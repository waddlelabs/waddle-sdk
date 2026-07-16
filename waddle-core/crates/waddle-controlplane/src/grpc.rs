//! The real tonic gRPC [`ControlTransport`] (`tonic-transport` feature).
//!
//! Tokio confinement: each successful [`ControlTransport::connect`] spawns
//! ONE dedicated `waddle-controlplane-grpc` thread owning a private
//! current-thread tokio runtime; the trait surface stays sync/channel-based
//! and no tokio type appears in any public signature. The client crate keeps
//! sole ownership of backoff, offline buffering, and replay — this module
//! only maps one live connection onto the eight RPCs and reports death by
//! severing the [`ControlConn`] channels:
//!
//! - `Register` / `Negotiate` / `ClaimEpisode` / `HandoffLease` — unary.
//! - `GateActions` + `Heartbeat` — long-lived bidi streams, opened eagerly at
//!   connect (they carry the plane's directives and demotions).
//! - `StreamObservations` — opened lazily on the first observation; acks are
//!   drained (they carry nothing the client consumes).
//! - `RequestReset` — server-streaming; each `ResetProgress` funnels back
//!   through the single ordered rx like every other server message.
//!
//! Any transport-level error (a failed unary, a broken stream) is fatal to
//! the connection: the worker exits, both channels sever, and the client's
//! existing disconnect → backoff → replay machinery takes over.
//!
//! Authentication is transport metadata per services.proto: a configured
//! token rides every RPC as `authorization: Bearer <token>`.

use std::sync::Arc;
use std::sync::mpsc::{Receiver as StdReceiver, Sender as StdSender};
use std::time::Duration;

use tokio::sync::mpsc as tokio_mpsc;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::UnboundedReceiverStream;
use tonic::metadata::{Ascii, MetadataValue};
use tonic::service::interceptor::InterceptedService;
use tonic::transport::{Channel, ClientTlsConfig, Endpoint};
use waddle_types::pb::v0 as pb;

use crate::PlaneError;
use crate::transport::{ClientMsg, ControlConn, ControlTransport, ServerMsg};

/// The generated `ControlPlane` service code (client + in-process test
/// server). Messages are `waddle_types::pb::v0` — codegen emits service
/// glue only (`extern_path`), so exactly one copy of the wire types exists.
#[allow(clippy::all, clippy::pedantic, missing_debug_implementations)]
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/waddle.v0.rs"));
}

use proto::control_plane_client::ControlPlaneClient as GeneratedClient;

/// Endpoint connect timeout: bounds how long the client thread blocks inside
/// a single `connect()` attempt (the client's backoff owns retry pacing).
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// Transport configuration: the plane's URL (`http://` or `https://`; TLS
/// uses the platform's native roots) and an optional bearer token.
#[derive(Clone)]
pub struct GrpcConfig {
    pub url: String,
    pub token: Option<String>,
}

impl std::fmt::Debug for GrpcConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GrpcConfig")
            .field("url", &self.url)
            .field("token", &self.token.as_ref().map(|_| "<redacted>"))
            .finish()
    }
}

impl GrpcConfig {
    #[must_use]
    pub fn new(url: impl Into<String>) -> Self {
        Self {
            url: url.into(),
            token: None,
        }
    }

    #[must_use]
    pub fn with_token(mut self, token: impl Into<String>) -> Self {
        self.token = Some(token.into());
        self
    }
}

/// The tonic-backed [`ControlTransport`]. Stateless between connections:
/// every `connect()` dials fresh (reconnect policy lives in the client).
#[derive(Debug)]
pub struct GrpcTransport {
    config: GrpcConfig,
}

impl GrpcTransport {
    pub fn new(config: GrpcConfig) -> Arc<Self> {
        Arc::new(Self { config })
    }
}

/// Convenience mirroring `InMemoryTransport::new`'s shape.
pub fn connect(config: GrpcConfig) -> Arc<dyn ControlTransport> {
    GrpcTransport::new(config)
}

impl ControlTransport for GrpcTransport {
    fn connect(&self) -> Result<ControlConn, PlaneError> {
        let config = self.config.clone();
        let (client_tx, cmd_std_rx) = std::sync::mpsc::channel::<ClientMsg>();
        let (server_tx, client_rx) = std::sync::mpsc::channel::<ServerMsg>();
        let (ready_tx, ready_rx) = std::sync::mpsc::channel::<Result<(), PlaneError>>();

        std::thread::Builder::new()
            .name("waddle-controlplane-grpc".into())
            .spawn(move || {
                let rt = match tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        let _ = ready_tx
                            .send(Err(PlaneError::Transport(format!("tokio runtime: {e}"))));
                        return;
                    }
                };
                rt.block_on(run_conn(config, cmd_std_rx, server_tx, &ready_tx));
                // Dropping the runtime here cancels every in-flight task and
                // drops their `ServerMsg` senders: the client's rx severs.
            })
            .expect("spawn waddle-controlplane-grpc");

        match ready_rx.recv() {
            Ok(Ok(())) => Ok(ControlConn {
                tx: client_tx,
                rx: client_rx,
            }),
            Ok(Err(e)) => Err(e),
            Err(_) => Err(PlaneError::Transport(
                "grpc worker exited before signalling readiness".into(),
            )),
        }
    }
}

/// Per-RPC bearer-token metadata.
#[derive(Clone)]
struct BearerAuth {
    header: Option<MetadataValue<Ascii>>,
}

impl tonic::service::Interceptor for BearerAuth {
    fn call(
        &mut self,
        mut request: tonic::Request<()>,
    ) -> Result<tonic::Request<()>, tonic::Status> {
        if let Some(h) = &self.header {
            request.metadata_mut().insert("authorization", h.clone());
        }
        Ok(request)
    }
}

type Client = GeneratedClient<InterceptedService<Channel, BearerAuth>>;

fn build_endpoint(config: &GrpcConfig) -> Result<Endpoint, PlaneError> {
    let mut endpoint = Endpoint::from_shared(config.url.clone())
        .map_err(|e| PlaneError::Transport(format!("invalid endpoint {:?}: {e}", config.url)))?
        .connect_timeout(CONNECT_TIMEOUT);
    if config.url.starts_with("https://") {
        endpoint = endpoint
            .tls_config(ClientTlsConfig::new().with_enabled_roots())
            .map_err(|e| PlaneError::Transport(format!("tls config: {e}")))?;
    }
    Ok(endpoint)
}

/// The connection worker: dial, open the long-lived streams, signal ready,
/// then pump until anything breaks or the client drops the conn.
async fn run_conn(
    config: GrpcConfig,
    cmd_std_rx: StdReceiver<ClientMsg>,
    out: StdSender<ServerMsg>,
    ready: &StdSender<Result<(), PlaneError>>,
) {
    let fail = |e: PlaneError| {
        let _ = ready.send(Err(e));
    };

    let header = match &config.token {
        Some(t) => match MetadataValue::try_from(format!("Bearer {t}")) {
            Ok(h) => Some(h),
            Err(e) => {
                return fail(PlaneError::Transport(format!("invalid bearer token: {e}")));
            }
        },
        None => None,
    };
    let endpoint = match build_endpoint(&config) {
        Ok(ep) => ep,
        Err(e) => return fail(e),
    };

    // Everything before `ready` blocks the client thread inside `connect()`,
    // so the WHOLE pre-ready phase is deadline-bounded: `connect_timeout`
    // covers TCP establishment, but a plane that accepts and then stalls the
    // handshake or the stream opens must also fail fast (the client's
    // backoff owns retrying).
    let opened = tokio::time::timeout(CONNECT_TIMEOUT, async {
        let channel = endpoint
            .connect()
            .await
            .map_err(|e| PlaneError::Transport(format!("connect: {e}")))?;
        let mut client: Client = GeneratedClient::with_interceptor(channel, BearerAuth { header });

        // The two eager long-lived streams: the plane's down-paths.
        let (gate_tx, gate_rx) = tokio_mpsc::unbounded_channel::<pb::GateClientMessage>();
        let gate_in = client
            .gate_actions(UnboundedReceiverStream::new(gate_rx))
            .await
            .map_err(|e| PlaneError::Transport(format!("GateActions open: {e}")))?
            .into_inner();
        let (hb_tx, hb_rx) = tokio_mpsc::unbounded_channel::<pb::HeartbeatPing>();
        let hb_in = client
            .heartbeat(UnboundedReceiverStream::new(hb_rx))
            .await
            .map_err(|e| PlaneError::Transport(format!("Heartbeat open: {e}")))?
            .into_inner();
        Ok((client, gate_tx, gate_in, hb_tx, hb_in))
    })
    .await
    .unwrap_or_else(|_| {
        Err(PlaneError::Transport(format!(
            "connection setup timed out after {CONNECT_TIMEOUT:?}"
        )))
    });
    let (mut client, gate_tx, mut gate_in, hb_tx, mut hb_in) = match opened {
        Ok(parts) => parts,
        Err(e) => return fail(e),
    };

    let _ = ready.send(Ok(()));

    // sync → async bridge for the conn's tx side. The forwarder exits when
    // the client drops the `ControlConn` (recv errs) or the worker dies
    // (send errs on the next forward).
    let (cmd_tx, mut cmd_rx) = tokio_mpsc::unbounded_channel::<ClientMsg>();
    std::thread::Builder::new()
        .name("waddle-controlplane-grpc-tx".into())
        .spawn(move || {
            while let Ok(msg) = cmd_std_rx.recv() {
                if cmd_tx.send(msg).is_err() {
                    break;
                }
            }
        })
        .expect("spawn waddle-controlplane-grpc-tx");

    // Fatal-error funnel for spawned per-RPC tasks.
    let (fatal_tx, mut fatal_rx) = tokio_mpsc::unbounded_channel::<()>();
    let mut obs_tx: Option<tokio_mpsc::UnboundedSender<pb::ObservationUpdate>> = None;

    loop {
        tokio::select! {
            cmd = cmd_rx.recv() => {
                let Some(msg) = cmd else { break }; // conn dropped by the client
                if !dispatch(msg, &mut client, &gate_tx, &hb_tx, &mut obs_tx, &out, &fatal_tx).await {
                    break;
                }
            }
            item = gate_in.next() => match item {
                Some(Ok(m)) => {
                    if out.send(ServerMsg::Gate(m)).is_err() {
                        break;
                    }
                }
                Some(Err(_)) | None => break,
            },
            item = hb_in.next() => match item {
                Some(Ok(m)) => {
                    if out.send(ServerMsg::HeartbeatAck(m)).is_err() {
                        break;
                    }
                }
                Some(Err(_)) | None => break,
            },
            _ = fatal_rx.recv() => break,
        }
    }
}

/// Route one client message to its RPC. Returns `false` on a fatal
/// transport condition (the caller tears the connection down).
async fn dispatch(
    msg: ClientMsg,
    client: &mut Client,
    gate_tx: &tokio_mpsc::UnboundedSender<pb::GateClientMessage>,
    hb_tx: &tokio_mpsc::UnboundedSender<pb::HeartbeatPing>,
    obs_tx: &mut Option<tokio_mpsc::UnboundedSender<pb::ObservationUpdate>>,
    out: &StdSender<ServerMsg>,
    fatal: &tokio_mpsc::UnboundedSender<()>,
) -> bool {
    /// Fire a unary RPC as a task; the response funnels back through the
    /// single ordered rx, an RPC error tears the connection down.
    macro_rules! unary {
        ($method:ident, $req:expr, $wrap:expr) => {{
            let mut c = client.clone();
            let out = out.clone();
            let fatal = fatal.clone();
            let req = $req;
            tokio::spawn(async move {
                match c.$method(req).await {
                    Ok(resp) => {
                        let _ = out.send($wrap(resp.into_inner()));
                    }
                    Err(_) => {
                        let _ = fatal.send(());
                    }
                }
            });
            true
        }};
    }

    match msg {
        ClientMsg::Gate(m) => gate_tx.send(m).is_ok(),
        ClientMsg::Heartbeat(m) => hb_tx.send(m).is_ok(),
        ClientMsg::Observation(o) => {
            if obs_tx.is_none() {
                let (tx, rx) = tokio_mpsc::unbounded_channel();
                match client
                    .stream_observations(UnboundedReceiverStream::new(rx))
                    .await
                {
                    Ok(resp) => {
                        // Acks carry nothing the client consumes; drain them
                        // and treat a broken ack stream as connection death.
                        let mut acks = resp.into_inner();
                        let fatal = fatal.clone();
                        tokio::spawn(async move {
                            loop {
                                match acks.next().await {
                                    Some(Ok(_)) => {}
                                    Some(Err(_)) | None => {
                                        let _ = fatal.send(());
                                        break;
                                    }
                                }
                            }
                        });
                        *obs_tx = Some(tx);
                    }
                    Err(_) => return false,
                }
            }
            obs_tx.as_ref().is_some_and(|tx| tx.send(o).is_ok())
        }
        ClientMsg::Register(r) => unary!(register, r, ServerMsg::Registered),
        ClientMsg::Negotiate(r) => unary!(negotiate, r, ServerMsg::Negotiated),
        ClientMsg::ClaimEpisode(r) => unary!(claim_episode, r, ServerMsg::ClaimResponse),
        ClientMsg::HandoffLease(r) => unary!(handoff_lease, r, ServerMsg::LeaseResponse),
        ClientMsg::RequestReset(r) => {
            // Server-streaming: pump every ResetProgress back through the
            // ordered rx. Normal end-of-stream (after DONE) is not an error.
            let mut c = client.clone();
            let out = out.clone();
            let fatal = fatal.clone();
            tokio::spawn(async move {
                match c.request_reset(r).await {
                    Ok(resp) => {
                        let mut progress = resp.into_inner();
                        loop {
                            match progress.next().await {
                                Some(Ok(p)) => {
                                    if out.send(ServerMsg::ResetProgress(p)).is_err() {
                                        break;
                                    }
                                }
                                Some(Err(_)) => {
                                    let _ = fatal.send(());
                                    break;
                                }
                                None => break,
                            }
                        }
                    }
                    Err(_) => {
                        let _ = fatal.send(());
                    }
                }
            });
            true
        }
    }
}
