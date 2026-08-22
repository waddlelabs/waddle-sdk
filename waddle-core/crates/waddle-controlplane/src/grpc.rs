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
//! Backpressure: a plane can be connected and yet not draining — it accepts
//! `StreamObservations` and then stops reading it (or is simply slower than
//! the declared still rate, closing the h2 flow-control window). Nothing
//! errors in that state, so the client never sees `Disconnected` and its
//! offline classification never runs. Every stream sender here is therefore
//! metered by its own [`InflightLimit`]: droppable messages (stills,
//! heartbeats) stop at the cap instead of piling up in the channel behind a
//! stream h2 has stopped polling, and the shed count is readable via
//! [`GrpcTransport::droppable_dropped`]. History is never shed.
//!
//! Authentication and correlation are transport metadata per services.proto:
//! a configured token rides every RPC as `authorization: Bearer <token>`;
//! connector transports also carry one exact customer/project/workspace
//! binding and the per-connection session nonce on every RPC.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
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
use crate::inflight::{DEFAULT_INFLIGHT_CAP, Inflight, InflightLimit};
use crate::transport::{ClientMsg, ControlConn, ControlTransport, ServerMsg};

/// The generated `ControlPlane` service code (client + in-process test
/// server). Messages are `waddle_types::pb::v0` — codegen emits service
/// glue only (`extern_path`), so exactly one copy of the wire types exists.
#[allow(clippy::all, clippy::pedantic, missing_debug_implementations)]
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/waddle.v0.rs"));
}

use proto::control_plane_client::ControlPlaneClient as GeneratedClient;

/// Non-secret exact-binding metadata required together on every connector
/// RPC. The bearer credential remains project-scoped; these values select
/// the exact authorized workspace within that project.
pub const CUSTOMER_ID_METADATA: &str = "x-waddle-customer-id";
pub const PROJECT_ID_METADATA: &str = "x-waddle-project-id";
pub const WORKSPACE_ID_METADATA: &str = "x-waddle-workspace-id";
/// Non-secret per-connection correlation metadata. It equals
/// `RegisterRequest.session_nonce` and rotates on every reconnect.
pub const SESSION_NONCE_METADATA: &str = "x-waddle-session-nonce";

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

/// The tonic-backed [`ControlTransport`]. Stateless between connections
/// except for the shed counter: every `connect()` dials fresh (reconnect
/// policy lives in the client).
#[derive(Debug)]
pub struct GrpcTransport {
    config: GrpcConfig,
    /// Droppable messages shed by the in-flight bound, summed over every
    /// connection this transport has made (see [`crate::inflight`]).
    dropped: Arc<AtomicU64>,
}

impl GrpcTransport {
    pub fn new(config: GrpcConfig) -> Arc<Self> {
        Arc::new(Self {
            config,
            dropped: Arc::new(AtomicU64::new(0)),
        })
    }

    /// How many droppable messages (control-plane stills, heartbeats) this
    /// transport has shed to stay bounded while the plane was connected but
    /// not draining. Non-zero means the plane is not keeping up with the
    /// declared still rate — the designed degradation, never a loss of
    /// episode history, which is bounded only by the offline buffer.
    #[must_use]
    pub fn droppable_dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// Convenience mirroring `InMemoryTransport::new`'s shape.
pub fn connect(config: GrpcConfig) -> Arc<dyn ControlTransport> {
    GrpcTransport::new(config)
}

impl ControlTransport for GrpcTransport {
    fn connect(&self, registration: &pb::RegisterRequest) -> Result<ControlConn, PlaneError> {
        let config = self.config.clone();
        let registration = registration.clone();
        let dropped = self.dropped.clone();
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
                rt.block_on(run_conn(
                    config,
                    registration,
                    dropped,
                    cmd_std_rx,
                    server_tx,
                    &ready_tx,
                ));
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

/// Per-RPC bearer-token, exact-binding, and connection-correlation metadata.
#[derive(Clone)]
struct RpcMetadata {
    authorization: Option<MetadataValue<Ascii>>,
    customer_id: Option<MetadataValue<Ascii>>,
    project_id: Option<MetadataValue<Ascii>>,
    workspace_id: Option<MetadataValue<Ascii>>,
    session_nonce: MetadataValue<Ascii>,
}

impl tonic::service::Interceptor for RpcMetadata {
    fn call(
        &mut self,
        mut request: tonic::Request<()>,
    ) -> Result<tonic::Request<()>, tonic::Status> {
        if let Some(h) = &self.authorization {
            request.metadata_mut().insert("authorization", h.clone());
        }
        if let Some(value) = &self.customer_id {
            request
                .metadata_mut()
                .insert(CUSTOMER_ID_METADATA, value.clone());
        }
        if let Some(value) = &self.project_id {
            request
                .metadata_mut()
                .insert(PROJECT_ID_METADATA, value.clone());
        }
        if let Some(value) = &self.workspace_id {
            request
                .metadata_mut()
                .insert(WORKSPACE_ID_METADATA, value.clone());
        }
        request
            .metadata_mut()
            .insert(SESSION_NONCE_METADATA, self.session_nonce.clone());
        Ok(request)
    }
}

type Client = GeneratedClient<InterceptedService<Channel, RpcMetadata>>;

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
    registration: pb::RegisterRequest,
    dropped: Arc<AtomicU64>,
    cmd_std_rx: StdReceiver<ClientMsg>,
    out: StdSender<ServerMsg>,
    ready: &StdSender<Result<(), PlaneError>>,
) {
    let fail = |e: PlaneError| {
        let _ = ready.send(Err(e));
    };

    let authorization = match &config.token {
        Some(t) => match MetadataValue::try_from(format!("Bearer {t}")) {
            Ok(h) => Some(h),
            Err(e) => {
                return fail(PlaneError::Transport(format!("invalid bearer token: {e}")));
            }
        },
        None => None,
    };
    let metadata_value = |name: &str, value: &str| {
        MetadataValue::try_from(value)
            .map_err(|e| PlaneError::Transport(format!("invalid {name} metadata value: {e}")))
    };
    if registration.session_nonce.len() != 32
        || !registration
            .session_nonce
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
        || registration.session_nonce.as_bytes()[12] != b'4'
        || !matches!(
            registration.session_nonce.as_bytes()[16],
            b'8' | b'9' | b'a' | b'b'
        )
    {
        return fail(PlaneError::Transport(
            "invalid x-waddle-session-nonce: expected UUID-v4 lowercase hex".into(),
        ));
    }
    let session_nonce = match metadata_value(SESSION_NONCE_METADATA, &registration.session_nonce) {
        Ok(value) => value,
        Err(e) => return fail(e),
    };
    let binding_present = !registration.customer_id.is_empty()
        || !registration.workspace_id.is_empty()
        || registration.authorization_only;
    let (customer_id, project_id, workspace_id) = if binding_present {
        if registration.customer_id.is_empty()
            || registration.project.is_empty()
            || registration.workspace_id.is_empty()
        {
            return fail(PlaneError::Transport(
                "exact connector binding metadata values must be all present and non-empty".into(),
            ));
        }
        {
            let customer = match metadata_value(CUSTOMER_ID_METADATA, &registration.customer_id) {
                Ok(value) => value,
                Err(e) => return fail(e),
            };
            let project = match metadata_value(PROJECT_ID_METADATA, &registration.project) {
                Ok(value) => value,
                Err(e) => return fail(e),
            };
            let workspace = match metadata_value(WORKSPACE_ID_METADATA, &registration.workspace_id)
            {
                Ok(value) => value,
                Err(e) => return fail(e),
            };
            (Some(customer), Some(project), Some(workspace))
        }
    } else {
        (None, None, None)
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
        let mut client: Client = GeneratedClient::with_interceptor(
            channel,
            RpcMetadata {
                authorization,
                customer_id,
                project_id,
                workspace_id,
                session_nonce,
            },
        );

        // The two eager long-lived streams: the plane's down-paths. Each is
        // polled through `Inflight::into_inner`, which is where a metered
        // message releases its in-flight slot — the bound therefore tracks
        // what h2 has actually taken, not what was handed to the channel.
        let (gate_tx, gate_rx) = tokio_mpsc::unbounded_channel::<Inflight<pb::GateClientMessage>>();
        let gate_in = client
            .gate_actions(UnboundedReceiverStream::new(gate_rx).map(Inflight::into_inner))
            .await
            .map_err(|e| PlaneError::Transport(format!("GateActions open: {e}")))?
            .into_inner();
        let (hb_tx, hb_rx) = tokio_mpsc::unbounded_channel::<Inflight<pb::HeartbeatPing>>();
        let hb_in = client
            .heartbeat(UnboundedReceiverStream::new(hb_rx).map(Inflight::into_inner))
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

    // sync → async forwarder for the conn's tx side. The forwarder exits when
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
    // One in-flight bound PER stream: they drain independently, so a
    // saturated observation stream must not shed the heartbeats that keep
    // the session alive.
    let mut outbound = Outbound {
        gate_tx,
        gate_limit: InflightLimit::new(DEFAULT_INFLIGHT_CAP, dropped.clone()),
        hb_tx,
        hb_limit: InflightLimit::new(DEFAULT_INFLIGHT_CAP, dropped.clone()),
        obs_tx: None,
        obs_limit: InflightLimit::new(DEFAULT_INFLIGHT_CAP, dropped),
    };

    loop {
        tokio::select! {
            cmd = cmd_rx.recv() => {
                let Some(msg) = cmd else { break }; // conn dropped by the client
                if !dispatch(msg, &mut client, &mut outbound, &out, &fatal_tx) {
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

/// The connection's outbound stream senders, each with its own in-flight
/// bound. `obs_tx` is `None` until the first observation opens the stream.
struct Outbound {
    gate_tx: tokio_mpsc::UnboundedSender<Inflight<pb::GateClientMessage>>,
    gate_limit: Arc<InflightLimit>,
    hb_tx: tokio_mpsc::UnboundedSender<Inflight<pb::HeartbeatPing>>,
    hb_limit: Arc<InflightLimit>,
    obs_tx: Option<tokio_mpsc::UnboundedSender<Inflight<pb::ObservationUpdate>>>,
    obs_limit: Arc<InflightLimit>,
}

/// Queue one message on a stream sender, metered by that stream's in-flight
/// bound. `false` means the stream is gone (fatal to the connection); a
/// message shed by the bound returns `true` — shedding perception is the
/// declared degradation, not a connection failure.
fn queue<T>(
    tx: &tokio_mpsc::UnboundedSender<Inflight<T>>,
    limit: &Arc<InflightLimit>,
    value: T,
    droppable: bool,
) -> bool {
    match limit.admit(value, droppable) {
        Some(item) => tx.send(item).is_ok(),
        None => true,
    }
}

/// Route one client message to its RPC. Returns `false` on a fatal
/// transport condition (the caller tears the connection down).
///
/// Deliberately NOT async: every RPC here is fired as a spawned task (or a
/// non-blocking channel send), so nothing a stalled plane does can freeze
/// the connection's message pump — failures funnel back via `fatal`.
fn dispatch(
    msg: ClientMsg,
    client: &mut Client,
    outbound: &mut Outbound,
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

    // The classification, answered once per message (waddle-controlplane's
    // `ClientMsg::is_droppable`), then honoured by whichever stream carries
    // it: only a droppable message can ever be shed by the bound.
    let droppable = msg.is_droppable();
    match msg {
        ClientMsg::Gate(m) => queue(&outbound.gate_tx, &outbound.gate_limit, m, droppable),
        ClientMsg::Heartbeat(m) => queue(&outbound.hb_tx, &outbound.hb_limit, m, droppable),
        ClientMsg::Observation(o) => {
            let tx = outbound.obs_tx.get_or_insert_with(|| {
                // Lazy open, but spawned like every other RPC: a plane that
                // accepts the stream and then stalls must not freeze the
                // pump. Observations buffer in the channel until the stream
                // is live; an open failure funnels back as fatal.
                let (tx, rx) = tokio_mpsc::unbounded_channel::<Inflight<pb::ObservationUpdate>>();
                let mut c = client.clone();
                let fatal = fatal.clone();
                tokio::spawn(async move {
                    match c
                        .stream_observations(
                            UnboundedReceiverStream::new(rx).map(Inflight::into_inner),
                        )
                        .await
                    {
                        Ok(resp) => {
                            // Acks carry nothing the client consumes; drain
                            // them and treat a broken ack stream as
                            // connection death.
                            let mut acks = resp.into_inner();
                            loop {
                                match acks.next().await {
                                    Some(Ok(_)) => {}
                                    Some(Err(_)) | None => {
                                        let _ = fatal.send(());
                                        break;
                                    }
                                }
                            }
                        }
                        Err(_) => {
                            let _ = fatal.send(());
                        }
                    }
                });
                tx
            });
            queue(tx, &outbound.obs_limit, o, droppable)
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
