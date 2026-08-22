//! In-process integration tests for the tonic `ControlTransport`: a minimal
//! test plane (the generated tonic server) exercises connect → auto-Register,
//! gate-stream round-trips, bearer-token metadata, the in-flight bound on
//! droppable messages when the plane is connected but not draining, and the
//! kill → reconnect → in-order replay path the client crate already owns.
#![cfg(feature = "tonic-transport")]

use std::net::SocketAddr;
use std::pin::Pin;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc as std_mpsc;
use std::time::Duration;

use parking_lot::Mutex;
use tokio::sync::mpsc as tokio_mpsc;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::{TcpListenerStream, UnboundedReceiverStream};
use tonic::{Request, Response, Status, Streaming};
use waddle_controlplane::grpc::proto::control_plane_server::{ControlPlane, ControlPlaneServer};
use waddle_controlplane::grpc::{
    CUSTOMER_ID_METADATA, GrpcConfig, GrpcTransport, PROJECT_ID_METADATA, SESSION_NONCE_METADATA,
    WORKSPACE_ID_METADATA,
};
use waddle_controlplane::{
    Backoff, ClientConfig, ClientMsg, ControlPlaneClient, ControlTransport, PlaneEvent, ServerMsg,
};
use waddle_types::pb::v0 as pb;

// ---------------------------------------------------------------------------
// Test plane
// ---------------------------------------------------------------------------

#[derive(Default)]
struct PlaneState {
    registers: Mutex<u32>,
    register_nonces: Mutex<Vec<String>>,
    /// The `authorization` metadata seen on Register calls.
    auth_seen: Mutex<Vec<Option<String>>>,
    rpc_metadata: Mutex<Vec<RpcMetadataSeen>>,
    seen_gate: Mutex<Vec<pb::GateClientMessage>>,
    seen_obs: Mutex<Vec<pb::ObservationUpdate>>,
    /// When set, `StreamObservations` accepts the RPC but never answers
    /// (a stalled — not dead — plane).
    stall_obs: AtomicBool,
    /// Push handle for plane → client gate messages (set when GateActions opens).
    gate_push: Mutex<Option<tokio_mpsc::UnboundedSender<Result<pb::GateServerMessage, Status>>>>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct RpcMetadataSeen {
    rpc: &'static str,
    authorization: Option<String>,
    customer_id: Option<String>,
    project_id: Option<String>,
    workspace_id: Option<String>,
    session_nonce: Option<String>,
}

fn metadata_string<T>(request: &Request<T>, name: &'static str) -> Option<String> {
    request
        .metadata()
        .get(name)
        .and_then(|value| value.to_str().ok())
        .map(String::from)
}

fn record_rpc<T>(state: &PlaneState, rpc: &'static str, request: &Request<T>) {
    state.rpc_metadata.lock().push(RpcMetadataSeen {
        rpc,
        authorization: metadata_string(request, "authorization"),
        customer_id: metadata_string(request, CUSTOMER_ID_METADATA),
        project_id: metadata_string(request, PROJECT_ID_METADATA),
        workspace_id: metadata_string(request, WORKSPACE_ID_METADATA),
        session_nonce: metadata_string(request, SESSION_NONCE_METADATA),
    });
}

struct TestPlane {
    state: Arc<PlaneState>,
}

type ServerStream<T> = Pin<Box<dyn tokio_stream::Stream<Item = Result<T, Status>> + Send>>;

#[tonic::async_trait]
impl ControlPlane for TestPlane {
    async fn register(
        &self,
        request: Request<pb::RegisterRequest>,
    ) -> Result<Response<pb::RegisterResponse>, Status> {
        *self.state.registers.lock() += 1;
        record_rpc(&self.state, "Register", &request);
        self.state.auth_seen.lock().push(
            request
                .metadata()
                .get("authorization")
                .and_then(|v| v.to_str().ok())
                .map(String::from),
        );
        self.state
            .register_nonces
            .lock()
            .push(request.get_ref().session_nonce.clone());
        Ok(Response::new(pb::RegisterResponse {
            session_id: "s-grpc".into(),
            // A plane that means to receive stills has to say so: the client
            // never puts a flag-scoped message on a connection that did not
            // accept its flag (`ClientMsg::connection_scoped_flag`), so
            // without this the shedding test below would have nothing to
            // shed.
            accepted_feature_flags: vec![
                waddle_controlplane::flags::STILLS.to_owned(),
                waddle_controlplane::flags::CONNECTOR_BINDING.to_owned(),
            ],
            ..Default::default()
        }))
    }

    async fn negotiate(
        &self,
        request: Request<pb::NegotiateRequest>,
    ) -> Result<Response<pb::NegotiateResponse>, Status> {
        record_rpc(&self.state, "Negotiate", &request);
        Ok(Response::new(pb::NegotiateResponse::default()))
    }

    type StreamObservationsStream = ServerStream<pb::ObservationAck>;

    async fn stream_observations(
        &self,
        request: Request<Streaming<pb::ObservationUpdate>>,
    ) -> Result<Response<Self::StreamObservationsStream>, Status> {
        record_rpc(&self.state, "StreamObservations", &request);
        if self.state.stall_obs.load(Ordering::SeqCst) {
            std::future::pending::<()>().await;
        }
        let state = self.state.clone();
        let mut inbound = request.into_inner();
        let (tx, rx) = tokio_mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(Ok(obs)) = inbound.next().await {
                state.seen_obs.lock().push(obs);
                let _ = tx.send(Ok(pb::ObservationAck::default()));
            }
        });
        Ok(Response::new(Box::pin(UnboundedReceiverStream::new(rx))))
    }

    type GateActionsStream = ServerStream<pb::GateServerMessage>;

    async fn gate_actions(
        &self,
        request: Request<Streaming<pb::GateClientMessage>>,
    ) -> Result<Response<Self::GateActionsStream>, Status> {
        record_rpc(&self.state, "GateActions", &request);
        let state = self.state.clone();
        let mut inbound = request.into_inner();
        let (tx, rx) = tokio_mpsc::unbounded_channel();
        *state.gate_push.lock() = Some(tx);
        tokio::spawn(async move {
            while let Some(Ok(msg)) = inbound.next().await {
                state.seen_gate.lock().push(msg);
            }
        });
        Ok(Response::new(Box::pin(UnboundedReceiverStream::new(rx))))
    }

    async fn claim_episode(
        &self,
        request: Request<pb::ClaimEpisodeRequest>,
    ) -> Result<Response<pb::ClaimEpisodeResponse>, Status> {
        record_rpc(&self.state, "ClaimEpisode", &request);
        Ok(Response::new(pb::ClaimEpisodeResponse {
            granted: true,
            ..Default::default()
        }))
    }

    async fn handoff_lease(
        &self,
        request: Request<pb::HandoffLeaseRequest>,
    ) -> Result<Response<pb::HandoffLeaseResponse>, Status> {
        record_rpc(&self.state, "HandoffLease", &request);
        Ok(Response::new(pb::HandoffLeaseResponse::default()))
    }

    type RequestResetStream = ServerStream<pb::ResetProgress>;

    async fn request_reset(
        &self,
        request: Request<pb::ResetRequest>,
    ) -> Result<Response<Self::RequestResetStream>, Status> {
        record_rpc(&self.state, "RequestReset", &request);
        // Two-phase progress ending in DONE, then the stream closes normally.
        let (tx, rx) = tokio_mpsc::unbounded_channel();
        let _ = tx.send(Ok(pb::ResetProgress {
            phase: pb::ResetPhase::Executing as i32,
            ..Default::default()
        }));
        let _ = tx.send(Ok(pb::ResetProgress {
            phase: pb::ResetPhase::Done as i32,
            ..Default::default()
        }));
        Ok(Response::new(Box::pin(UnboundedReceiverStream::new(rx))))
    }

    type HeartbeatStream = ServerStream<pb::HeartbeatAck>;

    async fn heartbeat(
        &self,
        request: Request<Streaming<pb::HeartbeatPing>>,
    ) -> Result<Response<Self::HeartbeatStream>, Status> {
        record_rpc(&self.state, "Heartbeat", &request);
        let mut inbound = request.into_inner();
        let (tx, rx) = tokio_mpsc::unbounded_channel();
        tokio::spawn(async move {
            while let Some(Ok(ping)) = inbound.next().await {
                let _ = tx.send(Ok(pb::HeartbeatAck {
                    echo_t_ns: ping.t_ns,
                    ..Default::default()
                }));
            }
        });
        Ok(Response::new(Box::pin(UnboundedReceiverStream::new(rx))))
    }
}

// ---------------------------------------------------------------------------
// Server harness: hard-kill on demand (dropping the runtime resets sockets)
// ---------------------------------------------------------------------------

struct TestServer {
    addr: SocketAddr,
    stop_tx: std_mpsc::Sender<()>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl TestServer {
    /// Bind and serve on `127.0.0.1:port` (0 = pick a free port).
    fn start(state: Arc<PlaneState>, port: u16) -> Self {
        let (addr_tx, addr_rx) = std_mpsc::channel::<SocketAddr>();
        let (stop_tx, stop_rx) = std_mpsc::channel::<()>();
        let thread = std::thread::Builder::new()
            .name("waddle-test-plane".into())
            .spawn(move || {
                let rt = tokio::runtime::Builder::new_current_thread()
                    .enable_all()
                    .build()
                    .expect("test-plane runtime");
                rt.block_on(async move {
                    let listener = tokio::net::TcpListener::bind(("127.0.0.1", port))
                        .await
                        .expect("bind test plane");
                    addr_tx
                        .send(listener.local_addr().expect("local addr"))
                        .expect("report addr");
                    let (halt_tx, halt_rx) = tokio_mpsc::unbounded_channel::<()>();
                    // Forward the sync stop signal into the async world.
                    std::thread::spawn(move || {
                        let _ = stop_rx.recv();
                        let _ = halt_tx.send(());
                    });
                    let serve = tonic::transport::Server::builder()
                        .add_service(ControlPlaneServer::new(TestPlane { state }))
                        .serve_with_incoming(TcpListenerStream::new(listener));
                    let mut halt_rx = halt_rx;
                    tokio::select! {
                        res = serve => res.expect("test plane serve"),
                        _ = halt_rx.recv() => {} // hard kill: fall out, drop the runtime
                    }
                });
                // Runtime drops here: every live connection is reset.
            })
            .expect("spawn test plane");
        let addr = addr_rx
            .recv_timeout(Duration::from_secs(5))
            .expect("test plane never bound");
        Self {
            addr,
            stop_tx,
            thread: Some(thread),
        }
    }

    fn kill(mut self) {
        let _ = self.stop_tx.send(());
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for TestServer {
    fn drop(&mut self) {
        let _ = self.stop_tx.send(());
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn client_config() -> ClientConfig {
    let mut cfg = ClientConfig::new(pb::RegisterRequest {
        project: "project-grpc".into(),
        customer_id: "customer-grpc".into(),
        workspace_id: "workspace-grpc".into(),
        ..Default::default()
    });
    cfg.backoff = Backoff {
        steps_ns: vec![20_000_000, 40_000_000],
        plateau_ns: 40_000_000,
    };
    cfg
}

fn grpc_config(addr: SocketAddr) -> GrpcConfig {
    GrpcConfig::new(format!("http://{addr}"))
}

#[allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only
fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    while !done() {
        assert!(std::time::Instant::now() < deadline, "timed out: {what}");
        std::thread::sleep(Duration::from_millis(2));
    }
}

fn expect_connected_and_registered(client: &ControlPlaneClient) {
    wait_for("Connected event", || {
        matches!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Connected)
        )
    });
    wait_for("Registered event", || {
        matches!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Registered(r)) if r.session_id == "s-grpc"
        )
    });
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn connect_to_unreachable_server_fails_cleanly() {
    // TEST-NET-1 address: nothing listens there; connect_timeout bounds it.
    let transport = GrpcTransport::new(GrpcConfig::new("http://127.0.0.1:9"));
    let registration = pb::RegisterRequest {
        session_nonce: "00000000000040008000000000000000".into(),
        ..Default::default()
    };
    let err = transport.connect(&registration).expect_err("must fail");
    let msg = format!("{err}");
    assert!(msg.contains("transport"), "unexpected error: {msg}");
}

#[test]
fn partial_exact_binding_fails_before_dial() {
    let transport = GrpcTransport::new(GrpcConfig::new("http://127.0.0.1:9"));
    let registration = pb::RegisterRequest {
        project: "project-grpc".into(),
        customer_id: "customer-grpc".into(),
        session_nonce: "00000000000040008000000000000000".into(),
        ..Default::default()
    };
    let error = transport
        .connect(&registration)
        .expect_err("a partial exact binding must fail closed");
    assert!(format!("{error}").contains("all present and non-empty"));
}

#[test]
fn stalled_observation_stream_open_does_not_freeze_the_pump() {
    let state = Arc::new(PlaneState::default());
    state.stall_obs.store(true, Ordering::SeqCst);
    let server = TestServer::start(state.clone(), 0);

    let transport = GrpcTransport::new(grpc_config(server.addr));
    let client = ControlPlaneClient::spawn(transport, client_config());
    expect_connected_and_registered(&client);

    // The first observation opens StreamObservations lazily; this plane
    // accepts the RPC and then stalls forever (slow, not dead — nothing
    // errors, so `Disconnected` must NOT be the escape hatch here).
    client.send(ClientMsg::Observation(pb::ObservationUpdate {
        t_ns: 1,
        ..Default::default()
    }));

    // The connection's pump must keep flowing in both directions regardless.
    let status = pb::GateClientMessage {
        msg: Some(pb::gate_client_message::Msg::Status(pb::GateStatus {
            tick_rate_hz: 15.0,
            ..Default::default()
        })),
    };
    client.send(ClientMsg::Gate(status.clone()));
    wait_for("gate message pumped despite the stalled obs open", || {
        !state.seen_gate.lock().is_empty()
    });
    assert_eq!(state.seen_gate.lock()[0], status);

    let directive = pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Claim(pb::ClaimDirective {
            kind: pb::ClaimDirectiveKind::Grant as i32,
            claim: None,
            directive_id: None,
        })),
    };
    state
        .gate_push
        .lock()
        .as_ref()
        .expect("gate stream open")
        .send(Ok(directive.clone()))
        .expect("push directive");
    wait_for("ClaimDirective still arrives on the ordered rx", || {
        matches!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Server(ServerMsg::Gate(m))) if m == directive
        )
    });

    client.shutdown();
}

#[test]
fn every_rpc_carries_the_same_exact_binding_and_session_nonce() {
    let state = Arc::new(PlaneState::default());
    let server = TestServer::start(state.clone(), 0);
    let transport = GrpcTransport::new(grpc_config(server.addr).with_token("secret-token"));
    let client = ControlPlaneClient::spawn(transport, client_config());
    expect_connected_and_registered(&client);

    client.send(ClientMsg::Negotiate(pb::NegotiateRequest::default()));
    client.send(ClientMsg::Observation(pb::ObservationUpdate::default()));
    client.send(ClientMsg::Gate(pb::GateClientMessage::default()));
    client.send(ClientMsg::Heartbeat(pb::HeartbeatPing::default()));
    client.send(ClientMsg::ClaimEpisode(pb::ClaimEpisodeRequest::default()));
    client.send(ClientMsg::HandoffLease(pb::HandoffLeaseRequest::default()));
    client.send(ClientMsg::RequestReset(pb::ResetRequest::default()));

    let expected_rpcs = [
        "Register",
        "Negotiate",
        "StreamObservations",
        "GateActions",
        "ClaimEpisode",
        "HandoffLease",
        "RequestReset",
        "Heartbeat",
    ];
    wait_for("all eight RPCs reached the plane", || {
        let seen = state.rpc_metadata.lock();
        expected_rpcs
            .iter()
            .all(|rpc| seen.iter().any(|metadata| metadata.rpc == *rpc))
    });

    let register_nonce = state
        .register_nonces
        .lock()
        .first()
        .cloned()
        .expect("Register request nonce");
    let seen = state.rpc_metadata.lock();
    for rpc in expected_rpcs {
        let metadata = seen
            .iter()
            .find(|metadata| metadata.rpc == rpc)
            .unwrap_or_else(|| panic!("missing metadata for {rpc}"));
        assert_eq!(
            metadata.authorization.as_deref(),
            Some("Bearer secret-token")
        );
        assert_eq!(metadata.customer_id.as_deref(), Some("customer-grpc"));
        assert_eq!(metadata.project_id.as_deref(), Some("project-grpc"));
        assert_eq!(metadata.workspace_id.as_deref(), Some("workspace-grpc"));
        assert_eq!(metadata.session_nonce.as_deref(), Some(&*register_nonce));
    }

    client.shutdown();
}

/// A plane that is CONNECTED but not draining must not turn bounded-rate
/// perception into unbounded memory: nothing errors in that state (so the
/// client never sees `Disconnected`, and its offline classification never
/// runs), which before the in-flight bound left every sampled still queued
/// forever behind a stream h2 had stopped polling.
///
/// This plane accepts `StreamObservations` and never reads it, so the
/// flow-control window closes and stays closed. The stills offered after
/// that must be shed (counted), not queued — and shedding must not tear the
/// connection down: it is the declared degradation, not a failure.
#[test]
fn stalled_observation_stream_sheds_stills_instead_of_queueing_them() {
    let state = Arc::new(PlaneState::default());
    state.stall_obs.store(true, Ordering::SeqCst);
    let server = TestServer::start(state.clone(), 0);

    let transport = GrpcTransport::new(grpc_config(server.addr));
    let client = ControlPlaneClient::spawn(transport.clone(), client_config());
    expect_connected_and_registered(&client);

    // 16 MB of stills offered at 16 KB each — an order of magnitude past any
    // plausible h2 window, so the sink is unmistakably not draining.
    const STILLS: u64 = 1000;
    for seq in 0..STILLS {
        client.send(ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: seq as i64,
            payload: Some(pb::observation_update::Payload::Still(pb::FrameStill {
                camera: "overhead".into(),
                frame_seq: seq,
                encoding: pb::CameraEncoding::Jpeg as i32,
                width: 1280,
                height: 720,
                data: vec![0xab; 16 * 1024],
            })),
        }));
    }

    // Most of them never enter the transport at all.
    wait_for("stills shed by the in-flight bound", || {
        transport.droppable_dropped() >= STILLS * 4 / 5
    });

    // Shedding perception is not a connection failure: no Disconnected, and
    // the client keeps taking sends.
    assert_eq!(
        client.try_recv_event(),
        None,
        "a shed still must not surface as a plane event"
    );
    client.send(ClientMsg::Observation(pb::ObservationUpdate {
        t_ns: 1,
        ..Default::default()
    }));

    client.shutdown();
}

#[test]
fn registers_round_trips_and_replays_after_server_restart() {
    let state = Arc::new(PlaneState::default());
    let server = TestServer::start(state.clone(), 0);
    let addr = server.addr;

    let transport = GrpcTransport::new(grpc_config(addr).with_token("secret-token"));
    let client = ControlPlaneClient::spawn(transport, client_config());

    // Connect → auto-Register arrives (with the bearer token as metadata).
    expect_connected_and_registered(&client);
    wait_for("register seen server-side", || *state.registers.lock() >= 1);
    assert_eq!(
        state.auth_seen.lock().first().cloned().flatten().as_deref(),
        Some("Bearer secret-token"),
    );

    // A GateClientMessage round-trips onto the plane's ordered stream.
    let status = pb::GateClientMessage {
        msg: Some(pb::gate_client_message::Msg::Status(pb::GateStatus {
            tick_rate_hz: 30.0,
            ..Default::default()
        })),
    };
    client.send(ClientMsg::Gate(status.clone()));
    wait_for("gate message seen server-side", || {
        !state.seen_gate.lock().is_empty()
    });
    assert_eq!(state.seen_gate.lock()[0], status);

    // A plane-side ClaimDirective arrives on the single ordered rx.
    let directive = pb::GateServerMessage {
        msg: Some(pb::gate_server_message::Msg::Claim(pb::ClaimDirective {
            kind: pb::ClaimDirectiveKind::Grant as i32,
            claim: None,
            directive_id: None,
        })),
    };
    state
        .gate_push
        .lock()
        .as_ref()
        .expect("gate stream open")
        .send(Ok(directive.clone()))
        .expect("push directive");
    wait_for("ClaimDirective on the ordered rx", || {
        matches!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Server(ServerMsg::Gate(m))) if m == directive
        )
    });

    // Kill the plane: the transport reports disconnected.
    server.kill();
    wait_for("Disconnected event", || {
        matches!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Disconnected)
        )
    });

    // Send while down: buffered (heartbeats would be dropped; observations replay).
    let obs = pb::ObservationUpdate {
        t_ns: 42,
        ..Default::default()
    };
    client.send(ClientMsg::Observation(obs.clone()));

    // Restart on the SAME port: reconnect, re-Register, replay in order.
    let _server2 = TestServer::start(state.clone(), addr.port());
    expect_connected_and_registered(&client);
    wait_for("second register", || *state.registers.lock() >= 2);
    let nonces = state.register_nonces.lock().clone();
    assert_eq!(nonces.len(), 2);
    assert_ne!(nonces[0], nonces[1], "every reconnect rotates its nonce");
    assert!(nonces.iter().all(|nonce| {
        nonce.len() == 32
            && nonce
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    }));
    wait_for("buffered observation replayed", || {
        state.seen_obs.lock().first() == Some(&obs)
    });

    client.shutdown();
}
