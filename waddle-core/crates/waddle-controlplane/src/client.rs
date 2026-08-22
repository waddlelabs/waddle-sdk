//! The control-plane client: one thread owning connect → register →
//! pump, with backoff reconnect and in-order replay of buffered messages.
//!
//! The one thread is also what makes the offline classification real: it
//! drains the (unbounded) command channel into the bounded offline buffer
//! continuously while backing off, so nothing queues behind a sleeping
//! reconnect. See [`backoff_draining`].
//!
//! And it is the only place that knows WHICH connection a message leaves on,
//! which is what per-connection feature negotiation needs: a flag-scoped
//! message ([`ClientMsg::connection_scoped_flag`]) is filtered on the way out
//! against this connection's `RegisterResponse`, and never enters the offline
//! buffer, so it cannot reach a plane that did not accept its flag by either
//! route.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::Mutex;
use waddle_types::pb::v0 as pb;

use crate::backoff::Backoff;
use crate::buffer::OfflineBuffer;
use crate::transport::{ClientMsg, ControlTransport, RegistrationRejection, ServerMsg};

/// Events surfaced to the runtime's reducer.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // moved once per event, never stored in bulk
pub enum PlaneEvent {
    /// A physical transport connection exists. Registration has not yet
    /// succeeded, so this is never authority or recovery by itself.
    Connected,
    Registered(pb::RegisterResponse),
    /// Register failed with a stable transport-supplied code. The connection
    /// is still discarded and no buffered or live traffic is replayed.
    RegistrationRejected(RegistrationRejection),
    Server(ServerMsg),
    Disconnected,
    /// The offline buffer dropped its oldest entries (loud, never silent).
    BufferOverflowed {
        dropped: u64,
    },
}

#[derive(Debug, Clone)]
pub struct ClientConfig {
    pub register: pb::RegisterRequest,
    pub backoff: Backoff,
    pub buffer_capacity: usize,
    /// Poll cadence of the pump loop.
    pub poll: Duration,
}

impl ClientConfig {
    #[must_use]
    pub fn new(register: pb::RegisterRequest) -> Self {
        Self {
            register,
            backoff: Backoff::production(),
            buffer_capacity: 4096,
            poll: Duration::from_millis(1),
        }
    }
}

/// Handle owned by the runtime. Dropping it (or calling `shutdown`) stops
/// the client thread.
#[derive(Debug)]
pub struct ControlPlaneClient {
    cmd_tx: Sender<ClientMsg>,
    events_rx: Mutex<Receiver<PlaneEvent>>,
    shutdown: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl ControlPlaneClient {
    pub fn spawn(transport: Arc<dyn ControlTransport>, config: ClientConfig) -> Self {
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<ClientMsg>();
        let (events_tx, events_rx) = std::sync::mpsc::channel::<PlaneEvent>();
        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_flag = shutdown.clone();

        let thread = std::thread::Builder::new()
            .name("waddle-controlplane".into())
            .spawn(move || run(transport, config, &cmd_rx, &events_tx, &shutdown_flag))
            .expect("spawn control-plane client");

        Self {
            cmd_tx,
            events_rx: Mutex::new(events_rx),
            shutdown,
            thread: Some(thread),
        }
    }

    /// Queue a message (buffered while disconnected, except heartbeats).
    pub fn send(&self, msg: ClientMsg) {
        let _ = self.cmd_tx.send(msg);
    }

    pub fn try_recv_event(&self) -> Option<PlaneEvent> {
        self.events_rx.lock().try_recv().ok()
    }

    /// Blocking receive with timeout (for tests and the runtime pump).
    pub fn recv_event_timeout(&self, timeout: Duration) -> Option<PlaneEvent> {
        self.events_rx.lock().recv_timeout(timeout).ok()
    }

    pub fn shutdown(mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for ControlPlaneClient {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn run(
    transport: Arc<dyn ControlTransport>,
    config: ClientConfig,
    cmd_rx: &Receiver<ClientMsg>,
    events_tx: &Sender<PlaneEvent>,
    shutdown: &AtomicBool,
) {
    let mut buffer: OfflineBuffer<ClientMsg> = OfflineBuffer::new(config.buffer_capacity);
    let mut attempt: u32 = 0;

    'reconnect: while !shutdown.load(Ordering::SeqCst) {
        // One nonce identifies every independently opened RPC belonging to
        // this physical connection. It is intentionally rotated before each
        // dial, including a failed dial, and is also the value inside the
        // Register barrier so a gateway can reject mismatched streams.
        let session_nonce = uuid::Uuid::new_v4().simple().to_string();
        let mut register = config.register.clone();
        register.session_nonce.clone_from(&session_nonce);
        let conn = match transport.connect(&register) {
            Ok(conn) => conn,
            Err(_) => {
                // A failed initial dial is a partition too. The runtime must
                // not retain its optimistic startup state indefinitely when
                // the configured supervision plane cannot be reached.
                let _ = events_tx.send(PlaneEvent::Disconnected);
                let delay = config.backoff.delay_ns(attempt);
                attempt = attempt.saturating_add(1);
                backoff_draining(delay, shutdown, cmd_rx, &mut buffer, events_tx);
                continue 'reconnect;
            }
        };
        // What THIS connection accepted, from its own `RegisterResponse`.
        // Empty until it answers — and a connection that has not answered has
        // accepted nothing, so a flag-scoped message offered in that window
        // waits for no one (see the outbound filter below).
        let mut accepted: Vec<String> = Vec::new();
        let _ = events_tx.send(PlaneEvent::Connected);

        // Register is a barrier, not merely the first write. Nothing else
        // may reach a fresh connection until the plane has authenticated
        // this exact registration and returned the connection's feature
        // answer. In particular, a project-scoped connector credential is
        // not enough to route GateActions/observations to a workspace before
        // Register's customer/project/workspace tuple has been accepted.
        if conn.tx.send(ClientMsg::Register(register)).is_err() {
            let _ = events_tx.send(PlaneEvent::Disconnected);
            backoff_after_disconnect(
                &config,
                &mut attempt,
                shutdown,
                cmd_rx,
                &mut buffer,
                events_tx,
            );
            continue 'reconnect;
        }

        let before = buffer.dropped();
        let registered = await_registration(&conn, cmd_rx, &mut buffer, shutdown, config.poll);
        report_overflow(before, &buffer, events_tx);
        if shutdown.load(Ordering::SeqCst) {
            return;
        }

        let Some(registered) = registered else {
            let _ = events_tx.send(PlaneEvent::Disconnected);
            backoff_after_disconnect(
                &config,
                &mut attempt,
                shutdown,
                cmd_rx,
                &mut buffer,
                events_tx,
            );
            continue 'reconnect;
        };
        let registered = match registered {
            RegistrationOutcome::Accepted(response) => response,
            RegistrationOutcome::Rejected(rejection) => {
                let _ = events_tx.send(PlaneEvent::RegistrationRejected(rejection));
                let _ = events_tx.send(PlaneEvent::Disconnected);
                backoff_after_disconnect(
                    &config,
                    &mut attempt,
                    shutdown,
                    cmd_rx,
                    &mut buffer,
                    events_tx,
                );
                continue 'reconnect;
            }
        };
        accepted.clone_from(&registered.accepted_feature_flags);
        let connector_binding_required = !config.register.customer_id.is_empty()
            || !config.register.workspace_id.is_empty()
            || config.register.authorization_only;
        let connector_binding_accepted = accepted
            .iter()
            .any(|flag| flag == crate::flags::CONNECTOR_BINDING);
        // Surface the exact response so the runtime/SDK can fail the
        // hardware-free authorization probe legibly. It still is not a
        // usable connection when the binding flag was refused: no replay or
        // live traffic follows it, and the transport is discarded below.
        let _ = events_tx.send(PlaneEvent::Registered(registered));
        if connector_binding_required && !connector_binding_accepted {
            let _ = events_tx.send(PlaneEvent::Disconnected);
            backoff_after_disconnect(
                &config,
                &mut attempt,
                shutdown,
                cmd_rx,
                &mut buffer,
                events_tx,
            );
            continue 'reconnect;
        }

        attempt = 0;
        if !replay_buffer(&conn, &mut buffer) {
            let _ = events_tx.send(PlaneEvent::Disconnected);
            backoff_after_disconnect(
                &config,
                &mut attempt,
                shutdown,
                cmd_rx,
                &mut buffer,
                events_tx,
            );
            continue 'reconnect;
        }

        // Pump until the connection dies or shutdown.
        loop {
            if shutdown.load(Ordering::SeqCst) {
                return;
            }
            // Outbound.
            loop {
                match cmd_rx.try_recv() {
                    Ok(msg) => {
                        // VERSIONING §3, enforced where the connection is
                        // actually known: a message whose content is legal
                        // only under a negotiated flag
                        // ([`ClientMsg::connection_scoped_flag`]) goes out
                        // only on a connection that accepted that flag. The
                        // producer withholds these too, but it decides on
                        // another thread against a mirror of the last
                        // answer — this is the point that sees which
                        // connection the message would leave on.
                        if let Some(flag) = msg.connection_scoped_flag()
                            && !accepted.iter().any(|f| f == flag)
                        {
                            continue;
                        }
                        if let Err(failed) = conn.tx.send(msg) {
                            // The connection died mid-send: re-buffer the
                            // message so it replays in order, never lost.
                            if failed.0.buffer_when_offline() {
                                buffer.push(failed.0);
                            }
                            let _ = events_tx.send(PlaneEvent::Disconnected);
                            continue 'reconnect;
                        }
                    }
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => return,
                }
            }
            // Inbound.
            match conn.try_recv() {
                // One Register response belongs to the handshake above.
                // A second response on the same connection is a protocol
                // violation; forwarding it could replace a negotiated
                // answer without a new connection boundary.
                Ok(Some(ServerMsg::Registered(_))) => {
                    let _ = events_tx.send(PlaneEvent::Disconnected);
                    continue 'reconnect;
                }
                Ok(Some(msg)) => {
                    let _ = events_tx.send(PlaneEvent::Server(msg));
                }
                Ok(None) => std::thread::sleep(config.poll),
                Err(_) => {
                    let _ = events_tx.send(PlaneEvent::Disconnected);
                    // Buffer whatever arrives while we are down.
                    break;
                }
            }
        }

        // Disconnected: keep draining commands into the buffer while backing
        // off, so nothing is lost out of order.
        let delay = config.backoff.delay_ns(attempt);
        attempt = attempt.saturating_add(1);
        backoff_draining(delay, shutdown, cmd_rx, &mut buffer, events_tx);
    }
}

/// Back off before the next connection attempt, draining the command channel
/// into the offline buffer the whole time, and report any overflow it caused.
///
/// The drain must happen DURING the wait, not merely around it. The command
/// channel is unbounded and everything the session produces while the plane
/// is unreachable lands there, so a wait that does not drain is exactly the
/// unbounded queue the bounded offline buffer exists to prevent: a droppable
/// message ([`ClientMsg::is_droppable`] — control-plane stills, heartbeats)
/// would sit there for a whole backoff plateau (16 s in production) and then
/// be handed to the plane as a stale picture, instead of being dropped
/// within milliseconds of arriving, and history would grow without the
/// drop-oldest bound (or its loud `BufferOverflowed`) applying at all.
fn backoff_draining(
    delay_ns: i64,
    shutdown: &AtomicBool,
    cmd_rx: &Receiver<ClientMsg>,
    buffer: &mut OfflineBuffer<ClientMsg>,
    events_tx: &Sender<PlaneEvent>,
) {
    let before = buffer.dropped();
    let mut remaining = delay_ns.max(0) as u64;
    const SLICE: u64 = 5_000_000; // 5 ms
    loop {
        buffer_pending(cmd_rx, buffer);
        if remaining == 0 || shutdown.load(Ordering::SeqCst) {
            break;
        }
        let step = remaining.min(SLICE);
        std::thread::sleep(Duration::from_nanos(step));
        remaining -= step;
    }
    let dropped = buffer.dropped() - before;
    if dropped > 0 {
        let _ = events_tx.send(PlaneEvent::BufferOverflowed { dropped });
    }
}

fn buffer_pending(cmd_rx: &Receiver<ClientMsg>, buffer: &mut OfflineBuffer<ClientMsg>) {
    while let Ok(msg) = cmd_rx.try_recv() {
        if msg.buffer_when_offline() {
            buffer.push(msg);
        }
    }
}

fn await_registration(
    conn: &crate::transport::ControlConn,
    cmd_rx: &Receiver<ClientMsg>,
    buffer: &mut OfflineBuffer<ClientMsg>,
    shutdown: &AtomicBool,
    poll: Duration,
) -> Option<RegistrationOutcome> {
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return None;
        }
        // Commands produced during the handshake are classified as offline:
        // durable history stays bounded for replay, while liveness,
        // perception, and connection-scoped requests never cross into a
        // connection that has not accepted them.
        buffer_pending(cmd_rx, buffer);
        match conn.try_recv() {
            Ok(Some(ServerMsg::Registered(response))) => {
                return Some(RegistrationOutcome::Accepted(response));
            }
            Ok(Some(ServerMsg::RegistrationRejected(rejection))) => {
                return Some(RegistrationOutcome::Rejected(rejection));
            }
            // No server down-path is legal before Register completes. Treat
            // one as a broken connection rather than executing a directive
            // whose binding has not been authenticated.
            Ok(Some(_)) | Err(_) => return None,
            Ok(None) => std::thread::sleep(poll),
        }
    }
}

enum RegistrationOutcome {
    Accepted(pb::RegisterResponse),
    Rejected(RegistrationRejection),
}

fn backoff_after_disconnect(
    config: &ClientConfig,
    attempt: &mut u32,
    shutdown: &AtomicBool,
    cmd_rx: &Receiver<ClientMsg>,
    buffer: &mut OfflineBuffer<ClientMsg>,
    events_tx: &Sender<PlaneEvent>,
) {
    let delay = config.backoff.delay_ns(*attempt);
    *attempt = attempt.saturating_add(1);
    backoff_draining(delay, shutdown, cmd_rx, buffer, events_tx);
}

fn report_overflow(before: u64, buffer: &OfflineBuffer<ClientMsg>, events_tx: &Sender<PlaneEvent>) {
    let dropped = buffer.dropped() - before;
    if dropped > 0 {
        let _ = events_tx.send(PlaneEvent::BufferOverflowed { dropped });
    }
}

/// Replay the authenticated connection's durable backlog without losing the
/// failed write or the unattempted suffix if the connection dies mid-replay.
fn replay_buffer(
    conn: &crate::transport::ControlConn,
    buffer: &mut OfflineBuffer<ClientMsg>,
) -> bool {
    let pending: Vec<ClientMsg> = buffer.drain().collect();
    let mut pending = pending.into_iter();
    while let Some(msg) = pending.next() {
        if let Err(failed) = conn.tx.send(msg) {
            if failed.0.buffer_when_offline() {
                buffer.push(failed.0);
            }
            for remaining in pending {
                if remaining.buffer_when_offline() {
                    buffer.push(remaining);
                }
            }
            return false;
        }
    }
    true
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only
mod tests {
    use super::*;
    use crate::transport::InMemoryTransport;
    use parking_lot::Mutex;

    fn test_config() -> ClientConfig {
        let mut cfg = ClientConfig::new(pb::RegisterRequest {
            project: "p".into(),
            ..Default::default()
        });
        cfg.backoff = Backoff {
            steps_ns: vec![1_000_000, 2_000_000],
            plateau_ns: 2_000_000,
        };
        cfg
    }

    /// Block until the client has drained its command channel *while
    /// offline*: every message queued before this call is now classified —
    /// buffered as history, or dropped as droppable — and can only reach
    /// the plane as a replay.
    ///
    /// The offline loop is `connect` → [`backoff_draining`] → `connect`, so
    /// two further refused dials bracket one complete drain that began
    /// after this call returned from its first `connect_attempts()` read.
    /// That is a happens-before a test can assert on; "sleep less than the
    /// backoff step" is a wall-clock race a loaded machine loses, and
    /// losing it silently turns a live forward into what reads like a
    /// replay. The transport must be refusing dials
    /// ([`InMemoryTransport::refuse_connections`]) — a connected client
    /// dials no more.
    fn wait_offline_drain(transport: &InMemoryTransport) {
        let target = transport.connect_attempts().saturating_add(2);
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        while transport.connect_attempts() < target {
            assert!(
                std::time::Instant::now() < deadline,
                "the client stopped dialling a refused plane"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn registration_is_a_barrier_for_buffered_history() {
        let (wire_tx, wire_rx) = std::sync::mpsc::channel();
        let (server_tx, client_rx) = std::sync::mpsc::channel();
        let conn = crate::transport::ControlConn {
            tx: wire_tx,
            rx: client_rx,
        };
        let (cmd_tx, cmd_rx) = std::sync::mpsc::channel();
        let history = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 41,
            ..Default::default()
        });
        cmd_tx.send(history.clone()).unwrap();
        server_tx
            .send(ServerMsg::Registered(pb::RegisterResponse {
                session_id: "registered".into(),
                ..Default::default()
            }))
            .unwrap();

        let mut buffer = OfflineBuffer::new(8);
        let shutdown = AtomicBool::new(false);
        let response =
            await_registration(&conn, &cmd_rx, &mut buffer, &shutdown, Duration::ZERO).unwrap();

        assert!(matches!(
            response,
            RegistrationOutcome::Accepted(response) if response.session_id == "registered"
        ));
        assert!(
            wire_rx.try_recv().is_err(),
            "history must not reach the transport before registration"
        );
        assert_eq!(buffer.len(), 1);
        assert!(replay_buffer(&conn, &mut buffer));
        assert_eq!(wire_rx.recv().unwrap(), history);
    }

    #[test]
    fn a_pre_registration_directive_breaks_the_connection() {
        let (wire_tx, _wire_rx) = std::sync::mpsc::channel();
        let (server_tx, client_rx) = std::sync::mpsc::channel();
        let conn = crate::transport::ControlConn {
            tx: wire_tx,
            rx: client_rx,
        };
        let (_cmd_tx, cmd_rx) = std::sync::mpsc::channel();
        server_tx
            .send(ServerMsg::Gate(pb::GateServerMessage::default()))
            .unwrap();

        let mut buffer = OfflineBuffer::new(8);
        assert!(
            await_registration(
                &conn,
                &cmd_rx,
                &mut buffer,
                &AtomicBool::new(false),
                Duration::ZERO,
            )
            .is_none(),
            "a server down-path is not authenticated before Register"
        );
    }

    #[test]
    fn connects_registers_and_forwards() {
        let seen: Arc<Mutex<Vec<ClientMsg>>> = Arc::new(Mutex::new(Vec::new()));
        let seen2 = seen.clone();
        let transport = InMemoryTransport::new(move |msg, tx| {
            if let ClientMsg::Register(_) = &msg {
                let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                    session_id: "s1".into(),
                    ..Default::default()
                }));
            }
            seen2.lock().push(msg);
        });
        let client = ControlPlaneClient::spawn(transport, test_config());

        assert_eq!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Connected)
        );
        let registered = client.recv_event_timeout(Duration::from_secs(1));
        assert!(matches!(registered, Some(PlaneEvent::Registered(r)) if r.session_id == "s1"));

        client.send(ClientMsg::Heartbeat(pb::HeartbeatPing {
            session_id: "s1".into(),
            ..Default::default()
        }));
        // Wait until the server saw both messages.
        for _ in 0..200 {
            if seen.lock().len() >= 2 {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        assert!(matches!(seen.lock()[1], ClientMsg::Heartbeat(_)));
        client.shutdown();
    }

    /// A plane that is unreachable from the start is the same "not taking
    /// messages" condition as a partition, and the classification must hold
    /// there too: while connect attempts fail, the client drains its command
    /// channel into the bounded offline buffer on every backoff slice, so a
    /// still is dropped within milliseconds of arriving. Before that drain,
    /// the whole outage sat in the unbounded command channel and every stale
    /// still was handed to the plane the moment it came up.
    #[test]
    fn stills_offered_while_connects_fail_are_dropped_never_replayed() {
        let seen: Arc<Mutex<Vec<ClientMsg>>> = Arc::new(Mutex::new(Vec::new()));
        let seen2 = seen.clone();
        let transport = InMemoryTransport::new(move |msg, tx| {
            if matches!(&msg, ClientMsg::Register(_)) {
                let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            }
            seen2.lock().push(msg);
        });
        // Unreachable from the start and until this test heals it — the
        // offline window is bounded by the test's own progress, never by a
        // backoff step.
        transport.refuse_connections();
        let client = ControlPlaneClient::spawn(transport.clone(), test_config());
        assert_eq!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Disconnected),
            "an unreachable configured plane is a partition from startup"
        );

        let still = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 7,
            payload: Some(pb::observation_update::Payload::Still(pb::FrameStill {
                camera: "overhead".into(),
                data: vec![0xff, 0xd8, 0xff],
                ..Default::default()
            })),
        });
        let proprio = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 8,
            payload: Some(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    joint_pos: vec![1.0],
                    ..Default::default()
                },
            )),
        });
        client.send(still);
        client.send(proprio.clone());
        // Both are classified offline before the plane can come back, so
        // anything the plane sees below is a replay, never a live forward.
        wait_offline_drain(&transport);
        transport.allow_connections();

        // The connection now succeeds and replays history in order.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let msgs = seen.lock();
            if msgs.contains(&proprio) {
                assert!(
                    !msgs.iter().any(|m| matches!(
                        m,
                        ClientMsg::Observation(o)
                            if matches!(
                                o.payload,
                                Some(pb::observation_update::Payload::Still(_))
                            )
                    )),
                    "a still offered while the plane was unreachable must never reach it"
                );
                break;
            }
            drop(msgs);
            assert!(
                std::time::Instant::now() < deadline,
                "history never replayed"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        client.shutdown();
    }

    /// The offline buffer is where a per-connection answer could outlive the
    /// connection that gave it. The registration barrier now withholds all
    /// replay until the next connection answers Register, but a named-part
    /// `ProprioSample` (flag
    /// `waddle.v0.parts`) offered while the plane is unreachable never
    /// reaches it — VERSIONING §3 — while the same sample under the sole
    /// part is ordinary history and replays in full.
    #[test]
    fn named_part_observations_offered_while_offline_are_never_replayed() {
        let seen: Arc<Mutex<Vec<ClientMsg>>> = Arc::new(Mutex::new(Vec::new()));
        let seen2 = seen.clone();
        let transport = InMemoryTransport::new(move |msg, tx| {
            if matches!(&msg, ClientMsg::Register(_)) {
                let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            }
            seen2.lock().push(msg);
        });
        transport.refuse_connections();
        let client = ControlPlaneClient::spawn(transport.clone(), test_config());

        let named = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 9,
            payload: Some(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    part: "left".into(),
                    joint_pos: vec![0.5; 7],
                    ..Default::default()
                },
            )),
        });
        let sole = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 10,
            payload: Some(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    joint_pos: vec![0.5; 14],
                    ..Default::default()
                },
            )),
        });
        client.send(named);
        client.send(sole.clone());
        // Both are classified offline before the plane can come back, so
        // anything below is a replay, never a live forward.
        wait_offline_drain(&transport);
        transport.allow_connections();

        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let msgs = seen.lock();
            if msgs.contains(&sole) {
                assert!(
                    !msgs.iter().any(|m| matches!(
                        m,
                        ClientMsg::Observation(o)
                            if matches!(
                                &o.payload,
                                Some(pb::observation_update::Payload::Proprio(p))
                                    if !p.part.is_empty()
                            )
                    )),
                    "a named part must never ride a connection that did not accept the flag"
                );
                break;
            }
            drop(msgs);
            assert!(
                std::time::Instant::now() < deadline,
                "history never replayed"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        client.shutdown();
    }

    #[test]
    fn partition_buffers_and_replays_in_order_after_reconnect() {
        let seen: Arc<Mutex<Vec<ClientMsg>>> = Arc::new(Mutex::new(Vec::new()));
        let seen2 = seen.clone();
        let transport = InMemoryTransport::new(move |msg, tx| {
            if matches!(&msg, ClientMsg::Register(_)) {
                let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse::default()));
            }
            seen2.lock().push(msg);
        });
        let client = ControlPlaneClient::spawn(transport.clone(), test_config());
        assert_eq!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Connected)
        );

        // Partition: sever the live connection AND refuse redials, so the
        // offline window below lasts as long as this test needs.
        //
        // Waiting for `Disconnected` is load-bearing too: the event is only
        // emitted once the severed connection's server side is gone, so
        // nothing sent after it can still be recorded by the old
        // connection.
        transport.refuse_connections();
        transport.drop_connections();
        let deadline = std::time::Instant::now() + Duration::from_secs(2);
        loop {
            if client.try_recv_event() == Some(PlaneEvent::Disconnected) {
                break;
            }
            assert!(std::time::Instant::now() < deadline, "never disconnected");
            std::thread::sleep(Duration::from_millis(1));
        }
        let ev1 = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 1,
            ..Default::default()
        });
        let ev2 = ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 2,
            ..Default::default()
        });
        client.send(ev1.clone());
        client.send(ClientMsg::Heartbeat(pb::HeartbeatPing::default())); // dropped offline
        client.send(ev2.clone());
        // All three are classified offline before the plane can come back:
        // the heartbeat is dropped there and now cannot reach the plane by
        // any route, which is exactly what the assertion below reads.
        wait_offline_drain(&transport);
        transport.allow_connections();

        // The redial now succeeds; wait for the two observations to arrive
        // after the new Register, in order.
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        loop {
            let msgs = seen.lock();
            let obs: Vec<&ClientMsg> = msgs
                .iter()
                .filter(|m| matches!(m, ClientMsg::Observation(_)))
                .collect();
            if obs.len() >= 2 {
                assert_eq!(obs[0], &ev1);
                assert_eq!(obs[1], &ev2);
                assert!(
                    !msgs.iter().any(|m| matches!(m, ClientMsg::Heartbeat(_))),
                    "offline heartbeats must be dropped, not replayed"
                );
                break;
            }
            drop(msgs);
            assert!(
                std::time::Instant::now() < deadline,
                "buffered events never replayed"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
        client.shutdown();
    }
}
