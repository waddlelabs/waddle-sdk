//! The control-plane client: one thread owning connect → register →
//! pump, with backoff reconnect and in-order replay of buffered messages.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::Mutex;
use waddle_types::pb::v0 as pb;

use crate::backoff::Backoff;
use crate::buffer::OfflineBuffer;
use crate::transport::{ClientMsg, ControlTransport, ServerMsg};

/// Events surfaced to the runtime's reducer.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // moved once per event, never stored in bulk
pub enum PlaneEvent {
    Connected,
    Registered(pb::RegisterResponse),
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
        let conn = match transport.connect() {
            Ok(conn) => conn,
            Err(_) => {
                let delay = config.backoff.delay_ns(attempt);
                attempt = attempt.saturating_add(1);
                sleep_interruptible(delay, shutdown);
                continue 'reconnect;
            }
        };
        attempt = 0;
        let _ = events_tx.send(PlaneEvent::Connected);

        // Register first, then replay the offline buffer strictly in order.
        if conn
            .tx
            .send(ClientMsg::Register(config.register.clone()))
            .is_err()
        {
            let _ = events_tx.send(PlaneEvent::Disconnected);
            continue 'reconnect;
        }
        for msg in buffer.drain() {
            if conn.tx.send(msg).is_err() {
                let _ = events_tx.send(PlaneEvent::Disconnected);
                continue 'reconnect;
            }
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
                Ok(Some(ServerMsg::Registered(r))) => {
                    let _ = events_tx.send(PlaneEvent::Registered(r));
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
        let overflow_before = buffer.dropped();
        buffer_pending(cmd_rx, &mut buffer);
        sleep_interruptible(delay, shutdown);
        buffer_pending(cmd_rx, &mut buffer);
        let dropped = buffer.dropped() - overflow_before;
        if dropped > 0 {
            let _ = events_tx.send(PlaneEvent::BufferOverflowed { dropped });
        }
    }
}

fn buffer_pending(cmd_rx: &Receiver<ClientMsg>, buffer: &mut OfflineBuffer<ClientMsg>) {
    while let Ok(msg) = cmd_rx.try_recv() {
        if msg.buffer_when_offline() {
            buffer.push(msg);
        }
    }
}

fn sleep_interruptible(ns: i64, shutdown: &AtomicBool) {
    let mut remaining = ns.max(0) as u64;
    const SLICE: u64 = 5_000_000; // 5 ms
    while remaining > 0 && !shutdown.load(Ordering::SeqCst) {
        let step = remaining.min(SLICE);
        std::thread::sleep(Duration::from_nanos(step));
        remaining -= step;
    }
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

    #[test]
    fn partition_buffers_and_replays_in_order_after_reconnect() {
        let seen: Arc<Mutex<Vec<ClientMsg>>> = Arc::new(Mutex::new(Vec::new()));
        let seen2 = seen.clone();
        let transport = InMemoryTransport::new(move |msg, _tx| {
            seen2.lock().push(msg);
        });
        // A long first backoff step guarantees a wide offline window for the
        // sends below (no reconnect race).
        let mut cfg = test_config();
        cfg.backoff = Backoff {
            steps_ns: vec![300_000_000],
            plateau_ns: 300_000_000,
        };
        let client = ControlPlaneClient::spawn(transport.clone(), cfg);
        assert_eq!(
            client.recv_event_timeout(Duration::from_secs(1)),
            Some(PlaneEvent::Connected)
        );

        // Sever the connection and wait for the client to notice.
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

        // Reconnect happens on backoff; wait for the two observations to
        // arrive after the new Register, in order.
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
