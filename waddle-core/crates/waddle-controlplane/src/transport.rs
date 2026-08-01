//! The transport seam: typed client/server messages over a channel pair.
//! The in-memory implementation hosts a scriptable server for tests and
//! conformance; a tonic transport implements the same trait when the gRPC
//! integration lands.

use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, TryRecvError};

use parking_lot::Mutex;
use waddle_types::pb::v0 as pb;

use crate::PlaneError;

/// Client → plane messages (the eight RPC surfaces flattened onto one
/// ordered stream; the tonic transport maps them back to their RPCs).
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // moved once per message, never stored in bulk
pub enum ClientMsg {
    Register(pb::RegisterRequest),
    Negotiate(pb::NegotiateRequest),
    Observation(pb::ObservationUpdate),
    Gate(pb::GateClientMessage),
    Heartbeat(pb::HeartbeatPing),
    ClaimEpisode(pb::ClaimEpisodeRequest),
    HandoffLease(pb::HandoffLeaseRequest),
    RequestReset(pb::ResetRequest),
}

impl ClientMsg {
    /// Heartbeats are liveness, not history: they are dropped while
    /// disconnected. Everything else buffers and replays in order.
    ///
    /// Control-plane stills (`FrameStill`, flag `waddle.v0.obs.stills`) are
    /// the one other exception, for the same reason: they are perception,
    /// not history. They are droppable by declaration (the SDK already
    /// samples latest-wins per camera), each is orders of magnitude larger
    /// than any other message here, and replaying a partition's worth of
    /// them on reconnect would both evict real episode history from this
    /// bounded buffer and hand the plane pictures of a world that has since
    /// moved on. A `ProprioSample` observation still buffers — it is small,
    /// and its history is the point.
    #[must_use]
    pub fn buffer_when_offline(&self) -> bool {
        match self {
            Self::Heartbeat(_) => false,
            Self::Observation(update) => !matches!(
                update.payload,
                Some(pb::observation_update::Payload::Still(_))
            ),
            _ => true,
        }
    }
}

/// Plane → client messages.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)] // moved once per message, never stored in bulk
pub enum ServerMsg {
    Registered(pb::RegisterResponse),
    Negotiated(pb::NegotiateResponse),
    Gate(pb::GateServerMessage),
    HeartbeatAck(pb::HeartbeatAck),
    ClaimResponse(pb::ClaimEpisodeResponse),
    LeaseResponse(pb::HandoffLeaseResponse),
    ResetProgress(pb::ResetProgress),
}

/// One live connection: send fails when the connection is gone.
#[derive(Debug)]
pub struct ControlConn {
    pub tx: Sender<ClientMsg>,
    pub rx: Receiver<ServerMsg>,
}

impl ControlConn {
    /// Non-blocking receive; `Err` means the connection is dead.
    pub fn try_recv(&self) -> Result<Option<ServerMsg>, PlaneError> {
        match self.rx.try_recv() {
            Ok(m) => Ok(Some(m)),
            Err(TryRecvError::Empty) => Ok(None),
            Err(TryRecvError::Disconnected) => Err(PlaneError::Transport("connection lost".into())),
        }
    }
}

pub trait ControlTransport: Send + Sync + 'static {
    fn connect(&self) -> Result<ControlConn, PlaneError>;
}

type ServerHandler = dyn Fn(ClientMsg, &Sender<ServerMsg>) + Send + Sync;

struct ServerSide {
    rx: Receiver<ClientMsg>,
    tx: Sender<ServerMsg>,
}

/// In-memory transport with a scriptable server. Each successful `connect`
/// spawns a server thread that feeds every client message to the handler;
/// tests inject failures (`fail_next`) and cut live connections
/// (`drop_connections`) to exercise backoff and buffering.
pub struct InMemoryTransport {
    handler: Arc<ServerHandler>,
    fail_next: Mutex<u32>,
    /// Live server sides; dropping them severs the client's channels.
    live: Mutex<Vec<Sender<()>>>,
    connects: Mutex<u32>,
}

impl std::fmt::Debug for InMemoryTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("InMemoryTransport").finish_non_exhaustive()
    }
}

impl InMemoryTransport {
    pub fn new(
        handler: impl Fn(ClientMsg, &Sender<ServerMsg>) + Send + Sync + 'static,
    ) -> Arc<Self> {
        Arc::new(Self {
            handler: Arc::new(handler),
            fail_next: Mutex::new(0),
            live: Mutex::new(Vec::new()),
            connects: Mutex::new(0),
        })
    }

    /// Make the next `n` connection attempts fail.
    pub fn fail_next(&self, n: u32) {
        *self.fail_next.lock() = n;
    }

    /// Sever all live connections (simulates a partition).
    pub fn drop_connections(&self) {
        self.live.lock().clear();
    }

    #[must_use]
    pub fn connect_attempts(&self) -> u32 {
        *self.connects.lock()
    }
}

impl ControlTransport for InMemoryTransport {
    fn connect(&self) -> Result<ControlConn, PlaneError> {
        *self.connects.lock() += 1;
        {
            let mut fail = self.fail_next.lock();
            if *fail > 0 {
                *fail -= 1;
                return Err(PlaneError::Transport("injected connect failure".into()));
            }
        }
        let (client_tx, server_rx) = std::sync::mpsc::channel::<ClientMsg>();
        let (server_tx, client_rx) = std::sync::mpsc::channel::<ServerMsg>();
        // The kill channel: when the transport drops it, the server thread
        // exits and the client's channels sever.
        let (kill_tx, kill_rx) = std::sync::mpsc::channel::<()>();
        self.live.lock().push(kill_tx);

        let handler = self.handler.clone();
        let side = ServerSide {
            rx: server_rx,
            tx: server_tx,
        };
        std::thread::Builder::new()
            .name("waddle-inmem-plane".into())
            .spawn(move || {
                loop {
                    match kill_rx.try_recv() {
                        Err(TryRecvError::Disconnected) => break,
                        Ok(()) | Err(TryRecvError::Empty) => {}
                    }
                    match side.rx.try_recv() {
                        Ok(msg) => handler(msg, &side.tx),
                        Err(TryRecvError::Empty) => {
                            std::thread::sleep(std::time::Duration::from_micros(200));
                        }
                        Err(TryRecvError::Disconnected) => break,
                    }
                }
            })
            .expect("spawn in-memory plane server");

        Ok(ControlConn {
            tx: client_tx,
            rx: client_rx,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation(payload: pb::observation_update::Payload) -> ClientMsg {
        ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: 1,
            payload: Some(payload),
        })
    }

    /// The offline buffer holds history, not perception: a `FrameStill`
    /// (flag `waddle.v0.obs.stills`) is dropped while disconnected exactly
    /// like a heartbeat, so a partition can never evict episode history —
    /// or replay stale pictures — on its behalf. A `ProprioSample` on the
    /// same message type still buffers.
    #[test]
    fn stills_are_dropped_while_disconnected_but_proprio_still_buffers() {
        assert!(
            !observation(pb::observation_update::Payload::Still(pb::FrameStill {
                camera: "overhead".into(),
                data: vec![0xff, 0xd8, 0xff],
                ..Default::default()
            }))
            .buffer_when_offline()
        );
        assert!(
            observation(pb::observation_update::Payload::Proprio(
                pb::ProprioSample {
                    joint_pos: vec![0.0],
                    ..Default::default()
                }
            ))
            .buffer_when_offline()
        );
        assert!(!ClientMsg::Heartbeat(pb::HeartbeatPing::default()).buffer_when_offline());
        assert!(ClientMsg::Gate(pb::GateClientMessage::default()).buffer_when_offline());
    }
}
