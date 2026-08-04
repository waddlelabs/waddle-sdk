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
    /// Perception and liveness, never history — THE classification, and the
    /// only place this question is answered. It governs both moments a
    /// message can be shed:
    ///
    /// - plane offline: a droppable message is not buffered
    ///   ([`Self::buffer_when_offline`]);
    /// - plane connected but not draining: a droppable message is bounded in
    ///   flight inside the transport ([`crate::inflight::InflightLimit`]).
    ///
    /// Heartbeats are liveness: one that could not go out now says nothing
    /// the next one won't. Control-plane stills (`FrameStill`, flag
    /// `waddle.v0.obs.stills`) are perception: droppable by declaration (the
    /// SDK already samples them latest-wins per camera), each orders of
    /// magnitude larger than any other message here, and a late one is a
    /// picture of a world that has moved on. Everything else — including a
    /// `ProprioSample` observation, small and historical — is never dropped
    /// by Waddle for being late. (Whether it may be sent AT ALL on a given
    /// connection is the separate question
    /// [`Self::connection_scoped_flag`] answers.)
    ///
    /// A new droppable variant must also be routed through a metered sender
    /// in every transport, or only half of this contract holds for it.
    #[must_use]
    pub fn is_droppable(&self) -> bool {
        match self {
            Self::Heartbeat(_) => true,
            Self::Observation(update) => matches!(
                update.payload,
                Some(pb::observation_update::Payload::Still(_))
            ),
            _ => false,
        }
    }

    /// The feature flag ([`crate::flags`]) whose acceptance this message's
    /// CONTENT depends on, or `None` for the core surface every connection
    /// has. THE classification for VERSIONING §3 ("the control plane MUST
    /// NOT plan against or emit any message, field, or behavior a connection
    /// did not declare"), and the only place this question is answered.
    ///
    /// Acceptance is a property of ONE connection: flags are re-negotiated
    /// at every Register, and the client re-registers on every reconnect. So
    /// this is also the answer to "may this message cross a connection
    /// boundary?" — it may not, and [`Self::buffer_when_offline`] enforces
    /// that.
    ///
    /// The producer withholds these at generation time too (the runtime
    /// reads the negotiated answer off its status mirror). That guard and
    /// this one are not redundant: the producer decides against the answer
    /// it can see, on another thread, and a connection can die between the
    /// decision and the send — this is the only point that sees which
    /// connection a message actually leaves on.
    #[must_use]
    pub fn connection_scoped_flag(&self) -> Option<&'static str> {
        match self {
            Self::Observation(update) => match &update.payload {
                // A named `ProprioSample.part` is one part's joint vector.
                // On a connection that never accepted `waddle.v0.parts`
                // there is no field to read it as, and relabeling it `""`
                // would put one arm's joints on the wire as the whole
                // robot's — so it is withheld. Not history lost: the local
                // recorder keeps the full-rate archive, part and all.
                Some(pb::observation_update::Payload::Proprio(sample)) => {
                    (!sample.part.is_empty()).then_some(crate::flags::PARTS)
                }
                Some(pb::observation_update::Payload::Still(_)) => Some(crate::flags::STILLS),
                _ => None,
            },
            // An ack answers ONE directive, on the connection that sent it
            // and asked to be answered. A replayed one is both un-negotiated
            // on the new connection and stale on the old correlation.
            Self::Gate(msg) => matches!(msg.msg, Some(pb::gate_client_message::Msg::Ack(_)))
                .then_some(crate::flags::ACKS),
            _ => None,
        }
    }

    /// Whether this message survives a partition in the client's bounded
    /// offline buffer and replays in order on reconnect.
    ///
    /// Two classes do not. Droppable messages
    /// ([`Self::is_droppable`]): replaying a partition's worth of them would
    /// both evict real episode history from that bounded buffer and hand the
    /// plane a stale world. Connection-scoped messages
    /// ([`Self::connection_scoped_flag`]): the buffer replays onto the NEXT
    /// connection, which negotiates its own flags — and replays before that
    /// connection's `RegisterResponse` has even arrived, so there is no
    /// moment at which the question could be re-asked. Holding them back
    /// until it arrives is not open either: history replays in order, and a
    /// partial hold would reorder the stream it belongs to.
    #[must_use]
    pub fn buffer_when_offline(&self) -> bool {
        !self.is_droppable() && self.connection_scoped_flag().is_none()
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

/// A transport opens connections; the client owns backoff, offline
/// buffering, and replay.
///
/// Contract: a transport that buffers internally (anything that does not
/// write synchronously inside its `ControlConn` consumer) MUST bound what it
/// holds for [`ClientMsg::is_droppable`] messages — see
/// [`crate::inflight::InflightLimit`]. A plane that is connected but not
/// draining never severs the channels, so an unbounded internal queue turns
/// bounded-rate perception (stills) into unbounded memory growth.
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
///
/// It needs no in-flight bound of its own (see [`ControlTransport`]): the
/// server thread consumes each message synchronously, so nothing accumulates
/// behind it unless a test's own handler blocks.
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

    /// Refuse every dial until [`Self::allow_connections`] heals the
    /// partition.
    ///
    /// This is how a test opens an offline window it can reason about: the
    /// window lasts as long as the test needs, not as long as a backoff
    /// step happens to last. A long step only makes the window *probably*
    /// wide enough — a loaded machine (or the run right after a heavy
    /// build) loses that race, the client reconnects early, and messages
    /// the test meant to classify offline are forwarded live instead, which
    /// no assertion over the plane's received messages can tell apart from
    /// a replay.
    pub fn refuse_connections(&self) {
        self.fail_next(u32::MAX);
    }

    /// Accept dials again (the partition heals).
    pub fn allow_connections(&self) {
        self.fail_next(0);
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
    ///
    /// The two halves of the droppability contract are ONE classification:
    /// whatever is dropped while offline is also the only thing a transport
    /// may shed while connected-but-stalled (`crate::inflight`), and nothing
    /// else is ever droppable. (A connection-scoped message is withheld from
    /// the buffer for the OTHER reason — see
    /// [`super::ClientMsg::connection_scoped_flag`] — so this equivalence is
    /// asserted only where no flag is in play.)
    #[test]
    fn stills_are_dropped_while_disconnected_but_proprio_still_buffers() {
        let still = observation(pb::observation_update::Payload::Still(pb::FrameStill {
            camera: "overhead".into(),
            data: vec![0xff, 0xd8, 0xff],
            ..Default::default()
        }));
        let proprio = observation(pb::observation_update::Payload::Proprio(
            pb::ProprioSample {
                joint_pos: vec![0.0],
                ..Default::default()
            },
        ));
        let heartbeat = ClientMsg::Heartbeat(pb::HeartbeatPing::default());
        let gate = ClientMsg::Gate(pb::GateClientMessage::default());

        assert!(still.is_droppable());
        assert!(heartbeat.is_droppable());
        assert!(!proprio.is_droppable(), "history, not perception");
        assert!(!gate.is_droppable());
        for msg in [&still, &proprio, &heartbeat, &gate] {
            if msg.connection_scoped_flag().is_some() && !msg.is_droppable() {
                continue;
            }
            assert_eq!(
                msg.buffer_when_offline(),
                !msg.is_droppable(),
                "one classification governs both halves: {msg:?}"
            );
        }
    }

    /// A message whose content is legal only under a negotiated flag belongs
    /// to the connection that accepted it, and the offline buffer is exactly
    /// the place it could outlive one: the buffer replays onto the NEXT
    /// connection, before that connection's `RegisterResponse` has even
    /// arrived. A named-part `ProprioSample` replayed that way lands on a
    /// plane that may have refused `waddle.v0.parts` — VERSIONING §3 — so it
    /// does not enter the buffer at all. The same sample under the sole part
    /// (`""`, core surface) is ordinary history and does.
    #[test]
    fn a_flag_scoped_message_never_survives_a_partition() {
        let named = observation(pb::observation_update::Payload::Proprio(
            pb::ProprioSample {
                part: "left".into(),
                joint_pos: vec![0.1; 7],
                ..Default::default()
            },
        ));
        let sole = observation(pb::observation_update::Payload::Proprio(
            pb::ProprioSample {
                joint_pos: vec![0.1; 14],
                ..Default::default()
            },
        ));
        let ack = ClientMsg::Gate(pb::GateClientMessage {
            msg: Some(pb::gate_client_message::Msg::Ack(pb::DirectiveAck {
                directive_id: "d1".into(),
                accepted: true,
                ..Default::default()
            })),
        });
        let event = ClientMsg::Gate(pb::GateClientMessage {
            msg: Some(pb::gate_client_message::Msg::Event(
                pb::EpisodeEvent::default(),
            )),
        });

        assert_eq!(named.connection_scoped_flag(), Some(crate::flags::PARTS));
        assert_eq!(ack.connection_scoped_flag(), Some(crate::flags::ACKS));
        assert_eq!(sole.connection_scoped_flag(), None);
        assert_eq!(event.connection_scoped_flag(), None);

        assert!(!named.buffer_when_offline(), "cannot cross a connection");
        assert!(!ack.buffer_when_offline(), "cannot cross a connection");
        assert!(sole.buffer_when_offline(), "history, and core surface");
        assert!(event.buffer_when_offline(), "history, and core surface");
        // Withheld is not shed: none of this is droppable, so nothing here
        // may be discarded by a transport that is merely slow.
        for msg in [&named, &sole, &ack, &event] {
            assert!(!msg.is_droppable(), "{msg:?}");
        }
    }
}
