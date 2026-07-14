//! waddle-media — the media plane's transport seam.
//!
//! Camera/depth video and the teleop data topics ride WebRTC in production
//! (`media.proto` defines the payloads and the topic table, including
//! reliability classes). This crate owns the *trait boundary* and an
//! in-memory [`LoopbackMedia`] used by tests and the conformance harness;
//! the LiveKit/WebRTC integration is deferred behind the `livekit` feature
//! (a typed stub until that milestone).
//!
//! Nothing stateful ever rides the media plane; nothing high-bandwidth ever
//! rides the control plane.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, Sender, TryRecvError};

use bytes::Bytes;
use parking_lot::Mutex;
use prost::Message;
use waddle_types::pb::v0 as pb;

#[derive(Debug, thiserror::Error)]
pub enum MediaError {
    #[error("unknown track {0:?}")]
    UnknownTrack(String),
    #[error("data topic {0:?} is not open")]
    TopicClosed(&'static str),
    #[error("payload decode failed: {0}")]
    Decode(#[from] prost::DecodeError),
    #[error("the `livekit` feature is a stub until the WebRTC integration milestone")]
    Unimplemented,
}

/// The data topics of `media.proto`'s normative table, with their topic
/// strings and reliability classes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DataTopic {
    /// `waddle/v0/teleop/pose` — lossy, unordered, latest-wins.
    TeleopPose,
    /// `waddle/v0/teleop/clutch` — reliable, ordered.
    TeleopClutch,
    /// `waddle/v0/teleop/mark` — reliable, ordered.
    TeleopMark,
    /// `waddle/v0/telemetry` — lossy, latest-wins.
    Telemetry,
}

impl DataTopic {
    #[must_use]
    pub fn topic_str(self) -> &'static str {
        match self {
            Self::TeleopPose => "waddle/v0/teleop/pose",
            Self::TeleopClutch => "waddle/v0/teleop/clutch",
            Self::TeleopMark => "waddle/v0/teleop/mark",
            Self::Telemetry => "waddle/v0/telemetry",
        }
    }

    /// Lossy topics are latest-wins: transports may drop under pressure.
    #[must_use]
    pub fn is_lossy(self) -> bool {
        matches!(self, Self::TeleopPose | Self::Telemetry)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TrackHandle {
    pub name: String,
}

/// One encoded video frame headed for a track.
#[derive(Debug, Clone)]
pub struct EncodedFrame {
    pub t_ns: i64,
    pub keyframe: bool,
    pub data: Bytes,
}

/// Raw pixels in, encoded frames out. The production implementation is
/// H.264; [`PassthroughEncoder`] treats input as already encoded.
pub trait VideoEncoder: Send {
    fn encode(&mut self, t_ns: i64, raw: &[u8]) -> Result<EncodedFrame, MediaError>;
}

#[derive(Debug, Default)]
pub struct PassthroughEncoder;

impl VideoEncoder for PassthroughEncoder {
    fn encode(&mut self, t_ns: i64, raw: &[u8]) -> Result<EncodedFrame, MediaError> {
        Ok(EncodedFrame {
            t_ns,
            keyframe: true,
            data: Bytes::copy_from_slice(raw),
        })
    }
}

/// Receiving end of a data topic (payloads are serialized `media.proto`
/// messages). Channel-backed so no async runtime leaks into the trait.
#[derive(Debug)]
pub struct DataRx {
    rx: Receiver<Bytes>,
}

impl DataRx {
    pub fn try_recv(&self) -> Option<Bytes> {
        match self.rx.try_recv() {
            Ok(b) => Some(b),
            Err(TryRecvError::Empty | TryRecvError::Disconnected) => None,
        }
    }

    /// Decode the next pending teleop stream packet, if any.
    pub fn try_recv_pose(&self) -> Result<Option<pb::TeleopStreamPacket>, MediaError> {
        self.try_recv()
            .map(|b| pb::TeleopStreamPacket::decode(b).map_err(MediaError::from))
            .transpose()
    }
}

#[derive(Debug, Clone)]
pub struct DataTx {
    tx: Sender<Bytes>,
}

impl DataTx {
    pub fn send(&self, payload: Bytes) {
        // Loopback is unbounded; real transports apply the topic's
        // reliability class.
        let _ = self.tx.send(payload);
    }

    pub fn send_msg(&self, msg: &impl Message) {
        self.send(Bytes::from(msg.encode_to_vec()));
    }
}

/// The media plane boundary the runtime wires against.
pub trait MediaPlane: Send + Sync + 'static {
    fn publish_track(&self, camera: &str) -> Result<TrackHandle, MediaError>;
    fn push_frame(&self, track: &TrackHandle, frame: EncodedFrame) -> Result<(), MediaError>;
    /// Inbound data (teleop actions, clutch, marks).
    fn open_data_rx(&self, topic: DataTopic) -> Result<DataRx, MediaError>;
    /// Outbound data (telemetry).
    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError>;
}

#[derive(Debug, Default)]
struct LoopbackState {
    /// Far-end producers for topics the near end reads.
    inbound: HashMap<&'static str, Sender<Bytes>>,
    /// Far-end receivers for topics the near end writes.
    outbound: HashMap<&'static str, Receiver<Bytes>>,
    frames: Vec<(String, EncodedFrame)>,
}

/// In-memory media plane: the test/conformance "far end" is scripted through
/// [`LoopbackFarEnd`].
#[derive(Debug, Default)]
pub struct LoopbackMedia {
    state: Arc<Mutex<LoopbackState>>,
}

/// Handle for scripting the remote side: push teleop packets in, observe
/// telemetry out, inspect published frames.
#[derive(Debug)]
pub struct LoopbackFarEnd {
    state: Arc<Mutex<LoopbackState>>,
}

impl LoopbackMedia {
    #[must_use]
    pub fn new() -> (Arc<Self>, LoopbackFarEnd) {
        let state = Arc::new(Mutex::new(LoopbackState::default()));
        (
            Arc::new(Self {
                state: state.clone(),
            }),
            LoopbackFarEnd { state },
        )
    }
}

impl MediaPlane for LoopbackMedia {
    fn publish_track(&self, camera: &str) -> Result<TrackHandle, MediaError> {
        Ok(TrackHandle {
            name: camera.to_owned(),
        })
    }

    fn push_frame(&self, track: &TrackHandle, frame: EncodedFrame) -> Result<(), MediaError> {
        self.state.lock().frames.push((track.name.clone(), frame));
        Ok(())
    }

    fn open_data_rx(&self, topic: DataTopic) -> Result<DataRx, MediaError> {
        let (tx, rx) = std::sync::mpsc::channel();
        self.state.lock().inbound.insert(topic.topic_str(), tx);
        Ok(DataRx { rx })
    }

    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError> {
        let (tx, rx) = std::sync::mpsc::channel();
        self.state.lock().outbound.insert(topic.topic_str(), rx);
        Ok(DataTx { tx })
    }
}

impl LoopbackFarEnd {
    /// Script an inbound payload (e.g. a teleop pose packet).
    pub fn push(&self, topic: DataTopic, msg: &impl Message) -> Result<(), MediaError> {
        let state = self.state.lock();
        let tx = state
            .inbound
            .get(topic.topic_str())
            .ok_or(MediaError::TopicClosed(topic.topic_str()))?;
        let _ = tx.send(Bytes::from(msg.encode_to_vec()));
        Ok(())
    }

    /// Drain outbound payloads (e.g. telemetry the robot side sent).
    pub fn drain(&self, topic: DataTopic) -> Vec<Bytes> {
        let state = self.state.lock();
        let Some(rx) = state.outbound.get(topic.topic_str()) else {
            return Vec::new();
        };
        let mut out = Vec::new();
        while let Ok(b) = rx.try_recv() {
            out.push(b);
        }
        out
    }

    #[must_use]
    pub fn frames(&self) -> Vec<(String, EncodedFrame)> {
        self.state.lock().frames.clone()
    }
}

#[cfg(feature = "livekit")]
pub mod livekit {
    //! Typed stub: the LiveKit/WebRTC integration is a named-trigger
    //! deferral. Every constructor returns [`MediaError::Unimplemented`].

    use super::{MediaError, MediaPlane};
    use std::sync::Arc;

    pub fn connect(_url: &str, _token: &str) -> Result<Arc<dyn MediaPlane>, MediaError> {
        Err(MediaError::Unimplemented)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_round_trips_pose_packets_and_telemetry() {
        let (media, far) = LoopbackMedia::new();

        let rx = media.open_data_rx(DataTopic::TeleopPose).unwrap();
        let packet = pb::TeleopStreamPacket {
            t_client_ns: 42,
            seq: 7,
            clutch_engaged: true,
            ..Default::default()
        };
        far.push(DataTopic::TeleopPose, &packet).unwrap();
        let got = rx.try_recv_pose().unwrap().unwrap();
        assert_eq!(got.seq, 7);
        assert!(got.clutch_engaged);

        let tx = media.open_data_tx(DataTopic::Telemetry).unwrap();
        tx.send_msg(&pb::OperatorTelemetry {
            t_ns: 1,
            ..Default::default()
        });
        assert_eq!(far.drain(DataTopic::Telemetry).len(), 1);
    }

    #[test]
    fn tracks_collect_frames() {
        let (media, far) = LoopbackMedia::new();
        let track = media.publish_track("overhead").unwrap();
        media
            .push_frame(
                &track,
                EncodedFrame {
                    t_ns: 5,
                    keyframe: true,
                    data: Bytes::from_static(b"frame"),
                },
            )
            .unwrap();
        let frames = far.frames();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].0, "overhead");
    }
}
