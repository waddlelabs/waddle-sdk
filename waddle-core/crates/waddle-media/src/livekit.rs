//! Real LiveKit media plane (behind the `livekit` cargo feature).
//!
//! # Tokio confinement (repo invariant)
//!
//! This module owns the only tokio in the workspace. [`LiveKitMedia::connect`]
//! spawns ONE dedicated thread (`waddle-media-livekit`) that runs a private
//! current-thread tokio runtime; every [`MediaPlane`] method stays
//! synchronous and bridges to it over channels. No tokio type appears in any
//! public signature, and default (featureless) builds contain no tokio at
//! all.
//!
//! # Data topics
//!
//! `DataTopic` maps onto LiveKit data-channel publishes using the normative
//! topic strings and reliability classes from `media.proto`'s topic table:
//! lossy topics (`TeleopPose`, `Telemetry`) publish unreliable/latest-wins,
//! reliable topics (`TeleopClutch`, `TeleopMark`) publish reliable/ordered.
//! ("Latest-wins" dropping of stale packets is receiver-side behavior per
//! media.proto; the transport only chooses the delivery class.) Inbound
//! packets are routed by topic string into the standard [`DataRx`] seam.
//!
//! # Tracks and encodings
//!
//! LiveKit's native `VideoSource` consumes RAW frames (planar I420) — it
//! does not accept pre-encoded JPEG; libwebrtc encodes the uplink itself.
//! The mapping is therefore:
//!
//! - [`MediaPlane::push_frame`] expects `EncodedFrame::data` to be either
//!   raw RGB8 (`width * height * 3` bytes; converted via
//!   [`crate::rgb8_to_i420`]) or already-planar I420 at the resolution the
//!   track was published with ([`LiveKitConfig::with_track_resolution`],
//!   default [`DEFAULT_TRACK_RESOLUTION`]). Anything else is
//!   [`MediaError::BadFrame`]. The `keyframe` flag is ignored: raw uplink
//!   has no keyframes, libwebrtc decides.
//! - [`crate::JpegEncoder`] applies to the data-channel/recording path
//!   (e.g. sidecar frame capture), not to LiveKit track uplink.
//!
//! Frame conversion and validation run synchronously on the caller's
//! thread; `capture_frame` is a thread-safe libwebrtc call, so pushing
//! frames never round-trips through the worker.

use std::collections::HashMap;
use std::fmt;
use std::sync::Arc;
use std::sync::mpsc as std_mpsc;
use std::thread;

use ::livekit::options::TrackPublishOptions;
use ::livekit::prelude::*;
use ::livekit::webrtc::prelude::{
    I420Buffer, RtcVideoSource, VideoFrame, VideoResolution, VideoRotation,
};
use ::livekit::webrtc::video_source::native::NativeVideoSource;
use bytes::Bytes;
use parking_lot::Mutex;
use tokio::sync::mpsc as tokio_mpsc;

use crate::{
    DataRx, DataTopic, DataTx, EncodedFrame, MediaError, MediaPlane, TrackHandle, rgb8_to_i420,
};

/// Pixel dimensions used for cameras not named in
/// [`LiveKitConfig::track_resolutions`].
pub const DEFAULT_TRACK_RESOLUTION: (u32, u32) = (640, 480);

/// Connection configuration. Identity and room are implicit in the token;
/// token minting is the caller's problem (the supervision plane hands the
/// integration a token, it never mints one).
#[derive(Clone)]
pub struct LiveKitConfig {
    pub url: String,
    pub token: String,
    /// Camera name → published pixel dimensions `(width, height)`. Cameras
    /// absent from the map publish at [`DEFAULT_TRACK_RESOLUTION`]. This is
    /// needed because [`MediaPlane::push_frame`] carries opaque bytes: the
    /// declared resolution is what lets the transport interpret raw frames.
    pub track_resolutions: HashMap<String, (u32, u32)>,
}

impl LiveKitConfig {
    #[must_use]
    pub fn new(url: String, token: String) -> Self {
        Self {
            url,
            token,
            track_resolutions: HashMap::new(),
        }
    }

    /// Declare the pixel dimensions `camera` will be published at.
    #[must_use]
    pub fn with_track_resolution(mut self, camera: &str, width: u32, height: u32) -> Self {
        self.track_resolutions
            .insert(camera.to_owned(), (width, height));
        self
    }
}

impl fmt::Debug for LiveKitConfig {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LiveKitConfig")
            .field("url", &self.url)
            .field("token", &"<redacted>")
            .field("track_resolutions", &self.track_resolutions)
            .finish()
    }
}

/// Commands the synchronous side sends to the worker thread. The channel is
/// a tokio unbounded sender because its `send` is synchronous and
/// non-blocking — it is the sync→async bridge.
enum Command {
    PublishTrack {
        camera: String,
        width: u32,
        height: u32,
        reply: std_mpsc::Sender<Result<NativeVideoSource, MediaError>>,
    },
    PublishData {
        topic: DataTopic,
        payload: Bytes,
    },
    OpenRx {
        topic: DataTopic,
        tx: std_mpsc::Sender<Bytes>,
    },
    Shutdown,
}

struct TrackState {
    source: NativeVideoSource,
    width: u32,
    height: u32,
}

/// The real LiveKit [`MediaPlane`]. See the module docs for the topology.
pub struct LiveKitMedia {
    cmd: tokio_mpsc::UnboundedSender<Command>,
    /// Declared per-camera resolutions (from [`LiveKitConfig`]).
    resolutions: HashMap<String, (u32, u32)>,
    /// Published tracks; the stored [`NativeVideoSource`] is thread-safe,
    /// so `push_frame` captures directly without a worker round-trip.
    tracks: Mutex<HashMap<String, TrackState>>,
    worker: Mutex<Option<thread::JoinHandle<()>>>,
}

impl fmt::Debug for LiveKitMedia {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LiveKitMedia")
            .field("tracks", &self.tracks.lock().keys().collect::<Vec<_>>())
            .finish_non_exhaustive()
    }
}

impl LiveKitMedia {
    /// Connect to the room named by `config.token`. Blocks until the signal
    /// connection succeeds or fails (the SDK applies its own connect
    /// timeout); on failure the worker thread has already wound down.
    pub fn connect(config: LiveKitConfig) -> Result<Arc<Self>, MediaError> {
        let resolutions = config.track_resolutions.clone();
        let (cmd_tx, cmd_rx) = tokio_mpsc::unbounded_channel();
        let (ready_tx, ready_rx) = std_mpsc::channel();
        let handle = thread::Builder::new()
            .name("waddle-media-livekit".to_owned())
            .spawn(move || worker(config, cmd_rx, ready_tx))
            .map_err(|e| MediaError::Transport(format!("failed to spawn worker thread: {e}")))?;
        let connected = ready_rx
            .recv()
            .unwrap_or_else(|_| Err(MediaError::Transport("worker exited during connect".into())));
        match connected {
            Ok(()) => Ok(Arc::new(Self {
                cmd: cmd_tx,
                resolutions,
                tracks: Mutex::new(HashMap::new()),
                worker: Mutex::new(Some(handle)),
            })),
            Err(e) => {
                let _ = handle.join();
                Err(e)
            }
        }
    }
}

/// Convenience matching the pre-integration stub's shape: connect with no
/// declared track resolutions (tracks publish at
/// [`DEFAULT_TRACK_RESOLUTION`]).
pub fn connect(url: &str, token: &str) -> Result<Arc<dyn MediaPlane>, MediaError> {
    LiveKitMedia::connect(LiveKitConfig::new(url.to_owned(), token.to_owned()))
        .map(|m| m as Arc<dyn MediaPlane>)
}

impl Drop for LiveKitMedia {
    fn drop(&mut self) {
        let _ = self.cmd.send(Command::Shutdown);
        if let Some(handle) = self.worker.lock().take() {
            let _ = handle.join();
        }
    }
}

impl MediaPlane for LiveKitMedia {
    fn publish_track(&self, camera: &str) -> Result<TrackHandle, MediaError> {
        // The registry lock is held across the worker round-trip so
        // concurrent publishes of the same camera cannot double-publish;
        // publishing is setup-time, so the contention window is idle.
        let mut tracks = self.tracks.lock();
        // Idempotent: re-publishing an existing camera returns its handle.
        if !tracks.contains_key(camera) {
            let (width, height) = self
                .resolutions
                .get(camera)
                .copied()
                .unwrap_or(DEFAULT_TRACK_RESOLUTION);
            let (reply_tx, reply_rx) = std_mpsc::channel();
            self.cmd
                .send(Command::PublishTrack {
                    camera: camera.to_owned(),
                    width,
                    height,
                    reply: reply_tx,
                })
                .map_err(|_| MediaError::Transport("livekit worker is gone".into()))?;
            let source = reply_rx
                .recv()
                .map_err(|_| MediaError::Transport("livekit worker is gone".into()))??;
            tracks.insert(
                camera.to_owned(),
                TrackState {
                    source,
                    width,
                    height,
                },
            );
        }
        Ok(TrackHandle {
            name: camera.to_owned(),
        })
    }

    fn push_frame(&self, track: &TrackHandle, frame: EncodedFrame) -> Result<(), MediaError> {
        // Snapshot the (cheaply clonable) source handle and drop the lock
        // before the pixel work so concurrent pushes on other tracks don't
        // serialize behind this frame's conversion.
        let (source, w, h) = {
            let tracks = self.tracks.lock();
            let state = tracks
                .get(&track.name)
                .ok_or_else(|| MediaError::UnknownTrack(track.name.clone()))?;
            (state.source.clone(), state.width, state.height)
        };
        let (cw, ch) = ((w as usize).div_ceil(2), (h as usize).div_ceil(2));
        let i420_len = (w as usize) * (h as usize) + 2 * cw * ch;
        let rgb_len = (w as usize) * (h as usize) * 3;
        let converted;
        let i420: &[u8] = if frame.data.len() == rgb_len {
            converted = rgb8_to_i420(w, h, &frame.data)?;
            &converted
        } else if frame.data.len() == i420_len {
            &frame.data
        } else {
            return Err(MediaError::BadFrame {
                got: frame.data.len(),
                expected: rgb_len,
                layout: "RGB8 or planar I420 at the track's declared resolution",
            });
        };

        let mut buffer = I420Buffer::new(w, h);
        let (sy, su, sv) = buffer.strides();
        let (dy, du, dv) = buffer.data_mut();
        let (y_plane, rest) = i420.split_at((w as usize) * (h as usize));
        let (u_plane, v_plane) = rest.split_at(cw * ch);
        copy_plane(dy, sy as usize, y_plane, w as usize, h as usize);
        copy_plane(du, su as usize, u_plane, cw, ch);
        copy_plane(dv, sv as usize, v_plane, cw, ch);

        source.capture_frame(&VideoFrame {
            rotation: VideoRotation::VideoRotation0,
            timestamp_us: frame.t_ns / 1_000,
            frame_metadata: None,
            buffer,
        });
        Ok(())
    }

    fn open_data_rx(&self, topic: DataTopic) -> Result<DataRx, MediaError> {
        let (tx, rx) = std_mpsc::channel();
        self.cmd
            .send(Command::OpenRx { topic, tx })
            .map_err(|_| MediaError::Transport("livekit worker is gone".into()))?;
        Ok(DataRx { rx })
    }

    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError> {
        // DataTx is a plain std channel by contract; a small forwarder
        // thread drains it into the worker, which applies the topic's
        // reliability class. The forwarder exits when the DataTx (all
        // senders) drops or the worker goes away.
        let (tx, rx) = std_mpsc::channel::<Bytes>();
        let cmd = self.cmd.clone();
        thread::Builder::new()
            .name(format!("waddle-media-lk-tx-{}", topic.topic_str()))
            .spawn(move || {
                while let Ok(payload) = rx.recv() {
                    if cmd.send(Command::PublishData { topic, payload }).is_err() {
                        break;
                    }
                }
            })
            .map_err(|e| {
                MediaError::Transport(format!("failed to spawn data-tx forwarder: {e}"))
            })?;
        Ok(DataTx { tx })
    }
}

/// Copy a tightly-packed plane into a possibly-strided destination.
fn copy_plane(dst: &mut [u8], stride: usize, src: &[u8], width: usize, rows: usize) {
    for r in 0..rows {
        dst[r * stride..r * stride + width].copy_from_slice(&src[r * width..(r + 1) * width]);
    }
}

/// The dedicated worker: owns the private current-thread runtime, the Room,
/// and inbound routing. Everything async lives below this line.
fn worker(
    config: LiveKitConfig,
    mut cmd_rx: tokio_mpsc::UnboundedReceiver<Command>,
    ready_tx: std_mpsc::Sender<Result<(), MediaError>>,
) {
    let rt = match tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
    {
        Ok(rt) => rt,
        Err(e) => {
            let _ = ready_tx.send(Err(MediaError::Transport(format!(
                "failed to build tokio runtime: {e}"
            ))));
            return;
        }
    };
    rt.block_on(async move {
        let (room, mut events) =
            match Room::connect(&config.url, &config.token, RoomOptions::default()).await {
                Ok(v) => v,
                Err(e) => {
                    let _ = ready_tx.send(Err(MediaError::Transport(format!(
                        "livekit connect failed: {e}"
                    ))));
                    return;
                }
            };
        let _ = ready_tx.send(Ok(()));

        let mut rx_routes: HashMap<&'static str, std_mpsc::Sender<Bytes>> = HashMap::new();
        loop {
            tokio::select! {
                cmd = cmd_rx.recv() => match cmd {
                    None | Some(Command::Shutdown) => break,
                    Some(Command::PublishTrack { camera, width, height, reply }) => {
                        let source = NativeVideoSource::new(
                            VideoResolution { width, height },
                            false,
                        );
                        let track = LocalVideoTrack::create_video_track(
                            &camera,
                            RtcVideoSource::Native(source.clone()),
                        );
                        let res = room
                            .local_participant()
                            .publish_track(
                                LocalTrack::Video(track),
                                TrackPublishOptions {
                                    source: TrackSource::Camera,
                                    ..Default::default()
                                },
                            )
                            .await
                            .map(|_publication| source)
                            .map_err(|e| {
                                MediaError::Transport(format!("publish_track failed: {e}"))
                            });
                        let _ = reply.send(res);
                    }
                    Some(Command::PublishData { topic, payload }) => {
                        let res = room
                            .local_participant()
                            .publish_data(DataPacket {
                                payload: payload.to_vec(),
                                topic: Some(topic.topic_str().to_owned()),
                                reliable: !topic.is_lossy(),
                                ..Default::default()
                            })
                            .await;
                        if let Err(e) = res {
                            // DataTx::send is fire-and-forget by contract;
                            // lossy topics tolerate drops and reliable-topic
                            // failures surface here for the operator log.
                            tracing::warn!(
                                topic = topic.topic_str(),
                                error = %e,
                                "livekit data publish failed"
                            );
                        }
                    }
                    Some(Command::OpenRx { topic, tx }) => {
                        rx_routes.insert(topic.topic_str(), tx);
                    }
                },
                ev = events.recv() => match ev {
                    None => break,
                    Some(RoomEvent::DataReceived { payload, topic, .. }) => {
                        if let Some(topic) = topic.as_deref()
                            && let Some(tx) = rx_routes.get(topic)
                        {
                            let _ = tx.send(Bytes::copy_from_slice(&payload));
                        }
                    }
                    Some(_) => {}
                },
            }
        }
        let _ = room.close().await;
    });
}
