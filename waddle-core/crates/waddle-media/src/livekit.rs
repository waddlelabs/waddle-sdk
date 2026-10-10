//! Real LiveKit media plane (behind the `livekit` cargo feature).
//!
//! # Tokio confinement (repo invariant)
//!
//! This module is a tokio confinement point (the other is
//! waddle-controlplane's `tonic-transport` feature). [`LiveKitMedia::connect`]
//! spawns ONE dedicated thread (`waddle-media-livekit`) that runs a private
//! current-thread tokio runtime; every [`MediaPlane`] method stays
//! synchronous and forwards to it over channels. No tokio type appears in any
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
use std::time::Duration;

use ::livekit::options::{TrackPublishOptions, VideoEncoding};
use ::livekit::prelude::*;
use ::livekit::webrtc::prelude::{
    I420Buffer, RtcVideoSource, VideoFrame, VideoResolution, VideoRotation,
};
use ::livekit::webrtc::video_source::native::NativeVideoSource;
use bytes::Bytes;
use parking_lot::Mutex;
use tokio::sync::mpsc as tokio_mpsc;
use waddle_types::pb::v0 as pb;

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
    /// Optional presentation ceilings, applied off the capture/control thread.
    pub preview_width: Option<u32>,
    pub preview_fps: Option<f64>,
    pub preview_max_kbps: Option<u32>,
    pub depth_preview: bool,
    pub demand_driven: bool,
    pub video_only: bool,
}

impl LiveKitConfig {
    #[must_use]
    pub fn new(url: String, token: String) -> Self {
        Self {
            url,
            token,
            track_resolutions: HashMap::new(),
            preview_width: None,
            preview_fps: None,
            preview_max_kbps: None,
            depth_preview: true,
            demand_driven: false,
            video_only: false,
        }
    }

    /// Declare the pixel dimensions `camera` will be published at.
    #[must_use]
    pub fn with_track_resolution(mut self, camera: &str, width: u32, height: u32) -> Self {
        self.track_resolutions
            .insert(camera.to_owned(), (width, height));
        self
    }

    /// Declare every camera in `robot` at the dimensions the robot itself
    /// declared — the whole `RobotDescription.cameras` list at once.
    ///
    /// This is how an integration that HAS a robot declaration must build
    /// its config, and why it lives here rather than in each binding: a
    /// track publishes at ONE resolution and [`MediaPlane::push_frame`]
    /// rejects every frame that disagrees with it, so a camera left to
    /// [`DEFAULT_TRACK_RESOLUTION`] drops 100% of its frames unless it
    /// happens to be exactly VGA. Nothing raises when that happens — the
    /// uplink pump warns and counts the drop — so the failure presents as
    /// "the teleoperator sees nothing" with a session that reports success.
    ///
    /// The map mirrors the declaration the rest of the pipeline already
    /// validates frames against (`Session::publish_frame` rejects any frame
    /// whose dimensions disagree with the same `cameras` entry), so a
    /// camera declared with no dimensions can never produce a frame here
    /// either, and needs no special case.
    #[must_use]
    pub fn with_robot_cameras(self, robot: &pb::RobotDescription) -> Self {
        robot.cameras.iter().fold(self, |config, cam| {
            config
                .with_track_resolution(&cam.name, cam.width, cam.height)
                .with_track_resolution(&crate::depth_track_name(&cam.name), cam.width, cam.height)
        })
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
/// non-blocking — it is the sync→async forwarder.
enum Command {
    PublishTrack {
        camera: String,
        width: u32,
        height: u32,
        reply: std_mpsc::Sender<Result<(NativeVideoSource, LocalVideoTrack), MediaError>>,
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
    track: LocalVideoTrack,
    width: u32,
    height: u32,
}

/// The real LiveKit [`MediaPlane`]. See the module docs for the topology.
pub struct LiveKitMedia {
    cmd: tokio_mpsc::UnboundedSender<Command>,
    /// Declared per-camera resolutions (from [`LiveKitConfig`]).
    resolutions: HashMap<String, (u32, u32)>,
    preview_width: Option<u32>,
    preview_fps: Option<f64>,
    depth_preview: bool,
    demand_driven: bool,
    video_only: bool,
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
    /// Connect to the room named by `config.token`. Ordinary publication waits
    /// for signaling; demand-driven previews dial and retry in the worker so
    /// optional presentation cannot hold up the owning session's startup.
    pub fn connect(config: LiveKitConfig) -> Result<Arc<Self>, MediaError> {
        if config.preview_width == Some(0)
            || config.preview_max_kbps == Some(0)
            || config
                .preview_fps
                .is_some_and(|fps| !fps.is_finite() || fps <= 0.0)
        {
            return Err(MediaError::Transport("invalid preview ceiling".into()));
        }
        let resolutions = config.track_resolutions.clone();
        let preview_width = config.preview_width;
        let preview_fps = config.preview_fps;
        let depth_preview = config.depth_preview;
        let demand_driven = config.demand_driven;
        let video_only = config.video_only;
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
                preview_width,
                preview_fps,
                depth_preview,
                demand_driven,
                video_only,
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
    fn supports_data_topics(&self) -> bool {
        !self.video_only
    }
    fn max_video_fps(&self) -> Option<f64> {
        self.preview_fps
    }

    fn depth_preview_enabled(&self) -> bool {
        self.depth_preview
    }

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
            let (width, height) = preview_dimensions(width, height, self.preview_width);
            let (reply_tx, reply_rx) = std_mpsc::channel();
            self.cmd
                .send(Command::PublishTrack {
                    camera: camera.to_owned(),
                    width,
                    height,
                    reply: reply_tx,
                })
                .map_err(|_| MediaError::Transport("livekit worker is gone".into()))?;
            let (source, track) = reply_rx
                .recv_timeout(Duration::from_secs(5))
                .map_err(|e| MediaError::Transport(format!("livekit publication wait: {e}")))??;
            tracks.insert(
                camera.to_owned(),
                TrackState {
                    source,
                    track,
                    width,
                    height,
                },
            );
        }
        Ok(TrackHandle {
            name: camera.to_owned(),
        })
    }

    fn wants_video_frame(&self, track: &TrackHandle) -> bool {
        !self.demand_driven
            || self.tracks.lock().get(&track.name).is_some_and(|state| {
                state
                    .track
                    .publishing_layers()
                    .iter()
                    .any(|layer| layer.active)
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
            // SFU dynacast state gates expensive conversion before pixels
            // are touched. A published track exists even while nobody watches.
            if self.demand_driven
                && !state
                    .track
                    .publishing_layers()
                    .iter()
                    .any(|layer| layer.active)
            {
                return Ok(());
            }
            (state.source.clone(), state.width, state.height)
        };
        let (input_w, input_h) = self
            .resolutions
            .get(&track.name)
            .copied()
            .unwrap_or(DEFAULT_TRACK_RESOLUTION);
        let resized;
        let raw = if (input_w, input_h) != (w, h)
            && frame.data.len() == input_w as usize * input_h as usize * 3
        {
            resized = resize_rgb(input_w, input_h, w, h, &frame.data);
            resized.as_slice()
        } else {
            frame.data.as_ref()
        };
        let (cw, ch) = ((w as usize).div_ceil(2), (h as usize).div_ceil(2));
        let i420_len = (w as usize) * (h as usize) + 2 * cw * ch;
        let rgb_len = (w as usize) * (h as usize) * 3;
        let converted;
        let i420: &[u8] = if raw.len() == rgb_len {
            converted = rgb8_to_i420(w, h, raw)?;
            &converted
        } else if raw.len() == i420_len {
            raw
        } else {
            return Err(MediaError::BadFrame {
                got: raw.len(),
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
        if self.video_only {
            return Err(MediaError::TopicClosed(topic.topic_str()));
        }
        let (tx, rx) = std_mpsc::channel();
        self.cmd
            .send(Command::OpenRx { topic, tx })
            .map_err(|_| MediaError::Transport("livekit worker is gone".into()))?;
        Ok(DataRx { rx })
    }

    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError> {
        if self.video_only {
            return Err(MediaError::TopicClosed(topic.topic_str()));
        }
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

fn preview_dimensions(width: u32, height: u32, ceiling: Option<u32>) -> (u32, u32) {
    match ceiling {
        Some(max) if max < width => (
            max,
            ((u64::from(height) * u64::from(max)) / u64::from(width)).max(1) as u32,
        ),
        _ => (width, height),
    }
}

// Presentation-only nearest-neighbor sampling; never modifies source pixels.
fn resize_rgb(iw: u32, ih: u32, ow: u32, oh: u32, rgb: &[u8]) -> Vec<u8> {
    let mut out = vec![0; ow as usize * oh as usize * 3];
    for y in 0..oh as usize {
        for x in 0..ow as usize {
            let src =
                ((y * ih as usize / oh as usize) * iw as usize + x * iw as usize / ow as usize) * 3;
            let dst = (y * ow as usize + x) * 3;
            out[dst..dst + 3].copy_from_slice(&rgb[src..src + 3]);
        }
    }
    out
}

/// The dedicated worker: owns the private current-thread runtime, the Room,
/// and inbound routing. Everything async lives below this line.
fn worker(
    config: LiveKitConfig,
    mut cmd_rx: tokio_mpsc::UnboundedReceiver<Command>,
    ready_tx: std_mpsc::Sender<Result<(), MediaError>>,
) {
    crate::prepare_background_media_thread();
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
        let mut options = RoomOptions::default();
        options.auto_subscribe = false;
        options.dynacast = config.demand_driven;
        let mut rx_routes: HashMap<&'static str, std_mpsc::Sender<Bytes>> = HashMap::new();
        if config.demand_driven {
            let _ = ready_tx.send(Ok(()));
        }
        let (room, mut events) = loop {
            let attempt = Room::connect(&config.url, &config.token, options.clone());
            tokio::pin!(attempt);
            let result = loop {
                tokio::select! {
                    result = &mut attempt => break result,
                    command = cmd_rx.recv(), if config.demand_driven => {
                        if !reject_disconnected(command, "livekit signaling is connecting", &mut rx_routes) {
                            return;
                        }
                    }
                }
            };
            match result {
                Ok(connected) => break connected,
                Err(error) => {
                    let detail = format!("livekit connect failed: {error}");
                    if !config.demand_driven {
                        let _ = ready_tx.send(Err(MediaError::Transport(detail)));
                        return;
                    }
                    tracing::warn!(error = %detail, "optional camera preview unavailable");
                    let retry = tokio::time::sleep(Duration::from_secs(5));
                    tokio::pin!(retry);
                    loop {
                        tokio::select! {
                            () = &mut retry => break,
                            command = cmd_rx.recv() => {
                                if !reject_disconnected(command, &detail, &mut rx_routes) {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        };
        if !config.demand_driven {
            let _ = ready_tx.send(Ok(()));
        }

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
                        let mut options = TrackPublishOptions {
                            source: TrackSource::Camera,
                            simulcast: !config.demand_driven,
                            ..Default::default()
                        };
                        if config.preview_max_kbps.is_some() || config.preview_fps.is_some() {
                            options.video_encoding = Some(VideoEncoding {
                                max_bitrate: u64::from(config.preview_max_kbps.unwrap_or(128)) * 1000,
                                max_framerate: config.preview_fps.unwrap_or(30.0),
                            });
                        }
                        let res = tokio::time::timeout(Duration::from_secs(4), room
                            .local_participant()
                            .publish_track(
                                LocalTrack::Video(track.clone()), options,
                            ))
                            .await
                            .map_err(|_| MediaError::Transport("publish_track timed out".into()))
                            .and_then(|res| res.map_err(|e| MediaError::Transport(format!("publish_track failed: {e}"))))
                            .map(|_publication| (source, track));
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
                            // failures surface here for the integrator log.
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

/// Drain optional preview requests while signaling is unavailable. Never build
/// an offline queue of frames/data or retain abandoned publication requests.
fn reject_disconnected(
    command: Option<Command>,
    detail: &str,
    rx_routes: &mut HashMap<&'static str, std_mpsc::Sender<Bytes>>,
) -> bool {
    match command {
        None | Some(Command::Shutdown) => false,
        Some(Command::PublishTrack { reply, .. }) => {
            let _ = reply.send(Err(MediaError::Transport(detail.to_owned())));
            true
        }
        Some(Command::OpenRx { topic, tx }) => {
            rx_routes.insert(topic.topic_str(), tx);
            true
        }
        Some(Command::PublishData { topic, .. }) => {
            tracing::warn!(topic = topic.topic_str(), error = %detail, "livekit data publish unavailable");
            true
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera(name: &str, width: u32, height: u32) -> pb::CameraDescription {
        pb::CameraDescription {
            name: name.to_owned(),
            width,
            height,
            ..Default::default()
        }
    }

    /// Every declared camera publishes at ITS declared resolution, not the
    /// VGA default: a track carries one resolution and `push_frame` rejects
    /// every frame that disagrees, so a camera missing from this map is a
    /// camera whose frames are all dropped — with nothing raising, and only
    /// a warn plus the uplink's drop counter to show for it.
    #[test]
    fn robot_cameras_are_declared_at_their_own_resolutions() {
        let robot = pb::RobotDescription {
            cameras: vec![camera("overhead", 1280, 720), camera("wrist", 320, 240)],
            ..Default::default()
        };

        let config = LiveKitConfig::new("ws://plane.invalid".to_owned(), "token".to_owned())
            .with_robot_cameras(&robot);

        assert_eq!(
            config.track_resolutions.get("overhead").copied(),
            Some((1280, 720)),
            "a declared camera must publish at its declared resolution"
        );
        assert_eq!(
            config.track_resolutions.get("wrist").copied(),
            Some((320, 240)),
            "EVERY declared camera, not just the first"
        );
        assert_eq!(
            config.track_resolutions.get("overhead/depth").copied(),
            Some((1280, 720)),
            "the paired depth-preview track must never fall back to VGA"
        );
        assert_eq!(
            config.track_resolutions.get("wrist/depth").copied(),
            Some((320, 240)),
            "every camera's depth-preview track keeps the camera dimensions"
        );
        assert_eq!(config.track_resolutions.len(), robot.cameras.len() * 2);
        assert!(
            !config
                .track_resolutions
                .values()
                .any(|&res| res == DEFAULT_TRACK_RESOLUTION),
            "no declared camera may silently inherit the VGA default"
        );
    }

    /// A robot that declares no cameras declares no resolutions — the
    /// default stands, and nothing is invented on the robot's behalf.
    #[test]
    fn a_robot_without_cameras_declares_no_resolutions() {
        let config = LiveKitConfig::new("ws://plane.invalid".to_owned(), "token".to_owned())
            .with_robot_cameras(&pb::RobotDescription::default());
        assert!(config.track_resolutions.is_empty());
    }

    #[test]
    fn preview_sampling_preserves_source_and_aspect_ratio() {
        assert_eq!(preview_dimensions(640, 480, Some(320)), (320, 240));
        assert_eq!(preview_dimensions(1280, 720, Some(320)), (320, 180));
        assert_eq!(preview_dimensions(96, 64, Some(320)), (96, 64));
        let pixels: Vec<u8> = (0..48).collect();
        let small = resize_rgb(4, 4, 2, 2, &pixels);
        assert_eq!(small, [0, 1, 2, 6, 7, 8, 24, 25, 26, 30, 31, 32]);
        assert_eq!(pixels.len(), 48);
    }

    #[test]
    fn disconnected_preview_does_not_delay_session_or_shutdown() {
        let started = std::time::Instant::now();
        let mut config = LiveKitConfig::new("ws://127.0.0.1:1".into(), "scoped-test".into());
        config.demand_driven = true;
        let media = LiveKitMedia::connect(config).expect("optional preview worker starts");
        assert!(started.elapsed() < Duration::from_secs(2));
        assert!(matches!(
            media.publish_track("camera"),
            Err(MediaError::Transport(_))
        ));
        drop(media);
        assert!(started.elapsed() < Duration::from_secs(3));
    }
}
