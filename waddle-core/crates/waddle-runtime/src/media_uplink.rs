//! Frame ingestion: `Session::publish_frame`'s uplink seam.
//!
//! `publish_frame` runs on the CUSTOMER's thread and must stay cheap: it
//! validates the camera + frame shape against the robot's declaration,
//! applies the declared uplink fps throttle (a wait-free atomic-timestamp
//! check — dropping a too-soon frame is the policy working, not a fault),
//! and otherwise only copies the frame onto a small bounded per-camera
//! queue. Everything expensive — the (lazy, once-per-camera) `publish_track`
//! call and the actual encode/`push_frame` — runs off that thread, on the
//! single dedicated `waddle-media-uplink` pump this module spawns.
//!
//! Control-plane stills (flag `waddle.v0.obs.stills`): a camera declaring
//! `StreamPolicy.still_fps > 0` additionally tees each published frame into
//! a latest-wins per-camera slot; the SAME pump samples that slot at
//! `still_fps` (a frame-timeline throttle mirroring the media fps throttle),
//! JPEG-encodes the sampled frame, and sends it as
//! `ObservationUpdate{ still: FrameStill }` on the CONTROL plane. This is
//! bounded-rate agent perception, never a video path — LiveKit media remains
//! the only video transport, and the tee works with or without a media plane
//! wired (an agent-only session has no LiveKit). Stills are sent only while
//! the current connection has accepted the flag at Register
//! (`crate::Status::stills_negotiated` — VERSIONING §3: a behavior the
//! connection did not accept is never emitted); an unaccepted flag leaves the
//! slot holding at most one frame per camera, never a growing queue.
//!
//! The two uplink paths are deliberately independent in BOTH directions: a
//! still never rides (or is rate-limited by) the reducer's 10 Hz
//! `StreamObservations` proprio cadence, and a still never displaces a
//! proprio sample — they are separate producers of the same
//! `ClientMsg::Observation` message, each with its own declared rate, joined
//! only at the client's send queue.
//!
//! `frame_seq` is minted here, at intake, once per validated
//! `publish_frame` call: THE per-camera `FrameNotice` sequence counter
//! (services.proto pins `FrameStill` to "FrameNotice's per-camera sequence
//! numbering"). Nothing emits `FrameNotice` yet; when something does, it
//! must draw from this same [`CameraUplink`] counter, never a parallel one.
//!
//! Backpressure: the bounded queue drops the OLDEST queued frame (not the
//! newest) when full — video wants the freshest frame, not a backlog — and
//! counts every such drop, surfaced as [`crate::Session::camera_frames_dropped`];
//! this is distinct from (and never conflated with) an fps-throttled frame,
//! which is simply never enqueued at all and never counted as a drop. Stills
//! are droppable by design (latest-wins: a slow pump loses intermediate
//! frames, never the freshest) and stay OUT of that counter, which means
//! exactly what it always meant: media-uplink loss. That same
//! droppable-by-design contract continues past this seam, in both directions
//! a slow plane can fail — `ClientMsg::is_droppable` classifies a still once
//! and the control-plane client honours it twice: a still is dropped rather
//! than buffered while the plane is PARTITIONED (`buffer_when_offline`), so
//! stills can never evict real episode history from the bounded offline
//! buffer, and it is dropped rather than queued once the transport's
//! in-flight bound is full while the plane is CONNECTED but not draining
//! (`waddle_controlplane::inflight`), so a plane that accepts the
//! observation stream and then stops reading it cannot turn this
//! bounded-rate tee into unbounded memory growth.
//!
//! Encoding: the one real transport this wires against,
//! `LiveKitMedia`, publishes a WebRTC video *track* — libwebrtc
//! encodes the uplink itself and its native video source ingests raw
//! RGB8/I420 only; a still-image byte stream (JPEG) is not a track format
//! at all. A declared `StreamPolicy.uplink.encoding` is therefore treated as
//! the customer's **bandwidth-intent**, not a promise of that literal byte
//! format landing on the wire: every encoding this pump can actually wire
//! onto a track — `CAMERA_ENCODING_UNSPECIFIED`/`RGB8`/`BGR8`/`JPEG` (and
//! `Z16` — depth is out of scope for this rgb8-only seam) — resolves to raw
//! passthrough, and the transport (`LiveKitMedia::push_frame`, or
//! `LoopbackMedia` in tests) converts to whatever the track needs
//! (`rgb8_to_i420`) and its own codec does the real compression. The one
//! genuine still-image byte stream is the control-plane stills tee above,
//! which is exactly what `waddle-media`'s `JpegEncoder` exists for — nothing
//! on the track path calls it. `CAMERA_ENCODING_H264` is the one genuinely
//! unsupported encoding and stays a build-time error (never a silent
//! per-frame failure): no encoder produces it, and no track can ingest it,
//! yet.
//!
//! Known limitations of the single-pump design (acceptable for this task's
//! scope; worth revisiting if multi-camera deployments need it):
//! `publish_track`/`push_frame` are synchronous `MediaPlane` trait calls —
//! a stalled transport for one camera blocks the ONE uplink thread's
//! round-robin, starving every other declared camera's queued frames (and
//! stills) for as long as the stall lasts (same trust model as
//! `ControlRegistry`'s synchronous verb callables: the integrator's
//! transport is expected to stay bounded). A failing `publish_track` is
//! retried on every subsequent frame with no backoff (no circuit breaker) —
//! a permanently broken transport keeps re-attempting the same failing call
//! once per admitted frame rather than degrading gracefully. Because this
//! same thread also owns thread-join at `Session::shutdown`, a transport
//! call that never returns would block shutdown indefinitely — again, the
//! same risk category `VerbDispatch` already carries for a hanging
//! integrator callback.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::Duration;

use bytes::Bytes;
use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, ControlPlaneClient};
use waddle_media::{
    JpegEncoder, MediaPlane, PassthroughEncoder, TrackHandle, VideoEncoder, VideoEncoding,
    make_encoder,
};
use waddle_types::pb::v0 as pb;

use crate::RuntimeError;
use crate::mirror::Mirror;

/// Flag `waddle.v0.obs.stills` (VERSIONING.md registry): declared at
/// Register iff a declared camera carries `StreamPolicy.still_fps > 0`;
/// stills are sent only when the current connection accepted it.
pub(crate) const STILLS_FLAG: &str = "waddle.v0.obs.stills";

/// The per-camera bounded queue's capacity. Small and fixed: video wants the
/// freshest frame, not a deep backlog — a few frames of slack absorb jitter
/// between the customer's loop and the uplink pump without building a real
/// queue depth.
const QUEUE_CAPACITY: usize = 4;

/// JPEG quality for control-plane stills. Fixed: stills are bounded-rate
/// agent perception, not an archival or video surface — a mid-range quality
/// keeps each `FrameStill` small on the control plane without a per-camera
/// knob nobody has asked for.
const STILL_JPEG_QUALITY: u8 = 80;

/// One raw video frame for [`crate::Session::publish_frame`]. RGB8 only in
/// this task; `pixels` is an enum (not a bare `Bytes` field) so a future
/// `Depth16` variant (declared `CameraEncoding::Z16`) can land without
/// breaking this constructor.
#[derive(Debug, Clone)]
pub struct FrameData {
    width: u32,
    height: u32,
    pixels: FramePixels,
}

/// The pixel payload of a [`FrameData`].
#[derive(Debug, Clone)]
#[non_exhaustive]
pub enum FramePixels {
    /// Packed row-major RGB8: `width * height * 3` bytes.
    Rgb8(Bytes),
}

impl FrameData {
    /// A packed row-major RGB8 frame (`width * height * 3` bytes).
    #[must_use]
    pub fn rgb8(width: u32, height: u32, pixels: impl Into<Bytes>) -> Self {
        Self {
            width,
            height,
            pixels: FramePixels::Rgb8(pixels.into()),
        }
    }

    #[must_use]
    pub fn width(&self) -> u32 {
        self.width
    }

    #[must_use]
    pub fn height(&self) -> u32 {
        self.height
    }

    fn as_bytes(&self) -> &[u8] {
        match &self.pixels {
            FramePixels::Rgb8(b) => b,
        }
    }

    /// The raw byte length, for `BadFrame`-shaped error reporting at the
    /// `Session::publish_frame` validation site.
    pub(crate) fn byte_len(&self) -> usize {
        self.as_bytes().len()
    }
}

/// Resolve a declared camera's `StreamPolicy.uplink` into what this task's
/// uplink pump enforces: the fps throttle (`0.0` = no uplink policy
/// declared at all = unthrottled; a *declared* policy always carries a
/// positive fps — see below) and the encoder selection. See the module doc
/// for the encoding mapping and its known LiveKit/JPEG gap.
fn resolve_policy(
    camera: &str,
    uplink: Option<&pb::stream_policy::UplinkPolicy>,
) -> Result<(f64, VideoEncoding), RuntimeError> {
    // `None` (no uplink policy declared at all) and `Some(fps: 0.0)` (a
    // present-but-degenerate policy) must NOT collapse onto the same
    // unthrottled sentinel: the SDK's own `Uplink` dataclass already
    // requires `fps > 0` whenever a policy is declared at all, so a
    // non-positive fps on a *present* policy only reaches here via a
    // direct (non-Python) `pb::CameraDescription` construction — fail
    // loudly rather than silently treating a declared "0 fps" as
    // unthrottled (the opposite of what a caller who bothered to declare a
    // policy almost certainly intended).
    let fps = match uplink {
        None => 0.0,
        Some(u) if u.fps > 0.0 => u.fps,
        Some(u) => {
            return Err(RuntimeError::InvalidCameraUplinkFps {
                camera: camera.to_owned(),
                fps: u.fps,
            });
        }
    };
    // Every declared encoding this pump can actually wire onto a track
    // resolves to raw passthrough — see the module doc: the declared
    // encoding is bandwidth-intent, not a promise of that literal byte
    // format on the wire, and the transport does the real conversion +
    // compression. `H264` is the one genuinely unsupported encoding: no
    // encoder produces it and no track can ingest it, so it stays a
    // build-time error rather than a silent per-frame failure.
    let encoding_tag = uplink.map_or(0, |u| u.encoding);
    let encoding = match pb::CameraEncoding::try_from(encoding_tag) {
        Ok(pb::CameraEncoding::H264) => {
            return Err(RuntimeError::UnsupportedCameraEncoding {
                camera: camera.to_owned(),
                encoding: "H264",
            });
        }
        _ => VideoEncoding::Passthrough,
    };
    Ok((fps, encoding))
}

/// A declared camera's `StreamPolicy.still_fps`, as this seam reads it:
/// `0.0` = no stills tee. Unlike `UplinkPolicy.fps` (where a *present*
/// policy carrying a non-positive fps is a loud build error, since presence
/// alone signals intent), `still_fps` has a documented in-band "off" value —
/// descriptors.proto pins "0/absent means no stills" — so there is no
/// presence-vs-value ambiguity to preserve, and a non-positive rate is
/// simply off.
fn resolve_still_fps(stream: Option<&pb::StreamPolicy>) -> f64 {
    stream
        .and_then(|s| s.still_fps)
        .filter(|fps| *fps > 0.0)
        .unwrap_or(0.0)
}

/// Whether this camera's declaration asks for control-plane stills — the
/// one place that question is answered, so `SessionBuilder::build`'s flag
/// declaration and the per-camera tee can never disagree about it.
pub(crate) fn declares_stills(cam: &pb::CameraDescription) -> bool {
    resolve_still_fps(cam.stream.as_ref()) > 0.0
}

struct QueuedFrame {
    t_ns: i64,
    /// The per-camera `FrameNotice` sequence number minted at intake (see
    /// the module doc): 1-based, one per validated `publish_frame` call.
    seq: u64,
    frame: FrameData,
}

/// Per-camera uplink state: the declared policy, the bounded queue
/// `Session::publish_frame` (customer thread) writes into and
/// [`spawn_media_uplink`] (the uplink pump) drains, the fps-throttle
/// timestamp, the drop counter, the lazily-published `TrackHandle`, and the
/// control-plane stills tee (latest-wins slot + frame-timeline throttle).
pub(crate) struct CameraUplink {
    name: String,
    /// Whether this camera publishes onto a media plane at all. False for a
    /// stills-only camera in a session with no media wired: "no media plane
    /// ⇒ no media uplink" stays true (nothing is ever enqueued for the
    /// media leg, so the bounded queue can't fill against a drain that
    /// isn't there), while the stills tee runs independently.
    media_wired: bool,
    fps: f64,
    encoding: VideoEncoding,
    last_sent_ns: AtomicI64,
    queue: Mutex<VecDeque<QueuedFrame>>,
    dropped: AtomicU64,
    track: Mutex<Option<TrackHandle>>,
    /// `StreamPolicy.still_fps`, `0.0` = no stills tee for this camera (see
    /// [`resolve_still_fps`]).
    still_fps: f64,
    /// The stills tee: latest-wins, capacity one — a still is droppable by
    /// design, and the pump always wants the freshest frame at each due
    /// instant, never a backlog.
    still_slot: Mutex<Option<QueuedFrame>>,
    /// The still throttle's "last sampled" stamp, on the FRAME timeline (the
    /// `SessionClock` stamp minted at `publish_frame`), never a pump-side
    /// clock read: the sampled rate is then a property of the frames
    /// themselves, which keeps this pump clock-free and the sampling
    /// deterministic under test.
    last_still_ns: AtomicI64,
    /// THE per-camera `FrameNotice` sequence counter (module doc): stores
    /// the last minted value; intake mints with `fetch_add(1) + 1` so
    /// sequences are 1-based (0 = "never", matching proto3's absent
    /// default).
    last_frame_seq: AtomicU64,
}

impl CameraUplink {
    fn new(
        name: String,
        media_wired: bool,
        fps: f64,
        encoding: VideoEncoding,
        still_fps: f64,
    ) -> Self {
        Self {
            name,
            media_wired,
            fps,
            encoding,
            last_sent_ns: AtomicI64::new(i64::MIN),
            queue: Mutex::new(VecDeque::with_capacity(QUEUE_CAPACITY)),
            dropped: AtomicU64::new(0),
            track: Mutex::new(None),
            still_fps,
            still_slot: Mutex::new(None),
            last_still_ns: AtomicI64::new(i64::MIN),
            last_frame_seq: AtomicU64::new(0),
        }
    }

    /// Mint this frame's per-camera `FrameNotice` sequence number. Called
    /// once per validated `publish_frame`, BEFORE either leg's throttle:
    /// the sequence numbers a camera's frames, not the subset some policy
    /// admitted, so a still's `frame_seq` locates it in the camera's own
    /// stream (which is the whole point of sharing `FrameNotice`'s
    /// numbering).
    fn next_seq(&self) -> u64 {
        self.last_frame_seq.fetch_add(1, Ordering::Relaxed) + 1
    }

    /// The fps throttle: wait-free (one atomic load, maybe a CAS retry — no
    /// lock, no syscall). `true` admits the frame (and claims this instant
    /// as the new "last sent"); `false` means drop it — the declared policy
    /// working as intended, never counted in [`Self::dropped`].
    fn admit(&self, now_ns: i64) -> bool {
        if self.fps <= 0.0 {
            return true; // no uplink policy declared: unthrottled
        }
        #[allow(clippy::cast_possible_truncation)]
        let period_ns = (1_000_000_000.0 / self.fps) as i64;
        loop {
            let last = self.last_sent_ns.load(Ordering::Relaxed);
            if last != i64::MIN && now_ns.saturating_sub(last) < period_ns {
                return false;
            }
            if self
                .last_sent_ns
                .compare_exchange_weak(last, now_ns, Ordering::Relaxed, Ordering::Relaxed)
                .is_ok()
            {
                return true;
            }
        }
    }

    /// Enqueue, dropping the OLDEST queued frame (not this new one) when the
    /// bounded queue is already full.
    fn enqueue(&self, t_ns: i64, seq: u64, frame: FrameData) {
        let mut q = self.queue.lock();
        if q.len() >= QUEUE_CAPACITY {
            q.pop_front();
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        q.push_back(QueuedFrame { t_ns, seq, frame });
    }

    /// The stills tee, on the customer's thread: latest-wins into a
    /// capacity-one slot. Never blocks and never grows — an overwritten
    /// frame is a still the pump did not get to in time, which is exactly
    /// the declared degradation (droppable, freshest-wins), not counted in
    /// [`Self::dropped`] (that counter means media-uplink loss).
    fn tee_still(&self, t_ns: i64, seq: u64, frame: FrameData) {
        *self.still_slot.lock() = Some(QueuedFrame { t_ns, seq, frame });
    }

    /// The stills throttle, on the pump: take the camera's newest published
    /// frame IFF it is due at the declared `still_fps`, measured on the
    /// FRAME timeline. A not-yet-due frame is deliberately LEFT in the slot
    /// (never discarded): it either becomes due on a later pass or is
    /// replaced by something fresher, so a sparse publisher still gets its
    /// stills and a fast one is sampled, not truncated.
    ///
    /// Only the uplink pump calls this — the single reader — so
    /// `last_still_ns` needs no CAS: the slot lock already serializes it
    /// against nothing but itself.
    fn take_due_still(&self) -> Option<QueuedFrame> {
        let mut slot = self.still_slot.lock();
        let t_ns = slot.as_ref()?.t_ns;
        #[allow(clippy::cast_possible_truncation)]
        let period_ns = (1_000_000_000.0 / self.still_fps) as i64;
        let last = self.last_still_ns.load(Ordering::Relaxed);
        if last != i64::MIN && t_ns.saturating_sub(last) < period_ns {
            return None;
        }
        self.last_still_ns.store(t_ns, Ordering::Relaxed);
        slot.take()
    }

    pub(crate) fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// Build one camera's [`CameraUplink`], or a build-time error (see
/// [`resolve_policy`]).
///
/// `media_wired` says whether a media plane exists for this session at all.
/// The declared `StreamPolicy.uplink` is resolved (and so validated) ONLY
/// when it is: an unsupported encoding must fail loudly exactly where it
/// would otherwise fail silently on the first frame, and stay inert for a
/// purely descriptive declaration nothing will ever publish. The stills tee
/// is orthogonal — a camera reaches this function whenever EITHER leg has
/// somewhere to go (see `SessionBuilder::build`).
pub(crate) fn build_camera_uplink(
    cam: &pb::CameraDescription,
    media_wired: bool,
) -> Result<CameraUplink, RuntimeError> {
    let (fps, encoding) = if media_wired {
        let uplink = cam.stream.as_ref().and_then(|s| s.uplink.as_ref());
        resolve_policy(&cam.name, uplink)?
    } else {
        (0.0, VideoEncoding::Passthrough)
    };
    Ok(CameraUplink::new(
        cam.name.clone(),
        media_wired,
        fps,
        encoding,
        resolve_still_fps(cam.stream.as_ref()),
    ))
}

/// `Session::publish_frame`'s intake, run on the customer's thread: mint the
/// frame's sequence number, tee it to the stills slot (latest-wins) when the
/// camera declared `still_fps`, and apply the media leg's fps throttle +
/// bounded enqueue when a media plane is wired. A media-throttled frame is
/// silently dropped here — the declared policy working as intended, never
/// counted against [`CameraUplink::dropped`] — and the two legs' rates are
/// independent (a frame can be admitted by one and not the other).
pub(crate) fn admit_and_enqueue(uplink: &CameraUplink, now_ns: i64, frame: FrameData) {
    let seq = uplink.next_seq();
    if uplink.still_fps > 0.0 {
        // Cheap: `FrameData` is a refcounted `Bytes` handle plus two u32s.
        uplink.tee_still(now_ns, seq, frame.clone());
    }
    if uplink.media_wired && uplink.admit(now_ns) {
        uplink.enqueue(now_ns, seq, frame);
    }
}

/// The single dedicated uplink pump (named `waddle-media-uplink`):
/// round-robins the declared cameras, draining at most one queued frame per
/// camera per pass onto the media plane (lazily `publish_track`-ing on a
/// camera's first frame, then encoding + `push_frame`-ing), and sampling
/// each camera's stills slot at its declared `still_fps` onto the control
/// plane. Either leg may be absent — an agent-only session has no media
/// plane, a media-only session has no stills — and the pump runs as long as
/// at least one camera has either. Joins the ordinary pump lifecycle (mirror
/// shutdown → exit before the session joins threads).
pub(crate) fn spawn_media_uplink(
    media: Option<Arc<dyn MediaPlane>>,
    plane: Option<Arc<ControlPlaneClient>>,
    cameras: Vec<Arc<CameraUplink>>,
    mirror: Arc<Mirror>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-media-uplink".into())
        .spawn(move || {
            // One stateful encoder per camera per leg (`VideoEncoder::encode`
            // takes `&mut self`), built lazily from the camera's first frame
            // — mirrors the lazy `publish_track` below.
            let mut encoders: HashMap<String, Box<dyn VideoEncoder>> = HashMap::new();
            let mut still_encoders: HashMap<String, JpegEncoder> = HashMap::new();
            loop {
                let status = mirror.read();
                if status.shutdown {
                    return;
                }
                // VERSIONING §3: stills are emitted only while the CURRENT
                // connection has accepted `waddle.v0.obs.stills` at Register
                // (the plane pump refreshes this on every re-registration).
                let stills_on = status.stills_negotiated;
                let mut idle = true;
                for cam in &cameras {
                    if cam.media_wired
                        && let Some(media) = media.as_deref()
                        && pump_media_frame(cam, media, &mut encoders)
                    {
                        idle = false;
                    }
                    if stills_on
                        && let Some(plane) = plane.as_deref()
                        && pump_still(cam, plane, &mut still_encoders)
                    {
                        idle = false;
                    }
                }
                if idle {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        })
        .expect("spawn media uplink")
}

/// One camera's media leg for a single pump pass: drain at most one queued
/// frame, lazily publishing the track on the first one, then encode and
/// `push_frame` it. Returns whether this camera had any work (a drained
/// frame — including one lost to a transport failure, which is counted).
fn pump_media_frame(
    cam: &CameraUplink,
    media: &dyn MediaPlane,
    encoders: &mut HashMap<String, Box<dyn VideoEncoder>>,
) -> bool {
    let Some(queued) = cam.queue.lock().pop_front() else {
        return false;
    };
    let track = {
        let mut guard = cam.track.lock();
        if guard.is_none() {
            match media.publish_track(&cam.name) {
                Ok(t) => *guard = Some(t),
                Err(err) => {
                    tracing::warn!(
                        camera = %cam.name,
                        error = %err,
                        "publish_track failed; dropping frame"
                    );
                    cam.dropped.fetch_add(1, Ordering::Relaxed);
                    return true;
                }
            }
        }
        guard.clone().expect("just set or already present")
    };
    let encoder = encoders.entry(cam.name.clone()).or_insert_with(|| {
        make_encoder(
            cam.encoding,
            clamp_u16(queued.frame.width()),
            clamp_u16(queued.frame.height()),
        )
        .unwrap_or_else(|_| Box::new(PassthroughEncoder))
    });
    match encoder.encode(queued.t_ns, queued.frame.as_bytes()) {
        Ok(encoded) => {
            if let Err(err) = media.push_frame(&track, encoded) {
                tracing::warn!(
                    camera = %cam.name,
                    error = %err,
                    "push_frame failed; dropping frame"
                );
                cam.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
        Err(err) => {
            tracing::warn!(
                camera = %cam.name,
                error = %err,
                "frame encode failed; dropping frame"
            );
            cam.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
    true
}

/// One camera's control-plane stills leg for a single pump pass: if the
/// camera's newest published frame is due at its declared `still_fps`,
/// JPEG-encode it (here, on the pump — never the customer's `publish_frame`
/// thread, and never the gate path) and send it as an
/// `ObservationUpdate{still}`. Returns whether a still was due.
///
/// The `t_ns` carried is the frame's own session-monotonic stamp, minted by
/// `SessionClock` at `publish_frame` — the observation is located where the
/// frame happened, not where the pump got to it.
fn pump_still(
    cam: &CameraUplink,
    plane: &ControlPlaneClient,
    encoders: &mut HashMap<String, JpegEncoder>,
) -> bool {
    let Some(due) = cam.take_due_still() else {
        return false;
    };
    let (width, height) = (due.frame.width(), due.frame.height());
    let encoder = encoders.entry(cam.name.clone()).or_insert_with(|| {
        JpegEncoder::new(clamp_u16(width), clamp_u16(height), STILL_JPEG_QUALITY)
    });
    match encoder.encode(due.t_ns, due.frame.as_bytes()) {
        Ok(encoded) => plane.send(ClientMsg::Observation(pb::ObservationUpdate {
            t_ns: due.t_ns,
            payload: Some(pb::observation_update::Payload::Still(pb::FrameStill {
                camera: cam.name.clone(),
                frame_seq: due.seq,
                encoding: pb::CameraEncoding::Jpeg as i32,
                width,
                height,
                data: encoded.data.to_vec(),
            })),
        })),
        // Droppable by design: a still that won't encode is skipped, never
        // retried and never counted against the media drop counter.
        Err(err) => tracing::warn!(
            camera = %cam.name,
            error = %err,
            "still encode failed; dropping still"
        ),
    }
    true
}

/// Pixel dimensions as the encoders take them. Saturating rather than
/// wrapping: a camera declaring more than 65535 px on a side is beyond
/// anything this seam supports, and the encoder's own length check rejects
/// the resulting frame loudly instead of encoding garbage.
#[allow(clippy::cast_possible_truncation)]
fn clamp_u16(v: u32) -> u16 {
    v.min(u32::from(u16::MAX)) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stills_camera(still_fps: f64) -> CameraUplink {
        CameraUplink::new(
            "overhead".into(),
            false,
            0.0,
            VideoEncoding::Passthrough,
            still_fps,
        )
    }

    fn frame(value: u8) -> FrameData {
        FrameData::rgb8(4, 4, vec![value; 4 * 4 * 3])
    }

    /// The headline sampling property, on a fully simulated frame timeline
    /// (the throttle reads the frame's own `SessionClock` stamp, so a
    /// synthetic timeline IS the clock): publishing at 20 Hz through one
    /// simulated second with `still_fps = 2` samples exactly two stills, at
    /// the two due instants — and each carries the `FrameNotice` sequence
    /// number of the frame it actually sampled, not of the frames the
    /// throttle passed over.
    #[test]
    fn stills_sample_at_the_declared_rate_on_the_frame_timeline() {
        let cam = stills_camera(2.0);
        let mut sampled = Vec::new();
        // 20 Hz for one second: t = 0ms, 50ms, ..., 950ms.
        for i in 0..20i64 {
            let t_ns = i * 50_000_000;
            #[allow(clippy::cast_possible_truncation)]
            admit_and_enqueue(&cam, t_ns, frame(i as u8));
            // The pump gets a pass after every publish (the pessimistic
            // case for sampling: it never sees a stale slot).
            if let Some(due) = cam.take_due_still() {
                sampled.push((due.t_ns, due.seq));
            }
        }
        assert_eq!(
            sampled,
            vec![(0, 1), (500_000_000, 11)],
            "2 fps over a simulated second must sample exactly the 0ms and 500ms frames"
        );
        assert_eq!(
            cam.last_frame_seq.load(Ordering::Relaxed),
            20,
            "every published frame numbers the camera's sequence, sampled or not"
        );
        assert!(
            cam.queue.lock().is_empty(),
            "no media plane wired: nothing may ever reach the media queue"
        );
        assert_eq!(
            cam.dropped(),
            0,
            "stills never touch the media drop counter"
        );
    }

    /// A pump that falls behind loses the intermediate frames, never the
    /// freshest one, and never grows a backlog.
    #[test]
    fn the_stills_slot_is_latest_wins() {
        let cam = stills_camera(1000.0); // every frame due
        for i in 0..5i64 {
            #[allow(clippy::cast_possible_truncation)]
            admit_and_enqueue(&cam, i * 1_000_000, frame(i as u8));
        }
        let due = cam.take_due_still().expect("a frame is waiting");
        assert_eq!((due.t_ns, due.seq), (4_000_000, 5), "newest frame wins");
        assert!(cam.take_due_still().is_none(), "the slot holds one frame");
    }

    /// A frame that is not yet due stays in the slot rather than being
    /// discarded: a publisher slower than its declared `still_fps` must
    /// still get every frame sampled.
    #[test]
    fn a_not_yet_due_frame_is_kept_for_the_next_pass() {
        let cam = stills_camera(2.0); // 500ms period
        admit_and_enqueue(&cam, 0, frame(0));
        assert_eq!(cam.take_due_still().map(|d| d.seq), Some(1));
        admit_and_enqueue(&cam, 100_000_000, frame(1));
        assert!(
            cam.take_due_still().is_none(),
            "100ms after the last sample is not due at 2 fps"
        );
        // No newer frame arrives; the held one becomes due by the pump's
        // own later passes only once the timeline advances past it, so the
        // NEXT published frame is what carries the sample.
        admit_and_enqueue(&cam, 600_000_000, frame(2));
        assert_eq!(cam.take_due_still().map(|d| d.seq), Some(3));
    }

    /// `still_fps` absent/0: the tee never runs at all — no slot writes, no
    /// sequence-number surprises for the media leg.
    #[test]
    fn no_declared_still_fps_means_no_tee() {
        let cam = stills_camera(0.0);
        for i in 0..10i64 {
            #[allow(clippy::cast_possible_truncation)]
            admit_and_enqueue(&cam, i * 1_000_000, frame(i as u8));
        }
        assert!(cam.still_slot.lock().is_none());
        assert!(cam.take_due_still().is_none());
    }

    /// `still_fps` is read from exactly one place, and a non-positive
    /// declaration is the documented "off" value (descriptors.proto), never
    /// a rate.
    #[test]
    fn still_fps_resolution_treats_non_positive_as_off() {
        let policy = |still_fps: Option<f64>| pb::StreamPolicy {
            local_full_rate: false,
            uplink: None,
            still_fps,
        };
        assert_eq!(resolve_still_fps(None), 0.0);
        assert_eq!(resolve_still_fps(Some(&policy(None))), 0.0);
        assert_eq!(resolve_still_fps(Some(&policy(Some(0.0)))), 0.0);
        assert_eq!(resolve_still_fps(Some(&policy(Some(-4.0)))), 0.0);
        assert_eq!(resolve_still_fps(Some(&policy(Some(2.0)))), 2.0);
    }
}
