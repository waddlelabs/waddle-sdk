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
//! Backpressure: the bounded queue drops the OLDEST queued frame (not the
//! newest) when full — video wants the freshest frame, not a backlog — and
//! counts every such drop, surfaced as [`crate::Session::camera_frames_dropped`];
//! this is distinct from (and never conflated with) an fps-throttled frame,
//! which is simply never enqueued at all and never counted as a drop.
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
//! (`rgb8_to_i420`) and its own codec does the real compression. `waddle-media`'s
//! `JpegEncoder` remains available for a genuine still-image byte stream
//! path (e.g. a future data-channel/recording snapshot) — nothing on the
//! track path calls it. `CAMERA_ENCODING_H264` is the one genuinely
//! unsupported encoding and stays a build-time error (never a silent
//! per-frame failure): no encoder produces it, and no track can ingest it,
//! yet.
//!
//! Known limitations of the single-pump design (acceptable for this task's
//! scope; worth revisiting if multi-camera deployments need it):
//! `publish_track`/`push_frame` are synchronous `MediaPlane` trait calls —
//! a stalled transport for one camera blocks the ONE uplink thread's
//! round-robin, starving every other declared camera's queued frames for
//! as long as the stall lasts (same trust model as `ControlRegistry`'s
//! synchronous verb callables: the integrator's transport is expected to
//! stay bounded). A failing `publish_track` is retried on every
//! subsequent frame with no backoff (no circuit breaker) — a permanently
//! broken transport keeps re-attempting the same failing call once per
//! admitted frame rather than degrading gracefully. Because this same
//! thread also owns thread-join at `Session::shutdown`, a transport call
//! that never returns would block shutdown indefinitely — again, the same
//! risk category `VerbDispatch` already carries for a hanging integrator
//! callback.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::Duration;

use bytes::Bytes;
use parking_lot::Mutex;
use waddle_media::{
    MediaPlane, PassthroughEncoder, TrackHandle, VideoEncoder, VideoEncoding, make_encoder,
};
use waddle_types::pb::v0 as pb;

use crate::RuntimeError;
use crate::mirror::Mirror;

/// The per-camera bounded queue's capacity. Small and fixed: video wants the
/// freshest frame, not a deep backlog — a few frames of slack absorb jitter
/// between the customer's loop and the uplink pump without building a real
/// queue depth.
const QUEUE_CAPACITY: usize = 4;

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

struct QueuedFrame {
    t_ns: i64,
    frame: FrameData,
}

/// Per-camera uplink state: the declared policy, the bounded queue
/// `Session::publish_frame` (customer thread) writes into and
/// [`spawn_media_uplink`] (the uplink pump) drains, the fps-throttle
/// timestamp, the drop counter, and the lazily-published `TrackHandle`.
pub(crate) struct CameraUplink {
    name: String,
    fps: f64,
    encoding: VideoEncoding,
    last_sent_ns: AtomicI64,
    queue: Mutex<VecDeque<QueuedFrame>>,
    dropped: AtomicU64,
    track: Mutex<Option<TrackHandle>>,
}

impl CameraUplink {
    fn new(name: String, fps: f64, encoding: VideoEncoding) -> Self {
        Self {
            name,
            fps,
            encoding,
            last_sent_ns: AtomicI64::new(i64::MIN),
            queue: Mutex::new(VecDeque::with_capacity(QUEUE_CAPACITY)),
            dropped: AtomicU64::new(0),
            track: Mutex::new(None),
        }
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
    fn enqueue(&self, t_ns: i64, frame: FrameData) {
        let mut q = self.queue.lock();
        if q.len() >= QUEUE_CAPACITY {
            q.pop_front();
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        q.push_back(QueuedFrame { t_ns, frame });
    }

    pub(crate) fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }
}

/// Build one camera's [`CameraUplink`] from its declared `StreamPolicy`, or
/// a build-time error (see [`resolve_policy`]) — called only for cameras
/// that will actually be wired (a media plane is present), so a
/// not-yet-implemented encoding is fatal exactly where it would otherwise
/// fail silently on the first frame.
pub(crate) fn build_camera_uplink(
    cam: &pb::CameraDescription,
) -> Result<CameraUplink, RuntimeError> {
    let uplink = cam.stream.as_ref().and_then(|s| s.uplink.as_ref());
    let (fps, encoding) = resolve_policy(&cam.name, uplink)?;
    Ok(CameraUplink::new(cam.name.clone(), fps, encoding))
}

/// `Session::publish_frame`'s throttle + enqueue step, run on the customer's
/// thread: applies the fps throttle and, if the frame is admitted, enqueues
/// it. A throttled frame is silently dropped here — the declared policy
/// working as intended, never counted against [`CameraUplink::dropped`].
pub(crate) fn admit_and_enqueue(uplink: &CameraUplink, now_ns: i64, frame: FrameData) {
    if uplink.admit(now_ns) {
        uplink.enqueue(now_ns, frame);
    }
}

/// The single dedicated uplink pump (named `waddle-media-uplink`):
/// round-robins the declared cameras, draining at most one queued frame per
/// camera per pass, lazily `publish_track`-ing on a camera's first frame,
/// and encoding + `push_frame`-ing the rest. Joins the ordinary pump
/// lifecycle (mirror shutdown → exit before the session joins threads).
pub(crate) fn spawn_media_uplink(
    media: Arc<dyn MediaPlane>,
    cameras: Vec<Arc<CameraUplink>>,
    mirror: Arc<Mirror>,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-media-uplink".into())
        .spawn(move || {
            // One stateful encoder per camera (`VideoEncoder::encode` takes
            // `&mut self`), built lazily from the camera's resolved policy
            // on its first frame — mirrors the lazy `publish_track` below.
            let mut encoders: HashMap<String, Box<dyn VideoEncoder>> = HashMap::new();
            loop {
                if mirror.read().shutdown {
                    return;
                }
                let mut idle = true;
                for cam in &cameras {
                    let Some(queued) = cam.queue.lock().pop_front() else {
                        continue;
                    };
                    idle = false;
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
                                    continue;
                                }
                            }
                        }
                        guard.clone().expect("just set or already present")
                    };
                    let encoder = encoders.entry(cam.name.clone()).or_insert_with(|| {
                        #[allow(clippy::cast_possible_truncation)]
                        let (w, h) = (
                            queued.frame.width().min(u32::from(u16::MAX)) as u16,
                            queued.frame.height().min(u32::from(u16::MAX)) as u16,
                        );
                        make_encoder(cam.encoding, w, h)
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
                }
                if idle {
                    std::thread::sleep(Duration::from_millis(1));
                }
            }
        })
        .expect("spawn media uplink")
}
