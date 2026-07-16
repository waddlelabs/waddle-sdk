//! The frame-ingestion seam: `Session::publish_frame` validates
//! the declared camera + frame shape, honors the declared `StreamPolicy`
//! uplink fps throttle, lazily `publish_track`s a camera on its first frame,
//! and drops (counted) under backpressure from a slow media plane — never
//! touching `Gate::gate()`'s fast path (this whole seam lives on the
//! customer thread plus one dedicated `waddle-media-uplink` pump).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::time::{Duration, Instant};

use waddle_media::{
    DataRx, DataTopic, DataTx, EncodedFrame, LoopbackMedia, MediaError, MediaPlane, TrackHandle,
};
use waddle_runtime::{ControlRegistry, FrameData, RuntimeError, Session, VerbError};
use waddle_types::pb::v0 as pb;

/// A registry with `hold`/`send` so a wired media plane's build-time
/// HOLD_FIRST check is satisfied (unrelated to this task; every test here
/// that wires `.media(...)` needs it).
fn registry() -> ControlRegistry {
    ControlRegistry {
        send: Some(Arc::new(
            |_chunk: &waddle_types::ActionChunk| -> Result<(), VerbError> { Ok(()) },
        )),
        hold: Some(Arc::new(|| Ok(()))),
        ..Default::default()
    }
}

/// A minimal 3-joint robot, optionally declaring cameras.
fn robot(cameras: Vec<pb::CameraDescription>) -> pb::RobotDescription {
    pb::RobotDescription {
        name: "media-uplink-bot".into(),
        robot_id: "media-uplink-01".into(),
        cell_id: "cell-media-uplink".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
                joints: (0..3)
                    .map(|i| pb::JointDescriptor {
                        name: format!("j{i}"),
                        ..Default::default()
                    })
                    .collect(),
            })),
            rate_hz: 50.0,
            chunking: None,
            gripper: None,
        }),
        cameras,
        ..Default::default()
    }
}

/// A tiny (4x4) declared camera, with an optional uplink policy.
fn camera(name: &str, uplink: Option<pb::stream_policy::UplinkPolicy>) -> pb::CameraDescription {
    pb::CameraDescription {
        name: name.to_owned(),
        width: 4,
        height: 4,
        fps: 30.0,
        encoding: pb::CameraEncoding::Rgb8 as i32,
        stream: Some(pb::StreamPolicy {
            local_full_rate: false,
            uplink,
        }),
        ..Default::default()
    }
}

/// A packed 4x4 RGB8 frame (all pixels the same value, for easy round-trip
/// assertions).
fn frame_4x4(value: u8) -> FrameData {
    FrameData::rgb8(4, 4, vec![value; 4 * 4 * 3])
}

fn wait_until(mut pred: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if pred() {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

// --- validation ------------------------------------------------------------

#[test]
fn unknown_camera_errors() {
    let (media, _far) = LoopbackMedia::new();
    let session = Session::builder("media-unknown")
        .robot(robot(vec![camera("front", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    let err = session
        .publish_frame("back", frame_4x4(1))
        .expect_err("an undeclared camera name must error");
    assert!(
        matches!(err, RuntimeError::UnknownCamera(ref name) if name == "back"),
        "expected UnknownCamera(\"back\"), got {err:?}"
    );
    session.shutdown();
}

#[test]
fn declared_camera_without_media_is_a_cheap_noop() {
    // No `.media(...)` at all: declared-but-no-media, per the brief — Ok(())
    // and nothing recorded (Local mode records no video in v0).
    let session = Session::builder("media-no-media")
        .robot(robot(vec![camera("front", None)]))
        .build()
        .unwrap();

    session
        .publish_frame("front", frame_4x4(1))
        .expect("declared camera + no media plane must be a cheap no-op");
    assert_eq!(session.camera_frames_dropped("front"), 0);
    session.shutdown();
}

#[test]
fn frame_dimensions_must_match_the_declared_camera() {
    let (media, _far) = LoopbackMedia::new();
    let session = Session::builder("media-baddims")
        .robot(robot(vec![camera("front", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    let wrong = FrameData::rgb8(8, 8, vec![0u8; 8 * 8 * 3]);
    let err = session
        .publish_frame("front", wrong)
        .expect_err("a frame whose dims disagree with the declared camera must error");
    assert!(
        matches!(err, RuntimeError::Media(MediaError::BadFrame { .. })),
        "expected a BadFrame media error, got {err:?}"
    );
    session.shutdown();
}

#[test]
fn h264_uplink_encoding_is_a_build_time_error() {
    let cam = camera(
        "front",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 30.0,
            encoding: pb::CameraEncoding::H264 as i32,
            max_kbps: None,
        }),
    );
    let (media, _far) = LoopbackMedia::new();
    let err = Session::builder("media-h264")
        .robot(robot(vec![cam]))
        .control(registry())
        .media(media)
        .build()
        .expect_err("H264 is a typed TODO — declaring it (with media wired) must fail loudly");
    assert!(
        matches!(err, RuntimeError::UnsupportedCameraEncoding { ref camera, .. } if camera == "front"),
        "expected UnsupportedCameraEncoding, got {err:?}"
    );
}

/// A *present* uplink policy with a non-positive fps is always a
/// misconfiguration (the SDK's own `Uplink` dataclass never lets a
/// customer declare one) — it must never collapse onto the same
/// "unthrottled" sentinel an altogether-undeclared policy uses (a
/// self-review regression: the two were briefly conflated, which would have
/// silently let a declared-but-broken `fps: 0.0` policy publish
/// unthrottled).
#[test]
fn a_present_uplink_policy_with_non_positive_fps_is_a_build_time_error() {
    let cam = camera(
        "front",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 0.0,
            encoding: pb::CameraEncoding::Rgb8 as i32,
            max_kbps: None,
        }),
    );
    let (media, _far) = LoopbackMedia::new();
    let err = Session::builder("media-zero-fps")
        .robot(robot(vec![cam]))
        .control(registry())
        .media(media)
        .build()
        .expect_err("a present uplink policy with fps <= 0 must fail loudly, not unthrottle");
    assert!(
        matches!(err, RuntimeError::InvalidCameraUplinkFps { ref camera, fps } if camera == "front" && fps == 0.0),
        "expected InvalidCameraUplinkFps, got {err:?}"
    );
}

#[test]
fn h264_uplink_encoding_without_media_still_builds() {
    // Nothing will ever be published for this camera (no media plane), so
    // the build-time check is scoped to cameras that would actually be
    // wired — never a fatal surprise for a purely descriptive declaration.
    let cam = camera(
        "front",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 30.0,
            encoding: pb::CameraEncoding::H264 as i32,
            max_kbps: None,
        }),
    );
    let session = Session::builder("media-h264-no-media")
        .robot(robot(vec![cam]))
        .build()
        .expect("no media wired: an unsupported encoding is inert, not build-fatal");
    session.shutdown();
}

// --- lazy publish_track ------------------------------------------------------

#[test]
fn first_publish_frame_lazily_publishes_the_track_exactly_once() {
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("media-lazy-track")
        .robot(robot(vec![camera("overhead", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    // Pushed faster than the tiny bounded queue can hold (by design — see
    // the backpressure test below), so not all 10 necessarily survive to
    // the far end; what this test asserts is that `publish_track` itself
    // ran exactly once despite that many pushes.
    for i in 0..10u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        std::thread::sleep(Duration::from_millis(5)); // let the pump keep up
    }
    assert!(
        wait_until(|| !far.frames().is_empty(), Duration::from_secs(2)),
        "the uplink pump never drained any queued frame"
    );
    assert_eq!(
        far.published_tracks(),
        vec!["overhead".to_string()],
        "publish_track must run exactly once despite many frames"
    );
    session.shutdown();
}

// --- fps throttle ------------------------------------------------------------

#[test]
fn fps_throttle_drops_frames_beyond_the_declared_rate() {
    let cam = camera(
        "overhead",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 5.0,
            encoding: pb::CameraEncoding::Rgb8 as i32,
            max_kbps: None,
        }),
    );
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("media-throttle")
        .robot(robot(vec![cam]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    // ~200ms of pushes at 5fps (200ms period) should admit only a couple of
    // frames, nowhere near the 100 pushed.
    for i in 0..100u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        std::thread::sleep(Duration::from_millis(2));
    }
    std::thread::sleep(Duration::from_millis(50)); // let the pump drain
    let got = far.frames().len();
    assert!(
        (1..=10).contains(&got),
        "expected roughly 5fps worth of frames over ~200ms admitted, got {got}"
    );
    session.shutdown();
}

// --- backpressure / drop counter --------------------------------------------

/// Delegates every `MediaPlane` call to a real `LoopbackMedia`, but sleeps
/// before completing `push_frame` — simulating a stalled encode/uplink step
/// downstream of the bounded per-camera queue (the runtime has no public
/// seam to swap in a custom `VideoEncoder`, so a slow `MediaPlane` — an
/// equally public trait — exercises the same backpressure path: the uplink
/// pump blocks in this call exactly as it would blocked inside a slow
/// encoder).
struct SlowMedia {
    inner: Arc<LoopbackMedia>,
    delay: Duration,
}

impl MediaPlane for SlowMedia {
    fn publish_track(&self, camera: &str) -> Result<TrackHandle, MediaError> {
        self.inner.publish_track(camera)
    }

    fn push_frame(&self, track: &TrackHandle, frame: EncodedFrame) -> Result<(), MediaError> {
        std::thread::sleep(self.delay);
        self.inner.push_frame(track, frame)
    }

    fn open_data_rx(&self, topic: DataTopic) -> Result<DataRx, MediaError> {
        self.inner.open_data_rx(topic)
    }

    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError> {
        self.inner.open_data_tx(topic)
    }
}

#[test]
fn backpressure_from_a_stalled_uplink_step_drops_the_oldest_frame_and_counts_it() {
    let (loopback, _far) = LoopbackMedia::new();
    let media = Arc::new(SlowMedia {
        inner: loopback,
        delay: Duration::from_millis(200),
    });
    // No declared uplink policy: unthrottled, so every push reaches the
    // bounded queue (isolating backpressure from the fps throttle).
    let session = Session::builder("media-backpressure")
        .robot(robot(vec![camera("overhead", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    for i in 0..40u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
    }
    assert!(
        wait_until(
            || session.camera_frames_dropped("overhead") > 0,
            Duration::from_secs(2)
        ),
        "expected the bounded queue to overflow against a stalled uplink step"
    );
    session.shutdown();
}

// --- uniform per-encoding behavior on the track path ----------------------

/// Mimics `LiveKitMedia::push_frame`'s real validation (see
/// `waddle-media::livekit`): a video track only ever ingests raw RGB8 or
/// already-planar I420 bytes at the track's resolution — never a
/// pre-encoded still-image byte stream (JPEG). Wrapping `LoopbackMedia` with
/// this exact shape check exercises the track path's real constraint
/// without the `livekit` feature or a live server — the same technique
/// `SlowMedia` above uses to exercise backpressure through the public
/// `MediaPlane` trait alone.
struct TrackShapedMedia {
    inner: Arc<LoopbackMedia>,
    width: u32,
    height: u32,
}

impl MediaPlane for TrackShapedMedia {
    fn publish_track(&self, camera: &str) -> Result<TrackHandle, MediaError> {
        self.inner.publish_track(camera)
    }

    fn push_frame(&self, track: &TrackHandle, frame: EncodedFrame) -> Result<(), MediaError> {
        let (w, h) = (self.width as usize, self.height as usize);
        let (cw, ch) = (w.div_ceil(2), h.div_ceil(2));
        let rgb_len = w * h * 3;
        let i420_len = w * h + 2 * cw * ch;
        if frame.data.len() != rgb_len && frame.data.len() != i420_len {
            return Err(MediaError::BadFrame {
                got: frame.data.len(),
                expected: rgb_len,
                layout: "RGB8 or planar I420 at the track's declared resolution",
            });
        }
        self.inner.push_frame(track, frame)
    }

    fn open_data_rx(&self, topic: DataTopic) -> Result<DataRx, MediaError> {
        self.inner.open_data_rx(topic)
    }

    fn open_data_tx(&self, topic: DataTopic) -> Result<DataTx, MediaError> {
        self.inner.open_data_tx(topic)
    }
}

/// A declared `CameraEncoding` on `StreamPolicy.uplink` is the
/// customer's bandwidth-intent for the track, not a promise that literal
/// byte format lands on the wire — the transport always receives raw frames
/// and converts to whatever the track actually needs (raw RGB8/I420;
/// libwebrtc's own codec does the real compression). RGB8 and JPEG
/// declarations must therefore behave identically here (both "accept": the
/// session builds cleanly and the frame reaches the track unmodified);
/// H264 remains the one genuinely unsupported encoding — a clear
/// build-time error, never a silent per-frame failure.
#[test]
fn rgb8_jpeg_and_h264_uplink_declarations_are_treated_uniformly_on_the_track_path() {
    for encoding in [pb::CameraEncoding::Rgb8, pb::CameraEncoding::Jpeg] {
        let cam = camera(
            "overhead",
            Some(pb::stream_policy::UplinkPolicy {
                fps: 30.0,
                encoding: encoding as i32,
                max_kbps: None,
            }),
        );
        let (loopback, far) = LoopbackMedia::new();
        let media = Arc::new(TrackShapedMedia {
            inner: loopback,
            width: 4,
            height: 4,
        });
        let session = Session::builder(format!("media-uniform-{encoding:?}"))
            .robot(robot(vec![cam]))
            .control(registry())
            .media(media)
            .build()
            .unwrap_or_else(|e| panic!("{encoding:?} uplink must build cleanly: {e:?}"));

        session.publish_frame("overhead", frame_4x4(3)).unwrap();
        assert!(
            wait_until(|| !far.frames().is_empty(), Duration::from_secs(2)),
            "{encoding:?}: a declared uplink encoding must still publish through the track path"
        );
        assert_eq!(
            session.camera_frames_dropped("overhead"),
            0,
            "{encoding:?}: the track-shaped media plane must accept the raw frame, not reject it"
        );
        let (_, encoded) = &far.frames()[0];
        assert_eq!(
            encoded.data.len(),
            4 * 4 * 3,
            "{encoding:?}: uplink must route raw RGB8 bytes to the track, not an actual re-encode"
        );
        session.shutdown();
    }

    // H264 is the one genuinely unsupported encoding: a clear build-time
    // error, never a silent per-frame failure once wired.
    let cam = camera(
        "overhead",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 30.0,
            encoding: pb::CameraEncoding::H264 as i32,
            max_kbps: None,
        }),
    );
    let (loopback, _far) = LoopbackMedia::new();
    let media = Arc::new(TrackShapedMedia {
        inner: loopback,
        width: 4,
        height: 4,
    });
    let err = Session::builder("media-uniform-h264")
        .robot(robot(vec![cam]))
        .control(registry())
        .media(media)
        .build()
        .expect_err("H264 must remain a clear build-time error");
    assert!(
        matches!(err, RuntimeError::UnsupportedCameraEncoding { ref camera, .. } if camera == "overhead"),
        "expected UnsupportedCameraEncoding, got {err:?}"
    );
}

#[test]
fn fps_throttle_does_not_count_as_a_drop() {
    // A steady stream well beyond a slow declared fps must be silently
    // throttled (Ok, no error) without ever touching the drop counter —
    // throttling is the policy working, never data loss.
    let cam = camera(
        "overhead",
        Some(pb::stream_policy::UplinkPolicy {
            fps: 1.0,
            encoding: pb::CameraEncoding::Rgb8 as i32,
            max_kbps: None,
        }),
    );
    let (media, _far) = LoopbackMedia::new();
    let session = Session::builder("media-throttle-not-drop")
        .robot(robot(vec![cam]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    for i in 0..20u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
    }
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        session.camera_frames_dropped("overhead"),
        0,
        "fps-throttled frames must never increment the drop counter"
    );
    session.shutdown();
}
