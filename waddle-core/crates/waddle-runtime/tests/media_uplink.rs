//! The frame-ingestion seam: `Session::publish_frame` validates
//! the declared camera + frame shape, honors the declared `StreamPolicy`
//! uplink fps throttle, lazily `publish_track`s a camera on its first frame,
//! drops (counted) under backpressure from a slow media plane, and — for a
//! camera declaring `StreamPolicy.still_fps` (flag `waddle.v0.obs.stills`) —
//! samples bounded-rate JPEG `FrameStill`s onto the CONTROL plane. Never
//! touching `Gate::gate()`'s fast path (this whole seam lives on the
//! customer thread plus one dedicated `waddle-media-uplink` pump).

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::sync::Arc;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use waddle_controlplane::{ClientMsg, InMemoryTransport, ServerMsg};
use waddle_media::{
    DataRx, DataTopic, DataTx, EncodedFrame, LoopbackMedia, MediaError, MediaPlane, TrackHandle,
};
use waddle_runtime::{ControlRegistry, FrameData, RuntimeError, Session, VerbError};
use waddle_types::pb::v0 as pb;

const STILLS_FLAG: &str = "waddle.v0.obs.stills";

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

/// The same robot, additionally declaring the grants an episode needs — for
/// the stills tests that also run gate ticks (to prove stills and the
/// reducer's own proprio uplink don't rate-limit each other).
fn robot_granted(cameras: Vec<pb::CameraDescription>) -> pb::RobotDescription {
    let mut robot = robot(cameras);
    robot.grants = vec![
        pb::Grant {
            verb: pb::Verb::Hold as i32,
            declared_latency_bound_ns: Some(50_000_000),
            ..Default::default()
        },
        pb::Grant {
            verb: pb::Verb::Send as i32,
            send_interfaces: vec![pb::SpaceKind::JointPosition as i32],
            ..Default::default()
        },
    ];
    robot
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
            ..Default::default()
        }),
        ..Default::default()
    }
}

/// The same tiny camera, additionally declaring control-plane stills.
fn stills_camera(name: &str, still_fps: f64) -> pb::CameraDescription {
    let mut cam = camera(name, None);
    cam.stream
        .as_mut()
        .expect("the helper always declares a StreamPolicy")
        .still_fps = Some(still_fps);
    cam
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

#[test]
fn depth_preview_is_an_independent_lazy_sibling_track() {
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("media-depth-track")
        .robot(robot(vec![camera("overhead", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    session.publish_frame("overhead", frame_4x4(7)).unwrap();
    session
        .publish_depth_preview("overhead", frame_4x4(19))
        .unwrap();

    assert!(
        wait_until(|| far.frames().len() == 2, Duration::from_secs(2)),
        "the RGB and depth-preview queues did not both drain"
    );
    let frames = far.frames();
    assert!(
        frames.iter().any(|(name, frame)| {
            name == "overhead" && frame.data.iter().all(|value| *value == 7)
        })
    );
    assert!(frames.iter().any(|(name, frame)| {
        name == "overhead/depth" && frame.data.iter().all(|value| *value == 19)
    }));
    let mut tracks = far.published_tracks();
    tracks.sort();
    assert_eq!(tracks, vec!["overhead", "overhead/depth"]);
    session.shutdown();
}

#[test]
fn depth_preview_reuses_camera_declaration_validation() {
    let (media, _far) = LoopbackMedia::new();
    let session = Session::builder("media-depth-validation")
        .robot(robot(vec![camera("overhead", None)]))
        .control(registry())
        .media(media)
        .build()
        .unwrap();

    let undeclared = session
        .publish_depth_preview("back", frame_4x4(1))
        .expect_err("depth cannot invent an undeclared camera");
    assert!(matches!(undeclared, RuntimeError::UnknownCamera(ref name) if name == "back"));

    let bad_shape = FrameData::rgb8(8, 8, vec![0; 8 * 8 * 3]);
    let malformed = session
        .publish_depth_preview("overhead", bad_shape)
        .expect_err("depth preview dimensions must match RGB/declaration dimensions");
    assert!(matches!(
        malformed,
        RuntimeError::Media(MediaError::BadFrame { .. })
    ));
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

// --- control-plane stills (flag `waddle.v0.obs.stills`) ---------------------

/// Everything the plane saw: the feature flags each `Register` declared, and
/// every `ObservationUpdate` uplinked.
#[derive(Default)]
struct PlaneLog {
    registered_flags: Vec<Vec<String>>,
    observations: Vec<pb::ObservationUpdate>,
}

impl PlaneLog {
    fn stills(&self) -> Vec<pb::FrameStill> {
        self.observations
            .iter()
            .filter_map(|o| match &o.payload {
                Some(pb::observation_update::Payload::Still(still)) => Some(still.clone()),
                _ => None,
            })
            .collect()
    }

    fn proprio_count(&self) -> usize {
        self.observations
            .iter()
            .filter(|o| matches!(o.payload, Some(pb::observation_update::Payload::Proprio(_))))
            .count()
    }
}

/// A control plane that records what it was told and accepts exactly
/// `accepted` at `Register` — the one knob every stills test turns.
fn logging_transport(accepted: &[&str]) -> (Arc<InMemoryTransport>, Arc<Mutex<PlaneLog>>) {
    let log: Arc<Mutex<PlaneLog>> = Arc::new(Mutex::new(PlaneLog::default()));
    let sink = log.clone();
    let accepted: Vec<String> = accepted.iter().map(|f| (*f).to_owned()).collect();
    let transport = InMemoryTransport::new(move |msg, tx: &Sender<ServerMsg>| match msg {
        ClientMsg::Register(req) => {
            sink.lock().registered_flags.push(req.feature_flags.clone());
            let _ = tx.send(ServerMsg::Registered(pb::RegisterResponse {
                accepted_feature_flags: accepted.clone(),
                ..Default::default()
            }));
        }
        ClientMsg::Observation(update) => sink.lock().observations.push(update),
        _ => {}
    });
    (transport, log)
}

/// The headline: a camera declaring `still_fps` with a transport but NO
/// media plane at all (the agent-only session shape — no LiveKit anywhere)
/// still gets bounded-rate JPEG stills onto the control plane, carrying the
/// camera's own `FrameNotice` sequence numbers.
#[test]
fn stills_ride_the_control_plane_without_a_media_plane() {
    const STILL_FPS: f64 = 20.0;
    let (transport, log) = logging_transport(&["waddle.v0.core", STILLS_FLAG]);
    let session = Session::builder("stills-no-media")
        .robot(robot(vec![stills_camera("overhead", STILL_FPS)]))
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    // 20 fps stills = a 50ms period on the frame timeline; 60 frames ~5ms
    // apart is ~300ms of publishing, comfortably several stills.
    let started = Instant::now();
    let mut published = 0u64;
    for i in 0..60u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        published += 1;
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(|| log.lock().stills().len() >= 3, Duration::from_secs(2)),
        "a declared still_fps must publish stills with no media plane wired at all"
    );
    let elapsed = started.elapsed();
    session.shutdown();

    let stills = log.lock().stills();
    let mut last_seq = 0;
    for still in &stills {
        assert_eq!(still.camera, "overhead");
        assert_eq!(still.encoding, pb::CameraEncoding::Jpeg as i32);
        assert_eq!((still.width, still.height), (4, 4));
        assert_eq!(
            still.data.get(..3),
            Some([0xff, 0xd8, 0xff].as_slice()),
            "a FrameStill must carry real JPEG bytes (SOI marker)"
        );
        assert!(
            still.frame_seq > last_seq,
            "frame_seq must advance with the camera's own frame sequence: {} after {last_seq}",
            still.frame_seq
        );
        assert!(
            still.frame_seq <= published,
            "frame_seq must number published frames (1-based), got {} of {published}",
            still.frame_seq
        );
        last_seq = still.frame_seq;
    }
    // The declared rate is a CAP, and the only load-independent way to say
    // so: however long the publishing loop actually took, no more stills may
    // have been sampled than that window allows (+1 for the first frame,
    // which is always due, and +1 for rounding).
    #[allow(clippy::cast_precision_loss)]
    let allowed = elapsed.as_secs_f64().mul_add(STILL_FPS, 2.0);
    assert!(
        (stills.len() as f64) <= allowed,
        "stills must be SAMPLED at the declared {STILL_FPS} fps, never one per published frame: \
         {} stills in {elapsed:?} (cap {allowed})",
        stills.len()
    );
}

/// VERSIONING §3: a behavior the connection did not accept is never
/// emitted. Same wiring as the test above, one flag removed — and the
/// session must stay otherwise alive (its proprio uplink keeps flowing), so
/// this pins the flag gate specifically, not a dead session.
#[test]
fn stills_stay_silent_when_the_flag_was_not_accepted() {
    let (transport, log) = logging_transport(&["waddle.v0.core"]);
    let session = Session::builder("stills-unnegotiated")
        .robot(robot_granted(vec![stills_camera("overhead", 100.0)]))
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("task").unwrap();
    for i in 0..60u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        let _ = ep.gate(&[0.0; 3], None, Some(&[0.1, 0.2, 0.3]));
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(|| log.lock().proprio_count() >= 2, Duration::from_secs(3)),
        "the session must still be uplinking observations — otherwise this proves nothing"
    );
    session.shutdown();

    assert!(
        log.lock().stills().is_empty(),
        "stills must never be emitted on a connection that did not accept the flag"
    );
}

/// The flag is declared at `Register` from the ROBOT's declaration: exactly
/// when some camera asks for stills, never otherwise (declaring it with no
/// camera asking would claim a behavior the session cannot produce).
#[test]
fn the_stills_flag_is_declared_only_when_a_camera_declares_still_fps() {
    for (cameras, expected) in [
        (vec![stills_camera("overhead", 2.0)], true),
        (vec![camera("overhead", None)], false),
        (vec![stills_camera("overhead", 0.0)], false),
        (
            vec![camera("front", None), stills_camera("wrist", 1.0)],
            true,
        ),
    ] {
        let (transport, log) = logging_transport(&[]);
        let session = Session::builder("stills-flag")
            .robot(robot(cameras))
            .control(registry())
            .transport(transport)
            .build()
            .unwrap();
        assert!(
            wait_until(
                || !log.lock().registered_flags.is_empty(),
                Duration::from_secs(2)
            ),
            "the session must register"
        );
        let flags = log.lock().registered_flags[0].clone();
        assert_eq!(
            flags.iter().any(|f| f == STILLS_FLAG),
            expected,
            "declared flags {flags:?} disagree with the cameras' still_fps"
        );
        session.shutdown();
    }
}

/// The two `ObservationUpdate` producers are independent in both
/// directions: stills are not capped by the reducer's 10 Hz proprio
/// cadence, and proprio keeps flowing while stills do.
#[test]
fn stills_and_proprio_observations_do_not_rate_limit_each_other() {
    let (transport, log) = logging_transport(&["waddle.v0.core", STILLS_FLAG]);
    let session = Session::builder("stills-vs-proprio")
        .robot(robot_granted(vec![stills_camera("overhead", 50.0)]))
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    let mut ep = session.start_episode("task").unwrap();
    for i in 0..80u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        let _ = ep.gate(&[0.0; 3], None, Some(&[0.1, 0.2, 0.3]));
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(
            || {
                let log = log.lock();
                log.stills().len() >= 5 && log.proprio_count() >= 2
            },
            Duration::from_secs(3)
        ),
        "both observation producers must keep flowing alongside each other"
    );
    session.shutdown();

    let log = log.lock();
    assert!(
        log.stills().len() > log.proprio_count(),
        "a 50 fps stills declaration must outpace the 10 Hz proprio cadence, \
         not inherit it: {} stills vs {} proprio",
        log.stills().len(),
        log.proprio_count()
    );
}

/// Stills and media are separate legs of the same intake: a camera wired to
/// both publishes video frames onto the track AND samples stills onto the
/// control plane, from the same `publish_frame` calls.
#[test]
fn a_camera_with_both_legs_feeds_the_track_and_the_control_plane() {
    let (transport, log) = logging_transport(&["waddle.v0.core", STILLS_FLAG]);
    let (media, far) = LoopbackMedia::new();
    let session = Session::builder("stills-and-media")
        .robot(robot(vec![stills_camera("overhead", 20.0)]))
        .control(registry())
        .media(media)
        .transport(transport)
        .build()
        .unwrap();

    for i in 0..40u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(
        wait_until(
            || !far.frames().is_empty() && log.lock().stills().len() >= 2,
            Duration::from_secs(2)
        ),
        "both legs must run from the same publish_frame intake"
    );
    // The track carries RAW frames (the transport does its own encoding);
    // only the stills leg ever produces a JPEG byte stream.
    let (_, encoded) = &far.frames()[0];
    assert_eq!(
        encoded.data.len(),
        4 * 4 * 3,
        "the media track must still receive raw RGB8, never the stills JPEG"
    );
    session.shutdown();
}

/// A camera with no `still_fps` and no media plane stays exactly the cheap
/// no-op it always was, even with a transport configured — no uplink state,
/// no stills, nothing counted.
#[test]
fn no_still_fps_and_no_media_is_still_a_cheap_noop_with_a_transport() {
    let (transport, log) = logging_transport(&["waddle.v0.core", STILLS_FLAG]);
    let session = Session::builder("stills-absent")
        .robot(robot(vec![camera("overhead", None)]))
        .control(registry())
        .transport(transport)
        .build()
        .unwrap();

    for i in 0..40u8 {
        session.publish_frame("overhead", frame_4x4(i)).unwrap();
    }
    std::thread::sleep(Duration::from_millis(100));
    assert_eq!(session.camera_frames_dropped("overhead"), 0);
    session.shutdown();
    assert!(
        log.lock().stills().is_empty(),
        "a camera that declared no still_fps must never produce stills"
    );
}
