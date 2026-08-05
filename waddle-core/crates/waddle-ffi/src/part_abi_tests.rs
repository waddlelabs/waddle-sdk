//! What a part-addressed step looks like from C (`Action.part`, flag
//! `waddle.v0.parts`): the name a Substitute carries in
//! [`WaddleGateResult::part`], and the name a dispatched step carries into
//! the `send` callback.
//!
//! `tests/roundtrip.rs` covers the other half — a declaration with no parts,
//! where the tag is NULL on every send and empty on every decision — and it
//! lives outside the crate because it needs nothing but the C entry points.
//! This half cannot: an intervention enters through the runtime `Session`
//! behind the opaque handle, and that handle is opaque BY CONTRACT (see the
//! module docs). Widening the crate's Rust surface so an external test could
//! reach inside would put a hole in the one thing this crate promises C
//! callers, so the injection seam is taken from the inside instead. What is
//! under assertion is still only what crossed `extern "C"`.
//!
//! Why it is worth the trouble: the field's own contract says a consumer
//! that misreads it "writes one arm's setpoint into another's". The inline
//! copy, its truncation bound, and the `CString` the send callback allocates
//! are the three places that can get that wrong.

#![allow(clippy::disallowed_methods)] // wall-clock deadlines are test-only

use std::ffi::{CStr, c_char, c_void};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use prost::Message;
use waddle_runtime::{Session, Status, grant_and_engage, push_intervention_chunk, release_claim};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActorKind, GateMode};

use crate::{
    WADDLE_MAX_PART_NAME, WADDLE_STATUS_OK, WaddleControl, WaddleEpisode, WaddleGateKind,
    WaddleGateResult, WaddleSession, waddle_episode_close, waddle_episode_start,
    waddle_episode_terminate, waddle_gate, waddle_session_close, waddle_session_open,
};

/// Seven rows per arm — six joints plus the gripper folded in as the last
/// row, the canonical bimanual declaration.
const ARM_DIMS: usize = 7;

/// A wait's failure bound, never a synchronisation device: the condition is
/// reached or the test fails.
const PATIENCE: Duration = Duration::from_secs(5);

/// Every step the C `send` callback was handed, read the way the ABI tells a
/// consumer to read it: NULL becomes `None` (the whole declared space).
#[derive(Default)]
struct SendLog(Mutex<Vec<(Option<String>, Vec<f64>)>>);

unsafe extern "C" fn log_send(
    user: *mut c_void,
    values: *const f64,
    len: usize,
    _gripper: *const f64,
    part: *const c_char,
) -> i32 {
    let log = unsafe { &*user.cast::<SendLog>() };
    let part = (!part.is_null()).then(|| {
        unsafe { CStr::from_ptr(part) }
            .to_string_lossy()
            .into_owned()
    });
    let values = unsafe { std::slice::from_raw_parts(values, len) }.to_vec();
    log.0.lock().unwrap().push((part, values));
    0
}

unsafe extern "C" fn ok_verb(_user: *mut c_void) -> i32 {
    0
}

fn control(log: &SendLog) -> WaddleControl {
    WaddleControl {
        send: Some(log_send),
        hold: Some(ok_verb),
        resume: Some(ok_verb),
        home: None,
        estop: None,
        user_data: std::ptr::from_ref(log).cast_mut().cast::<c_void>(),
    }
}

fn arm(prefix: &str) -> pb::ActionSpace {
    pb::ActionSpace {
        space: Some(pb::action_space::Space::JointPosition(pb::JointPosition {
            joints: (0..ARM_DIMS)
                .map(|i| pb::JointDescriptor {
                    name: format!("{prefix}{i}"),
                    ..Default::default()
                })
                .collect(),
        })),
        rate_hz: 50.0,
        chunking: None,
        gripper: None,
    }
}

/// Two named parts in declaration order (which IS the concatenated 14-row
/// layout). The second part's name is the caller's, so one test can hand it
/// a name too long for the ABI's buffer.
fn bimanual_bytes(right: &str) -> Vec<u8> {
    pb::RobotDescription {
        name: "ffi-bimanual".into(),
        robot_id: "ffi-bi-01".into(),
        cell_id: "cell-ffi".into(),
        action_space: Some(pb::ActionSpace {
            space: Some(pb::action_space::Space::Composite(pb::Composite {
                parts: vec![
                    pb::composite::Part {
                        name: "left".into(),
                        space: Some(arm("l")),
                    },
                    pb::composite::Part {
                        name: right.into(),
                        space: Some(arm("r")),
                    },
                ],
            })),
            rate_hz: 50.0,
            chunking: Some(pb::ChunkingSemantics {
                horizon_steps: 20,
                replan: pb::ReplanPolicy::Immediate as i32,
                interpolation: pb::Interpolation::Hold as i32,
            }),
            gripper: None,
        }),
        grants: vec![
            pb::Grant {
                verb: pb::Verb::Hold as i32,
                declared_latency_bound_ns: Some(40_000_000),
                ..Default::default()
            },
            pb::Grant {
                verb: pb::Verb::Send as i32,
                send_interfaces: vec![pb::SpaceKind::JointPosition as i32],
                ..Default::default()
            },
        ],
        ..Default::default()
    }
    .encode_to_vec()
}

/// One intervention chunk addressing a single declared part by name.
fn part_chunk(part: &str, value: f64) -> pb::ActionChunk {
    pb::ActionChunk {
        actions: vec![pb::Action {
            target: Some(pb::action::Target::JointPosition(pb::JointVector {
                values: vec![value; ARM_DIMS],
            })),
            gripper: None,
            t_offset_ns: 0,
            part: part.into(),
        }],
        seq: 1,
        source_id: "ffi-test".into(),
        ..Default::default()
    }
}

fn open(robot: &[u8], control: &WaddleControl) -> *mut WaddleSession {
    let project = std::ffi::CString::new("ffi-parts").unwrap();
    let mut session = std::ptr::null_mut();
    let rc = unsafe {
        waddle_session_open(
            project.as_ptr(),
            robot.as_ptr(),
            robot.len(),
            std::ptr::from_ref(control),
            std::ptr::null(),
            &raw mut session,
        )
    };
    assert_eq!(rc, WADDLE_STATUS_OK, "session open failed");
    session
}

fn start(session: *mut WaddleSession, task: &str) -> *mut WaddleEpisode {
    let task = std::ffi::CString::new(task).unwrap();
    let mut episode = std::ptr::null_mut();
    let rc = unsafe { waddle_episode_start(session, task.as_ptr(), &raw mut episode) };
    assert_eq!(rc, WADDLE_STATUS_OK, "episode start failed");
    episode
}

/// The runtime session behind the opaque handle — the injection seam this
/// module exists to reach (see the module docs). The borrow is laundered to
/// `'static` because the handle has no Rust lifetime to borrow from; every
/// caller below closes it with [`close`] at the end of the test, after the
/// last use.
fn inner(session: *mut WaddleSession) -> &'static Session {
    // SAFETY: a live handle from `waddle_session_open`, closed only by
    // `close` after the last borrow.
    &unsafe { &*session }.session
}

fn tick(episode: *mut WaddleEpisode, out: &mut std::mem::MaybeUninit<WaddleGateResult>) {
    let whole = [0.0f64; 2 * ARM_DIMS];
    let rc = unsafe {
        waddle_gate(
            episode,
            whole.as_ptr(),
            whole.len(),
            std::ptr::null(),
            std::ptr::null(),
            0,
            out.as_mut_ptr(),
        )
    };
    assert_eq!(rc, WADDLE_STATUS_OK);
}

fn wait_for(session: &Session, what: &str, pred: impl Fn(&Status) -> bool) {
    let deadline = Instant::now() + PATIENCE;
    loop {
        if pred(&session.status()) {
            return;
        }
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

fn close(session: *mut WaddleSession, episode: *mut WaddleEpisode) {
    assert_eq!(
        unsafe { waddle_episode_terminate(episode, 1, std::ptr::null()) },
        WADDLE_STATUS_OK
    );
    assert_eq!(unsafe { waddle_episode_close(episode) }, WADDLE_STATUS_OK);
    // Before the SendLog the callbacks point at goes out of scope.
    assert_eq!(unsafe { waddle_session_close(session) }, WADDLE_STATUS_OK);
}

/// The gate-decision half: a Substitute for one arm crosses with that arm's
/// rows AND that arm's name. Without the name a C consumer sees seven values
/// where its declaration has fourteen rows and cannot tell which end of the
/// robot they belong to.
#[test]
fn a_part_scoped_substitute_crosses_the_abi_with_its_part_name() {
    let robot = bimanual_bytes("right");
    let log = SendLog::default();
    let ctl = control(&log);
    let session = open(&robot, &ctl);
    let episode = start(session, "part-scoped-gate");
    let mut out = std::mem::MaybeUninit::<WaddleGateResult>::uninit();

    // The caller's own action commands the whole declared space even on a
    // declaration WITH parts: no tag, on the same session that is about to
    // produce one.
    tick(episode, &mut out);
    let result = unsafe { &*out.as_ptr() };
    assert!(matches!(result.kind, WaddleGateKind::Pass));
    assert_eq!(result.part_len, 0);

    wait_for(inner(session), "the episode to run", |s| {
        s.gate_mode == Some(GateMode::Passthrough)
    });
    grant_and_engage(inner(session), "claim-abi", "agent-plane", ActorKind::Agent);
    wait_for(inner(session), "the claim to engage", |s| {
        s.gate_mode == Some(GateMode::Intervention)
    });
    push_intervention_chunk(inner(session), part_chunk("right", 0.75));

    let deadline = Instant::now() + PATIENCE;
    let (values, part, part_len) = loop {
        tick(episode, &mut out);
        let result = unsafe { &*out.as_ptr() };
        if matches!(result.kind, WaddleGateKind::Substitute) {
            // Read exactly as the header tells a consumer to: a
            // NUL-terminated name, with its true length alongside.
            let part = unsafe { CStr::from_ptr(result.part.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            break (
                result.values[..result.values_len].to_vec(),
                part,
                result.part_len,
            );
        }
        assert!(
            Instant::now() < deadline,
            "the part-scoped chunk never substituted"
        );
        std::thread::sleep(Duration::from_millis(5));
    };

    assert_eq!(
        values,
        vec![0.75; ARM_DIMS],
        "a part-scoped action carries THAT part's width, not the whole robot's"
    );
    assert_eq!(part, "right", "the decision must name the part it commands");
    assert_eq!(part_len, "right".len());

    release_claim(inner(session), "claim-abi");
    close(session, episode);
}

/// A name longer than the ABI's buffer is reported truncated and SAYS SO:
/// `part_len` is the true length, so a consumer can refuse rather than match
/// a prefix (two declared parts may share one) or dispatch as whole-robot.
#[test]
fn a_part_name_too_long_for_the_buffer_reports_its_true_length() {
    let long = "r".repeat(WADDLE_MAX_PART_NAME + 16);
    let robot = bimanual_bytes(&long);
    let log = SendLog::default();
    let ctl = control(&log);
    let session = open(&robot, &ctl);
    let episode = start(session, "part-scoped-truncation");
    let mut out = std::mem::MaybeUninit::<WaddleGateResult>::uninit();

    tick(episode, &mut out);
    wait_for(inner(session), "the episode to run", |s| {
        s.gate_mode == Some(GateMode::Passthrough)
    });
    grant_and_engage(
        inner(session),
        "claim-long",
        "agent-plane",
        ActorKind::Agent,
    );
    wait_for(inner(session), "the claim to engage", |s| {
        s.gate_mode == Some(GateMode::Intervention)
    });
    push_intervention_chunk(inner(session), part_chunk(&long, 0.5));

    let deadline = Instant::now() + PATIENCE;
    let (part_bytes, part_len) = loop {
        tick(episode, &mut out);
        let result = unsafe { &*out.as_ptr() };
        if matches!(result.kind, WaddleGateKind::Substitute) {
            break (result.part, result.part_len);
        }
        assert!(
            Instant::now() < deadline,
            "the part-scoped chunk never substituted"
        );
        std::thread::sleep(Duration::from_millis(5));
    };

    assert_eq!(
        part_len,
        long.len(),
        "the true length is what makes the truncation detectable"
    );
    assert_eq!(
        part_bytes[WADDLE_MAX_PART_NAME - 1],
        0,
        "the buffer stays NUL-terminated even when the name did not fit"
    );
    let held = unsafe { CStr::from_ptr(part_bytes.as_ptr()) }
        .to_string_lossy()
        .into_owned();
    assert_eq!(
        held,
        long[..WADDLE_MAX_PART_NAME - 1],
        "what fits is a prefix of the name, never a different name"
    );

    release_claim(inner(session), "claim-long");
    close(session, episode);
}

/// The dispatch half: when the caller's loop stalls under a claim, the bypass
/// pump drives `send` directly — and the step it hands the callback names its
/// part, as a C string the callback can read for the duration of the call.
#[test]
fn a_dispatched_part_scoped_step_names_its_part_to_the_send_callback() {
    let robot = bimanual_bytes("right");
    let log = SendLog::default();
    let ctl = control(&log);
    let session = open(&robot, &ctl);
    let episode = start(session, "part-scoped-send");
    let mut out = std::mem::MaybeUninit::<WaddleGateResult>::uninit();

    tick(episode, &mut out);
    wait_for(inner(session), "the episode to run", |s| {
        s.gate_mode == Some(GateMode::Passthrough)
    });
    grant_and_engage(
        inner(session),
        "claim-bypass",
        "agent-plane",
        ActorKind::Agent,
    );
    // The caller stops ticking here: the stall detector flips the session to
    // BYPASS, where the pump — not `waddle_gate` — drives the verb.
    wait_for(inner(session), "the bypass window", |s| {
        s.gate_mode == Some(GateMode::Bypass)
    });
    push_intervention_chunk(inner(session), part_chunk("right", 1.25));

    let deadline = Instant::now() + PATIENCE;
    while log.0.lock().unwrap().is_empty() {
        assert!(Instant::now() < deadline, "the bypass pump never sent");
        std::thread::sleep(Duration::from_millis(5));
    }
    let sent = log.0.lock().unwrap().clone();
    assert_eq!(
        sent[0],
        (Some("right".to_owned()), vec![1.25; ARM_DIMS]),
        "the dispatched step must name the part it commands, at that part's width"
    );

    release_claim(inner(session), "claim-bypass");
    close(session, episode);
}
