//! FFI round-trip: session open (pb-encoded declaration + C callbacks) →
//! episode → gate → terminate → close, plus null/error paths and
//! double-close safety. Exercises the extern "C" functions directly.

#![allow(unsafe_code)]
#![allow(clippy::undocumented_unsafe_blocks)] // FFI calls under the module contract

use std::ffi::{CString, c_char, c_void};
use std::sync::atomic::{AtomicU32, Ordering};

use prost::Message;
use waddle::{
    WADDLE_STATUS_DECODE, WADDLE_STATUS_NULL_ARGUMENT, WADDLE_STATUS_OK, WaddleControl,
    WaddleGateKind, WaddleGateResult, waddle_episode_close, waddle_episode_done,
    waddle_episode_start, waddle_episode_terminate, waddle_gate, waddle_last_error,
    waddle_session_close, waddle_session_open,
};
use waddle_types::pb::v0 as pb;

static HOLD_CALLS: AtomicU32 = AtomicU32::new(0);

unsafe extern "C" fn hold_cb(_user: *mut c_void) -> i32 {
    HOLD_CALLS.fetch_add(1, Ordering::SeqCst);
    0
}

unsafe extern "C" fn send_cb(
    _user: *mut c_void,
    _values: *const f64,
    _len: usize,
    _gripper: *const f64,
) -> i32 {
    0
}

fn robot_bytes() -> Vec<u8> {
    pb::RobotDescription {
        name: "ffi-bot".into(),
        robot_id: "ffi-01".into(),
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
        ..Default::default()
    }
    .encode_to_vec()
}

fn control() -> WaddleControl {
    WaddleControl {
        send: Some(send_cb),
        hold: Some(hold_cb),
        resume: None,
        home: None,
        estop: None,
        user_data: std::ptr::null_mut(),
    }
}

fn last_error() -> String {
    let mut buf = vec![0u8; 256];
    let n = unsafe { waddle_last_error(buf.as_mut_ptr().cast::<c_char>(), buf.len()) };
    String::from_utf8_lossy(&buf[..n.min(buf.len() - 1)]).into_owned()
}

#[test]
fn full_round_trip_with_recording() {
    let dir = tempfile::tempdir().unwrap();
    let project = CString::new("ffi-project").unwrap();
    let task = CString::new("pick").unwrap();
    let dir_c = CString::new(dir.path().to_str().unwrap()).unwrap();
    let robot = robot_bytes();
    let ctl = control();

    let mut session = std::ptr::null_mut();
    let rc = unsafe {
        waddle_session_open(
            project.as_ptr(),
            robot.as_ptr(),
            robot.len(),
            &raw const ctl,
            dir_c.as_ptr(),
            &raw mut session,
        )
    };
    assert_eq!(rc, WADDLE_STATUS_OK, "open failed: {}", last_error());

    let mut episode = std::ptr::null_mut();
    let rc = unsafe { waddle_episode_start(session, task.as_ptr(), &raw mut episode) };
    assert_eq!(rc, WADDLE_STATUS_OK, "start failed: {}", last_error());
    assert_eq!(unsafe { waddle_episode_done(episode) }, 0);

    let action = [0.1f64, 0.2, 0.3];
    let mut out = std::mem::MaybeUninit::<WaddleGateResult>::uninit();
    for _ in 0..20 {
        let rc = unsafe {
            waddle_gate(
                episode,
                action.as_ptr(),
                action.len(),
                std::ptr::null(),
                out.as_mut_ptr(),
            )
        };
        assert_eq!(rc, WADDLE_STATUS_OK);
        let result = unsafe { &*out.as_ptr() };
        assert!(matches!(result.kind, WaddleGateKind::Pass));
        assert_eq!(result.provenance, 0);
    }

    let rc = unsafe { waddle_episode_terminate(episode, 1, std::ptr::null()) };
    assert_eq!(rc, WADDLE_STATUS_OK);
    assert_eq!(unsafe { waddle_episode_done(episode) }, 1);
    assert_eq!(unsafe { waddle_episode_close(episode) }, WADDLE_STATUS_OK);
    assert_eq!(unsafe { waddle_session_close(session) }, WADDLE_STATUS_OK);

    // The sidecar landed on disk.
    let sidecars = std::fs::read_dir(dir.path())
        .unwrap()
        .filter_map(Result::ok)
        .filter(|e| e.file_name().to_string_lossy().ends_with(".sidecar.json"))
        .count();
    assert_eq!(sidecars, 1);
}

#[test]
fn error_paths_and_double_close_are_safe() {
    // Null args.
    let rc = unsafe {
        waddle_session_open(
            std::ptr::null(),
            std::ptr::null(),
            0,
            std::ptr::null(),
            std::ptr::null(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(rc, WADDLE_STATUS_NULL_ARGUMENT);

    // Garbage pb bytes. (A truncated valid message can decode as empty —
    // use bytes that cannot parse as a RobotDescription: wrong wire type.)
    let project = CString::new("p").unwrap();
    let garbage = [0xffu8; 8];
    let ctl = control();
    let mut session = std::ptr::null_mut();
    let rc = unsafe {
        waddle_session_open(
            project.as_ptr(),
            garbage.as_ptr(),
            garbage.len(),
            &raw const ctl,
            std::ptr::null(),
            &raw mut session,
        )
    };
    assert_eq!(rc, WADDLE_STATUS_DECODE);
    assert!(last_error().contains("decode"));

    // NULL close is a no-op, twice.
    assert_eq!(
        unsafe { waddle_session_close(std::ptr::null_mut()) },
        WADDLE_STATUS_OK
    );
    assert_eq!(
        unsafe { waddle_episode_close(std::ptr::null_mut()) },
        WADDLE_STATUS_OK
    );
}
