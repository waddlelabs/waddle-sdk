//! libwaddle — the C ABI over `waddle-runtime`.
//!
//! **ABI STABILITY: UNSTABLE (N5).** This surface is explicitly unstable
//! until both the Python SDK and `waddle_ros` consume it in anger; stability
//! is declared afterward, as an event, never assumed from birth. The
//! generated header carries `#define WADDLE_ABI_UNSTABLE 1`.
//!
//! Contract:
//! - opaque handles only (`WaddleSession*`, `WaddleEpisode*`);
//! - configuration crosses as serialized `waddle.v0.RobotDescription` bytes;
//! - every function returns a status code; `waddle_last_error` retrieves a
//!   thread-local message for the most recent failure on this thread;
//! - verb callbacks are C function pointers + `user_data`, invoked ONLY from
//!   the core's verb-dispatch thread (serialized, never concurrently) —
//!   `user_data` must remain valid until `waddle_session_close` and must be
//!   safe to use from that thread;
//! - every entry point null-checks and catches panics; a panic maps to
//!   `WADDLE_STATUS_PANIC`, never unwinds across the boundary.

// The C ABI is unsafe by nature; every entry point documents its contract
// and is wrapped in catch_unwind.
#![allow(unsafe_code)]
#![allow(clippy::missing_safety_doc)] // contracts are on the module + fields
#![allow(clippy::undocumented_unsafe_blocks)] // per-fn contracts in the module docs
#![allow(missing_debug_implementations)] // opaque C handles never Debug-print

use std::cell::RefCell;
use std::ffi::{CStr, c_char, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;

use prost::Message;
use waddle_runtime::{ControlRegistry, Episode, Session, VerbError};
use waddle_types::pb::v0 as pb;
use waddle_types::{ActionChunk, TerminalOutcome};

pub const WADDLE_STATUS_OK: i32 = 0;
pub const WADDLE_STATUS_NULL_ARGUMENT: i32 = -1;
pub const WADDLE_STATUS_DECODE: i32 = -2;
pub const WADDLE_STATUS_RUNTIME: i32 = -3;
pub const WADDLE_STATUS_RESET_FAILED: i32 = -4;
pub const WADDLE_STATUS_PANIC: i32 = -5;
pub const WADDLE_STATUS_UTF8: i32 = -6;

/// Maximum action width crossing the ABI per tick.
pub const WADDLE_MAX_ACTION_DIMS: usize = 32;

/// Buffer size for a part name crossing the ABI, NUL included.
pub const WADDLE_MAX_PART_NAME: usize = 64;

thread_local! {
    static LAST_ERROR: RefCell<String> = const { RefCell::new(String::new()) };
}

fn set_error(msg: impl Into<String>) {
    LAST_ERROR.with(|e| *e.borrow_mut() = msg.into());
}

/// Retrieve the thread-local message for the most recent failure on this
/// thread. Copies up to `cap - 1` bytes plus a NUL; returns the full message
/// length in bytes (call again with a bigger buffer if truncated). Safe with
/// `buf == NULL` or `cap == 0` (returns the length only).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_last_error(buf: *mut c_char, cap: usize) -> usize {
    LAST_ERROR.with(|e| {
        let msg = e.borrow();
        let bytes = msg.as_bytes();
        if !buf.is_null() && cap > 0 {
            let n = bytes.len().min(cap - 1);
            // SAFETY: caller guarantees buf points to at least cap bytes.
            unsafe {
                std::ptr::copy_nonoverlapping(bytes.as_ptr().cast::<c_char>(), buf, n);
                *buf.add(n) = 0;
            }
        }
        bytes.len()
    })
}

/// Opaque session handle.
pub struct WaddleSession {
    session: Session,
}

/// Opaque episode handle. NOT thread-safe: use from one thread (the control
/// loop) only.
pub struct WaddleEpisode {
    episode: Episode,
}

/// The integrator's five-verb control contract as C callbacks. Any pointer
/// may be NULL (the verb is then not granted). Callbacks return 0 for
/// success, nonzero for failure. They are invoked ONLY from the core's
/// verb-dispatch thread.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct WaddleControl {
    /// send(user_data, values, len, gripper_or_null, part_or_null)
    ///
    /// `part` is the declared part this step addresses (`Action.part`, flag
    /// `waddle.v0.parts`) as a NUL-terminated string, or NULL for the whole
    /// declared space — which is every step on a declaration without parts.
    /// When it is set, `values`/`len` are THAT part's rows in that part's
    /// order, and the parts it does not name are commanded nothing this
    /// step ("move this part, hold the rest"): a consumer that writes a
    /// part-scoped row as the whole robot writes one arm's setpoint into
    /// another's. The pointer is valid only for the duration of the call.
    pub send: Option<
        unsafe extern "C" fn(*mut c_void, *const f64, usize, *const f64, *const c_char) -> i32,
    >,
    pub hold: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    pub resume: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    pub home: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    pub estop: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
    /// Opaque pointer passed back to every callback. Must remain valid until
    /// `waddle_session_close` and be safe to use from the dispatch thread.
    pub user_data: *mut c_void,
}

/// What `waddle_gate` returned for one tick.
#[repr(C)]
pub enum WaddleGateKind {
    /// Dispatch your own action.
    Pass = 0,
    /// Dispatch `values` instead (intervention).
    Substitute = 1,
    /// Dispatch `values` (cross-fade window; `progress` in [0,1]).
    Blend = 2,
    /// Do NOT dispatch: the runtime is driving `send` directly
    /// (claimed-while-stalled bypass); you are a spectator.
    Noop = 3,
    /// Hold position this tick.
    Hold = 4,
}

#[repr(C)]
pub struct WaddleGateResult {
    pub kind: WaddleGateKind,
    /// Populated for Substitute/Blend.
    pub values: [f64; WADDLE_MAX_ACTION_DIMS],
    pub values_len: usize,
    pub has_gripper: bool,
    pub gripper: f64,
    pub progress: f32,
    /// Provenance: 0 policy, 1 teleop, 2 agent, 3 custom.
    pub provenance: i32,
    /// The declared part a Substitute/Blend addressed (`Action.part`, flag
    /// `waddle.v0.parts`), NUL-terminated. EMPTY means the whole declared
    /// space — every action on a declaration without parts, and every
    /// whole-robot action on one with them. When it is set, `values` are
    /// THAT part's rows: a consumer that dispatches them as the whole robot
    /// writes one arm's setpoint into another's.
    ///
    /// Copied inline rather than pointed at: the name is owned by the
    /// decision, which this call returns by value and then drops.
    pub part: [c_char; WADDLE_MAX_PART_NAME],
    /// The part name's true length in bytes, which is 0 for the whole robot.
    /// `>= WADDLE_MAX_PART_NAME` means the name did not fit and `part` holds
    /// a PREFIX: never match a prefix against your declaration (two parts
    /// can share one) and never dispatch that tick as whole-robot — hold.
    pub part_len: usize,
}

struct UserData(*mut c_void);
// SAFETY: the ABI contract (struct docs) requires user_data to be valid for
// the session lifetime and usable from the dispatch thread.
unsafe impl Send for UserData {}
// SAFETY: as above; callbacks are additionally serialized by the dispatch
// thread, so there is no concurrent use.
unsafe impl Sync for UserData {}

impl UserData {
    /// Accessor (not field access) so closures capture the whole wrapper —
    /// disjoint capture of the raw field would bypass the Send/Sync impls.
    fn get(&self) -> *mut c_void {
        self.0
    }
}

fn registry_from(control: &WaddleControl) -> ControlRegistry {
    let mut registry = ControlRegistry::default();
    let user = control.user_data;

    if let Some(send) = control.send {
        let user = UserData(user);
        registry.send = Some(Arc::new(
            move |chunk: &ActionChunk| -> Result<(), VerbError> {
                for step in &chunk.steps {
                    let gripper = step.gripper;
                    let g_ptr = gripper
                        .as_ref()
                        .map_or(std::ptr::null(), std::ptr::from_ref);
                    // One allocation per part-scoped step, on the verb
                    // dispatch thread — never the gate's. A name that cannot
                    // cross as a C string fails the verb rather than crossing
                    // as NULL, which would report one part's rows as the
                    // whole robot's.
                    let part = step
                        .part
                        .as_deref()
                        .map(std::ffi::CString::new)
                        .transpose()
                        .map_err(|_| {
                            VerbError::Failed(
                                "part name contains an interior NUL and cannot cross the C ABI"
                                    .to_owned(),
                            )
                        })?;
                    let part_ptr = part.as_ref().map_or(std::ptr::null(), |c| c.as_ptr());
                    // SAFETY: pointers are valid for the duration of the call;
                    // the callback contract is documented on WaddleControl.
                    let rc = unsafe {
                        send(
                            user.get(),
                            step.values.as_ptr(),
                            step.values.len(),
                            g_ptr,
                            part_ptr,
                        )
                    };
                    if rc != 0 {
                        return Err(VerbError::Failed(format!("send callback returned {rc}")));
                    }
                }
                Ok(())
            },
        ));
    }

    let unit = |cb: Option<unsafe extern "C" fn(*mut c_void) -> i32>,
                user: *mut c_void|
     -> Option<Arc<dyn waddle_runtime::UnitVerb>> {
        let cb = cb?;
        let user = UserData(user);
        Some(Arc::new(move || -> Result<(), VerbError> {
            // SAFETY: per the WaddleControl contract.
            let rc = unsafe { cb(user.get()) };
            if rc == 0 {
                Ok(())
            } else {
                Err(VerbError::Failed(format!("verb callback returned {rc}")))
            }
        }))
    };
    registry.hold = unit(control.hold, user);
    registry.resume = unit(control.resume, user);
    registry.home = unit(control.home, user);
    registry.estop = unit(control.estop, user).map(|cb| (cb, waddle_runtime::EstopDecl::default()));
    registry
}

unsafe fn cstr_arg<'a>(ptr: *const c_char) -> Result<&'a str, i32> {
    if ptr.is_null() {
        return Err(WADDLE_STATUS_NULL_ARGUMENT);
    }
    // SAFETY: caller guarantees a NUL-terminated string.
    unsafe { CStr::from_ptr(ptr) }.to_str().map_err(|_| {
        set_error("argument is not valid UTF-8");
        WADDLE_STATUS_UTF8
    })
}

/// Open a session.
///
/// `robot_pb` / `robot_pb_len`: a serialized `waddle.v0.RobotDescription`.
/// `recording_dir`: optional (NULL) directory for Local-mode recording.
/// `control`: the five-verb contract (may have NULL members).
/// On success writes an owned handle to `out`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_session_open(
    project: *const c_char,
    robot_pb: *const u8,
    robot_pb_len: usize,
    control: *const WaddleControl,
    recording_dir: *const c_char,
    out: *mut *mut WaddleSession,
) -> i32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        if robot_pb.is_null() || out.is_null() || control.is_null() {
            set_error("null argument");
            return WADDLE_STATUS_NULL_ARGUMENT;
        }
        let project = match unsafe { cstr_arg(project) } {
            Ok(s) => s,
            Err(code) => return code,
        };
        // SAFETY: caller guarantees robot_pb points to robot_pb_len bytes.
        let bytes = unsafe { std::slice::from_raw_parts(robot_pb, robot_pb_len) };
        let robot = match pb::RobotDescription::decode(bytes) {
            Ok(r) => r,
            Err(e) => {
                set_error(format!("RobotDescription decode: {e}"));
                return WADDLE_STATUS_DECODE;
            }
        };
        // SAFETY: control checked non-null above; repr(C) POD copy.
        let control = unsafe { *control };
        let mut builder = Session::builder(project)
            .robot(robot)
            .control(registry_from(&control));
        if !recording_dir.is_null() {
            match unsafe { cstr_arg(recording_dir) } {
                Ok(dir) => builder = builder.recording_dir(dir),
                Err(code) => return code,
            }
        }
        match builder.build() {
            Ok(session) => {
                let handle = Box::new(WaddleSession { session });
                // SAFETY: out checked non-null above.
                unsafe { *out = Box::into_raw(handle) };
                WADDLE_STATUS_OK
            }
            Err(e) => {
                set_error(e.to_string());
                WADDLE_STATUS_RUNTIME
            }
        }
    }));
    result.unwrap_or_else(|_| {
        set_error("panic in waddle_session_open");
        WADDLE_STATUS_PANIC
    })
}

/// Close a session: joins all core threads and flushes recorders. The handle
/// is consumed (double-close is safe only via NULL).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_session_close(session: *mut WaddleSession) -> i32 {
    if session.is_null() {
        return WADDLE_STATUS_OK;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: ownership transferred back from waddle_session_open.
        let handle = unsafe { Box::from_raw(session) };
        handle.session.shutdown();
        WADDLE_STATUS_OK
    }));
    result.unwrap_or_else(|_| {
        set_error("panic in waddle_session_close");
        WADDLE_STATUS_PANIC
    })
}

/// Open an episode; blocks through the reset pipeline (the design contract).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_episode_start(
    session: *mut WaddleSession,
    task: *const c_char,
    out: *mut *mut WaddleEpisode,
) -> i32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        if session.is_null() || out.is_null() {
            set_error("null argument");
            return WADDLE_STATUS_NULL_ARGUMENT;
        }
        let task = match unsafe { cstr_arg(task) } {
            Ok(s) => s,
            Err(code) => return code,
        };
        // SAFETY: session is a live handle from waddle_session_open.
        let handle = unsafe { &*session };
        match handle.session.start_episode(task) {
            Ok(episode) => {
                // SAFETY: out checked non-null above.
                unsafe { *out = Box::into_raw(Box::new(WaddleEpisode { episode })) };
                WADDLE_STATUS_OK
            }
            Err(e) => {
                set_error(e.to_string());
                WADDLE_STATUS_RESET_FAILED
            }
        }
    }));
    result.unwrap_or_else(|_| {
        set_error("panic in waddle_episode_start");
        WADDLE_STATUS_PANIC
    })
}

/// The gate fast path. `values`/`values_len` is the policy action;
/// `gripper` may be NULL. `obs`/`obs_len` is the observation the action was
/// computed from (NULL or 0 = no observation; logged into the gate record).
/// Writes the decision to `out`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_gate(
    episode: *mut WaddleEpisode,
    values: *const f64,
    values_len: usize,
    gripper: *const f64,
    obs: *const f64,
    obs_len: usize,
    out: *mut WaddleGateResult,
) -> i32 {
    if episode.is_null() || values.is_null() || out.is_null() {
        set_error("null argument");
        return WADDLE_STATUS_NULL_ARGUMENT;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: live handle; values points to values_len doubles.
        let handle = unsafe { &mut *episode };
        let action = unsafe { std::slice::from_raw_parts(values, values_len) };
        // SAFETY: gripper is NULL or points to one double.
        let gripper = if gripper.is_null() {
            None
        } else {
            Some(unsafe { *gripper })
        };
        // SAFETY: obs is NULL or points to obs_len doubles.
        let obs = if obs.is_null() || obs_len == 0 {
            None
        } else {
            Some(unsafe { std::slice::from_raw_parts(obs, obs_len) })
        };
        let output = handle.episode.gate(action, gripper, obs);

        // SAFETY: out checked non-null; fully initialized below.
        let result = unsafe { &mut *out };
        result.values = [0.0; WADDLE_MAX_ACTION_DIMS];
        result.values_len = 0;
        result.has_gripper = false;
        result.gripper = 0.0;
        result.progress = 1.0;
        result.provenance = 0;
        result.part = [0; WADDLE_MAX_PART_NAME];
        result.part_len = 0;

        use waddle_gate::gate::GateOutput;
        let mut fill = |action: &waddle_gate::gate::OwnedAction| {
            let n = action.values.len().min(WADDLE_MAX_ACTION_DIMS);
            result.values[..n].copy_from_slice(&action.values[..n]);
            result.values_len = n;
            if let Some(g) = action.gripper {
                result.has_gripper = true;
                result.gripper = g;
            }
            // The part tag rides the decision the same way: copied in, with
            // the true length reported so a truncation is detectable rather
            // than silently answering to another part's name.
            if let Some(part) = action.part.as_deref() {
                let bytes = part.as_bytes();
                result.part_len = bytes.len();
                let n = bytes.len().min(WADDLE_MAX_PART_NAME - 1);
                for (slot, byte) in result.part[..n].iter_mut().zip(bytes) {
                    *slot = *byte as c_char;
                }
            }
        };
        match output {
            GateOutput::Pass { provenance } => {
                result.kind = WaddleGateKind::Pass;
                result.provenance = provenance_code(&provenance);
            }
            GateOutput::Substitute { action, provenance } => {
                result.kind = WaddleGateKind::Substitute;
                fill(&action);
                result.provenance = provenance_code(&provenance);
            }
            GateOutput::Blend {
                action,
                progress,
                provenance,
            } => {
                result.kind = WaddleGateKind::Blend;
                fill(&action);
                result.progress = progress;
                result.provenance = provenance_code(&provenance);
            }
            GateOutput::Noop { provenance } => {
                result.kind = WaddleGateKind::Noop;
                result.provenance = provenance_code(&provenance);
            }
            GateOutput::Hold => {
                result.kind = WaddleGateKind::Hold;
            }
        }
        WADDLE_STATUS_OK
    }));
    result.unwrap_or_else(|_| {
        set_error("panic in waddle_gate");
        WADDLE_STATUS_PANIC
    })
}

fn provenance_code(tag: &waddle_types::ProvenanceTag) -> i32 {
    match tag.provenance {
        waddle_types::Provenance::Policy => 0,
        waddle_types::Provenance::Teleop => 1,
        waddle_types::Provenance::Agent => 2,
        waddle_types::Provenance::Custom(_) => 3,
    }
}

/// 1 once the episode's outcome is decided — it reached a terminal outcome
/// or entered POST_RESET (where the outcome is already pinned and only the
/// scene cleanup, which self-resolves, is still running) — else 0.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_episode_done(episode: *const WaddleEpisode) -> i32 {
    if episode.is_null() {
        return 1;
    }
    // SAFETY: live handle.
    i32::from(unsafe { &*episode }.episode.done())
}

/// Terminate with an outcome: 1 success, 2 failure, 3 abort.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_episode_terminate(
    episode: *mut WaddleEpisode,
    outcome: i32,
    reason: *const c_char,
) -> i32 {
    let result = catch_unwind(AssertUnwindSafe(|| {
        if episode.is_null() {
            set_error("null argument");
            return WADDLE_STATUS_NULL_ARGUMENT;
        }
        let outcome = match outcome {
            1 => TerminalOutcome::Success,
            2 => TerminalOutcome::Failure,
            _ => TerminalOutcome::Abort,
        };
        let reason = if reason.is_null() {
            ""
        } else {
            match unsafe { cstr_arg(reason) } {
                Ok(s) => s,
                Err(code) => return code,
            }
        };
        // SAFETY: live handle.
        unsafe { &*episode }.episode.terminate(outcome, reason);
        WADDLE_STATUS_OK
    }));
    result.unwrap_or_else(|_| {
        set_error("panic in waddle_episode_terminate");
        WADDLE_STATUS_PANIC
    })
}

/// Release the episode handle (does not terminate; call terminate first for
/// a clean outcome).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn waddle_episode_close(episode: *mut WaddleEpisode) -> i32 {
    if episode.is_null() {
        return WADDLE_STATUS_OK;
    }
    // SAFETY: ownership transferred back from waddle_episode_start.
    drop(unsafe { Box::from_raw(episode) });
    WADDLE_STATUS_OK
}
