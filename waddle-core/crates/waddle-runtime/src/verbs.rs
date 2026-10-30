//! The five-verb dispatch: integrator callables invoked ONLY on one
//! dedicated core-owned thread, serialized (a guarantee the callable can
//! rely on), wrapped in `catch_unwind` (a panicking callback becomes a
//! `VerbError`, never a poisoned core), and timed (feeding the N11
//! callback-dispatch proxy signal). Estop bypasses the queue via a priority
//! slot checked before every dequeue.

use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::Mutex;
use waddle_types::time::Clock;
use waddle_types::{ActionChunk, EpisodeId, Verb, VerbRequest};

use crate::mirror::Mirror;
use crate::playback::execution::{ExecutionSlot, ExecutionTask};
use crate::playback::{
    ExecutionFailure, ExecutionHandle, FiniteReferences, PlaybackError, PlaybackIo,
};

enum Work {
    Verb(VerbRequest),
    Playback(Box<ExecutionTask>),
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum VerbError {
    #[error("verb {0:?} is not granted (no callable registered)")]
    NotRegistered(Verb),
    #[error("verb callable failed: {0}")]
    Failed(String),
    #[error("verb callable panicked")]
    Panicked,
}

/// The integrator's `send` callable for one canonical space.
pub trait SendVerb: Send + Sync + 'static {
    fn send(&self, chunk: &ActionChunk) -> Result<(), VerbError>;
}

impl<F> SendVerb for F
where
    F: Fn(&ActionChunk) -> Result<(), VerbError> + Send + Sync + 'static,
{
    fn send(&self, chunk: &ActionChunk) -> Result<(), VerbError> {
        self(chunk)
    }
}

/// hold / resume / home / estop callables.
pub trait UnitVerb: Send + Sync + 'static {
    fn call(&self) -> Result<(), VerbError>;
}

impl<F> UnitVerb for F
where
    F: Fn() -> Result<(), VerbError> + Send + Sync + 'static,
{
    fn call(&self) -> Result<(), VerbError> {
        self()
    }
}

/// The declared estop guarantee (rides the Grant; recorded here for
/// dispatch).
#[derive(Debug, Clone, Copy, Default)]
pub struct EstopDecl {
    pub hardware: bool,
    pub declared_latency_bound_ns: Option<i64>,
}

/// The declared control contract: which verbs exist (grants are negotiated
/// from exactly this).
#[derive(Default)]
pub struct ControlRegistry {
    pub send: Option<Arc<dyn SendVerb>>,
    pub hold: Option<Arc<dyn UnitVerb>>,
    pub resume: Option<Arc<dyn UnitVerb>>,
    pub home: Option<Arc<dyn UnitVerb>>,
    pub estop: Option<(Arc<dyn UnitVerb>, EstopDecl)>,
}

impl std::fmt::Debug for ControlRegistry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ControlRegistry")
            .field("send", &self.send.is_some())
            .field("hold", &self.hold.is_some())
            .field("resume", &self.resume.is_some())
            .field("home", &self.home.is_some())
            .field("estop", &self.estop.is_some())
            .finish()
    }
}

/// The result of one dispatched verb, reported back to the reducer.
#[derive(Debug, Clone)]
pub struct VerbOutcome {
    pub verb: Verb,
    pub result: Result<(), VerbError>,
    pub latency_ns: i64,
    pub at: waddle_types::MonoNs,
}

/// Recent callback-dispatch latencies (N11 proxy signal).
#[derive(Debug, Default)]
pub struct DispatchStats {
    samples: Mutex<Vec<i64>>,
}

impl DispatchStats {
    fn record(&self, latency_ns: i64) {
        let mut s = self.samples.lock();
        if s.len() >= 256 {
            s.remove(0);
        }
        s.push(latency_ns);
    }

    #[must_use]
    pub fn percentiles(&self) -> (i64, i64) {
        let mut s = self.samples.lock().clone();
        if s.is_empty() {
            return (0, 0);
        }
        s.sort_unstable();
        (s[s.len() / 2], s[(s.len() * 95) / 100])
    }
}

/// Handle to the dispatch thread.
#[derive(Debug)]
pub struct VerbDispatch {
    tx: Sender<Work>,
    estop_flag: Arc<AtomicUsize>,
    priority_hold_flag: Arc<AtomicUsize>,
    pending_verbs: Arc<AtomicUsize>,
    playback: ExecutionSlot,
    playback_available: bool,
    pub stats: Arc<DispatchStats>,
    thread: Option<JoinHandle<()>>,
    shutdown: Arc<AtomicBool>,
}

impl VerbDispatch {
    pub fn spawn<C: Clock>(
        registry: ControlRegistry,
        clock: C,
        outcomes: Sender<VerbOutcome>,
    ) -> Self {
        let (tx, rx) = std::sync::mpsc::channel::<Work>();
        let estop_flag = Arc::new(AtomicUsize::new(0));
        let priority_hold_flag = Arc::new(AtomicUsize::new(0));
        let pending_verbs = Arc::new(AtomicUsize::new(0));
        let playback_available = registry.hold.is_some();
        let stats = Arc::new(DispatchStats::default());
        let shutdown = Arc::new(AtomicBool::new(false));

        let flag = estop_flag.clone();
        let hold_flag = priority_hold_flag.clone();
        let stats_thread = stats.clone();
        let shutdown_thread = shutdown.clone();
        let pending_thread = pending_verbs.clone();
        let thread = std::thread::Builder::new()
            .name("waddle-verbs".into())
            .spawn(move || {
                dispatch_loop(
                    &registry,
                    &clock,
                    &rx,
                    &outcomes,
                    &flag,
                    &hold_flag,
                    &stats_thread,
                    &shutdown_thread,
                    &pending_thread,
                );
            })
            .expect("spawn verb dispatch");

        Self {
            tx,
            estop_flag,
            priority_hold_flag,
            stats,
            thread: Some(thread),
            shutdown,
            pending_verbs,
            playback: ExecutionSlot::default(),
            playback_available,
        }
    }

    /// Queue a verb request (serialized, in order). Estop never queues: it
    /// rides the priority flag, checked before every dequeue.
    pub fn request(&self, req: VerbRequest) {
        self.playback.interrupt("control verb requested", || {
            if matches!(req, VerbRequest::Estop) {
                self.estop_flag.fetch_add(1, Ordering::SeqCst);
            } else {
                self.pending_verbs.fetch_add(1, Ordering::SeqCst);
                if self.tx.send(Work::Verb(req)).is_err() {
                    self.pending_verbs.fetch_sub(1, Ordering::SeqCst);
                }
            }
        });
    }

    /// Queue a local HOLD ahead of ordinary verb traffic. Unlike e-stop this
    /// does not revoke authority; it is the core-owned energized hold path.
    pub fn request_priority_hold(&self) {
        self.playback.interrupt("priority Hold requested", || {
            self.priority_hold_flag.fetch_add(1, Ordering::SeqCst);
        });
    }

    pub(crate) fn start_playback(
        &self,
        mirror: Arc<Mirror>,
        episode: EpisodeId,
        references: FiniteReferences,
        io: Arc<dyn PlaybackIo>,
        sample_period_ns: i64,
    ) -> Result<ExecutionHandle, PlaybackError> {
        if !self.playback_available {
            return Err(PlaybackError::Invalid(
                "requires a registered Hold callback",
            ));
        }
        let handle = self
            .playback
            .register(episode.clone(), references.len(), || {
                self.shutdown.load(Ordering::SeqCst)
                    || self.pending_verbs.load(Ordering::SeqCst) != 0
                    || self.estop_flag.load(Ordering::SeqCst) != 0
                    || self.priority_hold_flag.load(Ordering::SeqCst) != 0
            })?;
        let task = ExecutionTask::new(mirror, episode, references, io, &handle, sample_period_ns);
        self.tx
            .send(Work::Playback(Box::new(task)))
            .map_err(|_| PlaybackError::Cancelled("local execution worker stopped".into()))?;
        Ok(handle)
    }

    pub(crate) fn playback_active(&self, episode: &EpisodeId) -> bool {
        self.playback.active(episode)
    }

    pub(crate) fn drain_playback(&self, episode: &EpisodeId) {
        if let Some(handle) = self.playback.cancel_episode(episode) {
            handle.wait();
        }
    }

    pub fn shutdown(mut self) {
        self.stop();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }

    /// Signal shutdown without consuming (the thread joins on Drop). Used by
    /// the session's ordered teardown: stopping dispatch drops the outcome
    /// sender, which lets the outcome pump exit.
    pub fn stop(&self) {
        self.playback.interrupt("session shutdown", || {
            self.shutdown.store(true, Ordering::SeqCst);
        });
    }
}

impl Drop for VerbDispatch {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

#[allow(clippy::too_many_arguments)] // independent dispatch lanes and observability sinks
fn dispatch_loop<C: Clock>(
    registry: &ControlRegistry,
    clock: &C,
    rx: &Receiver<Work>,
    outcomes: &Sender<VerbOutcome>,
    estop_flag: &AtomicUsize,
    priority_hold_flag: &AtomicUsize,
    stats: &DispatchStats,
    shutdown: &AtomicBool,
    pending_verbs: &AtomicUsize,
) {
    let mut active: Option<Box<ExecutionTask>> = None;
    loop {
        if shutdown.load(Ordering::SeqCst) {
            if active.is_some() {
                let stop = if estop_flag.load(Ordering::SeqCst) != 0 {
                    VerbRequest::Estop
                } else {
                    VerbRequest::Hold
                };
                execute(registry, clock, &stop, outcomes, stats);
            }
            return;
        }
        // Estop preempts anything queued.
        let estops = estop_flag.load(Ordering::SeqCst);
        if estops != 0 {
            execute(registry, clock, &VerbRequest::Estop, outcomes, stats);
            finish_cancelled(&mut active);
            estop_flag.fetch_sub(estops, Ordering::SeqCst);
            continue;
        }
        // An energized HOLD preempts ordinary traffic but never e-stop.
        let holds = priority_hold_flag.load(Ordering::SeqCst);
        if holds != 0 {
            execute(registry, clock, &VerbRequest::Hold, outcomes, stats);
            finish_cancelled(&mut active);
            priority_hold_flag.fetch_sub(holds, Ordering::SeqCst);
            continue;
        }
        let wait = active.as_ref().map_or(Duration::from_millis(5), |task| {
            task.wait_duration(clock.stamp_now().mono_ns().0)
        });
        match rx.recv_timeout(wait) {
            Ok(Work::Verb(req)) => {
                execute(registry, clock, &req, outcomes, stats);
                finish_cancelled(&mut active);
                pending_verbs.fetch_sub(1, Ordering::SeqCst);
            }
            Ok(Work::Playback(task)) => {
                if let Some(result) = task.cancelled_result() {
                    task.finish(result);
                } else {
                    debug_assert!(active.is_none());
                    active = Some(task);
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                if let Some(task) = active.as_mut()
                    && let Some(result) = task.tick(clock)
                {
                    // A selected substitute stream retains its own normal gate
                    // semantics. Other failures stop this task through Hold;
                    // an already requested e-stop takes priority over that Hold.
                    // A queued control request owns cancellation and its stop
                    // semantics. Process it before marking this loop drained.
                    if task.cancelled_result().is_some()
                        && (pending_verbs.load(Ordering::SeqCst) != 0
                            || estop_flag.load(Ordering::SeqCst) != 0
                            || priority_hold_flag.load(Ordering::SeqCst) != 0)
                    {
                        continue;
                    }
                    if result
                        .as_ref()
                        .is_err_and(|e| !matches!(e, ExecutionFailure::Superseded(_)))
                        && !task.scope_ended()
                    {
                        let estops = estop_flag.load(Ordering::SeqCst);
                        if estops != 0 {
                            execute(registry, clock, &VerbRequest::Estop, outcomes, stats);
                            estop_flag.fetch_sub(estops, Ordering::SeqCst);
                        } else {
                            execute(registry, clock, &VerbRequest::Hold, outcomes, stats);
                        }
                    }
                    if task.finish(result) {
                        active = None;
                    }
                }
            }
            Err(RecvTimeoutError::Disconnected) => return,
        }
    }
}

fn finish_cancelled(active: &mut Option<Box<ExecutionTask>>) {
    if let Some(task) = active.take()
        && let Some(result) = task.terminal_failure().or_else(|| task.cancelled_result())
    {
        task.finish(result);
    }
    // Drop retains a cancellation even when the request raced admission.
}

fn execute<C: Clock>(
    registry: &ControlRegistry,
    clock: &C,
    req: &VerbRequest,
    outcomes: &Sender<VerbOutcome>,
    stats: &DispatchStats,
) {
    let verb = req.verb();
    let started = clock.stamp_now();

    let result = {
        let call = || -> Result<(), VerbError> {
            match req {
                VerbRequest::Send { chunk } => registry
                    .send
                    .as_ref()
                    .ok_or(VerbError::NotRegistered(Verb::Send))?
                    .send(chunk),
                VerbRequest::Hold => unit(&registry.hold, Verb::Hold),
                VerbRequest::Resume => unit(&registry.resume, Verb::Resume),
                VerbRequest::Home => unit(&registry.home, Verb::Home),
                VerbRequest::Estop => match &registry.estop {
                    Some((cb, _)) => cb.call(),
                    None => Err(VerbError::NotRegistered(Verb::Estop)),
                },
            }
        };
        std::panic::catch_unwind(AssertUnwindSafe(call)).unwrap_or(Err(VerbError::Panicked))
    };

    let finished = clock.stamp_now();
    let latency_ns = finished.mono_ns().0 - started.mono_ns().0;
    stats.record(latency_ns);
    let _ = outcomes.send(VerbOutcome {
        verb,
        result,
        latency_ns,
        at: finished.mono_ns(),
    });
}

fn unit(slot: &Option<Arc<dyn UnitVerb>>, verb: Verb) -> Result<(), VerbError> {
    slot.as_ref().ok_or(VerbError::NotRegistered(verb))?.call()
}

#[cfg(test)]
#[path = "playback/worker_tests.rs"]
mod playback_tests;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU32;
    use waddle_ingest::FakeClock;

    #[test]
    fn dispatch_serializes_times_and_survives_panics() {
        let calls = Arc::new(AtomicU32::new(0));
        let calls2 = calls.clone();
        let registry = ControlRegistry {
            hold: Some(Arc::new(move || {
                calls2.fetch_add(1, Ordering::SeqCst);
                Ok(())
            })),
            home: Some(Arc::new(|| -> Result<(), VerbError> {
                panic!("integrator bug")
            })),
            ..Default::default()
        };
        let (out_tx, out_rx) = std::sync::mpsc::channel();
        let dispatch = VerbDispatch::spawn(registry, FakeClock::default(), out_tx);

        dispatch.request(VerbRequest::Hold);
        dispatch.request(VerbRequest::Home);
        dispatch.request(VerbRequest::Resume); // not registered

        let o1 = out_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        assert_eq!(o1.verb, Verb::Hold);
        assert!(o1.result.is_ok());
        let o2 = out_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        assert_eq!(o2.result, Err(VerbError::Panicked));
        let o3 = out_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        assert_eq!(o3.result, Err(VerbError::NotRegistered(Verb::Resume)));
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        dispatch.shutdown();
    }

    #[test]
    fn estop_has_a_priority_path() {
        let order: Arc<Mutex<Vec<&'static str>>> = Arc::new(Mutex::new(Vec::new()));
        let (slow_order, estop_order) = (order.clone(), order.clone());
        let registry = ControlRegistry {
            hold: Some(Arc::new(move || {
                std::thread::sleep(Duration::from_millis(50));
                slow_order.lock().push("hold");
                Ok(())
            })),
            estop: Some((
                Arc::new(move || {
                    estop_order.lock().push("estop");
                    Ok(())
                }),
                EstopDecl::default(),
            )),
            ..Default::default()
        };
        let (out_tx, out_rx) = std::sync::mpsc::channel();
        let dispatch = VerbDispatch::spawn(registry, FakeClock::default(), out_tx);

        // Queue several holds, then estop: estop must run before the queued
        // holds that have not started yet.
        dispatch.request(VerbRequest::Hold);
        dispatch.request(VerbRequest::Hold);
        dispatch.request(VerbRequest::Hold);
        std::thread::sleep(Duration::from_millis(10)); // first hold starts
        dispatch.request(VerbRequest::Estop);

        for _ in 0..4 {
            let _ = out_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        }
        let seq = order.lock().clone();
        let estop_pos = seq.iter().position(|s| *s == "estop").unwrap();
        assert!(
            estop_pos <= 1,
            "estop must preempt queued verbs, got order {seq:?}"
        );
        dispatch.shutdown();
    }
    #[test]
    fn priority_hold_preempts_ordinary_traffic_but_not_estop() {
        let order: Arc<Mutex<Vec<&'static str>>> = Arc::new(Mutex::new(Vec::new()));
        let calls = Arc::new(AtomicU32::new(0));
        let (started_tx, started_rx) = std::sync::mpsc::channel();
        let (release_tx, release_rx) = std::sync::mpsc::channel();
        let release_rx = Arc::new(Mutex::new(release_rx));

        let hold_order = order.clone();
        let hold_calls = calls.clone();
        let hold_release = release_rx.clone();
        let home_order = order.clone();
        let estop_order = order.clone();
        let registry = ControlRegistry {
            hold: Some(Arc::new(move || {
                let nth = hold_calls.fetch_add(1, Ordering::SeqCst);
                if nth == 0 {
                    hold_order.lock().push("hold-queued");
                    let _ = started_tx.send(());
                    let _ = hold_release.lock().recv();
                } else {
                    hold_order.lock().push("hold-priority");
                }
                Ok(())
            })),
            home: Some(Arc::new(move || {
                home_order.lock().push("home");
                Ok(())
            })),
            estop: Some((
                Arc::new(move || {
                    estop_order.lock().push("estop");
                    Ok(())
                }),
                EstopDecl::default(),
            )),
            ..Default::default()
        };
        let (out_tx, out_rx) = std::sync::mpsc::channel();
        let dispatch = VerbDispatch::spawn(registry, FakeClock::default(), out_tx);

        dispatch.request(VerbRequest::Hold);
        started_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        dispatch.request(VerbRequest::Home);
        dispatch.request_priority_hold();
        dispatch.request(VerbRequest::Estop);
        let _ = release_tx.send(());

        for _ in 0..4 {
            let _ = out_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        }
        assert_eq!(
            *order.lock(),
            ["hold-queued", "estop", "hold-priority", "home"]
        );
        dispatch.shutdown();
    }
}
