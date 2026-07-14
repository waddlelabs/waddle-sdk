//! The five-verb dispatch: integrator callables invoked ONLY on one
//! dedicated core-owned thread, serialized (a guarantee the callable can
//! rely on), wrapped in `catch_unwind` (a panicking callback becomes a
//! `VerbError`, never a poisoned core), and timed (feeding the N11
//! callback-dispatch proxy signal). Estop bypasses the queue via a priority
//! slot checked before every dequeue.

use std::panic::AssertUnwindSafe;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use parking_lot::Mutex;
use waddle_types::time::Clock;
use waddle_types::{ActionChunk, Verb, VerbRequest};

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
    tx: Sender<VerbRequest>,
    estop_flag: Arc<AtomicBool>,
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
        let (tx, rx) = std::sync::mpsc::channel::<VerbRequest>();
        let estop_flag = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(DispatchStats::default());
        let shutdown = Arc::new(AtomicBool::new(false));

        let flag = estop_flag.clone();
        let stats_thread = stats.clone();
        let shutdown_thread = shutdown.clone();
        let thread = std::thread::Builder::new()
            .name("waddle-verbs".into())
            .spawn(move || {
                dispatch_loop(
                    &registry,
                    &clock,
                    &rx,
                    &outcomes,
                    &flag,
                    &stats_thread,
                    &shutdown_thread,
                );
            })
            .expect("spawn verb dispatch");

        Self {
            tx,
            estop_flag,
            stats,
            thread: Some(thread),
            shutdown,
        }
    }

    /// Queue a verb request (serialized, in order). Estop never queues: it
    /// rides the priority flag, checked before every dequeue.
    pub fn request(&self, req: VerbRequest) {
        if matches!(req, VerbRequest::Estop) {
            self.estop_flag.store(true, Ordering::SeqCst);
            return;
        }
        let _ = self.tx.send(req);
    }

    pub fn shutdown(mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
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

fn dispatch_loop<C: Clock>(
    registry: &ControlRegistry,
    clock: &C,
    rx: &Receiver<VerbRequest>,
    outcomes: &Sender<VerbOutcome>,
    estop_flag: &AtomicBool,
    stats: &DispatchStats,
    shutdown: &AtomicBool,
) {
    loop {
        if shutdown.load(Ordering::SeqCst) {
            return;
        }
        // Estop preempts anything queued.
        if estop_flag.swap(false, Ordering::SeqCst) {
            execute(registry, clock, &VerbRequest::Estop, outcomes, stats);
            continue;
        }
        match rx.recv_timeout(Duration::from_millis(5)) {
            Ok(req) => execute(registry, clock, &req, outcomes, stats),
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return,
        }
    }
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
}
