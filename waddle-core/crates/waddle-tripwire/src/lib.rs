//! waddle-tripwire — Waddle-side local watchdogs.
//!
//! Vocabulary (normative, GLOSSARY.md): a **tripwire** is Waddle-side and
//! REQUESTS safety actions through the integrator's declared verbs. It is
//! weaker than an **envelope** by definition — the envelope is the hard,
//! non-bypassable gate chain owned by whoever owns the hardware. Waddle
//! never claims to provide an envelope.
//!
//! The evaluator is pure ([`Evaluator::evaluate`]); the thread harnesses
//! ([`spawn_evaluator`], [`spawn_heartbeat_watchdog`]) run it on dedicated
//! OS threads whose lifecycle the runtime owns. Tripwires fail safe locally:
//! they keep running through a control-plane partition.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::thread::JoinHandle;
use std::time::Duration;

use smallvec::SmallVec;
use waddle_types::time::Clock;
use waddle_types::{MonoNs, Verb};

/// A proprioceptive snapshot the evaluator reads. Produced by the runtime
/// from the ingest hub's latest-value slots.
#[derive(Debug, Clone, Default)]
pub struct ObsSnapshot {
    pub at: MonoNs,
    pub joint_pos: SmallVec<[f64; 16]>,
    /// End-effector position in the base frame, when known.
    pub ee_pos: Option<[f64; 3]>,
    /// Magnitude of the measured external force (N), when known.
    pub force_n: Option<f64>,
}

/// Where the evaluator gets its snapshots.
pub trait ObsSource: Send + Sync + 'static {
    fn latest(&self) -> Option<ObsSnapshot>;
}

/// Where tripwire fires go: the runtime maps them to verb-invocation
/// requests and `TripwireEvent` emissions. Tripwires never act directly.
pub trait TripwireSink: Send + Sync + 'static {
    fn request(&self, fire: TripwireFire);
}

#[derive(Debug, Clone, PartialEq)]
pub struct TripwireFire {
    pub name: String,
    pub requested_verb: Verb,
    pub detail: String,
    pub at: MonoNs,
}

/// An axis-aligned workspace box in the base frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    #[must_use]
    pub fn contains(&self, p: [f64; 3]) -> bool {
        (0..3).all(|i| p[i] >= self.min[i] && p[i] <= self.max[i])
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum TripwireKind {
    /// EE position must stay inside the box.
    WorkspaceAabb(Aabb),
    /// Any joint within `margin_rad` of its declared limit fires.
    JointLimitMargin {
        margin_rad: f64,
        limits: Vec<(f64, f64)>,
    },
    /// External force magnitude above `max_n` fires.
    ForceThreshold { max_n: f64 },
    /// The snapshot itself is stale: the observation path died.
    Staleness { max_age_ns: i64 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Tripwire {
    pub name: String,
    pub kind: TripwireKind,
    /// The verb this tripwire requests when it fires (HOLD unless declared
    /// otherwise).
    pub requested_verb: Verb,
}

impl Tripwire {
    #[must_use]
    pub fn holds(name: impl Into<String>, kind: TripwireKind) -> Self {
        Self {
            name: name.into(),
            kind,
            requested_verb: Verb::Hold,
        }
    }
}

/// Pure evaluation — unit-testable without threads.
#[derive(Debug, Clone, Default)]
pub struct Evaluator {
    pub wires: Vec<Tripwire>,
}

impl Evaluator {
    #[must_use]
    pub fn new(wires: Vec<Tripwire>) -> Self {
        Self { wires }
    }

    #[must_use]
    pub fn evaluate(&self, snap: &ObsSnapshot, now: MonoNs) -> SmallVec<[TripwireFire; 2]> {
        let mut fires = SmallVec::new();
        for wire in &self.wires {
            let detail = match &wire.kind {
                TripwireKind::WorkspaceAabb(aabb) => match snap.ee_pos {
                    Some(p) if !aabb.contains(p) => {
                        Some(format!("ee at {p:?} outside workspace box"))
                    }
                    _ => None,
                },
                TripwireKind::JointLimitMargin { margin_rad, limits } => snap
                    .joint_pos
                    .iter()
                    .zip(limits)
                    .enumerate()
                    .find(|(_, (q, (lo, hi)))| **q < lo + margin_rad || **q > hi - margin_rad)
                    .map(|(i, (q, _))| format!("joint {i} at {q:.4} rad within limit margin")),
                TripwireKind::ForceThreshold { max_n } => match snap.force_n {
                    Some(f) if f > *max_n => Some(format!("force {f:.1} N over {max_n:.1} N")),
                    _ => None,
                },
                TripwireKind::Staleness { max_age_ns } => {
                    let age = now.0 - snap.at.0;
                    (age > *max_age_ns).then(|| format!("observation stale by {age} ns"))
                }
            };
            if let Some(detail) = detail {
                fires.push(TripwireFire {
                    name: wire.name.clone(),
                    requested_verb: wire.requested_verb,
                    detail,
                    at: now,
                });
            }
        }
        fires
    }
}

/// Cooperative shutdown for the thread harnesses.
#[derive(Debug, Clone, Default)]
pub struct ShutdownToken(Arc<AtomicBool>);

impl ShutdownToken {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn shutdown(&self) {
        self.0.store(true, Ordering::SeqCst);
    }

    #[must_use]
    pub fn is_shutdown(&self) -> bool {
        self.0.load(Ordering::SeqCst)
    }
}

/// Run the evaluator on a dedicated OS thread. Fires are edge-triggered per
/// wire (a wire re-arms once it stops firing) so a persistent condition
/// requests once, not at poll rate.
pub fn spawn_evaluator<C: Clock>(
    evaluator: Evaluator,
    clock: C,
    source: Arc<dyn ObsSource>,
    sink: Arc<dyn TripwireSink>,
    period: Duration,
    shutdown: ShutdownToken,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-tripwire".into())
        .spawn(move || {
            let mut firing: Vec<bool> = vec![false; evaluator.wires.len()];
            while !shutdown.is_shutdown() {
                if let Some(snap) = source.latest() {
                    let now = clock.stamp_now().mono_ns();
                    let fires = evaluator.evaluate(&snap, now);
                    for (i, wire) in evaluator.wires.iter().enumerate() {
                        let hit = fires.iter().find(|f| f.name == wire.name);
                        match (hit, firing[i]) {
                            (Some(f), false) => {
                                firing[i] = true;
                                sink.request(f.clone());
                            }
                            (None, true) => firing[i] = false,
                            _ => {}
                        }
                    }
                }
                std::thread::sleep(period);
            }
        })
        .expect("spawn tripwire evaluator")
}

/// The control-plane heartbeat watchdog: fires (once per outage) when the
/// last heartbeat is older than `timeout_ns`. `last_heartbeat_ns` is stamped
/// by the control-plane client on every ack.
pub fn spawn_heartbeat_watchdog<C: Clock>(
    clock: C,
    last_heartbeat_ns: Arc<AtomicI64>,
    timeout_ns: i64,
    sink: Arc<dyn TripwireSink>,
    period: Duration,
    shutdown: ShutdownToken,
) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name("waddle-heartbeat-watchdog".into())
        .spawn(move || {
            let mut fired = false;
            while !shutdown.is_shutdown() {
                let now = clock.stamp_now().mono_ns();
                let last = last_heartbeat_ns.load(Ordering::SeqCst);
                let stale = last != 0 && now.0 - last > timeout_ns;
                if stale && !fired {
                    fired = true;
                    sink.request(TripwireFire {
                        name: "control-plane-heartbeat".into(),
                        requested_verb: Verb::Hold,
                        detail: format!("no heartbeat for {} ns", now.0 - last),
                        at: now,
                    });
                } else if !stale {
                    fired = false;
                }
                std::thread::sleep(period);
            }
        })
        .expect("spawn heartbeat watchdog")
}

#[cfg(test)]
mod tests {
    use super::*;
    use parking_lot::Mutex;
    use smallvec::smallvec;

    #[test]
    fn workspace_and_force_wires_fire_with_detail() {
        let ev = Evaluator::new(vec![
            Tripwire::holds(
                "workspace",
                TripwireKind::WorkspaceAabb(Aabb {
                    min: [-1.0, -1.0, 0.0],
                    max: [1.0, 1.0, 1.0],
                }),
            ),
            Tripwire::holds("force", TripwireKind::ForceThreshold { max_n: 40.0 }),
        ]);
        let snap = ObsSnapshot {
            at: MonoNs(0),
            joint_pos: smallvec![],
            ee_pos: Some([2.0, 0.0, 0.5]),
            force_n: Some(55.0),
        };
        let fires = ev.evaluate(&snap, MonoNs(1));
        assert_eq!(fires.len(), 2);
        assert!(fires.iter().all(|f| f.requested_verb == Verb::Hold));
    }

    #[test]
    fn joint_margin_and_staleness() {
        let ev = Evaluator::new(vec![
            Tripwire::holds(
                "margin",
                TripwireKind::JointLimitMargin {
                    margin_rad: 0.05,
                    limits: vec![(-3.0, 3.0), (-3.0, 3.0)],
                },
            ),
            Tripwire::holds("stale", TripwireKind::Staleness { max_age_ns: 1_000 }),
        ]);
        let ok = ObsSnapshot {
            at: MonoNs(500),
            joint_pos: smallvec![0.0, 2.96],
            ee_pos: None,
            force_n: None,
        };
        let fires = ev.evaluate(&ok, MonoNs(600));
        assert_eq!(fires.len(), 1); // margin only
        let fires = ev.evaluate(&ok, MonoNs(2_000));
        assert_eq!(fires.len(), 2); // margin + staleness
    }

    struct VecSink(Mutex<Vec<TripwireFire>>);
    impl TripwireSink for VecSink {
        fn request(&self, fire: TripwireFire) {
            self.0.lock().push(fire);
        }
    }
    struct StaticSource(ObsSnapshot);
    impl ObsSource for StaticSource {
        fn latest(&self) -> Option<ObsSnapshot> {
            Some(self.0.clone())
        }
    }

    #[test]
    fn evaluator_thread_is_edge_triggered_and_shuts_down() {
        let ev = Evaluator::new(vec![Tripwire::holds(
            "force",
            TripwireKind::ForceThreshold { max_n: 1.0 },
        )]);
        let sink = Arc::new(VecSink(Mutex::new(Vec::new())));
        let source = Arc::new(StaticSource(ObsSnapshot {
            at: MonoNs(0),
            joint_pos: smallvec![],
            ee_pos: None,
            force_n: Some(10.0), // persistently over
        }));
        let shutdown = ShutdownToken::new();
        let handle = spawn_evaluator(
            ev,
            waddle_ingest::FakeClock::default(),
            source,
            sink.clone(),
            Duration::from_millis(1),
            shutdown.clone(),
        );
        std::thread::sleep(Duration::from_millis(30));
        shutdown.shutdown();
        handle.join().unwrap();
        // Edge-triggered: one request despite ~30 polls.
        assert_eq!(sink.0.lock().len(), 1);
    }
}
