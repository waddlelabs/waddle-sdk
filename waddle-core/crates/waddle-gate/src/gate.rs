//! The gate: `gate()` does three things in one synchronous call — stamps and
//! records the tick, consults claim state, and returns the action (or the
//! intervention substitute) tagged with its provenance.

use std::sync::Arc;

use arc_swap::ArcSwap;
use parking_lot::Mutex;
use waddle_types::action::{ActionValues, ObsValues};
use waddle_types::time::Clock;
use waddle_types::{MonoNs, ProvenanceTag};

use crate::blend::blend_step;
use crate::jitter::{JitterBuffer, TimedAction};
use crate::plan::{GatePlan, PlanMode};
use crate::record::{GateDecision, GateRecord};
use crate::stats::TickStats;

/// One executable action leaving the gate. Inline up to 16 dims: bimanual
/// plus grippers without a heap allocation.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnedAction {
    pub values: ActionValues,
    pub gripper: Option<f64>,
}

/// What the gate returned for one tick.
#[derive(Debug, Clone, PartialEq)]
pub enum GateOutput {
    /// Nominal: dispatch your own action.
    Pass { provenance: ProvenanceTag },
    /// An intervention action replaces yours; dispatch it.
    Substitute {
        action: OwnedAction,
        provenance: ProvenanceTag,
    },
    /// Cross-fade window of an IMMEDIATE handoff.
    Blend {
        action: OwnedAction,
        progress: f32,
        provenance: ProvenanceTag,
    },
    /// Bypass mode: the runtime pump is driving `send` directly. Do NOT
    /// dispatch — you are a spectator (claimed-while-stalled).
    Noop { provenance: ProvenanceTag },
    /// Hold position (HOLD_FIRST engage, tripwire hold, or no intervention
    /// action due yet).
    Hold,
}

/// The intervention-stream consumer: SPSC ring drained into the jitter
/// buffer. Owned behind a mutex so consumption can move between the caller
/// thread (claimed) and the runtime pump (bypass) without rewiring; the
/// mutex is never touched on the passthrough fast path.
#[derive(Debug)]
pub struct StreamIntake {
    rx: rtrb::Consumer<TimedAction>,
    jitter: JitterBuffer,
}

impl StreamIntake {
    /// Drain arrivals, then pop the action due at `now`.
    pub fn pop_due(&mut self, now: MonoNs) -> Option<OwnedAction> {
        while let Ok(ta) = self.rx.pop() {
            self.jitter.ingest(ta);
        }
        self.jitter.pop_due(now)
    }
}

/// State shared between the gate (caller thread), the FSM reducer (plan
/// writes), the runtime pump (bypass consumption), and the heartbeat
/// (tick stats).
#[derive(Debug)]
pub struct GateShared {
    plan: ArcSwap<GatePlan>,
    pub stats: TickStats,
    pub stream: Mutex<StreamIntake>,
}

impl GateShared {
    /// Build the shared state plus the media-intake producer end of the
    /// intervention stream.
    #[must_use]
    pub fn new(
        initial: GatePlan,
        stream_capacity: usize,
        playout_delay_ns: i64,
    ) -> (Arc<Self>, rtrb::Producer<TimedAction>) {
        let (tx, rx) = rtrb::RingBuffer::new(stream_capacity);
        (
            Arc::new(Self {
                plan: ArcSwap::from_pointee(initial),
                stats: TickStats::new(),
                stream: Mutex::new(StreamIntake {
                    rx,
                    jitter: JitterBuffer::new(playout_delay_ns),
                }),
            }),
            tx,
        )
    }

    /// Written only by the runtime's FSM reducer (`Effect::SetGateMode`).
    pub fn store_plan(&self, plan: GatePlan) {
        self.plan.store(Arc::new(plan));
    }

    #[must_use]
    pub fn load_plan(&self) -> Arc<GatePlan> {
        self.plan.load_full()
    }
}

/// The per-episode gate. Owned by the caller's thread; `gate()` is the only
/// core code that ever executes there.
#[derive(Debug)]
pub struct Gate<C: Clock> {
    shared: Arc<GateShared>,
    clock: C,
    records_tx: rtrb::Producer<GateRecord>,
    seq: u64,
    /// Blend anchor: the last action that left the gate.
    last_action: Option<OwnedAction>,
    records_dropped: u64,
}

impl<C: Clock> Gate<C> {
    /// Returns the gate and the consumer end of the record ring (drained by
    /// the runtime onto the sidecar).
    pub fn new(
        shared: Arc<GateShared>,
        clock: C,
        record_capacity: usize,
    ) -> (Self, rtrb::Consumer<GateRecord>) {
        let (records_tx, records_rx) = rtrb::RingBuffer::new(record_capacity);
        (
            Self {
                shared,
                clock,
                records_tx,
                seq: 0,
                last_action: None,
                records_dropped: 0,
            },
            records_rx,
        )
    }

    #[inline]
    fn record(
        &mut self,
        stamp: waddle_types::time::Stamp,
        decision: GateDecision,
        provenance: ProvenanceTag,
        action: Option<OwnedAction>,
        obs: Option<ObsValues>,
    ) {
        let rec = GateRecord {
            stamp,
            seq: self.seq,
            decision,
            provenance,
            action,
            obs,
        };
        // Never block the caller's loop: a full ring drops loudly.
        if self.records_tx.push(rec).is_err() {
            self.records_dropped += 1;
        }
    }

    /// The synchronous fast path. Passthrough: one plan load, one clock
    /// read, one ring push — wait-free, and allocation-free up to 16 action
    /// dims and 32 obs dims.
    pub fn gate(
        &mut self,
        values: &[f64],
        gripper: Option<f64>,
        obs: Option<&[f64]>,
    ) -> GateOutput {
        let stamp = self.clock.stamp_now();
        let now = stamp.mono_ns();
        self.shared.stats.record(now);
        self.seq += 1;

        // Recorded on every decision arm: the (obs, action) pair is the
        // contract, whatever the gate decides.
        let obs = obs.map(ObsValues::from_slice);

        let plan = self.shared.plan.load();
        match &plan.mode {
            PlanMode::Passthrough => {
                let action = OwnedAction {
                    values: ActionValues::from_slice(values),
                    gripper,
                };
                let provenance = ProvenanceTag::policy();
                self.record(
                    stamp,
                    GateDecision::Pass,
                    provenance.clone(),
                    Some(action.clone()),
                    obs,
                );
                self.last_action = Some(action);
                GateOutput::Pass { provenance }
            }
            PlanMode::Held => {
                self.record(
                    stamp,
                    GateDecision::Hold,
                    ProvenanceTag::policy(),
                    None,
                    obs,
                );
                GateOutput::Hold
            }
            PlanMode::Bypass { provenance } => {
                self.record(stamp, GateDecision::Noop, provenance.clone(), None, obs);
                GateOutput::Noop {
                    provenance: provenance.clone(),
                }
            }
            // D7 edge 3: a stale caller handle ticking during a remote reset
            // window must dispatch nothing. Same cost class as Bypass (one
            // record push, one marker return) — no locks/syscalls/allocs.
            PlanMode::Reset { provenance } => {
                self.record(
                    stamp,
                    GateDecision::ResetActive,
                    provenance.clone(),
                    None,
                    obs,
                );
                GateOutput::Noop {
                    provenance: provenance.clone(),
                }
            }
            PlanMode::Claimed { provenance, blend } => {
                let due = self.shared.stream.lock().pop_due(now);
                match due {
                    None => {
                        self.record(stamp, GateDecision::Hold, provenance.clone(), None, obs);
                        GateOutput::Hold
                    }
                    Some(target) => {
                        let progress = blend.as_ref().map_or(1.0, |b| b.progress(now));
                        if progress < 1.0 {
                            let schedule = blend.as_ref().expect("progress < 1 implies schedule");
                            let from = self.last_action.clone().unwrap_or_else(|| target.clone());
                            match blend_step(&from, &target, progress, schedule.interp) {
                                Some(blended) => {
                                    self.last_action = Some(blended.clone());
                                    self.record(
                                        stamp,
                                        GateDecision::Blend,
                                        provenance.clone(),
                                        Some(blended.clone()),
                                        obs,
                                    );
                                    GateOutput::Blend {
                                        action: blended,
                                        progress,
                                        provenance: provenance.clone(),
                                    }
                                }
                                None => {
                                    // Defense in depth: a dims mismatch here
                                    // means intake validation was bypassed.
                                    // Never truncate — hold instead.
                                    self.record(
                                        stamp,
                                        GateDecision::Hold,
                                        provenance.clone(),
                                        None,
                                        obs,
                                    );
                                    GateOutput::Hold
                                }
                            }
                        } else {
                            self.last_action = Some(target.clone());
                            self.record(
                                stamp,
                                GateDecision::Substitute,
                                provenance.clone(),
                                Some(target.clone()),
                                obs,
                            );
                            GateOutput::Substitute {
                                action: target,
                                provenance: provenance.clone(),
                            }
                        }
                    }
                }
            }
        }
    }

    #[must_use]
    pub fn records_dropped(&self) -> u64 {
        self.records_dropped
    }

    #[must_use]
    pub fn shared(&self) -> &Arc<GateShared> {
        &self.shared
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::BlendSchedule;
    use smallvec::smallvec;
    use waddle_ingest::FakeClock;
    use waddle_types::{Interp, Provenance};

    fn teleop_tag() -> ProvenanceTag {
        ProvenanceTag {
            provenance: Provenance::Teleop,
            actor: None,
            bypass_approval: false,
        }
    }

    fn setup() -> (
        Gate<FakeClock>,
        Arc<GateShared>,
        rtrb::Producer<TimedAction>,
        rtrb::Consumer<GateRecord>,
        FakeClock,
    ) {
        let clock = FakeClock::default();
        let (shared, tx) = GateShared::new(GatePlan::passthrough(MonoNs(0)), 64, 0);
        let (gate, records) = Gate::new(shared.clone(), clock.clone(), 256);
        (gate, shared, tx, records, clock)
    }

    #[test]
    fn passthrough_passes_and_records() {
        let (mut gate, _shared, _tx, mut records, clock) = setup();
        clock.advance(1_000);
        let out = gate.gate(&[1.0, 2.0], Some(0.5), None);
        assert!(matches!(out, GateOutput::Pass { .. }));
        let rec = records.pop().unwrap();
        assert_eq!(rec.decision, GateDecision::Pass);
        assert_eq!(rec.action.unwrap().values.as_slice(), &[1.0, 2.0]);
    }

    #[test]
    fn records_carry_the_obs_on_every_decision_arm() {
        let (mut gate, shared, _tx, mut records, clock) = setup();
        let obs = [0.5f64; 30];

        clock.advance(1_000);
        gate.gate(&[1.0, 2.0], None, Some(&obs));
        let rec = records.pop().unwrap();
        assert_eq!(rec.decision, GateDecision::Pass);
        assert_eq!(rec.obs.unwrap().as_slice(), &obs);

        // Hold (claimed, stream empty) records the obs too: it is still a
        // tick the caller observed.
        shared.store_plan(GatePlan {
            mode: PlanMode::Claimed {
                provenance: teleop_tag(),
                blend: None,
            },
            since: MonoNs(0),
        });
        clock.advance(1_000);
        gate.gate(&[1.0, 2.0], None, Some(&obs));
        let rec = records.pop().unwrap();
        assert_eq!(rec.decision, GateDecision::Hold);
        assert_eq!(rec.obs.unwrap().as_slice(), &obs);
    }

    #[test]
    fn claimed_substitutes_from_the_stream_and_holds_when_empty() {
        let (mut gate, shared, mut tx, _records, clock) = setup();
        shared.store_plan(GatePlan {
            mode: PlanMode::Claimed {
                provenance: teleop_tag(),
                blend: None,
            },
            since: MonoNs(0),
        });
        clock.advance(1_000);
        assert!(matches!(gate.gate(&[0.0], None, None), GateOutput::Hold));

        tx.push(TimedAction {
            channel: crate::jitter::StreamChannel::Teleop,
            seq: 1,
            received: MonoNs(1_000),
            action: OwnedAction {
                values: smallvec![9.0],
                gripper: None,
            },
        })
        .unwrap();
        clock.advance(1_000);
        match gate.gate(&[0.0], None, None) {
            GateOutput::Substitute { action, provenance } => {
                assert_eq!(action.values.as_slice(), &[9.0]);
                assert_eq!(provenance.provenance, Provenance::Teleop);
            }
            other => panic!("expected substitute, got {other:?}"),
        }
    }

    #[test]
    fn blend_window_progresses_from_last_pass_to_stream() {
        let (mut gate, shared, mut tx, _records, clock) = setup();
        // Establish a blend anchor at 0.0.
        clock.advance(1_000);
        gate.gate(&[0.0], None, None);

        shared.store_plan(GatePlan {
            mode: PlanMode::Claimed {
                provenance: teleop_tag(),
                blend: Some(BlendSchedule {
                    start: MonoNs(1_000),
                    blend_ns: 1_000,
                    interp: Interp::Linear,
                }),
            },
            since: MonoNs(1_000),
        });
        tx.push(TimedAction {
            channel: crate::jitter::StreamChannel::Teleop,
            seq: 1,
            received: MonoNs(1_000),
            action: OwnedAction {
                values: smallvec![10.0],
                gripper: None,
            },
        })
        .unwrap();
        clock.set(MonoNs(1_500)); // halfway through the window
        match gate.gate(&[0.0], None, None) {
            GateOutput::Blend {
                action, progress, ..
            } => {
                assert!((f64::from(progress) - 0.5).abs() < 1e-6);
                assert!((action.values[0] - 5.0).abs() < 1e-6);
            }
            other => panic!("expected blend, got {other:?}"),
        }
    }

    #[test]
    fn bypass_returns_noops() {
        let (mut gate, shared, _tx, mut records, clock) = setup();
        shared.store_plan(GatePlan {
            mode: PlanMode::Bypass {
                provenance: teleop_tag(),
            },
            since: MonoNs(0),
        });
        clock.advance(1_000);
        assert!(matches!(
            gate.gate(&[1.0], None, None),
            GateOutput::Noop { .. }
        ));
        assert_eq!(records.pop().unwrap().decision, GateDecision::Noop);
    }

    /// D7 edge 3: a caller ticking `gate()` on a stale handle while a remote
    /// actor holds the reset window dispatches nothing — same shape as
    /// bypass, distinct decision so the reducer can render
    /// `NoopReason::RESET_ACTIVE` instead of `BYPASS_ACTIVE`.
    #[test]
    fn reset_active_returns_noop_and_records_distinctly() {
        let (mut gate, shared, _tx, mut records, clock) = setup();
        shared.store_plan(GatePlan {
            mode: PlanMode::Reset {
                provenance: teleop_tag(),
            },
            since: MonoNs(0),
        });
        clock.advance(1_000);
        assert!(matches!(
            gate.gate(&[1.0], None, None),
            GateOutput::Noop { .. }
        ));
        let rec = records.pop().unwrap();
        assert_eq!(rec.decision, GateDecision::ResetActive);
        assert_ne!(
            rec.decision,
            GateDecision::Noop,
            "reset-active must be distinguishable from bypass for the reducer's marker mapping"
        );
    }
}
