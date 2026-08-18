//! The teleop jitter buffer: reorders a lossy latest-wins action stream and
//! plays out "the action due now" after a fixed playout delay.
//!
//! Pure and deterministic: time is an argument. The media intake thread
//! calls [`JitterBuffer::ingest`]; the consumer calls
//! [`JitterBuffer::pop_due`] each tick.
//!
//! Three producers write into the ONE ring `waddle-runtime` builds per
//! session (`StreamProducer`, Mutex-shared):
//!
//! * the media-intake thread — teleop packets, wire-ordered by
//!   `TeleopStreamPacket.seq`, genuinely unordered over the network;
//! * `forward_server_msg`'s `InterventionChunk` arm — agent chunks off the
//!   plane, both the Reset-mode window actuation and the general Claimed-mode
//!   intake;
//! * `session::push_intervention_chunk` — the local seam that injects a chunk
//!   with no plane behind it (the SDK's testing hooks, and any embedder
//!   driving an intervention directly).
//!
//! Each [`TimedAction`] carries a [`StreamChannel`] tag, and this buffer
//! keeps a SEPARATE reorder/late-drop cursor per CHANNEL — never one shared
//! high-water mark — so the teleop stream's activity (e.g. an ordinary teleop
//! claim earlier in the session) can never starve or permanently drop an
//! agent chunk's arrivals (e.g. the first chunk of a later claim).
//!
//! Note what that does and does not buy: the cursor is per channel, not per
//! producer, and the last two producers above share `AgentChunk`. So they
//! must also share ONE seq counter, which is why `waddle-runtime` keeps a
//! single `ChunkIntakeState` per session rather than one per intake — two
//! counters would have the second producer's seq 1 land at or behind a cursor
//! the first had already advanced, and the arrival would be dropped as late
//! and in silence. One stamping authority per channel; see waddle-runtime's
//! `pumps.rs` for the producer side.
//!
//! Per-channel cursors alone are not enough: an arrival that is pushed but
//! never popped before its claim/window ends (still within its own playout
//! delay, say) is left sitting in that channel's `pending` map with nothing
//! left to drain it -- the caller stopped ticking `Claimed`, and the bypass
//! pump only polls while `Bypass`/`Reset` is active. It resurfaces the next
//! time ANYTHING pops that channel, which may be a much later and entirely
//! unrelated claim/window, dispatched under THAT claimant's mirror
//! provenance. [`JitterBuffer::clear_pending`] is the fix: the reducer calls
//! it on every transition back to `GateMode::Passthrough` (claim released,
//! or a reset window closed) so a dead claim/window's leftovers can never
//! outlive it.
//!
//! ## Chunk horizon (the `AgentChunk` channel only)
//!
//! Unlike a teleop packet (one action per wire message), an agent's
//! `ActionChunk` carries a HORIZON of steps computed together from one
//! observation. A later chunk can arrive before the earlier one's horizon
//! has finished playing out — `ChunkingSemantics.replan`
//! (`descriptors.proto`) declares how that supersede is handled:
//! `REPLAN_POLICY_IMMEDIATE`/`REPLAN_POLICY_BLEND` drop the executing
//! chunk's still-pending steps and switch to the new one;
//! `REPLAN_POLICY_CHUNK_BOUNDARY` lets them finish first (the new chunk's
//! steps simply queue behind — no special handling needed, since per-item
//! `seq` already keeps them FIFO-ordered). `BLEND`'s normative comment
//! itself steers implementations away from it ("prefer IMMEDIATE + clamp");
//! `ChunkingSemantics` declares no blend duration/curve for a chunk-to-chunk
//! splice (unlike `HandoffPolicy::Immediate.blend_ns`), so this buffer maps
//! it onto the same replace-remaining behavior as IMMEDIATE rather than
//! inventing an underspecified cross-fade — flagged in the task report.
//!
//! Each `AgentChunk` [`TimedAction`] carries the [`ChunkMeta`] of the wire
//! chunk it came from. `ingest` detects a chunk BOUNDARY (the first step of
//! a chunk different from the channel's currently-tracked
//! [`ChannelState::active_chunk`]) and, at that instant only: rejects the
//! whole chunk as stale (`dropped_stale_chunks`) if it is not strictly newer
//! by `chunk_seq` — the one field `control.proto` normatively requires to be
//! monotone per stream — additionally rejecting on `t_emitted_ns` only when
//! BOTH chunks declare a nonzero value and the new one is not strictly newer
//! (a wire-legal producer that leaves `t_emitted_ns` at the proto3 default 0,
//! or ties it, is never penalized for a field the protocol never requires to
//! be set or increasing); else applies the declared replan policy. An EMPTY
//! chunk (zero steps) never reaches `ingest` at all (the producer has
//! nothing to push), so it is a true no-op by construction — it can neither
//! supersede nor be rejected.

use std::collections::BTreeMap;

use waddle_types::{MonoNs, ReplanPolicy};

use crate::gate::OwnedAction;

/// Which stream supplied an arrival — see the module doc for why this can't
/// be inferred from `seq` alone (the two channels' seq spaces are independent
/// and may overlap).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StreamChannel {
    /// `spawn_media_intake`'s teleop pose stream.
    Teleop,
    /// Intervention chunks: `forward_server_msg`'s `InterventionChunk` arm
    /// (Reset-mode window actuation or general Claimed-mode intake) and the
    /// local `push_intervention_chunk` seam, which share one seq counter
    /// because they share this cursor.
    AgentChunk,
}

/// Identifies which wire `ActionChunk` an `AgentChunk` arrival came from:
/// `seq` is `ActionChunk.seq` ("monotone per stream" per the proto comment —
/// the only field the protocol normatively requires for ordering, and hence
/// the PRIMARY staleness signal), `t_emitted_ns` is `ActionChunk.t_emitted_ns`
/// (an ADDITIONAL, defense-in-depth signal, consulted only when a producer
/// bothers to set it on both the executing and the candidate chunk — proto3
/// cannot distinguish "unset" from 0, so a lone/zero value is never treated
/// as evidence of staleness). Used ONLY to detect a chunk boundary and decide
/// stale-vs-supersede (see the module doc) — never for playout scheduling
/// (that stays session-receive-time + `t_offset_ns`, matching the
/// Reset-mode arm: `ActionChunk`'s `_ns` fields are already session-timeline
/// per `VERSIONING.md` §7, not `_client_ns`, but there is no guarantee a
/// remote agent's own chunk-seq numbering is claim-scoped or session-scoped,
/// so trusting it as an absolute playout anchor is not assumed here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkMeta {
    pub chunk_seq: u64,
    pub t_emitted_ns: i64,
}

#[derive(Debug, Clone)]
pub struct TimedAction {
    pub channel: StreamChannel,
    pub seq: u64,
    pub received: MonoNs,
    pub action: OwnedAction,
    /// `Some` for `AgentChunk` arrivals (which chunk this step came from);
    /// always `None` for `Teleop` (no chunk horizon on that channel).
    pub chunk: Option<ChunkMeta>,
}

/// One channel's own reorder window and playout cursor.
#[derive(Debug, Default)]
struct ChannelState {
    pending: BTreeMap<u64, TimedAction>,
    last_popped_seq: Option<u64>,
    /// The chunk currently considered "executing" on this channel (see the
    /// module doc's chunk-horizon section). Always `None` for `Teleop`.
    /// Reset to `None` by `clear_pending` — scoped to the claim/window that
    /// is executing it, not a session-lifetime property like
    /// `last_popped_seq`: a brand-new claim's first chunk must never be
    /// wrongly rejected as "stale" against a previous, unrelated claim's
    /// last chunk just because the wire's chunk-seq numbering happens to be
    /// claim-scoped on the sender's side.
    active_chunk: Option<ChunkMeta>,
}

#[derive(Debug)]
pub struct JitterBuffer {
    playout_delay_ns: i64,
    /// `ChunkingSemantics.replan` for the `AgentChunk` channel — a session
    /// (declared action-space) constant, never per-message. Unused for
    /// `Teleop` (which carries no chunk metadata to replan against).
    agent_chunk_replan: ReplanPolicy,
    teleop: ChannelState,
    agent_chunk: ChannelState,
    dropped_late: u64,
    dropped_stale_chunks: u64,
}

impl JitterBuffer {
    #[must_use]
    pub fn new(playout_delay_ns: i64, agent_chunk_replan: ReplanPolicy) -> Self {
        Self {
            playout_delay_ns: playout_delay_ns.max(0),
            agent_chunk_replan,
            teleop: ChannelState::default(),
            agent_chunk: ChannelState::default(),
            dropped_late: 0,
            dropped_stale_chunks: 0,
        }
    }

    fn state_mut(&mut self, channel: StreamChannel) -> &mut ChannelState {
        match channel {
            StreamChannel::Teleop => &mut self.teleop,
            StreamChannel::AgentChunk => &mut self.agent_chunk,
        }
    }

    /// Ingest an arrival. Actions at-or-before their OWN channel's playout
    /// cursor are late — dropped and counted, never reordered backwards (a
    /// late pose is a wrong pose) — but a channel's cursor never looks at
    /// another channel's arrivals.
    ///
    /// For an `AgentChunk` arrival carrying `chunk: Some(meta)` that differs
    /// from the channel's current `active_chunk` (a chunk boundary — see the
    /// module doc), this first decides stale-vs-supersede BEFORE the
    /// late-check below ever runs (a superseded chunk's steps are dropped by
    /// the replan policy, not by the late-drop cursor).
    pub fn ingest(&mut self, action: TimedAction) {
        if let Some(meta) = action.chunk {
            enum Transition {
                /// Same chunk as already executing: no boundary logic.
                Same,
                /// Not strictly newer than the executing chunk: reject the
                /// whole chunk (this and every other step of it, since
                /// `active_chunk` never advances on a stale arrival).
                Stale,
                /// The first chunk ever, or a genuinely newer one: apply the
                /// declared replan policy.
                Replan,
            }
            let transition = {
                let state = self.state_mut(action.channel);
                match state.active_chunk {
                    Some(active) if active == meta => Transition::Same,
                    // `seq` is the ONLY field `control.proto` normatively
                    // requires for ordering ("Monotone per stream; gaps are
                    // visible, reordering is detectable" — nothing says
                    // `t_emitted_ns` must be set or strictly increasing, and
                    // proto3 can't distinguish "unset" from 0). So `seq` is
                    // the primary and sufficient staleness signal; a NOT
                    // strictly newer seq is stale regardless of
                    // `t_emitted_ns`. `t_emitted_ns` is consulted as an
                    // ADDITIONAL rejection only when BOTH chunks declare a
                    // nonzero value (so a producer that legally leaves the
                    // default 0 on every chunk, or ties it, is never
                    // penalized for a field the protocol doesn't require) —
                    // this still catches a genuinely newer `seq` whose
                    // declared emission time regressed, which would
                    // otherwise indicate clock/reordering trouble upstream.
                    Some(active)
                        if meta.chunk_seq <= active.chunk_seq
                            || (meta.t_emitted_ns != 0
                                && active.t_emitted_ns != 0
                                && meta.t_emitted_ns <= active.t_emitted_ns) =>
                    {
                        Transition::Stale
                    }
                    _ => Transition::Replan,
                }
            };
            match transition {
                Transition::Same => {}
                Transition::Stale => {
                    self.dropped_stale_chunks += 1;
                    return;
                }
                Transition::Replan => {
                    let replan = self.agent_chunk_replan;
                    let state = self.state_mut(action.channel);
                    if matches!(replan, ReplanPolicy::Immediate | ReplanPolicy::Blend) {
                        // Replace-remaining: the executing chunk's
                        // still-pending steps never play out.
                        state.pending.clear();
                    }
                    // ChunkBoundary: leave `pending` untouched — the new
                    // chunk's steps get higher per-item `seq` values (see
                    // `pumps.rs`'s `next_chunk_seq`), so they naturally queue
                    // FIFO-behind the executing chunk's remaining steps.
                    state.active_chunk = Some(meta);
                }
            }
        }

        let state = self.state_mut(action.channel);
        if let Some(last) = state.last_popped_seq
            && action.seq <= last
        {
            self.dropped_late += 1;
            return;
        }
        state.pending.insert(action.seq, action);
    }

    /// Pop the next in-order action whose playout delay has elapsed, across
    /// both channels. In practice only one channel is ever populated at a
    /// time (one engaged claimant drives either teleop or agent-chunk
    /// actions, never both) but this makes no assumption of that: each
    /// channel's own head is checked independently, and whichever is ready
    /// and arrived first wins ties.
    pub fn pop_due(&mut self, now: MonoNs) -> Option<OwnedAction> {
        let playout_delay_ns = self.playout_delay_ns;
        let mut best: Option<(StreamChannel, u64, MonoNs)> = None;
        for channel in [StreamChannel::Teleop, StreamChannel::AgentChunk] {
            let state = self.state_mut(channel);
            if let Some((&seq, head)) = state.pending.iter().next()
                && head.received.0 + playout_delay_ns <= now.0
                && best.is_none_or(|(_, _, received)| head.received.0 < received.0)
            {
                best = Some((channel, seq, head.received));
            }
        }
        let (channel, seq, _) = best?;
        let state = self.state_mut(channel);
        let action = state.pending.remove(&seq).expect("first key exists");
        state.last_popped_seq = Some(seq);
        Some(action.action)
    }

    /// Discard every not-yet-due arrival on EVERY channel, leaving each
    /// channel's own late-drop cursor (`last_popped_seq`) untouched.
    ///
    /// Called once per claim/window teardown (the reducer's transition back
    /// to `GateMode::Passthrough` — see `waddle-runtime`'s `reducer.rs`):
    /// whatever is still `pending` at that instant was pushed under a claim
    /// or reset window that has just ended, and nothing pops the ring again
    /// until some LATER, unrelated claim/window starts polling it. Without
    /// this, those leftovers sit in the buffer indefinitely and eventually
    /// get popped and dispatched under a completely different, later
    /// claimant's mirror provenance — the exact defect
    /// `waddle-runtime`'s `remote_post_reset_window_agent_chunk_survives_prior_teleop_claim_activity`
    /// test guards against. The cursor is deliberately left alone: it is
    /// the per-channel late-drop watermark, not a scope of "this claim" —
    /// resetting it would let an already-delivered (or already-late) seq
    /// look fresh again on a channel whose producer's seq space persists
    /// across claims (the media-intake teleop seq is the wire's own,
    /// session-lifetime-monotonic counter).
    pub fn clear_pending(&mut self) {
        self.teleop.pending.clear();
        self.teleop.active_chunk = None;
        self.agent_chunk.pending.clear();
        self.agent_chunk.active_chunk = None;
    }

    #[must_use]
    pub fn dropped_late(&self) -> u64 {
        self.dropped_late
    }

    /// Arrivals rejected outright as belonging to a stale/out-of-order chunk
    /// (`ingest`'s `Transition::Stale`) — distinct from `dropped_late`, which
    /// counts individual late steps within an already-accepted chunk. Counts
    /// per ARRIVAL, not per distinct chunk: a stale chunk with N steps
    /// increments this by N (every step independently re-evaluates against
    /// the same unchanged `active_chunk`, since a stale arrival never
    /// advances it), not by 1.
    #[must_use]
    pub fn dropped_stale_chunks(&self) -> u64 {
        self.dropped_stale_chunks
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.teleop.pending.is_empty() && self.agent_chunk.pending.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;
    use smallvec::smallvec;

    fn ta(seq: u64, received: i64) -> TimedAction {
        ta_on(StreamChannel::Teleop, seq, received)
    }

    fn ta_on(channel: StreamChannel, seq: u64, received: i64) -> TimedAction {
        TimedAction {
            channel,
            seq,
            received: MonoNs(received),
            action: OwnedAction {
                #[allow(clippy::cast_precision_loss)]
                values: smallvec![seq as f64],
                velocity_feedforward: None,
                gripper: None,
                part: None,
            },
            chunk: None,
        }
    }

    /// An `AgentChunk` arrival tagged with its originating chunk's metadata.
    /// `value` rides in `values[0]` so tests can tell which chunk a popped
    /// step came from without decoding `seq`.
    fn ta_chunk(
        seq: u64,
        received: i64,
        chunk_seq: u64,
        t_emitted_ns: i64,
        value: f64,
    ) -> TimedAction {
        TimedAction {
            channel: StreamChannel::AgentChunk,
            seq,
            received: MonoNs(received),
            action: OwnedAction {
                values: smallvec![value],
                velocity_feedforward: None,
                gripper: None,
                part: None,
            },
            chunk: Some(ChunkMeta {
                chunk_seq,
                t_emitted_ns,
            }),
        }
    }

    fn jb(playout_delay_ns: i64, replan: ReplanPolicy) -> JitterBuffer {
        JitterBuffer::new(playout_delay_ns, replan)
    }

    #[test]
    fn respects_playout_delay() {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        jb.ingest(ta(1, 0));
        assert!(jb.pop_due(MonoNs(999)).is_none());
        assert!(jb.pop_due(MonoNs(1_000)).is_some());
    }

    #[test]
    fn reorders_within_the_window_and_drops_late() {
        let mut jb = jb(100, ReplanPolicy::Immediate);
        jb.ingest(ta(2, 0));
        jb.ingest(ta(1, 10));
        assert_eq!(jb.pop_due(MonoNs(200)).unwrap().values[0], 1.0);
        assert_eq!(jb.pop_due(MonoNs(200)).unwrap().values[0], 2.0);
        // seq 1 again: behind the cursor → dropped.
        jb.ingest(ta(1, 50));
        assert!(jb.pop_due(MonoNs(500)).is_none());
        assert_eq!(jb.dropped_late(), 1);
    }

    /// The critical regression this module exists to prevent: every producer
    /// shares one buffer, but the CHANNELS do not share a cursor. A teleop
    /// claimant's activity (advancing `Teleop`'s cursor arbitrarily high)
    /// must never cause the FIRST agent-chunk arrival — whose counter starts
    /// at 1, independent of the teleop wire's (see `pumps.rs`'s
    /// `next_chunk_seq` doc comment) — to collide with that high-water mark
    /// and be dropped as late.
    #[test]
    fn channels_have_independent_reorder_cursors() {
        let mut jb = jb(0, ReplanPolicy::Immediate);
        // A teleop claimant runs for a while: seq climbs well past any small
        // number an unrelated later producer might start from.
        for seq in 1..=50u64 {
            jb.ingest(ta_on(StreamChannel::Teleop, seq, 0));
            assert!(jb.pop_due(MonoNs(0)).is_some());
        }
        assert_eq!(jb.dropped_late(), 0);

        // A LATER reset window's agent-chunk producer starts its own seq
        // space fresh at 1 — with a single shared cursor this would be
        // "seq <= 50" and silently dropped forever; with independent
        // cursors it must ingest and pop normally.
        jb.ingest(ta_on(StreamChannel::AgentChunk, 1, 100));
        let popped = jb.pop_due(MonoNs(100));
        assert!(
            popped.is_some(),
            "agent-chunk seq=1 must not collide with the teleop channel's cursor"
        );
        assert_eq!(jb.dropped_late(), 0, "no cross-channel drop must occur");

        // And the teleop channel's own late-drop discipline still holds:
        // replaying an already-popped teleop seq is still dropped.
        jb.ingest(ta_on(StreamChannel::Teleop, 30, 200));
        assert!(jb.pop_due(MonoNs(200)).is_none());
        assert_eq!(jb.dropped_late(), 1);
    }

    // --- Chunk horizon: expansion, ordering, ReplanPolicy, staleness ------

    /// A chunk's several steps expand into the buffer as independent timed
    /// arrivals and pop out in the same order they were declared, each only
    /// once its own playout delay has elapsed.
    #[test]
    fn a_chunks_steps_expand_and_pop_in_order_at_their_own_playout_time() {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        jb.ingest(ta_chunk(
            1, 0, /* chunk_seq */ 1, /* t_emitted */ 0, 10.0,
        ));
        jb.ingest(ta_chunk(2, 500, 1, 0, 20.0));
        jb.ingest(ta_chunk(3, 1_000, 1, 0, 30.0));

        assert!(jb.pop_due(MonoNs(500)).is_none(), "step 1 not due yet");
        assert_eq!(jb.pop_due(MonoNs(1_000)).unwrap().values[0], 10.0);
        assert!(jb.pop_due(MonoNs(1_000)).is_none(), "step 2 not due yet");
        assert_eq!(jb.pop_due(MonoNs(1_500)).unwrap().values[0], 20.0);
        assert_eq!(jb.pop_due(MonoNs(2_000)).unwrap().values[0], 30.0);
        assert_eq!(jb.dropped_stale_chunks(), 0);
    }

    /// `REPLAN_POLICY_IMMEDIATE`: a newer chunk arriving mid-horizon drops
    /// the executing chunk's still-pending steps outright — they never play
    /// out, even though they were accepted (not late) when ingested.
    #[test]
    fn immediate_replan_drops_the_executing_chunks_remaining_steps() {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        // Chunk 1: three steps, well spread out.
        jb.ingest(ta_chunk(1, 0, 1, 100, 1.0));
        jb.ingest(ta_chunk(2, 1_000, 1, 100, 2.0));
        jb.ingest(ta_chunk(3, 2_000, 1, 100, 3.0));
        // Step 1 plays out normally.
        assert_eq!(jb.pop_due(MonoNs(1_000)).unwrap().values[0], 1.0);

        // Chunk 2 supersedes mid-horizon (newer by both seq and emitted-time).
        jb.ingest(ta_chunk(4, 1_100, 2, 200, 9.0));

        // Chunk 1's remaining steps (2.0, 3.0) are gone; only chunk 2's step
        // pops, however long we wait.
        assert_eq!(jb.pop_due(MonoNs(10_000)).unwrap().values[0], 9.0);
        assert!(jb.pop_due(MonoNs(10_000)).is_none());
    }

    /// `REPLAN_POLICY_BLEND`'s normative comment offers no blend
    /// duration/curve for a chunk-to-chunk splice (unlike
    /// `HandoffPolicy::Immediate.blend_ns`) and itself steers implementers
    /// away from it ("prefer IMMEDIATE + clamp") — this buffer treats it as
    /// replace-remaining, identically to IMMEDIATE (documented in the module
    /// doc and the task report).
    #[test]
    fn blend_replan_is_treated_as_immediate_replace() {
        let mut jb = jb(0, ReplanPolicy::Blend);
        jb.ingest(ta_chunk(1, 0, 1, 100, 1.0));
        jb.ingest(ta_chunk(2, 100, 1, 100, 2.0));
        assert_eq!(jb.pop_due(MonoNs(0)).unwrap().values[0], 1.0);

        jb.ingest(ta_chunk(3, 50, 2, 200, 9.0));
        assert_eq!(jb.pop_due(MonoNs(1_000)).unwrap().values[0], 9.0);
        assert!(jb.pop_due(MonoNs(1_000)).is_none(), "step 2.0 must be gone");
    }

    /// `REPLAN_POLICY_CHUNK_BOUNDARY`: the executing chunk's remaining steps
    /// finish first; the newer chunk's steps simply queue in behind (their
    /// per-item `seq` is higher, so ordering falls out for free).
    #[test]
    fn chunk_boundary_replan_lets_the_executing_chunk_finish_then_queues_the_new_one() {
        let mut jb = jb(0, ReplanPolicy::ChunkBoundary);
        jb.ingest(ta_chunk(1, 0, 1, 100, 1.0));
        jb.ingest(ta_chunk(2, 1_000, 1, 100, 2.0));
        jb.ingest(ta_chunk(3, 2_000, 1, 100, 3.0));
        assert_eq!(jb.pop_due(MonoNs(1_000)).unwrap().values[0], 1.0);

        // Chunk 2 arrives mid-horizon; under CHUNK_BOUNDARY nothing is
        // dropped.
        jb.ingest(ta_chunk(4, 1_100, 2, 200, 9.0));

        // Chunk 1's remaining steps still pop first, in order...
        assert_eq!(jb.pop_due(MonoNs(2_000)).unwrap().values[0], 2.0);
        assert_eq!(jb.pop_due(MonoNs(3_000)).unwrap().values[0], 3.0);
        // ...then chunk 2's.
        assert_eq!(jb.pop_due(MonoNs(3_000)).unwrap().values[0], 9.0);
    }

    /// A chunk is rejected wholesale if its `chunk_seq` is not strictly
    /// newer than the currently-executing one (the primary, protocol-backed
    /// signal — regardless of `t_emitted_ns`), OR — when BOTH chunks declare
    /// a nonzero `t_emitted_ns` — that emitted-time is not strictly newer
    /// either (a defense-in-depth signal, but never on its own veto power
    /// over a bare/zero value). Either way, the executing chunk's horizon is
    /// undisturbed, and no step of the stale chunk is ever inserted (so it
    /// can never accidentally play out).
    #[test]
    fn a_stale_chunk_is_rejected_by_either_seq_or_emitted_time() {
        let mut jb = jb(0, ReplanPolicy::Immediate);
        jb.ingest(ta_chunk(
            1, 0, /* chunk_seq */ 5, /* t_emitted */ 500, 1.0,
        ));
        assert_eq!(jb.dropped_stale_chunks(), 0);

        // Lower seq is stale on its own, no matter how much higher
        // t_emitted claims to be (seq is the primary signal).
        jb.ingest(ta_chunk(2, 100, 4, 900, 99.0));
        assert_eq!(jb.dropped_stale_chunks(), 1);

        // Higher seq, but both sides declare a nonzero t_emitted and the new
        // one isn't newer: also stale (the defense-in-depth signal fires).
        jb.ingest(ta_chunk(3, 100, 6, 500, 99.0));
        assert_eq!(jb.dropped_stale_chunks(), 2);

        // Neither rejected arrival's step is ever observable.
        assert_eq!(jb.pop_due(MonoNs(10_000)).unwrap().values[0], 1.0);
        assert!(jb.pop_due(MonoNs(10_000)).is_none());

        // Strictly newer by both signals: accepted normally.
        jb.ingest(ta_chunk(4, 100, 6, 600, 2.0));
        assert_eq!(jb.pop_due(MonoNs(10_000)).unwrap().values[0], 2.0);
        assert_eq!(jb.dropped_stale_chunks(), 2, "no further rejection");
    }

    /// `dropped_stale_chunks` counts per ARRIVAL, not per distinct chunk: a
    /// multi-step stale chunk increments it once per step (every step
    /// independently re-evaluates against the same unchanged
    /// `active_chunk`), never per whole chunk. All of them must still be
    /// rejected — none may leak into `pending`.
    #[test]
    fn a_multi_step_stale_chunk_increments_dropped_stale_chunks_once_per_step() {
        let mut jb = jb(0, ReplanPolicy::Immediate);
        jb.ingest(ta_chunk(
            1, 0, /* chunk_seq */ 5, /* t_emitted */ 500, 1.0,
        ));

        // A 3-step stale chunk (chunk_seq=2, older than active's 5).
        jb.ingest(ta_chunk(2, 100, 2, 100, 91.0));
        jb.ingest(ta_chunk(3, 100, 2, 100, 92.0));
        jb.ingest(ta_chunk(4, 100, 2, 100, 93.0));
        assert_eq!(
            jb.dropped_stale_chunks(),
            3,
            "each step of the stale chunk is independently rejected"
        );

        // Only chunk 1's own step is ever observable.
        assert_eq!(jb.pop_due(MonoNs(10_000)).unwrap().values[0], 1.0);
        assert!(jb.pop_due(MonoNs(10_000)).is_none());
    }

    /// Regression (review finding on `jitter.rs:184`): `control.proto` makes
    /// only `ActionChunk.seq` normative for ordering ("Monotone per stream;
    /// gaps are visible, reordering is detectable") — nothing requires
    /// `t_emitted_ns` to be set, and proto3 cannot distinguish "unset" from
    /// its default 0. A wire-legal producer that leaves `t_emitted_ns` at 0
    /// on every chunk (or ties it) must still have its genuinely newer `seq`
    /// chunks accepted — the old `meta.t_emitted_ns <= active.t_emitted_ns`
    /// check treated `0 <= 0` as stale, which would have wrongly rejected
    /// chunk 2 here and then, because a stale arrival never advances
    /// `active_chunk`, silently rejected EVERY subsequent chunk in the claim
    /// window too (the failure this test locks in against a regression).
    #[test]
    fn a_producer_that_leaves_t_emitted_ns_at_the_proto3_default_is_never_treated_as_stale_by_that_alone()
     {
        let mut jb = jb(0, ReplanPolicy::Immediate);
        // Chunk 1: t_emitted_ns left at the proto3 default 0.
        jb.ingest(ta_chunk(
            1, 0, /* chunk_seq */ 1, /* t_emitted */ 0, 1.0,
        ));
        assert_eq!(jb.pop_due(MonoNs(0)).unwrap().values[0], 1.0);

        // Chunk 2: genuinely newer by seq, but the same producer still
        // leaves (or ties) t_emitted_ns at 0 — must be accepted, not
        // rejected as stale.
        jb.ingest(ta_chunk(
            2, 100, /* chunk_seq */ 2, /* t_emitted */ 0, 2.0,
        ));
        assert_eq!(
            jb.dropped_stale_chunks(),
            0,
            "an unset/tied t_emitted_ns must never veto a strictly newer seq"
        );
        assert_eq!(jb.pop_due(MonoNs(100)).unwrap().values[0], 2.0);

        // Chunk 3: newer seq again, this time BOTH sides tie a genuinely
        // nonzero t_emitted_ns — the defense-in-depth signal only fires when
        // it can't be, so this ties on t_emitted while still being newer by
        // seq alone: must still be accepted (seq is primary, sufficient on
        // its own).
        jb.ingest(ta_chunk(
            3, 200, /* chunk_seq */ 3, /* t_emitted */ 0, 3.0,
        ));
        assert_eq!(jb.dropped_stale_chunks(), 0);
        assert_eq!(jb.pop_due(MonoNs(200)).unwrap().values[0], 3.0);

        // Every subsequent chunk in the window keeps substituting normally —
        // the exact regression: under the old logic, chunk 2 above would
        // have been rejected as stale and NEVER updated `active_chunk`, so
        // chunk 3 (chunk_seq=3 <= active's still-1) would ALSO have been
        // rejected, and so on forever.
        jb.ingest(ta_chunk(
            4, 300, /* chunk_seq */ 4, /* t_emitted */ 0, 4.0,
        ));
        assert_eq!(jb.dropped_stale_chunks(), 0);
        assert_eq!(jb.pop_due(MonoNs(300)).unwrap().values[0], 4.0);
    }

    /// A newer `seq` whose declared emission time actually regresses (both
    /// sides nonzero) is still caught — the additional `t_emitted_ns` signal
    /// is defense-in-depth, not a no-op, when the producer does supply it.
    #[test]
    fn a_newer_seq_with_a_regressing_nonzero_t_emitted_ns_is_still_stale() {
        let mut jb = jb(0, ReplanPolicy::Immediate);
        jb.ingest(ta_chunk(
            1, 0, /* chunk_seq */ 1, /* t_emitted */ 1_000, 1.0,
        ));
        assert_eq!(jb.pop_due(MonoNs(0)).unwrap().values[0], 1.0);

        // seq is newer (2 > 1), but t_emitted_ns regressed (500 < 1_000) and
        // both sides are nonzero: the additional signal rejects it.
        jb.ingest(ta_chunk(
            2, 100, /* chunk_seq */ 2, /* t_emitted */ 500, 9.0,
        ));
        assert_eq!(jb.dropped_stale_chunks(), 1);
        assert!(
            jb.pop_due(MonoNs(100)).is_none(),
            "the stale step never queued"
        );
    }

    /// An empty wire chunk never reaches `ingest` at all (the producer has
    /// no steps to push) — by construction this can neither supersede the
    /// executing chunk nor be rejected as stale. Proven here by simulating
    /// "chunk 2 arrives empty": no call is made for it, and chunk 1's
    /// still-pending steps are completely unaffected.
    #[test]
    fn empty_chunk_is_a_no_op() {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        jb.ingest(ta_chunk(1, 500, 1, 100, 1.0));
        jb.ingest(ta_chunk(2, 600, 1, 100, 2.0));
        // "Chunk 2 (empty)" contributes zero steps — nothing to ingest.
        assert_eq!(jb.pop_due(MonoNs(2_000)).unwrap().values[0], 1.0);
        assert_eq!(jb.pop_due(MonoNs(2_000)).unwrap().values[0], 2.0);
        assert_eq!(jb.dropped_stale_chunks(), 0);
    }

    /// `clear_pending` must also forget the executing-chunk pointer: a
    /// brand-new claim's first chunk can legitimately use a small,
    /// claim-scoped `chunk_seq`/`t_emitted_ns` (nothing in the protocol
    /// guarantees these are session-lifetime-monotonic across claims) and
    /// must never be wrongly rejected as "stale" against a previous,
    /// unrelated claim's last chunk.
    #[test]
    fn clear_pending_forgets_the_executing_chunk_so_a_new_claims_first_chunk_is_never_wrongly_stale()
     {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        // Claim 1's last chunk: high seq/emitted-time, still pending when
        // the claim ends.
        jb.ingest(ta_chunk(1, 500, 100, 100_000, 1.0));
        jb.clear_pending();

        // Claim 2's first chunk: a small, claim-scoped chunk_seq/t_emitted —
        // must be accepted, not rejected as stale against claim 1's chunk.
        jb.ingest(ta_chunk(2, 600, 1, 1, 2.0));
        assert_eq!(jb.pop_due(MonoNs(2_000)).unwrap().values[0], 2.0);
        assert_eq!(
            jb.dropped_stale_chunks(),
            0,
            "a new claim's first chunk must never be measured against a prior claim's"
        );
    }

    /// The stale-residue regression `clear_pending` exists to prevent: an
    /// arrival that is pushed but never popped before its claim ends (still
    /// within its own playout delay) must not resurface once a LATER,
    /// unrelated claim/window starts polling the buffer again — per-channel
    /// cursors alone don't help here, since it's the SAME channel both
    /// times (e.g. an ordinary teleop claim, then a later teleop-driven
    /// reset window).
    #[test]
    fn clear_pending_discards_stale_not_yet_due_arrivals_but_keeps_the_cursor() {
        let mut jb = jb(1_000, ReplanPolicy::Immediate);
        // Claim 1 delivers seq=1 normally (sets the channel's cursor).
        jb.ingest(ta_on(StreamChannel::Teleop, 1, 0));
        assert_eq!(jb.pop_due(MonoNs(1_000)).unwrap().values[0], 1.0);

        // A later packet in the SAME claim arrives right as the claim ends,
        // still within its own playout delay — pending, not yet due.
        jb.ingest(ta_on(StreamChannel::Teleop, 2, 900));
        assert!(jb.pop_due(MonoNs(1_000)).is_none(), "not due yet");

        // The claim releases (gate mode -> Passthrough): the reducer clears
        // the buffer right here.
        jb.clear_pending();

        // A LATER, unrelated teleop claim/window starts polling. Without
        // the clear, `now` has long since passed seq 2's playout deadline
        // and it would pop here, dispatched under the later claim's
        // provenance.
        assert!(
            jb.pop_due(MonoNs(5_000)).is_none(),
            "claim 1's stale pending arrival must not survive into a later claim"
        );
        assert_eq!(
            jb.dropped_late(),
            0,
            "a clear is a discard, not a late-drop (no double counting)"
        );

        // The cursor is untouched: a replay of an already-DELIVERED seq
        // from claim 1's own channel is still correctly rejected as late,
        // not accepted as fresh (see the method's own doc comment for why
        // that stays deliberate).
        jb.ingest(ta_on(StreamChannel::Teleop, 1, 6_000));
        assert!(jb.pop_due(MonoNs(7_000)).is_none());
        assert_eq!(jb.dropped_late(), 1);

        // The later claim's own fresh arrivals still work normally.
        jb.ingest(ta_on(StreamChannel::Teleop, 3, 6_000));
        assert!(jb.pop_due(MonoNs(7_000)).is_some());
    }

    proptest! {
        /// Pops are strictly seq-increasing regardless of arrival order, and
        /// nothing pops before its playout delay.
        #[test]
        fn pops_are_ordered_and_delayed(
            arrivals in proptest::collection::vec((0u64..64, 0i64..10_000), 1..64),
            delay in 0i64..2_000,
        ) {
            let mut jb = jb(delay, ReplanPolicy::Immediate);
            let mut now = 0i64;
            let mut last_seq: Option<u64> = None;
            for (seq, gap) in arrivals {
                now += gap;
                jb.ingest(ta(seq, now));
                while let Some(a) = jb.pop_due(MonoNs(now)) {
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                    let popped = a.values[0] as u64;
                    if let Some(prev) = last_seq {
                        prop_assert!(popped > prev, "out-of-order pop");
                    }
                    last_seq = Some(popped);
                }
            }
        }
    }
}
