//! Sequenced input liveness on the SDK clock, independent of planner/status RPCs.

use std::sync::Arc;

use parking_lot::Mutex;
use waddle_types::EpisodeId;
use waddle_types::time::Clock;

use super::{PlaybackError, PlaybackWatchdog};

/// Retained input evidence. An active input permits no motion by itself.
#[derive(Clone, Debug)]
pub struct PlaybackInputSnapshot {
    pub id: String,
    pub episode_id: EpisodeId,
    pub sequence: u64,
    /// `observed_ns` is local input acceptance time, not robot feedback time.
    pub watchdog: PlaybackWatchdog,
    /// Release and expiry are permanent for this input identity.
    pub failure: Option<PlaybackError>,
}

pub(super) struct InputControl {
    pub registry: Arc<()>,
    pub episode: EpisodeId,
    clock: Arc<dyn Clock>,
    state: Mutex<PlaybackInputSnapshot>,
}

impl std::fmt::Debug for InputControl {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("InputControl")
            .field("state", &*self.state.lock())
            .finish_non_exhaustive()
    }
}

/// One run's press/input stream. Renew only on newly accepted human input;
/// polling, planning and motion completion never renew it. Each renewal carries
/// a strictly increasing sequence; duplicates and reordered messages do not
/// extend expiry. The existing worker checks it before and after local I/O.
#[derive(Clone, Debug)]
pub struct PlaybackInput(pub(super) Arc<InputControl>);

impl PlaybackInput {
    pub(super) fn new(
        registry: Arc<()>,
        episode: EpisodeId,
        clock: Arc<dyn Clock>,
        timeout_ns: i64,
    ) -> Result<Self, PlaybackError> {
        let now = clock.stamp_now().mono_ns().0;
        if !(1..=180_000_000_000).contains(&timeout_ns)
            || now < 0
            || now.checked_add(timeout_ns).is_none()
        {
            return Err(PlaybackError::Invalid("input requires a bounded timeout"));
        }
        Ok(Self(Arc::new(InputControl {
            registry,
            episode: episode.clone(),
            clock,
            state: Mutex::new(PlaybackInputSnapshot {
                id: uuid::Uuid::new_v4().to_string(),
                episode_id: episode,
                sequence: 0,
                watchdog: PlaybackWatchdog {
                    observed_ns: now,
                    timeout_ns,
                },
                failure: None,
            }),
        })))
    }

    fn check(state: &mut PlaybackInputSnapshot, now: i64) {
        if state.failure.is_none() {
            if now < state.watchdog.observed_ns {
                state.failure = Some(PlaybackError::Invalid("input clock moved backwards"));
            } else if now >= state.watchdog.deadline_ns() {
                state.failure = Some(PlaybackError::WatchdogExpired);
            }
        }
    }

    #[must_use]
    pub fn snapshot(&self) -> PlaybackInputSnapshot {
        let mut state = self.0.state.lock();
        Self::check(&mut state, self.0.clock.stamp_now().mono_ns().0);
        state.clone()
    }

    /// Accept new input on the SDK clock. Expiry is checked before renewal,
    /// even while the execution worker is blocked in an owner callback.
    pub fn refresh(&self, sequence: u64) -> Result<PlaybackInputSnapshot, PlaybackError> {
        let mut state = self.0.state.lock();
        let now = self.0.clock.stamp_now().mono_ns().0;
        Self::check(&mut state, now);
        if let Some(error) = &state.failure {
            return Err(error.clone());
        }
        if sequence > state.sequence {
            if now.checked_add(state.watchdog.timeout_ns).is_none() {
                return Err(PlaybackError::Invalid("input deadline overflows"));
            }
            state.sequence = sequence;
            state.watchdog.observed_ns = now;
        }
        Ok(state.clone())
    }

    /// Fence this input immediately; the existing worker performs normal Hold
    /// and publishes its cleanup receipt. Release alone is not stop completion.
    pub fn release(&self, reason: &str) {
        let mut state = self.0.state.lock();
        Self::check(&mut state, self.0.clock.stamp_now().mono_ns().0);
        state
            .failure
            .get_or_insert_with(|| PlaybackError::Cancelled(reason.into()));
    }
}
