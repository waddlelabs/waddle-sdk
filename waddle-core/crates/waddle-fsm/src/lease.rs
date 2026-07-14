//! The single-writer lease machine. A direct port of the production broker's
//! lease semantics (FSM.md §3, rows L1–L8):
//!
//! - acquire is idempotent per client (the holder gets its existing token),
//! - handoff is atomic and installs a FRESHLY MINTED token,
//! - release requires a token match,
//! - revoke-all (the e-stop path) kills every outstanding token.
//!
//! Tokens are minted by the caller (runtime/conformance target), never here.

use waddle_types::{ClientId, LeaseId};

#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub enum LeaseState {
    Vacant,
    Held { lease: LeaseId, client: ClientId },
}

impl LeaseState {
    #[must_use]
    pub fn holder(&self) -> Option<(&LeaseId, &ClientId)> {
        match self {
            Self::Vacant => None,
            Self::Held { lease, client } => Some((lease, client)),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LeaseCmd {
    /// Idempotent per client: the current holder re-acquiring gets its
    /// existing token back and `minted` is discarded.
    Acquire {
        client: ClientId,
        minted: LeaseId,
    },
    Release {
        lease: LeaseId,
    },
    /// Atomic: fails unless `from` is the current token; on success the new
    /// holder receives `minted`.
    Handoff {
        from: LeaseId,
        to: ClientId,
        minted: LeaseId,
    },
    /// The e-stop path.
    RevokeAll,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LeaseOutcome {
    Granted {
        lease: LeaseId,
        client: ClientId,
        /// False when this was an idempotent re-acquire by the holder.
        fresh: bool,
    },
    HandedOff {
        old: LeaseId,
        new: LeaseId,
        to: ClientId,
    },
    Released {
        lease: LeaseId,
    },
    RevokedAll {
        was: Option<LeaseId>,
    },
    Denied {
        detail: &'static str,
    },
}

/// Pure transition. Denials leave the state unchanged (returned by value,
/// identical to the input).
#[must_use]
pub fn step(state: &LeaseState, cmd: &LeaseCmd) -> (LeaseState, LeaseOutcome) {
    match (state, cmd) {
        (LeaseState::Vacant, LeaseCmd::Acquire { client, minted }) => (
            LeaseState::Held {
                lease: minted.clone(),
                client: client.clone(),
            },
            LeaseOutcome::Granted {
                lease: minted.clone(),
                client: client.clone(),
                fresh: true,
            },
        ),
        (LeaseState::Held { lease, client }, LeaseCmd::Acquire { client: who, .. }) => {
            if who == client {
                (
                    state.clone(),
                    LeaseOutcome::Granted {
                        lease: lease.clone(),
                        client: client.clone(),
                        fresh: false,
                    },
                )
            } else {
                (
                    state.clone(),
                    LeaseOutcome::Denied {
                        detail: "held by another client",
                    },
                )
            }
        }
        (LeaseState::Held { lease, .. }, LeaseCmd::Release { lease: token }) => {
            if token == lease {
                (
                    LeaseState::Vacant,
                    LeaseOutcome::Released {
                        lease: lease.clone(),
                    },
                )
            } else {
                (
                    state.clone(),
                    LeaseOutcome::Denied {
                        detail: "stale or wrong lease token",
                    },
                )
            }
        }
        (LeaseState::Vacant, LeaseCmd::Release { .. }) => (
            state.clone(),
            LeaseOutcome::Denied {
                detail: "no lease held",
            },
        ),
        (LeaseState::Held { lease, .. }, LeaseCmd::Handoff { from, to, minted }) => {
            if from == lease {
                (
                    LeaseState::Held {
                        lease: minted.clone(),
                        client: to.clone(),
                    },
                    LeaseOutcome::HandedOff {
                        old: lease.clone(),
                        new: minted.clone(),
                        to: to.clone(),
                    },
                )
            } else {
                (
                    state.clone(),
                    LeaseOutcome::Denied {
                        detail: "handoff from a token that is not the current holder",
                    },
                )
            }
        }
        (LeaseState::Vacant, LeaseCmd::Handoff { .. }) => (
            state.clone(),
            LeaseOutcome::Denied {
                detail: "no lease to hand off",
            },
        ),
        (_, LeaseCmd::RevokeAll) => {
            let was = state.holder().map(|(l, _)| l.clone());
            (LeaseState::Vacant, LeaseOutcome::RevokedAll { was })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(s: &str) -> ClientId {
        ClientId::new(s)
    }
    fn l(s: &str) -> LeaseId {
        LeaseId::new(s)
    }

    #[test]
    fn acquire_is_idempotent_for_holder() {
        let (s1, _) = step(
            &LeaseState::Vacant,
            &LeaseCmd::Acquire {
                client: c("loop"),
                minted: l("t1"),
            },
        );
        let (s2, out) = step(
            &s1,
            &LeaseCmd::Acquire {
                client: c("loop"),
                minted: l("t2"),
            },
        );
        assert_eq!(s1, s2);
        assert!(
            matches!(out, LeaseOutcome::Granted { lease, fresh: false, .. } if lease == l("t1"))
        );
    }

    #[test]
    fn handoff_is_atomic_and_mints_fresh() {
        let (s1, _) = step(
            &LeaseState::Vacant,
            &LeaseCmd::Acquire {
                client: c("loop"),
                minted: l("t1"),
            },
        );
        let (s2, out) = step(
            &s1,
            &LeaseCmd::Handoff {
                from: l("t1"),
                to: c("teleop"),
                minted: l("t2"),
            },
        );
        assert!(matches!(
            out,
            LeaseOutcome::HandedOff { old, new, .. } if old == l("t1") && new == l("t2")
        ));
        // The old token is dead.
        let (_, out) = step(&s2, &LeaseCmd::Release { lease: l("t1") });
        assert!(matches!(out, LeaseOutcome::Denied { .. }));
    }

    #[test]
    fn stale_handoff_is_denied_without_state_change() {
        let (s1, _) = step(
            &LeaseState::Vacant,
            &LeaseCmd::Acquire {
                client: c("loop"),
                minted: l("t1"),
            },
        );
        let (s2, out) = step(
            &s1,
            &LeaseCmd::Handoff {
                from: l("stale"),
                to: c("teleop"),
                minted: l("t2"),
            },
        );
        assert_eq!(s1, s2);
        assert!(matches!(out, LeaseOutcome::Denied { .. }));
    }

    #[test]
    fn revoke_all_empties_from_any_state() {
        let (s1, _) = step(
            &LeaseState::Vacant,
            &LeaseCmd::Acquire {
                client: c("loop"),
                minted: l("t1"),
            },
        );
        let (s2, out) = step(&s1, &LeaseCmd::RevokeAll);
        assert_eq!(s2, LeaseState::Vacant);
        assert!(matches!(out, LeaseOutcome::RevokedAll { was: Some(w) } if w == l("t1")));
    }
}
