//! The N11 heartbeat: safely-measurable proxy signals. You cannot
//! continuously measure the latency of a verb you dare not call — hold() on
//! a robot mid-task is an incident, not a probe. The ping carries proxies
//! (control RTT, gate-tick jitter, host load, callback dispatch); actual
//! verb measurements ride along only when taken in a safe window.

use waddle_types::MonoNs;
use waddle_types::pb::v0 as pb;

/// Builds pings and derives control-plane RTT from acks.
#[derive(Debug, Default)]
pub struct HeartbeatTracker {
    last_rtt_ns: Option<i64>,
}

/// Inputs sampled at ping time.
#[derive(Debug, Clone, Default)]
pub struct ProxyInputs {
    pub gate_tick_p50_ns: i64,
    pub gate_tick_p95_ns: i64,
    pub gate_tick_max_ns: i64,
    pub gate_tick_samples: u32,
    pub callback_p50_ns: i64,
    pub callback_p95_ns: i64,
    pub host_load_1m: f64,
    pub host_load_5m: f64,
}

impl HeartbeatTracker {
    #[must_use]
    pub fn build_ping(
        &self,
        session_id: &str,
        now: MonoNs,
        inputs: &ProxyInputs,
        verb_measurements: Vec<pb::VerbMeasurement>,
    ) -> pb::HeartbeatPing {
        pb::HeartbeatPing {
            session_id: session_id.to_owned(),
            t_ns: now.0,
            signals: Some(pb::ProxySignals {
                control_rtt_ns: self.last_rtt_ns.unwrap_or_default(),
                gate_tick: Some(pb::JitterStats {
                    p50_ns: inputs.gate_tick_p50_ns,
                    p95_ns: inputs.gate_tick_p95_ns,
                    p99_ns: inputs.gate_tick_p95_ns,
                    max_ns: inputs.gate_tick_max_ns,
                    rate_hz: 0.0,
                    samples: inputs.gate_tick_samples,
                }),
                host_load_1m: inputs.host_load_1m,
                host_load_5m: inputs.host_load_5m,
                cpu_utilization: 0.0,
                callback_dispatch: Some(pb::JitterStats {
                    p50_ns: inputs.callback_p50_ns,
                    p95_ns: inputs.callback_p95_ns,
                    p99_ns: inputs.callback_p95_ns,
                    max_ns: inputs.callback_p95_ns,
                    rate_hz: 0.0,
                    samples: 0,
                }),
            }),
            verb_measurements,
        }
    }

    /// Fold an ack: `echo_t_ns` is our ping's `t_ns`, so RTT needs no second
    /// clock.
    pub fn note_ack(&mut self, ack: &pb::HeartbeatAck, now: MonoNs) -> Option<i64> {
        if ack.echo_t_ns == 0 {
            return None;
        }
        let rtt = now.0 - ack.echo_t_ns;
        (rtt >= 0).then(|| {
            self.last_rtt_ns = Some(rtt);
            rtt
        })
    }

    #[must_use]
    pub fn last_rtt_ns(&self) -> Option<i64> {
        self.last_rtt_ns
    }
}

/// Host load, sampled from /proc/loadavg (zeros where unavailable). Outer
/// crate: I/O is allowed here, never in the inner crates.
#[derive(Debug, Clone, Copy, Default)]
pub struct HostLoad {
    pub one_minute: f64,
    pub five_minutes: f64,
}

impl HostLoad {
    #[must_use]
    pub fn sample() -> Self {
        let Ok(contents) = std::fs::read_to_string("/proc/loadavg") else {
            return Self::default();
        };
        let mut parts = contents.split_whitespace();
        let one = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
        let five = parts.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
        Self {
            one_minute: one,
            five_minutes: five,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rtt_derives_from_the_echo() {
        let mut hb = HeartbeatTracker::default();
        let ping = hb.build_ping("s", MonoNs(1_000_000), &ProxyInputs::default(), vec![]);
        let ack = pb::HeartbeatAck {
            echo_t_ns: ping.t_ns,
            ..Default::default()
        };
        assert_eq!(hb.note_ack(&ack, MonoNs(3_000_000)), Some(2_000_000));
        assert_eq!(hb.last_rtt_ns(), Some(2_000_000));
    }

    #[test]
    fn host_load_never_panics() {
        let _ = HostLoad::sample();
    }
}
