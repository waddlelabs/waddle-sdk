//! Reconnect backoff as data (adopted from production: fixed steps, then a
//! plateau — retry forever, stay honest about connection state).

#[derive(Debug, Clone)]
pub struct Backoff {
    pub steps_ns: Vec<i64>,
    pub plateau_ns: i64,
}

impl Backoff {
    #[must_use]
    pub fn production() -> Self {
        Self {
            steps_ns: vec![1_000_000_000, 4_000_000_000, 16_000_000_000],
            plateau_ns: 16_000_000_000,
        }
    }

    /// Delay before reconnect attempt `attempt` (0-based).
    #[must_use]
    pub fn delay_ns(&self, attempt: u32) -> i64 {
        self.steps_ns
            .get(attempt as usize)
            .copied()
            .unwrap_or(self.plateau_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_then_plateau_forever() {
        let b = Backoff::production();
        assert_eq!(b.delay_ns(0), 1_000_000_000);
        assert_eq!(b.delay_ns(1), 4_000_000_000);
        assert_eq!(b.delay_ns(2), 16_000_000_000);
        assert_eq!(b.delay_ns(100), 16_000_000_000);
    }
}
