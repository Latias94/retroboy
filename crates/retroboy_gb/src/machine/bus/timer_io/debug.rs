use super::super::GameBoyBus;

impl GameBoyBus {
    /// Test helper: return the number of T-cycles the timer has been advanced
    /// since power-on. Only available in tests.
    #[cfg(test)]
    pub fn timer_debug_ticks(&self) -> u64 {
        self.timer.tick_debug_count
    }

    /// Test helper: return how many times TIMA has been incremented (including
    /// increments that overflowed). Only available in tests.
    #[cfg(test)]
    pub fn timer_debug_tima_increments(&self) -> u64 {
        self.timer.tima_increment_count
    }

    /// Test helper: return how many overflow events have been serviced (i.e.
    /// how many times TIMA was reloaded from TMA and the timer interrupt
    /// requested). Only available in tests.
    #[cfg(test)]
    pub fn timer_debug_overflows(&self) -> u64 {
        self.timer.overflow_event_count
    }

    /// Test-only helper that returns a snapshot of the timer/interrupt visible
    /// state without advancing time.
    ///
    /// This is used by lockstep tests so that we can inspect
    /// DIV/TIMA/TMA/TAC/IF/IE without triggering the side effects associated
    /// with real MMIO reads.
    #[cfg(test)]
    pub(crate) fn debug_timer_snapshot(&self) -> (u8, u8, u8, u8, u8, u8) {
        let div = (self.timer.internal_counter >> 6) as u8;
        let tima = self.timer.tima;
        let tma = self.timer.tma;
        let tac = self.timer.tac | 0b1111_1000;
        let if_reg = self.if_reg | 0b1110_0000;
        let ie_reg = self.ie_reg;
        (div, tima, tma, tac, if_reg, ie_reg)
    }

    /// Test-only helper to reset the Timer into a "mooneye-like" power-on
    /// state where DIV/TIMA/TMA/TAC and the internal counter all start from
    /// zero. This is used by lockstep tests that compare against
    /// `mooneye-gb-core`, which does not apply the DMG boot ROM's DIV seeding.
    #[cfg(test)]
    pub(crate) fn reset_timer_for_mooneye_tests(&mut self) {
        self.timer.internal_counter = 0;
        self.timer.tima = 0;
        self.timer.tma = 0;
        self.timer.tac = 0;
        self.timer.overflow = false;
        self.timer.enabled = false;
    }
}
