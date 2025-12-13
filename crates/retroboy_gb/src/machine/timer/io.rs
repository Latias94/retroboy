use super::Timer;

impl Timer {
    /// DIV read cycle.
    pub(in super::super) fn div_read(&mut self, if_reg: &mut u8) -> u8 {
        // On DMG, reading DIV still advances the internal counter by one
        // T-cycle. We model this by ticking once here, mirroring the
        // behaviour of the reference Mooneye GB timer.
        self.tick_tcycle(if_reg);
        (self.internal_counter >> 6) as u8
    }

    /// DIV write cycle.
    ///
    /// Writing any value to DIV resets the internal counter. On DMG this
    /// can cause a single TIMA increment when the selected divider bit
    /// falls from 1 to 0 due to the reset.
    pub(in super::super) fn div_write(&mut self, if_reg: &mut u8) {
        // First advance the timer by one T-cycle so that writes to DIV
        // are aligned with the normal divider progression. This matches
        // the scheme used by Mooneye GB's `div_write_cycle`.
        self.tick_tcycle(if_reg);
        if self.counter_bit() {
            self.increment_tima();
        }
        // Reset counter; on DMG this clears all divider bits.
        self.internal_counter = 0;
    }

    /// TIMA read cycle.
    pub(in super::super) fn tima_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        self.tima
    }

    /// TIMA write cycle.
    ///
    /// Writes during the overflow/reload window have special behaviour:
    /// if an overflow is pending (TIMA just overflowed on the previous
    /// tick), writing to TIMA cancels the reload and interrupt.
    pub(in super::super) fn tima_write(&mut self, value: u8, if_reg: &mut u8) {
        self.tick_tcycle(if_reg);
        let was_overflow = self.overflow;
        self.tima = value;
        if was_overflow {
            self.overflow = false;
        }
    }

    /// TMA read cycle.
    pub(in super::super) fn tma_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        self.tma
    }

    /// TMA write cycle.
    ///
    /// On DMG, writing to TMA during an overflow/reload window affects
    /// what value gets loaded into TIMA (and also what gets exposed if
    /// TIMA is read during the window). We approximate this by applying
    /// the write after advancing the timer state for this cycle.
    pub(in super::super) fn tma_write(&mut self, value: u8, if_reg: &mut u8) {
        self.tick_tcycle(if_reg);
        let was_overflow = self.overflow;
        self.tma = value;
        // If overflow is pending, writing TMA can also update the value
        // that will be reloaded.
        if was_overflow {
            self.tima = value;
        }
    }

    /// TAC read cycle.
    pub(in super::super) fn tac_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        self.tac | 0b1111_1000
    }

    /// TAC write cycle.
    ///
    /// On DMG, disabling the timer or switching the clock source while
    /// the selected input bit is 1 triggers a single TIMA increment.
    pub(in super::super) fn tac_write(&mut self, value: u8, if_reg: &mut u8) {
        // Writing TAC also corresponds to a real T-cycle on hardware, so
        // we advance the timer state before applying the new control
        // bits. This closely matches Mooneye GB's `tac_write_cycle`.
        self.tick_tcycle(if_reg);
        let old_bit = self.enabled && self.counter_bit();
        self.tac = value & 0x07;
        self.enabled = (self.tac & 0x04) != 0;
        let new_bit = self.enabled && self.counter_bit();
        // Falling edge when timer is enabled can produce an extra tick.
        if old_bit && !new_bit {
            self.increment_tima();
        }
    }
}
