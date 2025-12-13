/// Timer / divider unit.
///
/// This models the behaviour described in Pandocs' "Timer and Divider
/// Registers" and "Timer obscure behaviour", and is closely aligned with
/// the state machine used in the Mooneye GB project. Time advances via
/// explicit calls to `tick_tcycle`, while the DIV/TIMA/TMA/TAC helpers
/// implement the observable register semantics without advancing time
/// themselves.
mod io;

pub(super) struct Timer {
    /// Hidden system counter; DIV exposes its upper bits.
    pub(super) internal_counter: u16,
    /// TIMA (FF05).
    pub(super) tima: u8,
    /// TMA (FF06).
    pub(super) tma: u8,
    /// TAC raw value (lower 3 bits meaningful).
    pub(super) tac: u8,
    /// Pending overflow: when true, the *next* tick_mcycle will reload
    /// TIMA from TMA and request the timer interrupt.
    pub(super) overflow: bool,
    /// Cached "timer enabled" flag derived from TAC.
    pub(super) enabled: bool,
    /// Test-only counter tracking how many timer cycles the timer has
    /// been advanced. This is useful when debugging acceptance tests
    /// that rely on precise timer frequencies.
    #[cfg(test)]
    pub(super) tick_debug_count: u64,
    /// Test-only counter tracking how many times TIMA has been
    /// incremented (including increments that lead to overflow).
    #[cfg(test)]
    pub(super) tima_increment_count: u64,
    /// Test-only counter tracking how many overflow events have been
    /// serviced (i.e. how many times TIMA was reloaded from TMA and
    /// the timer interrupt requested).
    #[cfg(test)]
    pub(super) overflow_event_count: u64,
}

impl Timer {
    pub(super) fn new() -> Self {
        Self {
            internal_counter: 0,
            tima: 0,
            tma: 0,
            tac: 0,
            overflow: false,
            enabled: false,
            #[cfg(test)]
            tick_debug_count: 0,
            #[cfg(test)]
            tima_increment_count: 0,
            #[cfg(test)]
            overflow_event_count: 0,
        }
    }

    /// Initialise timer to DMG/MGB power-on state.
    ///
    /// We seed `internal_counter` so that DIV (bits 13:6) reads back as
    /// 0xAB at PC=0x0100, as described in Pandocs.
    pub(super) fn init_dmg(&mut self) {
        self.internal_counter = 0x2AC0;
        self.tima = 0x00;
        self.tma = 0x00;
        self.tac = 0x00;
        self.overflow = false;
        self.enabled = false;
    }

    #[inline]
    pub(super) fn enabled(&self) -> bool {
        self.enabled
    }

    /// Return the currently selected timer input bit of the internal
    /// counter based on TAC[1:0]. The mapping is:
    /// - 00 → bit 7
    /// - 01 → bit 1
    /// - 10 → bit 3
    /// - 11 → bit 5
    #[inline]
    pub(super) fn counter_bit(&self) -> bool {
        let mask = match self.tac & 0x03 {
            0x00 => 1u16 << 7,
            0x01 => 1u16 << 1,
            0x02 => 1u16 << 3,
            0x03 => 1u16 << 5,
            _ => 1u16 << 7,
        };
        (self.internal_counter & mask) != 0
    }

    /// Increment TIMA by one and track overflow.
    #[inline]
    pub(super) fn increment_tima(&mut self) {
        let (next, overflow) = self.tima.overflowing_add(1);
        self.tima = next;
        #[cfg(test)]
        {
            self.tima_increment_count = self.tima_increment_count.saturating_add(1);
        }
        if overflow {
            // TIMA becomes 0x00 for this cycle; reload + IF happen on the
            // *next* tick_mcycle call.
            self.overflow = true;
        }
    }

    /// Advance timer by one T-cycle.
    ///
    /// This is called once per T-cycle by the bus, and also indirectly by
    /// timer register read/write helpers. The semantics follow the model
    /// used by Mooneye GB's timer implementation.
    pub(super) fn tick_tcycle(&mut self, if_reg: &mut u8) {
        #[cfg(test)]
        {
            self.tick_debug_count = self.tick_debug_count.saturating_add(1);
        }
        if self.overflow {
            // One cycle after overflow: reload TIMA from TMA and request INT $50.
            self.internal_counter = self.internal_counter.wrapping_add(1);
            self.tima = self.tma;
            *if_reg |= 0x04;
            self.overflow = false;
            #[cfg(test)]
            {
                self.overflow_event_count = self.overflow_event_count.saturating_add(1);
            }
        } else if self.enabled && self.counter_bit() {
            // Timer enabled and input bit currently 1: advance counter,
            // then detect a falling edge.
            self.internal_counter = self.internal_counter.wrapping_add(1);
            let new_bit = self.counter_bit();
            if !new_bit {
                self.increment_tima();
            }
        } else {
            // Timer disabled or input bit low: just advance the divider.
            self.internal_counter = self.internal_counter.wrapping_add(1);
        }
    }
}
