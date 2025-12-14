/// Abstraction over the Game Boy bus (memory and IO).
///
/// We keep this intentionally small at first; it may later grow to include
/// separate methods for VRAM, IO registers, and cartridge space.
pub trait Bus {
    fn read8(&mut self, addr: u16) -> u8;
    fn write8(&mut self, addr: u16, value: u8);
    /// Advance bus-side peripherals by a given number of CPU cycles.
    ///
    /// Default implementation does nothing; system buses can override this
    /// to drive timers, PPU, APU, etc.
    fn tick(&mut self, _cycles: u32) {}

    /// Advance non-timer peripherals (PPU/APU/etc.) by a single CPU
    /// M-cycle (4 T-cycles).
    ///
    /// Micro-step CPU paths (`step_mcycle`) use this to model generic
    /// bus activity while keeping timer advancement under explicit
    /// control via `timer_tick_mcycle` and the Timer IO helpers.
    fn tick_mcycle_generic(&mut self) {
        self.tick(4);
    }

    /// Advance the timer by one M-cycle worth of time when no explicit
    /// Timer IO (DIV/TIMA/TMA/TAC) access took place in the current
    /// CPU cycle.
    ///
    /// Game Boy-specific buses can override this to call into their
    /// Timer core; generic buses can leave it as a no-op.
    fn timer_tick_mcycle(&mut self) {}

    /// Hook that marks the beginning of a single CPU instruction (or
    /// interrupt entry) from the bus's point of view.
    ///
    /// Instruction-level stepping (`Cpu::step`) calls this once per
    /// instruction before any memory accesses. Buses that need to track
    /// per-instruction state (e.g. timer IO events) can override this;
    /// the default implementation is a no-op.
    fn begin_instruction(&mut self) {}

    /// Hook that finalises a single CPU instruction (or interrupt entry)
    /// from the bus's point of view.
    ///
    /// The `cycles` argument is the total number of CPU T-cycles consumed
    /// by the instruction. The default implementation simply forwards to
    /// `tick(cycles)` so existing buses keep their current behaviour.
    fn end_instruction(&mut self, cycles: u32) {
        self.tick(cycles);
    }

    /// Single T-cycle that performs a Timer register read at `addr`.
    ///
    /// The default implementation simply forwards to `read8` so that
    /// non-Game Boy buses do not need to care about Timer semantics.
    /// Game Boy-specific buses (such as `GameBoyBus`) can override this
    /// to model per-cycle Timer behaviour for DIV/TIMA/TMA/TAC accesses.
    fn timer_cycle_read(&mut self, addr: u16) -> u8 {
        self.read8(addr)
    }

    /// Single T-cycle that performs a Timer register write at `addr`.
    ///
    /// The default implementation forwards to `write8`. Game Boy-specific
    /// buses can override this to call into their Timer read/write helpers
    /// instead, so that each Timer IO access corresponds to exactly one
    /// T-cycle.
    fn timer_cycle_write(&mut self, addr: u16, value: u8) {
        self.write8(addr, value)
    }

    /// Handle the CGB "speed switch" mechanism (KEY1 + STOP).
    ///
    /// When running in CGB mode and the "prepare speed switch" latch is set,
    /// executing the `STOP` instruction toggles double-speed and returns
    /// immediately without entering the STOP low-power state.
    ///
    /// Returns `true` if a speed switch occurred.
    fn cgb_speed_switch(&mut self) -> bool {
        false
    }

    /// Returns `true` if the CPU is currently in the post-STOP speed switch pause.
    ///
    /// On CGB, after a successful KEY1+STOP speed switch the CPU is paused for
    /// a fixed duration while the LCD controller continues running.
    fn cgb_speed_switch_pause_active(&self) -> bool {
        false
    }

    /// Consume a single M-cycle (4 T-cycles) of the post-STOP speed switch pause.
    ///
    /// Implementations should advance non-timer peripherals (PPU, STAT/VBlank, etc.)
    /// but must not tick the timer/DIV during this pause.
    ///
    /// Returns `true` if a pause M-cycle was consumed and the CPU should remain idle.
    fn cgb_speed_switch_pause_mcycle(&mut self) -> bool {
        false
    }
}
