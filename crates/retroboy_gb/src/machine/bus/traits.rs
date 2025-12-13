use crate::cpu::Bus;

use super::GameBoyBus;

impl Bus for GameBoyBus {
    fn begin_instruction(&mut self) {
        // Reset per-instruction timer IO accounting so that we can
        // later derive how many additional timer ticks are needed to
        // maintain a consistent "one tick per machine cycle" rate.
        self.timer_io_events = 0;
    }

    fn end_instruction(&mut self, cycles: u32) {
        // Total number of timer ticks corresponding to this instruction
        // when the timer advances once per machine cycle (4 T-cycles).
        let total_timer_ticks = cycles / 4;

        // Timer register helpers (DIV/TIMA/TMA/TAC reads/writes) already
        // advanced the timer once per access. Any remaining ticks are
        // accounted for here.
        let extra_ticks = total_timer_ticks.saturating_sub(self.timer_io_events as u32);

        // Advance non-timer peripherals (PPU, STAT/VBlank, etc.) using
        // the existing per-T-cycle model.
        self.tick(cycles);

        // Apply any additional timer ticks that were not covered by
        // timer IO helpers. From the timer's point of view this is
        // equivalent to spacing them across the instruction's
        // non-timer cycles, since the helpers themselves already
        // handle the relative ordering around DIV/TIMA/TMA/TAC.
        for _ in 0..extra_ticks {
            self.timer.tick_tcycle(&mut self.if_reg);
        }
    }

    fn read8(&mut self, addr: u16) -> u8 {
        self.read8_mmio(addr)
    }

    fn write8(&mut self, addr: u16, value: u8) {
        self.write8_mmio(addr, value)
    }

    fn tick(&mut self, cycles: u32) {
        for _ in 0..cycles {
            self.cycle_generic();
        }
    }

    fn tick_mcycle_generic(&mut self) {
        // Advance PPU / LCD timing by one CPU M-cycle (4 T-cycles).
        for _ in 0..4 {
            self.cycle_generic();
        }
    }

    fn timer_tick_mcycle(&mut self) {
        // Single Timer T-cycle corresponding to one CPU M-cycle in the
        // micro-step model. This keeps the Timer frequency roughly
        // aligned with the instruction-level path where we advance the
        // timer once per machine cycle on average.
        self.timer.tick_tcycle(&mut self.if_reg);
    }

    fn timer_cycle_read(&mut self, addr: u16) -> u8 {
        self.cycle_timer_read(addr)
    }

    fn timer_cycle_write(&mut self, addr: u16, value: u8) {
        self.cycle_timer_write(addr, value)
    }
}
