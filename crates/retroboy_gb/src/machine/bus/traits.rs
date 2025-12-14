use crate::cpu::Bus;

use super::GameBoyBus;

impl Bus for GameBoyBus {
    fn begin_instruction(&mut self) {
        // Reset per-instruction timer IO accounting so that we can
        // later derive how many additional timer ticks are needed to
        // maintain a consistent "one tick per machine cycle" rate.
        self.timer_io_events = 0;
        // Track how many PPU cycles this instruction advanced (used by
        // `GameBoy::step_frame` for CGB double-speed timing).
        self.last_ppu_cycles = 0;
    }

    fn end_instruction(&mut self, cycles: u32) {
        // Total number of timer ticks corresponding to this instruction
        // when the timer advances once per machine cycle (4 T-cycles).
        let total_timer_ticks = cycles / 4;

        // Timer register helpers (DIV/TIMA/TMA/TAC reads/writes) already
        // advanced the timer once per access. Any remaining ticks are
        // accounted for here.
        let extra_ticks = total_timer_ticks.saturating_sub(self.timer_io_events as u32);

        // Advance non-timer peripherals (PPU, STAT/VBlank, etc.). In CGB
        // double-speed mode the LCD controller keeps running at the normal
        // rate, so we only advance the PPU by half the CPU clock.
        self.tick(cycles);

        // Apply any additional timer ticks that were not covered by
        // timer IO helpers. From the timer's point of view this is
        // equivalent to spacing them across the instruction's
        // non-timer cycles, since the helpers themselves already
        // handle the relative ordering around DIV/TIMA/TMA/TAC.
        for _ in 0..extra_ticks {
            self.timer.tick_tcycle(&mut self.if_reg);
        }

        // Apply any DMA-induced CPU stall time after the current instruction.
        // This advances the rest of the machine (PPU + timer), but the CPU
        // does not execute instructions during the pause.
        let stall = std::mem::take(&mut self.cpu_stall_tcycles);
        if stall != 0 {
            self.tick(stall);
            let stall_timer_ticks = stall / 4;
            for _ in 0..stall_timer_ticks {
                self.timer.tick_tcycle(&mut self.if_reg);
            }
        }

        // CGB speed switch pause after KEY1+STOP: advance non-timer peripherals
        // (PPU/STAT/etc.), but keep DIV/TIMA frozen during the pause.
        let pause = std::mem::take(&mut self.cgb_speed_switch_pause_tcycles);
        if pause != 0 {
            self.tick(pause);
        }
    }

    fn read8(&mut self, addr: u16) -> u8 {
        self.read8_mmio(addr)
    }

    fn write8(&mut self, addr: u16, value: u8) {
        self.write8_mmio(addr, value)
    }

    fn tick(&mut self, cycles: u32) {
        if matches!(self.model, super::super::GameBoyModel::Cgb) && self.cgb_double_speed {
            let total = self.cgb_ppu_subcycle as u32 + cycles;
            let ppu_cycles = total / 2;
            self.cgb_ppu_subcycle = (total & 1) as u8;
            self.last_ppu_cycles = self.last_ppu_cycles.saturating_add(ppu_cycles);
            for _ in 0..ppu_cycles {
                self.cycle_generic();
            }
        } else {
            self.cgb_ppu_subcycle = 0;
            self.last_ppu_cycles = self.last_ppu_cycles.saturating_add(cycles);
            for _ in 0..cycles {
                self.cycle_generic();
            }
        }
    }

    fn tick_mcycle_generic(&mut self) {
        // Advance PPU / LCD timing by one CPU M-cycle (4 T-cycles).
        // In double-speed mode, this corresponds to 2 PPU cycles.
        self.tick(4);
    }

    fn timer_tick_mcycle(&mut self) {
        // Single Timer T-cycle corresponding to one CPU M-cycle in the
        // micro-step model.
        //
        // When the CPU core performs a timer register access via the
        // regular `read8`/`write8` path (instead of `timer_cycle_*`),
        // the Timer helper already advanced the core and incremented
        // `timer_io_events`. In that case, skip the generic tick to avoid
        // double-counting.
        if self.timer_io_events != 0 {
            self.timer_io_events = self.timer_io_events.wrapping_sub(1);
            return;
        }

        self.timer.tick_tcycle(&mut self.if_reg);
    }

    fn timer_cycle_read(&mut self, addr: u16) -> u8 {
        self.cycle_timer_read(addr)
    }

    fn timer_cycle_write(&mut self, addr: u16, value: u8) {
        self.cycle_timer_write(addr, value)
    }

    fn cgb_speed_switch(&mut self) -> bool {
        if !matches!(self.model, super::super::GameBoyModel::Cgb) {
            return false;
        }
        if !self.cgb_key1_armed {
            return false;
        }

        self.cgb_key1_armed = false;
        self.cgb_double_speed = !self.cgb_double_speed;
        // Pandocs: CPU pauses for 2050 M-cycles (= 8200 T-cycles) after STOP.
        self.cgb_speed_switch_pause_tcycles = 8_200;
        true
    }

    fn cgb_speed_switch_pause_active(&self) -> bool {
        self.cgb_speed_switch_pause_tcycles != 0
    }

    fn cgb_speed_switch_pause_mcycle(&mut self) -> bool {
        if self.cgb_speed_switch_pause_tcycles == 0 {
            return false;
        }

        let step = 4u32.min(self.cgb_speed_switch_pause_tcycles);
        self.cgb_speed_switch_pause_tcycles = self.cgb_speed_switch_pause_tcycles.saturating_sub(step);
        self.tick(step);
        true
    }
}
