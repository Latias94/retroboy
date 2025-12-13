use crate::cpu::{Bus, Cpu};

use super::MicroInstrKind;

impl Cpu {
    /// Helper used by micro-ops to advance one M-cycle worth of time.
    ///
    /// `timer_used_this_cycle` should be set to `true` when the current
    /// micro-op M-cycle has already called into a Timer IO helper
    /// (`timer_cycle_read`/`timer_cycle_write`), which internally ticks
    /// the Timer once. In that case we avoid ticking the Timer a second
    /// time here.
    #[inline]
    pub(super) fn tick_mcycle_with_timer<B: Bus>(
        &mut self,
        bus: &mut B,
        timer_used_this_cycle: bool,
    ) {
        bus.tick_mcycle_generic();
        if !timer_used_this_cycle {
            bus.timer_tick_mcycle();
        }
    }

    /// Execute one M-cycle (4 T-cycles) worth of work for the current
    /// micro-stepped instruction, if any.
    pub(crate) fn step_instruction_mcycle<B: Bus>(&mut self, bus: &mut B) {
        match self.micro_instr {
            MicroInstrKind::CallA16 => self.step_call_a16_mcycle(bus),
            MicroInstrKind::Ret => self.step_ret_mcycle(bus),
            MicroInstrKind::RetCond => self.step_ret_cond_mcycle(bus),
            MicroInstrKind::JpA16 | MicroInstrKind::JpCondA16 => self.step_jp_a16_mcycle(bus),
            MicroInstrKind::LdSpA16 => self.step_ld_sp_a16_mcycle(bus),
            MicroInstrKind::Rst => self.step_rst_mcycle(bus),
            MicroInstrKind::Jr | MicroInstrKind::JrCond => self.step_jr_mcycle(bus),
            MicroInstrKind::CallCond => self.step_call_cond_mcycle(bus),
            MicroInstrKind::LdhAFromA8 => self.step_ldh_a_from_a8_mcycle(bus),
            MicroInstrKind::LdhAToA8 => self.step_ldh_a_to_a8_mcycle(bus),
            MicroInstrKind::None => {
                // Fallback: no instruction-specific micro-ops; just tick
                // the bus for one M-cycle.
                bus.tick(4);
                return;
            }
        }

        self.micro_stage = self.micro_stage.saturating_add(1);
    }
}
