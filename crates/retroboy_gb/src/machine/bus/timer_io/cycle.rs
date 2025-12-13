use crate::cpu::Bus;

use super::super::GameBoyBus;

impl GameBoyBus {
    /// Single T-cycle that performs a Timer register *read* at `addr`.
    ///
    /// This is a building block for a future per-cycle CPU/bus integration
    /// model where accesses to DIV/TIMA/TMA/TAC are modelled as their own
    /// cycles. For now it is unused by the instruction-level `step` path.
    pub(in super::super) fn cycle_timer_read(&mut self, addr: u16) -> u8 {
        match addr {
            0xFF04 => self.timer.div_read(&mut self.if_reg),
            0xFF05 => self.timer.tima_read(&mut self.if_reg),
            0xFF06 => self.timer.tma_read(&mut self.if_reg),
            0xFF07 => self.timer.tac_read(&mut self.if_reg),
            _ => self.read8(addr),
        }
    }

    /// Single T-cycle that performs a Timer register *write* at `addr`.
    ///
    /// Like `cycle_timer_read`, this is currently only a scaffold for more
    /// precise per-cycle modelling; the instruction-level CPU path still
    /// uses `write8` + `tick`.
    pub(in super::super) fn cycle_timer_write(&mut self, addr: u16, value: u8) {
        match addr {
            0xFF04 => self.timer.div_write(&mut self.if_reg),
            0xFF05 => self.timer.tima_write(value, &mut self.if_reg),
            0xFF06 => self.timer.tma_write(value, &mut self.if_reg),
            0xFF07 => self.timer.tac_write(value, &mut self.if_reg),
            _ => self.write8(addr, value),
        }
    }
}
