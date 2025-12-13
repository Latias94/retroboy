use super::super::GameBoyBus;

impl GameBoyBus {
    pub(in super::super) fn mmio_read_timer_register(&mut self, addr: u16) -> u8 {
        // Divider and timer registers. Each helper call advances the internal
        // timer state by one tick; we track these so that `end_instruction`
        // can avoid double-counting ticks.
        let value = match addr {
            0xFF04 => self.timer.div_read(&mut self.if_reg),
            0xFF05 => self.timer.tima_read(&mut self.if_reg),
            0xFF06 => self.timer.tma_read(&mut self.if_reg),
            0xFF07 => self.timer.tac_read(&mut self.if_reg),
            _ => unreachable!("mmio_read_timer_register called with non-timer addr {addr:#06X}"),
        };
        self.timer_io_events = self.timer_io_events.wrapping_add(1);
        value
    }

    pub(in super::super) fn mmio_write_timer_register(&mut self, addr: u16, value: u8) {
        match addr {
            0xFF04 => self.timer.div_write(&mut self.if_reg),
            0xFF05 => self.timer.tima_write(value, &mut self.if_reg),
            0xFF06 => self.timer.tma_write(value, &mut self.if_reg),
            0xFF07 => self.timer.tac_write(value, &mut self.if_reg),
            _ => unreachable!("mmio_write_timer_register called with non-timer addr {addr:#06X}"),
        }
        self.timer_io_events = self.timer_io_events.wrapping_add(1);
    }
}
