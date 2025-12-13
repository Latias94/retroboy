mod read;
mod write;

use super::GameBoyBus;

impl GameBoyBus {
    pub(super) fn read8_mmio(&mut self, addr: u16) -> u8 {
        self.read8_mmio_impl(addr)
    }

    pub(super) fn write8_mmio(&mut self, addr: u16, value: u8) {
        self.write8_mmio_impl(addr, value)
    }
}
