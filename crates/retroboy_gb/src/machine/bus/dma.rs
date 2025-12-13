use super::super::MEMORY_SIZE;
use super::GameBoyBus;

impl GameBoyBus {
    pub(super) fn do_oam_dma(&mut self, value: u8) {
        // OAM DMA: copy 160 bytes from source XX00..XX9F to FE00..FE9F.
        // We currently ignore the exact 160 M-cycle timing and CPU
        // bus conflicts, and perform the transfer immediately.
        let base = (value as u16) << 8;
        for i in 0u16..0xA0 {
            let src = base.wrapping_add(i);
            let byte = self.read8_mmio(src);
            let dst = 0xFE00u16.wrapping_add(i);
            if (dst as usize) < MEMORY_SIZE {
                self.memory[dst as usize] = byte;
            }
        }
        self.memory[0xFF46] = value;
    }
}
