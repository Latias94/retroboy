use super::super::{GameBoyBus, MEMORY_SIZE};

impl GameBoyBus {
    pub(super) fn write8_mmio_impl(&mut self, addr: u16, value: u8) {
        match addr {
            // Cartridge ROM / MBC area 0x0000..0x7FFF. On real hardware this is
            // read-only from the CPU's point of view; writes are interpreted
            // by the cartridge's MBC logic.
            0x0000..=0x7FFF => {
                if let Some(cart) = self.cartridge.as_mut() {
                    cart.rom_write(addr, value);
                } else {
                    // No MBC present: CPU writes have no effect.
                }
            }

            // Cartridge RAM area. With an MBC we forward to the mapper; for
            // simple cartridges without an MBC we keep treating this as
            // ordinary battery-backed RAM stored in `memory`.
            0xA000..=0xBFFF => {
                if let Some(cart) = self.cartridge.as_mut() {
                    cart.ram_write(addr, value);
                } else {
                    self.memory[addr as usize] = value;
                }
            }

            // VRAM: writes are ignored while the PPU owns the bus (mode 3).
            0x8000..=0x9FFF => {
                if self.vram_accessible() {
                    self.memory[addr as usize] = value;
                }
            }

            // Echo RAM: writes update both the echo and the underlying WRAM
            // region so that code observing either sees a coherent value.
            0xE000..=0xFDFF => {
                let base = addr.wrapping_sub(0x2000);
                self.memory[addr as usize] = value;
                if (base as usize) < MEMORY_SIZE {
                    self.memory[base as usize] = value;
                }
            }

            // Joypad input (P1).
            0xFF00 => self.write_joyp(value),

            // Serial transfer registers.
            0xFF01 => self.serial.write_sb(value),
            0xFF02 => self.serial.write_sc(value),

            0xFF04..=0xFF07 => self.mmio_write_timer_register(addr, value),

            // OAM DMA.
            0xFF46 => self.do_oam_dma(value),

            // Audio registers.
            0xFF10..=0xFF14 | 0xFF16..=0xFF1E | 0xFF20..=0xFF25 | 0xFF26 => {
                self.write_apu_register(addr, value);
            }

            0xFF40 => self.mmio_write_lcdc(value),
            0xFF41 => self.mmio_write_stat(value),
            0xFF44 => self.mmio_write_ly(),
            0xFF45 => self.mmio_write_lyc(value),

            // Interrupt flags and enable.
            0xFF0F => {
                // Only lower 5 bits are writable; upper bits always read as 1.
                self.if_reg = value & 0x1F;
            }
            0xFFFF => {
                self.ie_reg = value;
            }

            // OAM: writes are ignored while the PPU owns the bus (modes 2/3).
            0xFE00..=0xFE9F => {
                if self.oam_accessible() {
                    self.memory[addr as usize] = value;
                }
            }

            // Writes to the unusable area 0xFEA0..0xFEFF are ignored.
            0xFEA0..=0xFEFF => {}

            _ => {
                self.memory[addr as usize] = value;
            }
        }
    }
}
