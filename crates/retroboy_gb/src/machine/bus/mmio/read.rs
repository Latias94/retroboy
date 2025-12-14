use super::super::GameBoyBus;

impl GameBoyBus {
    pub(super) fn read8_mmio_impl(&mut self, addr: u16) -> u8 {
        match addr {
            // Cartridge ROM area: either flat ROM (no MBC) or routed through
            // the active mapper (currently only MBC1/MBC3).
            0x0000..=0x7FFF => {
                if let Some(cart) = &self.cartridge {
                    cart.rom_read(addr)
                } else {
                    self.memory[addr as usize]
                }
            }

            // VRAM region.
            0x8000..=0x9FFF => {
                if self.vram_accessible() {
                    self.vram_read(addr)
                } else {
                    // Undefined data when VRAM is blocked; most hardware
                    // observations see 0xFF.
                    0xFF
                }
            }

            // Work RAM.
            0xC000..=0xDFFF => self.wram_read(addr),

            // Cartridge RAM area 0xA000..0xBFFF. When a mapper is present we
            // route accesses through it; otherwise we fall back to internal
            // memory so simple "no MBC + RAM" cartridges still work.
            0xA000..=0xBFFF => {
                if let Some(cart) = &self.cartridge {
                    cart.ram_read(addr)
                } else {
                    self.memory[addr as usize]
                }
            }

            // Joypad input (P1).
            0xFF00 => self.read_joyp(),

            // Echo RAM: 0xE000..0xFDFF mirrors 0xC000..0xDDFF.
            0xE000..=0xFDFF => self.wram_echo_read(addr),

            // Serial transfer registers.
            0xFF01 => self.serial.sb,
            0xFF02 => self.serial.sc,

            0xFF04..=0xFF07 => self.mmio_read_timer_register(addr),

            // Interrupt flags and enable.
            0xFF0F => self.if_reg | 0b1110_0000,
            0xFFFF => self.ie_reg,

            // --- CGB-only registers ---
            0xFF4C | 0xFF56 => 0xFF,
            0xFF4D => {
                if self.is_cgb() {
                    self.cgb_key1_read()
                } else {
                    0xFF
                }
            }
            0xFF4F => {
                if self.is_cgb() {
                    self.cgb_vbk_read()
                } else {
                    0xFF
                }
            }
            0xFF51 => {
                if self.is_cgb() {
                    self.cgb_hdma1
                } else {
                    0xFF
                }
            }
            0xFF52 => {
                if self.is_cgb() {
                    self.cgb_hdma2
                } else {
                    0xFF
                }
            }
            0xFF53 => {
                if self.is_cgb() {
                    self.cgb_hdma3
                } else {
                    0xFF
                }
            }
            0xFF54 => {
                if self.is_cgb() {
                    self.cgb_hdma4
                } else {
                    0xFF
                }
            }
            0xFF55 => {
                if self.is_cgb() {
                    self.cgb_hdma5_read()
                } else {
                    0xFF
                }
            }
            0xFF68 => {
                if self.is_cgb() {
                    self.cgb_bgpi_read()
                } else {
                    0xFF
                }
            }
            0xFF69 => {
                if self.is_cgb() {
                    self.cgb_bcpd_read()
                } else {
                    0xFF
                }
            }
            0xFF6A => {
                if self.is_cgb() {
                    self.cgb_obpi_read()
                } else {
                    0xFF
                }
            }
            0xFF6B => {
                if self.is_cgb() {
                    self.cgb_ocpd_read()
                } else {
                    0xFF
                }
            }
            0xFF6C => {
                if self.is_cgb() {
                    self.cgb_opri_read()
                } else {
                    0xFF
                }
            }
            0xFF70 => {
                if self.is_cgb() {
                    self.cgb_svbk_read()
                } else {
                    0xFF
                }
            }
            0xFF72 => {
                if self.is_cgb() {
                    self.cgb_ff72
                } else {
                    0xFF
                }
            }
            0xFF73 => {
                if self.is_cgb() {
                    self.cgb_ff73
                } else {
                    0xFF
                }
            }
            0xFF74 => {
                if self.is_cgb() {
                    self.cgb_ff74
                } else {
                    0xFF
                }
            }
            0xFF75 => {
                if self.is_cgb() {
                    self.cgb_ff75_read()
                } else {
                    0xFF
                }
            }
            0xFF76 | 0xFF77 => {
                if self.is_cgb() {
                    0x00
                } else {
                    0xFF
                }
            }
            // --- End CGB-only registers ---

            // OAM region 0xFE00..0xFE9F: CPU access is restricted by PPU mode.
            0xFE00..=0xFE9F => {
                if self.oam_accessible() {
                    self.memory[addr as usize]
                } else {
                    0xFF
                }
            }

            // Unusable area 0xFEA0..0xFEFF reads as 0xFF.
            0xFEA0..=0xFEFF => 0xFF,

            _ => self.memory[addr as usize],
        }
    }
}
