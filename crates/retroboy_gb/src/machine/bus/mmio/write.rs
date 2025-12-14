use super::super::GameBoyBus;

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
                    self.vram_write(addr, value);
                }
            }

            // Work RAM.
            0xC000..=0xDFFF => self.wram_write(addr, value),

            // Echo RAM: writes update both the echo and the underlying WRAM
            // region so that code observing either sees a coherent value.
            0xE000..=0xFDFF => self.wram_echo_write(addr, value),

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
            0xFF47 => {
                // BGP palette register. CPU reads should see the new value
                // immediately, but the DMG pixel pipeline can observe palette
                // restores with a slight delay (tearoom:m3_bgp_change).
                self.memory[addr as usize] = value;
                if matches!(self.model, super::super::GameBoyModel::Dmg | super::super::GameBoyModel::CgbCompat) {
                    let lcd_enabled = (self.memory[0xFF40] & 0x80) != 0;
                    if lcd_enabled {
                        // Treat the MMIO write as occurring between dot ticks:
                        // the current `ppu_cycle_counter` corresponds to the
                        // last dot we processed, and the write should affect
                        // the *next* dot. Tearoom's `m3_bgp_change` relies on
                        // this 1-dot alignment.
                        let next_ppu = self.ppu_cycle_counter.wrapping_add(1);
                        let ly = ((next_ppu / 456) % 154) as u8;
                        let line_cycle = (next_ppu % 456) as u16;
                        let mode3_end = 80u16 + self.ppu_dmg_mode3_len.saturating_sub(1);
                        let in_mode3 = ly < 144 && (80..=mode3_end).contains(&line_cycle);
                        let did_delay = in_mode3;
                        if did_delay {
                            // During DMG mode 3 we observe a small visibility glitch on BGP
                            // writes: the pixel pipeline can see a short-lived OR of the old and
                            // new register values, then the write becomes stable one dot later.
                            self.ppu_dmg_bgp_effective |= value;
                            self.ppu_dmg_bgp_pending = value;
                            self.ppu_dmg_bgp_restore_delay = 1;
                        } else {
                            self.ppu_dmg_bgp_effective = value;
                            self.ppu_dmg_bgp_pending = value;
                            self.ppu_dmg_bgp_restore_delay = 0;
                        }

                        if std::env::var_os("RETROBOY_GB_TRACE_BGP").is_some() {
                            log::info!(
                                "BGP write: value=0x{value:02X} ppu_cycle={} LY={} line_cycle={} in_mode3={} base=0x{:02X} eff=0x{:02X} pending=0x{:02X} delay={} did_delay={} dmg_x={} fifo_len={} out_delay={}",
                                self.ppu_cycle_counter,
                                self.memory[0xFF44],
                                line_cycle,
                                in_mode3,
                                self.ppu_dmg_bgp_base,
                                self.ppu_dmg_bgp_effective,
                                self.ppu_dmg_bgp_pending,
                                self.ppu_dmg_bgp_restore_delay,
                                did_delay,
                                self.ppu_dmg_mode3_x,
                                self.ppu_dmg_fifo_len,
                                self.ppu_dmg_mode3_output_delay,
                            );
                        }
                    } else {
                        self.ppu_dmg_bgp_effective = value;
                        self.ppu_dmg_bgp_pending = value;
                        self.ppu_dmg_bgp_restore_delay = 0;
                    }
                }
            }

            // --- CGB-only registers ---
            0xFF4C | 0xFF56 => {}
            0xFF4D => {
                if self.is_cgb() {
                    self.cgb_key1_write(value);
                }
            }
            0xFF4F => {
                if self.is_cgb() {
                    self.cgb_vbk_write(value);
                }
            }
            0xFF51 => {
                if self.is_cgb() {
                    self.cgb_hdma1 = value;
                }
            }
            0xFF52 => {
                if self.is_cgb() {
                    self.cgb_hdma2 = value & 0xF0;
                }
            }
            0xFF53 => {
                if self.is_cgb() {
                    self.cgb_hdma3 = value & 0x1F;
                }
            }
            0xFF54 => {
                if self.is_cgb() {
                    self.cgb_hdma4 = value & 0xF0;
                }
            }
            0xFF55 => {
                if self.is_cgb() {
                    self.cgb_hdma5_write(value);
                }
            }
            0xFF68 => {
                if self.is_cgb() {
                    self.cgb_bgpi_write(value);
                }
            }
            0xFF69 => {
                if self.is_cgb() {
                    self.cgb_bcpd_write(value);
                }
            }
            0xFF6A => {
                if self.is_cgb() {
                    self.cgb_obpi_write(value);
                }
            }
            0xFF6B => {
                if self.is_cgb() {
                    self.cgb_ocpd_write(value);
                }
            }
            0xFF6C => {
                if self.is_cgb() {
                    self.cgb_opri_write(value);
                }
            }
            0xFF70 => {
                if self.is_cgb() {
                    self.cgb_svbk_write(value);
                }
            }
            0xFF72 => {
                if self.is_cgb() {
                    self.cgb_ff72 = value;
                }
            }
            0xFF73 => {
                if self.is_cgb() {
                    self.cgb_ff73 = value;
                }
            }
            0xFF74 => {
                if self.is_cgb() {
                    self.cgb_ff74 = value;
                }
            }
            0xFF75 => {
                if self.is_cgb() {
                    self.cgb_ff75_write(value);
                }
            }
            0xFF76 | 0xFF77 => {}
            // --- End CGB-only registers ---

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
