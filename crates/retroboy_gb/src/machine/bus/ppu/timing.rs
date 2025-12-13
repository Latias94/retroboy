use super::super::GameBoyBus;

impl GameBoyBus {
    /// Advance the machine by a single CPU T-cycle ("generic" cycle).
    ///
    /// Higher-level callers typically use `tick` to advance by a batch of
    /// cycles, but having this helper makes it easier to move towards a
    /// per-cycle CPU/bus integration model.
    #[allow(dead_code)]
    fn tick_cycle(&mut self) {
        self.cycle_generic();
    }

    /// Generic one-cycle helper used by both `tick_cycle` and `tick`.
    #[inline]
    pub(in super::super) fn cycle_generic(&mut self) {
        // PPU / LCD timing.
        let lcdc = self.memory[0xFF40];
        if (lcdc & 0x80) == 0 {
            // LCD display is disabled: LY stays at 0 and PPU timing is reset.
            self.ppu_cycle_counter = 0;
            self.memory[0xFF44] = 0;
            self.stat_irq_line = false;
            self.update_lcd_status();
            return;
        }

        let old_ppu_counter = self.ppu_cycle_counter;
        self.ppu_cycle_counter = self.ppu_cycle_counter.wrapping_add(1);

        // DMG timing: 154 lines per frame, 456 cycles per line.
        let old_ly = ((old_ppu_counter / 456) % 154) as u8;
        let new_ly = ((self.ppu_cycle_counter / 456) % 154) as u8;
        self.memory[0xFF44] = new_ly;

        // Generate a VBlank interrupt when we cross from LY <= 143 into LY >= 144.
        //
        // On DMG, enabling the STAT mode-2 interrupt also causes a STAT
        // interrupt to be generated at this LY=144 vblank edge (see
        // mooneye "vblank_stat_intr-GS" acceptance test). We model this
        // by additionally requesting INT $48 when STAT's mode-2 select
        // bit is set at the moment vblank begins.
        if old_ly < 144 && new_ly >= 144 {
            // VBlank interrupt.
            self.if_reg |= 0x01;

            // DMG quirk: STAT mode-2 interrupt at vblank start.
            let stat = self.memory[0xFF41];
            if (stat & 0x20) != 0 {
                self.if_reg |= 0x02;
            }

            log::debug!(
                "GB PPU: VBlank edge (LY {}->{}), IF=0x{:02X}, STAT=0x{:02X}",
                old_ly,
                new_ly,
                self.if_reg,
                stat,
            );
        }

        // Update STAT (mode, coincidence) and STAT interrupt line based on the
        // new LY and current line position.
        self.update_lcd_status();
    }

    /// Recompute STAT's mode and LYC=LY flag and update the STAT interrupt line.
    ///
    /// This helper is called from `cycle_generic` (after we've advanced the PPU timing)
    /// and from writes to LCDC/STAT/LY/LYC. It implements the basic Pandocs
    /// semantics for:
    /// - STAT bits 0-1 (PPU mode, read-only)
    /// - STAT bit 2   (LYC == LY flag, read-only)
    /// - STAT bits 3-6 (mode/LYC interrupt selects, read/write)
    /// - INT $48 (STAT interrupt) as a rising edge on the logically ORed line
    ///   of enabled sources.
    pub(super) fn update_lcd_status(&mut self) {
        let lcdc = self.memory[0xFF40];
        let ly = self.memory[0xFF44];
        let lyc = self.memory[0xFF45];

        let lcd_enabled = (lcdc & 0x80) != 0;

        // Derive the current PPU mode from LY and the position within the line.
        let mode = if !lcd_enabled {
            // When LCD is disabled, the PPU is effectively off and STAT reports mode 0.
            0
        } else if ly >= 144 {
            // VBlank (mode 1) for LY >= 144.
            1
        } else {
            let line_cycle = (self.ppu_cycle_counter % 456) as u16;
            if line_cycle < 80 {
                2 // OAM search
            } else if line_cycle < 80 + 172 {
                3 // LCD transfer
            } else {
                0 // HBlank
            }
        };

        let coincidence = ly == lyc;

        // Update STAT bits 0-2 while preserving bits 3-7 (interrupt enables and unused bit 7).
        let mut stat = self.memory[0xFF41];
        stat &= !0x07;
        stat |= mode as u8;
        if coincidence {
            stat |= 0x04;
        }
        self.memory[0xFF41] = stat;

        // If the LCD is disabled, STAT sources are effectively inactive; we still report mode 0
        // and LYC==LY in the register, but the STAT interrupt line stays low.
        if !lcd_enabled {
            self.stat_irq_line = false;
            return;
        }

        // Compute the logically ORed STAT interrupt line from enabled sources.
        let prev_line = self.stat_irq_line;
        let mut line = false;
        let lyc_int_en = (stat & 0x40) != 0;
        let mode2_en = (stat & 0x20) != 0;
        let mode1_en = (stat & 0x10) != 0;
        let mode0_en = (stat & 0x08) != 0;

        if lyc_int_en && coincidence {
            line = true;
        }
        if mode2_en && mode == 2 {
            line = true;
        }
        if mode1_en && mode == 1 {
            line = true;
        }
        if mode0_en && mode == 0 {
            line = true;
        }

        self.stat_irq_line = line;
        if !prev_line && line {
            // Rising edge on the STAT interrupt line.
            self.if_reg |= 0x02;
            log::debug!(
                "GB PPU: STAT IRQ rising edge (STAT=0x{:02X} LY={} mode={} IF=0x{:02X})",
                stat,
                ly,
                mode,
                self.if_reg
            );
        }
    }
}
