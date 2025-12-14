use super::super::GameBoyBus;
use crate::{SCREEN_HEIGHT, SCREEN_WIDTH};
use crate::machine::GameBoyModel;

impl GameBoyBus {
    fn cgb_palette_rgb(pal_data: &[u8; 0x40], palette: u8, color_index: u8) -> (u8, u8, u8) {
        let palette = palette & 0x07;
        let color_index = color_index & 0x03;
        let base = (palette as usize) * 8 + (color_index as usize) * 2;
        let lo = pal_data[base] as u16;
        let hi = pal_data[base + 1] as u16;
        let raw = lo | (hi << 8);
        let r5 = (raw & 0x1F) as u8;
        let g5 = ((raw >> 5) & 0x1F) as u8;
        let b5 = ((raw >> 10) & 0x1F) as u8;
        let to8 = |v5: u8| (v5 << 3) | (v5 >> 2);
        (to8(r5), to8(g5), to8(b5))
    }

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
            self.ppu_line_captured.fill(false);
            self.ppu_frame_valid = false;
            self.ppu_dmg_fb_valid = false;
            self.ppu_cgb_fb_valid = false;
            self.ppu_dmg_window_line = 0;
            self.ppu_dmg_window_drawn_this_line = false;
            self.ppu_cgb_window_line = 0;
            self.ppu_cgb_window_drawn_this_line = false;
            self.ppu_cgb_fetch_using_window = false;
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

        // Start of a new frame: clear scanline latches.
        if old_ly != new_ly && new_ly == 0 {
            self.ppu_line_captured.fill(false);
            self.ppu_dmg_fb_valid = false;
            self.ppu_dmg_window_line = 0;
            self.ppu_dmg_window_drawn_this_line = false;
            self.ppu_cgb_window_line = 0;
            self.ppu_cgb_window_drawn_this_line = false;
            self.ppu_cgb_fetch_using_window = false;
        }

        // Latch a per-scanline register snapshot at the start of mode 3
        // (line cycle 80). This is enough for line-based CGB test ROMs like
        // cgb-acid2 which only perform writes during mode 2.
        let line_cycle = (self.ppu_cycle_counter % 456) as u16;
        if new_ly < 144 && line_cycle == 80 {
            let idx = new_ly as usize;
            self.ppu_line_captured[idx] = true;
            self.ppu_line_lcdc[idx] = self.memory[0xFF40];
            self.ppu_line_scy[idx] = self.memory[0xFF42];
            self.ppu_line_scx[idx] = self.memory[0xFF43];
            self.ppu_line_bgp[idx] = self.memory[0xFF47];
            self.ppu_line_obp0[idx] = self.memory[0xFF48];
            self.ppu_line_obp1[idx] = self.memory[0xFF49];
            self.ppu_line_wy[idx] = self.memory[0xFF4A];
            self.ppu_line_wx[idx] = self.memory[0xFF4B];
            self.ppu_line_bgpd[idx] = self.cgb_bgpd;
            self.ppu_line_obpd[idx] = self.cgb_obpd;
        }

        // DMG dot-based pixel output during mode 3.
        if matches!(self.model(), GameBoyModel::Dmg | GameBoyModel::CgbCompat) && new_ly < 144 {
            if line_cycle == 0 {
                self.ppu_dmg_window_drawn_this_line = false;
            }

            if line_cycle == 80 {
                self.ppu_dmg_mode3_begin(new_ly);
            }

            let mode3_end = 80u16 + self.ppu_dmg_mode3_len.saturating_sub(1);
            if (80..=mode3_end).contains(&line_cycle) {
                self.ppu_dmg_mode3_tick(new_ly);
            }

            if line_cycle == 455 && self.ppu_dmg_window_drawn_this_line {
                self.ppu_dmg_window_line = self.ppu_dmg_window_line.wrapping_add(1);
            }
        }

        // CGB dot-based background output during mode 3 (scaffold).
        if matches!(self.model(), GameBoyModel::Cgb) && new_ly < 144 {
            if line_cycle == 0 {
                self.ppu_cgb_window_drawn_this_line = false;
                self.ppu_cgb_fetch_using_window = false;
            }

            if line_cycle == 80 {
                self.ppu_cgb_mode3_begin(new_ly);
            }

            let mode3_end = 80u16 + self.ppu_dmg_mode3_len.saturating_sub(1);
            if (80..=mode3_end).contains(&line_cycle) {
                self.ppu_cgb_mode3_tick(new_ly);
            }

            if line_cycle == 455 && self.ppu_cgb_window_drawn_this_line {
                self.ppu_cgb_window_line = self.ppu_cgb_window_line.wrapping_add(1);
            }
        }

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

            // Commit the last completed frame's line snapshots for the renderer.
            // Our outer stepping is not synchronized to vblank, so the renderer
            // should use a stable snapshot rather than partially-captured
            // scanlines from the current in-progress frame.
            for idx in 0..144usize {
                self.ppu_frame_lcdc[idx] = self.ppu_line_lcdc[idx];
                self.ppu_frame_scy[idx] = self.ppu_line_scy[idx];
                self.ppu_frame_scx[idx] = self.ppu_line_scx[idx];
                self.ppu_frame_bgp[idx] = self.ppu_line_bgp[idx];
                self.ppu_frame_obp0[idx] = self.ppu_line_obp0[idx];
                self.ppu_frame_obp1[idx] = self.ppu_line_obp1[idx];
                self.ppu_frame_wy[idx] = self.ppu_line_wy[idx];
                self.ppu_frame_wx[idx] = self.ppu_line_wx[idx];
                self.ppu_frame_bgpd[idx] = self.ppu_line_bgpd[idx];
                self.ppu_frame_obpd[idx] = self.ppu_line_obpd[idx];
            }
            self.ppu_frame_valid = true;
            self.ppu_dmg_fb_valid = true;
            if matches!(self.model(), GameBoyModel::Cgb) {
                self.ppu_cgb_fb_valid = true;
            }

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

        // CGB HBlank DMA: transfer one 0x10-byte block at the start of each HBlank.
        if self.is_cgb() && self.cgb_hdma_active {
            let ly = self.memory[0xFF44];
            if ly < 144 {
                let line_cycle = (self.ppu_cycle_counter % 456) as u16;
                let hblank_start = 80u16 + self.ppu_dmg_mode3_len;
                if line_cycle == hblank_start {
                    self.cgb_hdma_hblank_tick();
                }
            }
        }
    }

    fn ppu_dmg_mode3_begin(&mut self, ly: u8) {
        // Latch baseline state at start of mode 3. We intentionally keep this
        // conservative for now (enough for `m3_bgp_change`); later tearoom
        // tests require more nuanced mid-mode-3 latching rules.
        self.ppu_dmg_lcdc_latched = self.memory[0xFF40];
        self.ppu_dmg_scx_latched = self.memory[0xFF43];
        self.ppu_dmg_scy_latched = self.memory[0xFF42];
        self.ppu_dmg_bgp_base = self.memory[0xFF47];
        self.ppu_dmg_bgp_effective = self.ppu_dmg_bgp_base;
        self.ppu_dmg_bgp_pending = self.ppu_dmg_bgp_base;
        self.ppu_dmg_bgp_restore_delay = 0;

        let delay = std::env::var("RETROBOY_GB_DMG_MODE3_DELAY")
            .ok()
            .and_then(|v| v.parse::<u8>().ok())
            .unwrap_or(12);
        self.ppu_dmg_mode3_output_delay = delay;
        self.ppu_dmg_mode3_x = 0;
        self.ppu_dmg_mode3_discard = self.ppu_dmg_scx_latched & 0x07;
        self.ppu_dmg_mode3_len = 172u16 + self.ppu_dmg_mode3_discard as u16;
        self.ppu_dmg_fifo_head = 0;
        self.ppu_dmg_fifo_len = 0;
        self.ppu_dmg_fetch_phase = 0;
        self.ppu_dmg_fetch_tile_x = (self.ppu_dmg_scx_latched >> 3) & 0x1F;

        let bg_y = (ly as u16).wrapping_add(self.ppu_dmg_scy_latched as u16);
        self.ppu_dmg_fetch_tile_y = ((bg_y >> 3) as u8) & 0x1F;
        self.ppu_dmg_fetch_tile_line = (bg_y as u8) & 0x07;
    }

    fn ppu_cgb_fifo_push8(&mut self, lo: u8, hi: u8, palette: u8, priority: bool, xflip: bool) {
        if self.ppu_cgb_fifo_len > 8 {
            return;
        }
        let start = (self.ppu_cgb_fifo_head.wrapping_add(self.ppu_cgb_fifo_len)) & 0x0F;
        let pri_bit = if priority { 1u8 } else { 0u8 };
        for i in 0..8u8 {
            let bit = if xflip { i } else { 7 - i };
            let low = (lo >> bit) & 0x01;
            let high = (hi >> bit) & 0x01;
            let color_index = (high << 1) | low;
            let entry = (color_index & 0x03) | ((palette & 0x07) << 2) | (pri_bit << 5);
            let idx = (start.wrapping_add(i)) & 0x0F;
            self.ppu_cgb_fifo[idx as usize] = entry;
        }
        self.ppu_cgb_fifo_len = self.ppu_cgb_fifo_len.saturating_add(8);
    }

    fn ppu_cgb_fifo_pop(&mut self) -> Option<u8> {
        if self.ppu_cgb_fifo_len == 0 {
            return None;
        }
        let v = self.ppu_cgb_fifo[self.ppu_cgb_fifo_head as usize];
        self.ppu_cgb_fifo_head = (self.ppu_cgb_fifo_head.wrapping_add(1)) & 0x0F;
        self.ppu_cgb_fifo_len = self.ppu_cgb_fifo_len.wrapping_sub(1);
        Some(v)
    }

    fn ppu_cgb_mode3_begin(&mut self, ly: u8) {
        let scx = self.memory[0xFF43];
        let scy = self.memory[0xFF42];

        self.ppu_cgb_fetch_using_window = false;
        self.ppu_cgb_oam_scan(ly);

        let delay = std::env::var("RETROBOY_GB_CGB_MODE3_DELAY")
            .ok()
            .and_then(|v| v.parse::<u8>().ok())
            .unwrap_or(12);
        self.ppu_cgb_mode3_output_delay = delay;
        self.ppu_cgb_mode3_x = 0;
        self.ppu_cgb_mode3_discard = scx & 0x07;

        // Keep STAT/HBlank timings consistent with the DMG model for now.
        self.ppu_dmg_mode3_len = 172u16 + self.ppu_cgb_mode3_discard as u16;

        self.ppu_cgb_fifo_head = 0;
        self.ppu_cgb_fifo_len = 0;
        self.ppu_cgb_fetch_phase = 0;
        self.ppu_cgb_fetch_tile_x = (scx >> 3) & 0x1F;

        let bg_y = (ly as u16).wrapping_add(scy as u16);
        self.ppu_cgb_fetch_tile_y = ((bg_y >> 3) as u8) & 0x1F;
        self.ppu_cgb_fetch_tile_line = (bg_y as u8) & 0x07;
    }

    fn ppu_cgb_oam_scan(&mut self, ly: u8) {
        self.ppu_cgb_sprite_count = 0;

        let lcdc = self.memory[0xFF40];
        if (lcdc & 0x02) == 0 {
            return;
        }

        let obj_8x16 = (lcdc & 0x04) != 0;
        let sprite_height: i16 = if obj_8x16 { 16 } else { 8 };

        for i in 0..40u16 {
            if self.ppu_cgb_sprite_count == 10 {
                break;
            }

            let oam_base = 0xFE00u16.wrapping_add(i * 4);
            let sy = self.memory[oam_base as usize] as i16 - 16;
            let sx = self.memory[oam_base.wrapping_add(1) as usize] as i16 - 8;
            let tile_index = self.memory[oam_base.wrapping_add(2) as usize];
            let attrs = self.memory[oam_base.wrapping_add(3) as usize];

            let row = (ly as i16).wrapping_sub(sy);
            if row < 0 || row >= sprite_height {
                continue;
            }

            let flip_y = (attrs & 0x40) != 0;
            let src_row = if flip_y {
                (sprite_height - 1 - row) as u16
            } else {
                row as u16
            };

            let base_tile = if obj_8x16 { tile_index & 0xFE } else { tile_index };
            let (tile, tile_row) = if obj_8x16 && src_row >= 8 {
                (base_tile.wrapping_add(1), src_row - 8)
            } else {
                (base_tile, src_row)
            };

            let row_addr = 0x8000u16
                .wrapping_add((tile as u16) * 16)
                .wrapping_add(tile_row * 2);
            let bank = if (attrs & 0x08) != 0 { 1 } else { 0 };
            let lo = self.ppu_vram_read(bank, row_addr);
            let hi = self.ppu_vram_read(bank, row_addr.wrapping_add(1));

            let idx = self.ppu_cgb_sprite_count as usize;
            self.ppu_cgb_sprites[idx] = super::super::PpuCgbSpriteLine {
                oam_index: i as u8,
                x: sx,
                attrs,
                lo,
                hi,
            };
            self.ppu_cgb_sprite_order[idx] = idx as u8;
            self.ppu_cgb_sprite_count = self.ppu_cgb_sprite_count.wrapping_add(1);
        }

        // Priority mode: when OPRI bit0 is set, use DMG-style x-ordering.
        if self.cgb_opri() != 0 {
            let n = self.ppu_cgb_sprite_count as usize;
            // Insertion sort is fine for <=10 elements.
            for i in 1..n {
                let key = self.ppu_cgb_sprite_order[i];
                let key_sprite = self.ppu_cgb_sprites[key as usize];
                let mut j = i;
                while j > 0 {
                    let prev = self.ppu_cgb_sprite_order[j - 1];
                    let prev_sprite = self.ppu_cgb_sprites[prev as usize];
                    let prev_key = (prev_sprite.x, prev_sprite.oam_index);
                    let key_key = (key_sprite.x, key_sprite.oam_index);
                    if prev_key <= key_key {
                        break;
                    }
                    self.ppu_cgb_sprite_order[j] = prev;
                    j -= 1;
                }
                self.ppu_cgb_sprite_order[j] = key;
            }
        }
    }

    fn ppu_cgb_sprite_pixel(
        &self,
        x: u8,
        bg_color_index: u8,
        bg_priority: bool,
    ) -> Option<(u8, u8, u8)> {
        let lcdc = self.memory[0xFF40];
        if (lcdc & 0x02) == 0 {
            return None;
        }

        let n = self.ppu_cgb_sprite_count as usize;
        for i in 0..n {
            let idx = self.ppu_cgb_sprite_order[i] as usize;
            let s = self.ppu_cgb_sprites[idx];
            let col = x as i16 - s.x;
            if col < 0 || col >= 8 {
                continue;
            }

            let flip_x = (s.attrs & 0x20) != 0;
            let src_x = if flip_x { 7 - col } else { col } as u8;
            let bit = 7 - src_x;
            let low = (s.lo >> bit) & 0x01;
            let high = (s.hi >> bit) & 0x01;
            let color_index = (high << 1) | low;
            if color_index == 0 {
                continue;
            }

            let obj_to_bg = (s.attrs & 0x80) != 0;
            if (obj_to_bg || bg_priority) && bg_color_index != 0 {
                continue;
            }

            let palette = s.attrs & 0x07;
            let (r, g, b) = Self::cgb_palette_rgb(&self.cgb_obpd, palette, color_index);
            return Some((r, g, b));
        }

        None
    }

    fn ppu_cgb_mode3_tick(&mut self, ly: u8) {
        // Window switching: once the window starts on a scanline, it replaces
        // BG output for the remainder of that line. This is a simplified model
        // that flushes the FIFO at the window boundary.
        let lcdc = self.memory[0xFF40];
        // CGB quirk: LCDC bit 0 does NOT disable BG/Window rendering in CGB mode
        // (it controls BG/Window priority vs OBJ). We still use it to gate the
        // BG "priority" bit when mixing sprites.
        let bg_master_priority = (lcdc & 0x01) != 0;
        let win_enabled = (lcdc & 0x20) != 0;
        let wy = self.memory[0xFF4A];
        let wx = self.memory[0xFF4B];

        let win_start_x = (wx as i16) - 7;
        let win_start_x = win_start_x.max(0);
        let win_visible = win_enabled && ly >= wy && win_start_x < SCREEN_WIDTH as i16;

        let x_now = self.ppu_cgb_mode3_x as i16;
        let should_use_window = win_visible && x_now >= win_start_x;
        if should_use_window && !self.ppu_cgb_fetch_using_window {
            self.ppu_cgb_fetch_using_window = true;
            self.ppu_cgb_window_drawn_this_line = true;

            // Reset the fetcher to start from the left edge of the window.
            let win_x = (x_now - win_start_x).max(0) as u8;
            self.ppu_cgb_fifo_head = 0;
            self.ppu_cgb_fifo_len = 0;
            self.ppu_cgb_fetch_phase = 0;
            self.ppu_cgb_fetch_tile_x = (win_x >> 3) & 0x1F;
            // If the window starts mid-tile (WX not aligned), skip the leading pixels.
            self.ppu_cgb_mode3_discard = win_x & 0x07;
            let win_y = self.ppu_cgb_window_line;
            self.ppu_cgb_fetch_tile_y = ((win_y >> 3) as u8) & 0x1F;
            self.ppu_cgb_fetch_tile_line = win_y & 0x07;
        }

        // Output stage.
        if self.ppu_cgb_mode3_output_delay != 0 {
            self.ppu_cgb_mode3_output_delay = self.ppu_cgb_mode3_output_delay.wrapping_sub(1);
        } else if let Some(entry) = self.ppu_cgb_fifo_pop() {
            if self.ppu_cgb_mode3_discard != 0 {
                self.ppu_cgb_mode3_discard = self.ppu_cgb_mode3_discard.wrapping_sub(1);
            } else if (self.ppu_cgb_mode3_x as usize) < SCREEN_WIDTH && (ly as usize) < SCREEN_HEIGHT
            {
                let x = self.ppu_cgb_mode3_x as usize;
                let pixel_index = (ly as usize) * SCREEN_WIDTH + x;
                let out_idx = pixel_index * 3;

                let entry_color_index = entry & 0x03;
                let entry_palette = (entry >> 2) & 0x07;
                let entry_priority = (entry & (1 << 5)) != 0;

                let bg_color_index = entry_color_index;
                let bg_palette = entry_palette;
                let bg_priority = if bg_master_priority {
                    entry_priority
                } else {
                    false
                };

                let (r, g, b) = Self::cgb_palette_rgb(&self.cgb_bgpd, bg_palette, bg_color_index);
                self.ppu_cgb_fb[out_idx] = r;
                self.ppu_cgb_fb[out_idx + 1] = g;
                self.ppu_cgb_fb[out_idx + 2] = b;

                self.ppu_cgb_bg_color_index[pixel_index] = bg_color_index;
                self.ppu_cgb_bg_priority[pixel_index] = bg_priority as u8;

                // Sprite overlay.
                if let Some((sr, sg, sb)) =
                    self.ppu_cgb_sprite_pixel(self.ppu_cgb_mode3_x, bg_color_index, bg_priority)
                {
                    self.ppu_cgb_fb[out_idx] = sr;
                    self.ppu_cgb_fb[out_idx + 1] = sg;
                    self.ppu_cgb_fb[out_idx + 2] = sb;
                }

                self.ppu_cgb_mode3_x = self.ppu_cgb_mode3_x.wrapping_add(1);
            }
        }

        // Fetcher stage.
        if self.ppu_cgb_fifo_len > 8 {
            self.ppu_cgb_fetch_phase = 0;
            return;
        }

        let phase = self.ppu_cgb_fetch_phase;
        self.ppu_cgb_fetch_phase = self.ppu_cgb_fetch_phase.wrapping_add(1) & 0x07;
        match phase {
            0 => {
                let lcdc = self.memory[0xFF40];
                let map_base: u16 = if self.ppu_cgb_fetch_using_window {
                    if (lcdc & 0x40) != 0 { 0x9C00 } else { 0x9800 }
                } else if (lcdc & 0x08) != 0 {
                    0x9C00
                } else {
                    0x9800
                };
                let addr = map_base
                    .wrapping_add((self.ppu_cgb_fetch_tile_y as u16) * 32)
                    .wrapping_add(self.ppu_cgb_fetch_tile_x as u16);
                self.ppu_cgb_fetch_tile_id = self.ppu_vram_read(0, addr);
                self.ppu_cgb_fetch_tile_attr = self.ppu_vram_read(1, addr);
            }
            2 => {
                let lcdc = self.memory[0xFF40];
                let tile_data_unsigned = (lcdc & 0x10) != 0;
                let tile_index = self.ppu_cgb_fetch_tile_id;
                let tile_base = if tile_data_unsigned {
                    0x8000u16.wrapping_add((tile_index as u16) * 16)
                } else {
                    let idx_signed = tile_index as i8 as i32;
                    (0x9000i32 + idx_signed * 16) as u16
                };

                let attr = self.ppu_cgb_fetch_tile_attr;
                let yflip = (attr & 0x40) != 0;
                let bank = if (attr & 0x08) != 0 { 1 } else { 0 };
                let fine_y = if yflip {
                    7u16.wrapping_sub(self.ppu_cgb_fetch_tile_line as u16)
                } else {
                    self.ppu_cgb_fetch_tile_line as u16
                };

                let row_addr = tile_base.wrapping_add(fine_y * 2);
                self.ppu_cgb_fetch_tile_lo = self.ppu_vram_read(bank, row_addr);
            }
            4 => {
                let lcdc = self.memory[0xFF40];
                let tile_data_unsigned = (lcdc & 0x10) != 0;
                let tile_index = self.ppu_cgb_fetch_tile_id;
                let tile_base = if tile_data_unsigned {
                    0x8000u16.wrapping_add((tile_index as u16) * 16)
                } else {
                    let idx_signed = tile_index as i8 as i32;
                    (0x9000i32 + idx_signed * 16) as u16
                };

                let attr = self.ppu_cgb_fetch_tile_attr;
                let yflip = (attr & 0x40) != 0;
                let bank = if (attr & 0x08) != 0 { 1 } else { 0 };
                let fine_y = if yflip {
                    7u16.wrapping_sub(self.ppu_cgb_fetch_tile_line as u16)
                } else {
                    self.ppu_cgb_fetch_tile_line as u16
                };

                let row_addr = tile_base.wrapping_add(fine_y * 2).wrapping_add(1);
                self.ppu_cgb_fetch_tile_hi = self.ppu_vram_read(bank, row_addr);
            }
            6 => {
                let attr = self.ppu_cgb_fetch_tile_attr;
                let palette = attr & 0x07;
                let xflip = (attr & 0x20) != 0;
                let priority = (attr & 0x80) != 0;
                self.ppu_cgb_fifo_push8(
                    self.ppu_cgb_fetch_tile_lo,
                    self.ppu_cgb_fetch_tile_hi,
                    palette,
                    priority,
                    xflip,
                );
                self.ppu_cgb_fetch_tile_x = self.ppu_cgb_fetch_tile_x.wrapping_add(1) & 0x1F;
            }
            _ => {}
        }
    }

    fn ppu_dmg_fifo_push8(&mut self, lo: u8, hi: u8) {
        if self.ppu_dmg_fifo_len > 8 {
            return;
        }
        let start = (self.ppu_dmg_fifo_head.wrapping_add(self.ppu_dmg_fifo_len)) & 0x0F;
        for i in 0..8u8 {
            let bit = 7 - i;
            let low = (lo >> bit) & 0x01;
            let high = (hi >> bit) & 0x01;
            let color_index = (high << 1) | low;
            let idx = (start.wrapping_add(i)) & 0x0F;
            self.ppu_dmg_fifo[idx as usize] = color_index;
        }
        self.ppu_dmg_fifo_len = self.ppu_dmg_fifo_len.saturating_add(8);
    }

    fn ppu_dmg_fifo_pop(&mut self) -> Option<u8> {
        if self.ppu_dmg_fifo_len == 0 {
            return None;
        }
        let v = self.ppu_dmg_fifo[self.ppu_dmg_fifo_head as usize];
        self.ppu_dmg_fifo_head = (self.ppu_dmg_fifo_head.wrapping_add(1)) & 0x0F;
        self.ppu_dmg_fifo_len = self.ppu_dmg_fifo_len.wrapping_sub(1);
        Some(v)
    }

    fn ppu_dmg_mode3_tick(&mut self, ly: u8) {
        // Output stage first (so fetcher pushes become visible next cycle).
        if self.ppu_dmg_mode3_output_delay != 0 {
            self.ppu_dmg_mode3_output_delay = self.ppu_dmg_mode3_output_delay.wrapping_sub(1);
        } else if let Some(color_index) = self.ppu_dmg_fifo_pop() {
            if self.ppu_dmg_mode3_discard != 0 {
                self.ppu_dmg_mode3_discard = self.ppu_dmg_mode3_discard.wrapping_sub(1);
            } else if (self.ppu_dmg_mode3_x as usize) < SCREEN_WIDTH && (ly as usize) < SCREEN_HEIGHT
            {
                let x = self.ppu_dmg_mode3_x as usize;
                let bgp = self.ppu_dmg_bgp_effective;
                let palette_bits = (bgp >> (color_index * 2)) & 0x03;
                let (r, g, b) = match self.model() {
                    GameBoyModel::CgbCompat => match palette_bits {
                        // Mealybug tearoom "CGB compatibility mode" palette mapping.
                        0 => (0xFF, 0xFF, 0xFF),
                        1 => (0x7B, 0xFF, 0x31),
                        2 => (0x00, 0x63, 0xC6),
                        _ => (0x00, 0x00, 0x00),
                    },
                    _ => {
                        let shade = match palette_bits {
                            0 => 0xFF,
                            1 => 0xAA,
                            2 => 0x55,
                            _ => 0x00,
                        };
                        (shade, shade, shade)
                    }
                };

                let pixel_index = (ly as usize) * SCREEN_WIDTH + x;
                self.ppu_dmg_bg_color_index[pixel_index] = color_index;
                let out_idx = pixel_index * 3;
                self.ppu_dmg_fb[out_idx] = r;
                self.ppu_dmg_fb[out_idx + 1] = g;
                self.ppu_dmg_fb[out_idx + 2] = b;
                self.ppu_dmg_mode3_x = self.ppu_dmg_mode3_x.wrapping_add(1);
            }
        }

        // Apply any pending palette update after a 1-dot delay. This models the
        // DMG's small BGP write visibility lag/glitch in mode 3.
        if self.ppu_dmg_bgp_restore_delay != 0 {
            self.ppu_dmg_bgp_restore_delay = self.ppu_dmg_bgp_restore_delay.wrapping_sub(1);
            if self.ppu_dmg_bgp_restore_delay == 0 {
                self.ppu_dmg_bgp_effective = self.ppu_dmg_bgp_pending;
            }
        }

        // Fetcher stage.
        if self.ppu_dmg_fifo_len > 8 {
            // When the FIFO has more than 8 pixels buffered, the DMG fetcher
            // is stalled. Once it resumes, it starts a fresh 8-cycle tile
            // fetch sequence rather than continuing from the previous phase.
            self.ppu_dmg_fetch_phase = 0;
            return;
        }

        let phase = self.ppu_dmg_fetch_phase;
        self.ppu_dmg_fetch_phase = self.ppu_dmg_fetch_phase.wrapping_add(1) & 0x07;
        match phase {
            0 => {
                let lcdc = self.ppu_dmg_lcdc_latched;
                let map_base: u16 = if (lcdc & 0x08) != 0 { 0x9C00 } else { 0x9800 };
                let addr = map_base
                    .wrapping_add((self.ppu_dmg_fetch_tile_y as u16) * 32)
                    .wrapping_add(self.ppu_dmg_fetch_tile_x as u16);
                self.ppu_dmg_fetch_tile_id = self.memory[addr as usize];
            }
            2 => {
                let lcdc = self.ppu_dmg_lcdc_latched;
                let tile_data_unsigned = (lcdc & 0x10) != 0;
                let tile_index = self.ppu_dmg_fetch_tile_id;
                let tile_base = if tile_data_unsigned {
                    0x8000u16.wrapping_add((tile_index as u16) * 16)
                } else {
                    let idx_signed = tile_index as i8 as i32;
                    (0x9000i32 + idx_signed * 16) as u16
                };
                let row_addr = tile_base.wrapping_add((self.ppu_dmg_fetch_tile_line as u16) * 2);
                self.ppu_dmg_fetch_tile_lo = self.memory[row_addr as usize];
            }
            4 => {
                let lcdc = self.ppu_dmg_lcdc_latched;
                let tile_data_unsigned = (lcdc & 0x10) != 0;
                let tile_index = self.ppu_dmg_fetch_tile_id;
                let tile_base = if tile_data_unsigned {
                    0x8000u16.wrapping_add((tile_index as u16) * 16)
                } else {
                    let idx_signed = tile_index as i8 as i32;
                    (0x9000i32 + idx_signed * 16) as u16
                };
                let row_addr = tile_base.wrapping_add((self.ppu_dmg_fetch_tile_line as u16) * 2);
                self.ppu_dmg_fetch_tile_hi = self.memory[row_addr.wrapping_add(1) as usize];
            }
            6 => {
                let lo = self.ppu_dmg_fetch_tile_lo;
                let hi = self.ppu_dmg_fetch_tile_hi;
                self.ppu_dmg_fifo_push8(lo, hi);
                self.ppu_dmg_fetch_tile_x = (self.ppu_dmg_fetch_tile_x.wrapping_add(1)) & 0x1F;
            }
            _ => {}
        }
    }

    fn ppu_dmg_render_pixel(&mut self, ly: u8, x: usize) {
        if x >= SCREEN_WIDTH || (ly as usize) >= SCREEN_HEIGHT {
            return;
        }

        let lcdc = self.memory[0xFF40];
        let scy = self.memory[0xFF42];
        let scx = self.memory[0xFF43];
        let bgp = self.memory[0xFF47];
        let obp0 = self.memory[0xFF48];
        let obp1 = self.memory[0xFF49];
        let wy = self.memory[0xFF4A];
        let wx = self.memory[0xFF4B];

        let dmg_bg_enabled = (lcdc & 0x01) != 0;

        let dmg_shade = |palette_bits: u8| -> u8 {
            match palette_bits & 0x03 {
                0 => 0xFF,
                1 => 0xAA,
                2 => 0x55,
                _ => 0x00,
            }
        };

        // Default fill is DMG BG color 0 (or white if BG disabled).
        let bg0 = if dmg_bg_enabled {
            dmg_shade(bgp & 0x03)
        } else {
            0xFF
        };

        let mut bg_color_index: u8 = 0;
        let mut r = bg0;
        let mut g = bg0;
        let mut b = bg0;

        // Background + Window (DMG only; no attrs).
        if dmg_bg_enabled {
            let bg_tile_map_base: u16 = if (lcdc & 0x08) != 0 { 0x9C00 } else { 0x9800 };
            let win_tile_map_base: u16 = if (lcdc & 0x40) != 0 { 0x9C00 } else { 0x9800 };
            let tile_data_unsigned = (lcdc & 0x10) != 0;

            let window_enabled = (lcdc & 0x20) != 0 && wy < SCREEN_HEIGHT as u8;
            let window_x_start = wx as i16 - 7;
            let window_visible = window_enabled
                && ly >= wy
                && window_x_start < SCREEN_WIDTH as i16
                && (x as i16) >= window_x_start;

            let (map_base, map_x, map_y) = if window_visible {
                self.ppu_dmg_window_drawn_this_line = true;
                let win_x = (x as i16 - window_x_start) as u8;
                (win_tile_map_base, win_x, self.ppu_dmg_window_line)
            } else {
                let bg_x = (x as u8).wrapping_add(scx);
                let bg_y = ly.wrapping_add(scy);
                (bg_tile_map_base, bg_x, bg_y)
            };

            let tile_x = (map_x / 8) as u16;
            let tile_y = (map_y / 8) as u16;
            let tile_index_addr = map_base
                .wrapping_add(tile_y.saturating_mul(32))
                .wrapping_add(tile_x);

            let tile_index = self.memory[tile_index_addr as usize];
            let tile_base = if tile_data_unsigned {
                0x8000u16.wrapping_add((tile_index as u16) * 16)
            } else {
                let idx_signed = tile_index as i8 as i32;
                (0x9000i32 + idx_signed * 16) as u16
            };

            let fine_y = (map_y & 7) as u16;
            let fine_x = (map_x & 7) as u8;
            let row_addr = tile_base.wrapping_add(fine_y * 2);
            let lo = self.memory[row_addr as usize];
            let hi = self.memory[row_addr.wrapping_add(1) as usize];
            let bit = 7 - fine_x;
            let low = (lo >> bit) & 0x01;
            let high = (hi >> bit) & 0x01;
            bg_color_index = (high << 1) | low;

            let palette_bits = (bgp >> (bg_color_index * 2)) & 0x03;
            let shade = dmg_shade(palette_bits);
            r = shade;
            g = shade;
            b = shade;
        }

        // Cache BG color index for sprite priority tests.
        let pixel_index = (ly as usize) * SCREEN_WIDTH + x;
        self.ppu_dmg_bg_color_index[pixel_index] = bg_color_index;

        // Sprites (objects).
        if (lcdc & 0x02) != 0 {
            let obj_8x16 = (lcdc & 0x04) != 0;
            let sprite_height: i16 = if obj_8x16 { 16 } else { 8 };

            let mut best: Option<(i16, u8, u8, bool)> = None; // (sx, oam, color_index, use_obp1)
            let mut best_rgb: (u8, u8, u8) = (r, g, b);

            for si in 0..(self.ppu_dmg_sprite_count as usize) {
                let s = self.ppu_dmg_sprites[si];
                let col = x as i16 - s.x;
                if col < 0 || col >= 8 {
                    continue;
                }

                let row = (ly as i16).wrapping_sub(s.y);
                if row < 0 || row >= sprite_height {
                    continue;
                }

                let flip_x = (s.attrs & 0x20) != 0;
                let flip_y = (s.attrs & 0x40) != 0;
                let use_obp1 = (s.attrs & 0x10) != 0;
                let obj_to_bg_pri = (s.attrs & 0x80) != 0;

                let src_row = if flip_y {
                    (sprite_height - 1 - row) as u16
                } else {
                    row as u16
                };

                let base_tile = if obj_8x16 { s.tile & 0xFE } else { s.tile };
                let (tile, tile_row) = if obj_8x16 && src_row >= 8 {
                    (base_tile.wrapping_add(1), src_row - 8)
                } else {
                    (base_tile, src_row)
                };

                let src_x = if flip_x { 7 - col } else { col } as u8;
                let bit = 7 - src_x;
                let tile_base: u16 = 0x8000u16.wrapping_add((tile as u16) * 16);
                let row_addr = tile_base.wrapping_add(tile_row * 2);
                let lo = self.memory[row_addr as usize];
                let hi = self.memory[row_addr.wrapping_add(1) as usize];
                let low = (lo >> bit) & 0x01;
                let high = (hi >> bit) & 0x01;
                let color_index = (high << 1) | low;
                if color_index == 0 {
                    continue;
                }

                // OBJ-to-BG priority (DMG): if set and BG color is non-zero, hide sprite.
                if obj_to_bg_pri && bg_color_index != 0 {
                    continue;
                }

                let key = (s.x, s.oam_index);
                if let Some((best_x, best_oam, _, _)) = best {
                    if key >= (best_x, best_oam) {
                        continue;
                    }
                }

                let obp = if use_obp1 { obp1 } else { obp0 };
                let palette_bits = (obp >> (color_index * 2)) & 0x03;
                let shade = dmg_shade(palette_bits);

                best = Some((s.x, s.oam_index, color_index, use_obp1));
                best_rgb = (shade, shade, shade);
            }

            if best.is_some() {
                (r, g, b) = best_rgb;
            }
        }

        let out_idx = pixel_index * 3;
        self.ppu_dmg_fb[out_idx] = r;
        self.ppu_dmg_fb[out_idx + 1] = g;
        self.ppu_dmg_fb[out_idx + 2] = b;
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
            } else if line_cycle < 80 + self.ppu_dmg_mode3_len {
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
        let line_cycle = (self.ppu_cycle_counter % 456) as u16;
        let lyc_int_en = (stat & 0x40) != 0;
        let mode2_en = (stat & 0x20) != 0;
        let mode1_en = (stat & 0x10) != 0;
        let mode0_en = (stat & 0x08) != 0;

        if lyc_int_en && coincidence {
            line = true;
        }
        if mode2_en && mode == 2 {
            // DMG quirk: on LY=0, STAT mode-2 IRQ asserts a few cycles later
            // than on other visible lines. Mealybug tearoom uses this to
            // align mid-mode-3 palette changes.
            let line0_holdoff = matches!(self.model(), GameBoyModel::Dmg)
                && ly == 0
                && line_cycle < self.ppu_dmg_line0_mode2_extra as u16;
            if !line0_holdoff {
                line = true;
            }
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
            log::trace!(
                "GB PPU: STAT IRQ rising edge (STAT=0x{:02X} LY={} mode={} IF=0x{:02X})",
                stat,
                ly,
                mode,
                self.if_reg
            );
        }
    }
}
