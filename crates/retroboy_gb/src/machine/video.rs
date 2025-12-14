use crate::{SCREEN_HEIGHT, SCREEN_WIDTH};

use super::{GameBoyBus, GameBoyModel};

pub(super) fn render_video_frame(bus: &GameBoyBus, buffer: &mut [u8]) {
    // Expect an RGB buffer (3 bytes per pixel).
    let max_pixels = buffer.len() / 3;
    let width = SCREEN_WIDTH as u32;
    let height = SCREEN_HEIGHT as u32;
    let visible_pixels = (width * height) as usize;
    let pixels = max_pixels.min(visible_pixels);

    // If the LCD is disabled, fall back to a solid color (white).
    let lcdc_now = bus.memory[0xFF40];
    if (lcdc_now & 0x80) == 0 {
        for i in 0..pixels {
            let idx = i * 3;
            buffer[idx] = 0xFF;
            buffer[idx + 1] = 0xFF;
            buffer[idx + 2] = 0xFF;
        }
        return;
    }

    let dmg_shade = |palette_bits: u8| -> u8 {
        match palette_bits & 0x03 {
            0 => 0xFF,
            1 => 0xAA,
            2 => 0x55,
            _ => 0x00,
        }
    };

    let is_cgb = matches!(bus.model(), GameBoyModel::Cgb);
    let is_cgb_compat = matches!(bus.model(), GameBoyModel::CgbCompat);

    let compat_bg_rgb = |palette_bits: u8| -> (u8, u8, u8) {
        match palette_bits & 0x03 {
            0 => (0xFF, 0xFF, 0xFF),
            1 => (0x7B, 0xFF, 0x31),
            2 => (0x00, 0x63, 0xC6),
            _ => (0x00, 0x00, 0x00),
        }
    };

    let compat_obj_rgb = |palette_bits: u8| -> (u8, u8, u8) {
        match palette_bits & 0x03 {
            0 => (0xFF, 0xFF, 0xFF),
            1 => (0xFF, 0x84, 0x84),
            2 => (0x94, 0x39, 0x39),
            _ => (0x00, 0x00, 0x00),
        }
    };

    let cgb_color_rgb = |pal_data: &[u8; 0x40], palette: u8, color_index: u8| -> (u8, u8, u8) {
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
    };

    // Track BG pixel metadata for sprite priority decisions.
    let mut bg_color_index = vec![0u8; pixels];
    let mut bg_priority = vec![0u8; pixels];

    let resolve_tile_base = |tile_data_unsigned: bool, tile_index: u8| -> u16 {
        if tile_data_unsigned {
            // 0x8000-based, unsigned tile index.
            0x8000u16.wrapping_add((tile_index as u16) * 16)
        } else {
            // 0x8800-based, signed tile index (0x9000 + signed*16).
            let idx_signed = tile_index as i8 as i32;
            let addr = 0x9000i32 + idx_signed * 16;
            addr as u16
        }
    };

    // --- Background + Window (line-based) ---
    let mut window_line: u8 = 0;
    for y in 0..height {
        let ly = y as u8;
        let lcdc = bus.ppu_line_lcdc(ly);
        let scy = bus.ppu_line_scy(ly);
        let scx = bus.ppu_line_scx(ly);
        let bgp = bus.ppu_line_bgp(ly);
        let wy = bus.ppu_line_wy(ly);
        let wx = bus.ppu_line_wx(ly);

        // Choose the background tile map base.
        let bg_tile_map_base: u16 = if (lcdc & 0x08) != 0 { 0x9C00 } else { 0x9800 };
        // Choose the window tile map base.
        let win_tile_map_base: u16 = if (lcdc & 0x40) != 0 { 0x9C00 } else { 0x9800 };
        // Select tile data base and addressing mode for BG/Window.
        let tile_data_unsigned = (lcdc & 0x10) != 0;

        let dmg_bg_enabled = (lcdc & 0x01) != 0;
        let render_bg = is_cgb || dmg_bg_enabled;

        // Fill the scanline with "color 0". This matches the common case where
        // the BG/window pixel resolves to color index 0.
        let (bg0_r, bg0_g, bg0_b) = if is_cgb {
            let bgpd = bus.ppu_line_cgb_bgpd(ly);
            cgb_color_rgb(bgpd, 0, 0)
        } else if is_cgb_compat {
            // In CGB compatibility mode, DMG shade colors are mapped through a
            // fixed RGB palette (see mealybug tearoom docs).
            compat_bg_rgb(bgp & 0x03)
        } else if dmg_bg_enabled {
            let shade = dmg_shade(bgp & 0x03);
            (shade, shade, shade)
        } else {
            (0xFF, 0xFF, 0xFF)
        };
        for x in 0..width {
            let pixel_index = (y * width + x) as usize;
            if pixel_index >= pixels {
                break;
            }
            let idx = pixel_index * 3;
            buffer[idx] = bg0_r;
            buffer[idx + 1] = bg0_g;
            buffer[idx + 2] = bg0_b;
            bg_color_index[pixel_index] = 0;
            bg_priority[pixel_index] = 0;
        }

        if !render_bg {
            continue;
        }

        // Window enable: DMG overrides bit 5 when bit 0 is clear; CGB does not.
        let window_enabled = if is_cgb {
            (lcdc & 0x20) != 0 && wy < SCREEN_HEIGHT as u8
        } else {
            dmg_bg_enabled && (lcdc & 0x20) != 0 && wy < SCREEN_HEIGHT as u8
        };
        let window_x_start = wx as i16 - 7;
        let window_visible = window_enabled
            && ly >= wy
            && window_x_start < SCREEN_WIDTH as i16;

        let win_y = window_line;

        for x in 0..width {
            let pixel_index = (y * width + x) as usize;
            if pixel_index >= pixels {
                break;
            }

            let use_window = window_visible && (x as i16) >= window_x_start;

            // Select tile map and coordinates.
            let (map_base, map_x, map_y) = if use_window {
                let win_x = (x as i16 - window_x_start) as u8;
                (win_tile_map_base, win_x, win_y)
            } else {
                let bg_x = (x as u8).wrapping_add(scx);
                let bg_y = (ly as u8).wrapping_add(scy);
                (bg_tile_map_base, bg_x, bg_y)
            };

            let tile_x = (map_x / 8) as u16;
            let tile_y = (map_y / 8) as u16;
            let tile_index_addr = map_base
                .wrapping_add(tile_y.saturating_mul(32))
                .wrapping_add(tile_x);

            let tile_index = bus.ppu_vram_read(0, tile_index_addr);
            let attrs = if is_cgb {
                bus.ppu_vram_read(1, tile_index_addr)
            } else {
                0
            };

            let palette_index = attrs & 0x07;
            let tile_bank = if (attrs & 0x08) != 0 { 1 } else { 0 };
            let xflip = (attrs & 0x20) != 0;
            let yflip = (attrs & 0x40) != 0;
            let pri = if is_cgb && (attrs & 0x80) != 0 { 1 } else { 0 };

            let tile_base = resolve_tile_base(tile_data_unsigned, tile_index);

            let mut fine_y = (map_y & 7) as u16;
            let mut fine_x = (map_x & 7) as u8;
            if xflip {
                fine_x = 7 - fine_x;
            }
            if yflip {
                fine_y = 7 - fine_y;
            }

            let row_addr = tile_base.wrapping_add(fine_y * 2);
            let bank = if is_cgb { tile_bank } else { 0 };
            let lo = bus.ppu_vram_read(bank, row_addr);
            let hi = bus.ppu_vram_read(bank, row_addr.wrapping_add(1));
            let bit = 7 - fine_x;
            let low = (lo >> bit) & 0x01;
            let high = (hi >> bit) & 0x01;
            let color_index = (high << 1) | low;

            let (r, g, b) = if is_cgb {
                let bgpd = bus.ppu_line_cgb_bgpd(ly);
                cgb_color_rgb(bgpd, palette_index, color_index)
            } else if is_cgb_compat {
                let palette_bits = (bgp >> (color_index * 2)) & 0x03;
                compat_bg_rgb(palette_bits)
            } else {
                let palette_bits = (bgp >> (color_index * 2)) & 0x03;
                let shade = dmg_shade(palette_bits);
                (shade, shade, shade)
            };

            let idx = pixel_index * 3;
            buffer[idx] = r;
            buffer[idx + 1] = g;
            buffer[idx + 2] = b;
            bg_color_index[pixel_index] = color_index;
            bg_priority[pixel_index] = pri;
        }

        if window_visible {
            window_line = window_line.wrapping_add(1);
        }
    }

    // --- Sprite overlay (line-based, 10 sprites per line) ---
    #[derive(Clone, Copy)]
    struct SpriteLine {
        oam_index: u8,
        x: i16,
        attrs: u8,
        lo: u8,
        hi: u8,
        use_obp1: bool,
        cgb_palette: u8,
    }

    let cgb_dmg_priority = is_cgb && bus.cgb_opri() != 0;

    for y in 0..height {
        let ly = y as u8;
        let lcdc = bus.ppu_line_lcdc(ly);
        if (lcdc & 0x02) == 0 {
            continue;
        }

        let obj_8x16 = (lcdc & 0x04) != 0;
        let sprite_height: i16 = if obj_8x16 { 16 } else { 8 };
        let dmg_obp0 = bus.ppu_line_obp0(ly);
        let dmg_obp1 = bus.ppu_line_obp1(ly);

        // OAM scan: only the first 10 sprites (by OAM order) that intersect the
        // scanline are considered.
        let mut sprites: Vec<SpriteLine> = Vec::with_capacity(10);
        for i in 0..40u16 {
            let oam_base = 0xFE00u16.wrapping_add(i * 4);
            let sy = bus.memory[oam_base as usize] as i16 - 16;
            let sx = bus.memory[oam_base.wrapping_add(1) as usize] as i16 - 8;
            let tile_index = bus.memory[oam_base.wrapping_add(2) as usize];
            let attrs = bus.memory[oam_base.wrapping_add(3) as usize];

            let row = (ly as i16).wrapping_sub(sy);
            if row < 0 || row >= sprite_height {
                continue;
            }

            // Precompute this scanline's pattern bytes for the sprite.
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

            let tile_base: u16 = 0x8000u16.wrapping_add((tile as u16) * 16);
            let row_addr = tile_base.wrapping_add(tile_row * 2);

            let cgb_obj_bank = if (attrs & 0x08) != 0 { 1 } else { 0 };
            let bank = if is_cgb { cgb_obj_bank } else { 0 };
            let lo = bus.ppu_vram_read(bank, row_addr);
            let hi = bus.ppu_vram_read(bank, row_addr.wrapping_add(1));

            sprites.push(SpriteLine {
                oam_index: i as u8,
                x: sx,
                attrs,
                lo,
                hi,
                use_obp1: (attrs & 0x10) != 0,
                cgb_palette: attrs & 0x07,
            });

            if sprites.len() == 10 {
                break;
            }
        }

        if sprites.is_empty() {
            continue;
        }

        // Determine sprite priority ordering for this scanline.
        let mut order: Vec<usize> = (0..sprites.len()).collect();
        if !is_cgb || cgb_dmg_priority {
            order.sort_by(|&a, &b| {
                let sa = sprites[a];
                let sb = sprites[b];
                (sa.x, sa.oam_index).cmp(&(sb.x, sb.oam_index))
            });
        }

        let cgb_master_priority = is_cgb && (lcdc & 0x01) != 0;
        let cgb_obpd = bus.ppu_line_cgb_obpd(ly);

        for x in 0..width {
            let pixel_index = (y * width + x) as usize;
            if pixel_index >= pixels {
                break;
            }

            let mut chosen: Option<(SpriteLine, u8)> = None;
            for &si in &order {
                let s = sprites[si];
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

                chosen = Some((s, color_index));
                break;
            }

            let Some((sprite, color_index)) = chosen else {
                continue;
            };

            let bg_idx = bg_color_index[pixel_index];
            let bg_pri = bg_priority[pixel_index] != 0;

            let show_sprite = if is_cgb {
                if !cgb_master_priority {
                    true
                } else {
                    let obj_to_bg_pri = (sprite.attrs & 0x80) != 0;
                    if (obj_to_bg_pri || bg_pri) && bg_idx != 0 {
                        false
                    } else {
                        true
                    }
                }
            } else {
                let obj_to_bg_pri = (sprite.attrs & 0x80) != 0;
                !(obj_to_bg_pri && bg_idx != 0)
            };

            if !show_sprite {
                continue;
            }

            let (r, g, b) = if is_cgb {
                cgb_color_rgb(cgb_obpd, sprite.cgb_palette, color_index)
            } else if is_cgb_compat {
                let obp = if sprite.use_obp1 { dmg_obp1 } else { dmg_obp0 };
                let palette_bits = (obp >> (color_index * 2)) & 0x03;
                compat_obj_rgb(palette_bits)
            } else {
                let obp = if sprite.use_obp1 { dmg_obp1 } else { dmg_obp0 };
                let palette_bits = (obp >> (color_index * 2)) & 0x03;
                let shade = dmg_shade(palette_bits);
                (shade, shade, shade)
            };

            let idx = pixel_index * 3;
            buffer[idx] = r;
            buffer[idx + 1] = g;
            buffer[idx + 2] = b;
        }
    }
}
