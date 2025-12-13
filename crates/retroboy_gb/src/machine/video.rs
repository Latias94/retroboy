use crate::{SCREEN_HEIGHT, SCREEN_WIDTH};

use super::GameBoyBus;

pub(super) fn render_video_frame(bus: &GameBoyBus, buffer: &mut [u8]) {
    // Expect an RGB buffer (3 bytes per pixel).
    let max_pixels = buffer.len() / 3;
    let width = SCREEN_WIDTH as u32;
    let height = SCREEN_HEIGHT as u32;
    let visible_pixels = (width * height) as usize;
    let pixels = max_pixels.min(visible_pixels);

    // Snapshot the registers we care about.
    let lcdc = bus.memory[0xFF40];
    let scy = bus.memory[0xFF42];
    let scx = bus.memory[0xFF43];
    let bgp = bus.memory[0xFF47];

    // If LCD or BG is disabled, fall back to a solid color (white).
    if (lcdc & 0x80) == 0 || (lcdc & 0x01) == 0 {
        for i in 0..pixels {
            let idx = i * 3;
            buffer[idx] = 0xFF;
            buffer[idx + 1] = 0xFF;
            buffer[idx + 2] = 0xFF;
        }
        return;
    }

    // Choose the background tile map base.
    let bg_tile_map_base: u16 = if (lcdc & 0x08) != 0 { 0x9C00 } else { 0x9800 };

    // Select tile data base and addressing mode for BG.
    let tile_data_unsigned = (lcdc & 0x10) != 0;

    for y in 0..height {
        for x in 0..width {
            let pixel_index = (y * width + x) as usize;
            if pixel_index >= pixels {
                break;
            }

            // Scroll-adjusted coordinates in BG space.
            let bg_x = (x as u8).wrapping_add(scx);
            let bg_y = (y as u8).wrapping_add(scy);

            let tile_x = (bg_x / 8) as u16;
            let tile_y = (bg_y / 8) as u16;
            let tile_index_addr = bg_tile_map_base
                .wrapping_add(tile_y.saturating_mul(32))
                .wrapping_add(tile_x);

            let tile_index = bus.memory[tile_index_addr as usize];

            // Resolve tile data address.
            let tile_base: u16 = if tile_data_unsigned {
                // 0x8000‑based, unsigned tile index.
                0x8000u16.wrapping_add((tile_index as u16) * 16)
            } else {
                // 0x8800‑based, signed tile index (0x9000 + signed*16).
                let idx_signed = tile_index as i8 as i32;
                let addr = 0x9000i32 + idx_signed * 16;
                addr as u16
            };

            let fine_y = (bg_y & 7) as u16;
            let fine_x = (bg_x & 7) as u8;
            let row_addr = tile_base.wrapping_add(fine_y * 2);

            // Each row is 2 bytes: low bits then high bits.
            let lo = bus.memory[row_addr as usize];
            let hi = bus.memory[row_addr.wrapping_add(1) as usize];
            let bit = 7 - fine_x;
            let low = (lo >> bit) & 0x01;
            let high = (hi >> bit) & 0x01;
            let color_index = (high << 1) | low;

            // Map color index (0..3) through BGP palette to a DMG shade.
            let palette_bits = (bgp >> (color_index * 2)) & 0x03;
            // Simple DMG‑style grayscale: 0=white, 3=black.
            let shade = match palette_bits {
                0 => 0xFF,
                1 => 0xAA,
                2 => 0x55,
                _ => 0x00,
            };

            let idx = pixel_index * 3;
            buffer[idx] = shade;
            buffer[idx + 1] = shade;
            buffer[idx + 2] = shade;
        }
    }

    // --- Sprite overlay (very simplified) ---
    //
    // We render up to 40 sprites from OAM on top of the background.
    // This ignores many details (priority, OBJ size 8x16, window
    // interactions, etc.) but is enough to visualise simple test ROMs
    // and menus.
    let obp0 = bus.memory[0xFF48];
    let obp1 = bus.memory[0xFF49];
    let obj_8x16 = (lcdc & 0x04) != 0;

    // Iterate over all 40 sprite entries in OAM.
    for i in 0..40u16 {
        let oam_base = 0xFE00u16.wrapping_add(i * 4);
        let y = bus.memory[oam_base as usize] as i16 - 16;
        let x = bus.memory[oam_base.wrapping_add(1) as usize] as i16 - 8;
        let mut tile_index = bus.memory[oam_base.wrapping_add(2) as usize];
        let attrs = bus.memory[oam_base.wrapping_add(3) as usize];

        if y <= -8 || y as i32 >= height as i32 {
            continue;
        }
        if x <= -8 || x as i32 >= width as i32 {
            continue;
        }

        let use_obp1 = (attrs & 0x10) != 0;
        let flip_x = (attrs & 0x20) != 0;
        let flip_y = (attrs & 0x40) != 0;
        // Priority bit (attrs & 0x80) is currently ignored; sprites
        // always render on top of the background.

        if obj_8x16 {
            // In 8x16 mode, the low bit of the tile index is ignored.
            tile_index &= 0xFE;
        }
        let tile_base: u16 = 0x8000u16.wrapping_add((tile_index as u16) * 16);

        for row in 0..8i16 {
            let screen_y = y + row;
            if screen_y < 0 || screen_y as u32 >= height {
                continue;
            }

            let src_y = if flip_y { 7 - row } else { row } as u16;
            let row_addr = tile_base.wrapping_add(src_y * 2);
            let lo = bus.memory[row_addr as usize];
            let hi = bus.memory[row_addr.wrapping_add(1) as usize];

            for col in 0..8i16 {
                let screen_x = x + col;
                if screen_x < 0 || screen_x as u32 >= width {
                    continue;
                }

                let src_x = if flip_x { 7 - col } else { col } as u8;
                let bit = 7 - src_x;
                let low = (lo >> bit) & 0x01;
                let high = (hi >> bit) & 0x01;
                let color_index = (high << 1) | low;
                // Color index 0 is transparent for sprites.
                if color_index == 0 {
                    continue;
                }

                let obp = if use_obp1 { obp1 } else { obp0 };
                let palette_bits = (obp >> (color_index * 2)) & 0x03;
                let shade = match palette_bits {
                    0 => 0xFF,
                    1 => 0xAA,
                    2 => 0x55,
                    _ => 0x00,
                };

                let idx =
                    (screen_y as u32 * width + screen_x as u32) as usize * 3;
                if idx + 2 < buffer.len() {
                    buffer[idx] = shade;
                    buffer[idx + 1] = shade;
                    buffer[idx + 2] = shade;
                }
            }
        }
    }
}
