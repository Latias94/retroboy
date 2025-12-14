use super::{cartridge::Cartridge, serial::Serial, timer::Timer, GameBoyModel, MEMORY_SIZE};
use crate::{SCREEN_HEIGHT, SCREEN_WIDTH};

mod apu;
mod dma;
mod init;
mod joypad;
mod mmio;
mod ppu;
mod timer_io;
mod traits;

pub(crate) struct GameBoyBus {
    pub(crate) memory: [u8; MEMORY_SIZE],
    pub(crate) serial: Serial,
    pub(crate) if_reg: u8,
    pub(crate) ie_reg: u8,
    model: GameBoyModel,
    /// Optional cartridge with mapper (currently MBC1 or MBC3).
    cartridge: Option<Cartridge>,
    /// CGB VRAM bank (FF4F/VBK bit 0). Bank 0 lives in `memory`; bank 1 is
    /// stored separately in `vram_bank1`.
    vbk: u8,
    vram_bank1: [u8; 0x2000],
    /// CGB WRAM bank select (FF70/SVBK bits 0-2). Value 0 maps bank 1.
    svbk: u8,
    /// CGB WRAM banks 2-7 (bank 1 lives in `memory`).
    wram_banks: [[u8; 0x1000]; 6],
    /// CGB KEY1/SPD (FF4D): double-speed state and "prepare speed switch" latch.
    cgb_double_speed: bool,
    cgb_key1_armed: bool,
    /// Remaining CPU T-cycles of the post-STOP speed switch pause (Pandocs: 8200 T-cycles).
    cgb_speed_switch_pause_tcycles: u32,
    /// CGB palette registers (FF68-FF6B).
    cgb_bgpi: u8,
    cgb_bgpi_autoinc: bool,
    cgb_bgpd: [u8; 0x40],
    cgb_obpi: u8,
    cgb_obpi_autoinc: bool,
    cgb_obpd: [u8; 0x40],
    /// CGB OPRI (FF6C) - object priority mode (bit 0).
    cgb_opri: u8,
    /// CGB HDMA registers (FF51-FF55). Only stored for now.
    cgb_hdma1: u8,
    cgb_hdma2: u8,
    cgb_hdma3: u8,
    cgb_hdma4: u8,
    cgb_hdma5: u8,
    cgb_hdma_active: bool,
    /// Pending CPU stall time (in CPU T-cycles) caused by DMA engines such as HDMA.
    cpu_stall_tcycles: u32,
    /// CGB double-speed: remainder for mapping CPU T-cycles to PPU cycles.
    ///
    /// In double-speed mode, the CPU runs twice as fast as the LCD controller.
    /// We therefore advance the PPU by 1 cycle for every 2 CPU T-cycles, and
    /// keep the leftover here so odd `bus.tick(1)` calls remain consistent.
    cgb_ppu_subcycle: u8,
    /// Number of PPU cycles advanced by the most recent `tick` call.
    last_ppu_cycles: u32,
    /// CGB undocumented registers (FF72-FF75).
    cgb_ff72: u8,
    cgb_ff73: u8,
    cgb_ff74: u8,
    cgb_ff75: u8,
    /// Timer / divider state.
    timer: Timer,
    /// Number of timer register accesses (DIV/TIMA/TMA/TAC reads or
    /// writes) performed during the current CPU instruction.
    ///
    /// Each such access corresponds to one timer "tick" in the DMG
    /// model. We use this counter together with the total instruction
    /// length (in T-cycles) to keep the overall timer tick frequency
    /// aligned with hardware (one tick per machine cycle = 4 T-cycles)
    /// without double-counting ticks in the timer helpers.
    timer_io_events: u8,
    // Very small PPU timing model. We track a running counter of CPU cycles
    // to derive LY and an approximate LCD mode.
    ppu_cycle_counter: u32,
    /// DMG quirk after LCD enable: the very first scanline's mode-2 timing is
    /// offset by 4 cycles (tearoom relies on this for line-0 alignment).
    ppu_dmg_line0_mode2_extra: u8,
    /// DMG-only dot-based framebuffer used by mode-3 timing tests (mealybug-tearoom-tests).
    ///
    /// This is intentionally simple: each visible pixel is "output" during
    /// mode 3 at line cycles 80..239, using whatever register values are
    /// currently visible on the bus at that cycle.
    ppu_dmg_fb_valid: bool,
    ppu_dmg_window_line: u8,
    ppu_dmg_window_drawn_this_line: bool,
    ppu_dmg_mode3_output_delay: u8,
    ppu_dmg_mode3_len: u16,
    ppu_dmg_mode3_x: u8,
    ppu_dmg_mode3_discard: u8,
    /// BGP palette value latched at the start of mode 3.
    ppu_dmg_bgp_base: u8,
    /// Effective BGP value visible to the DMG pixel pipeline.
    ppu_dmg_bgp_effective: u8,
    /// Pending BGP value that will be applied after a short delay.
    ppu_dmg_bgp_pending: u8,
    /// Countdown (in dots) until the pending BGP value becomes visible.
    ppu_dmg_bgp_restore_delay: u8,
    ppu_dmg_fifo: [u8; 16],
    ppu_dmg_fifo_head: u8,
    ppu_dmg_fifo_len: u8,
    ppu_dmg_fetch_phase: u8,
    ppu_dmg_fetch_tile_x: u8,
    ppu_dmg_fetch_tile_y: u8,
    ppu_dmg_fetch_tile_line: u8,
    ppu_dmg_fetch_tile_id: u8,
    ppu_dmg_fetch_tile_lo: u8,
    ppu_dmg_fetch_tile_hi: u8,
    ppu_dmg_lcdc_latched: u8,
    ppu_dmg_scx_latched: u8,
    ppu_dmg_scy_latched: u8,
    ppu_dmg_bg_color_index: [u8; SCREEN_WIDTH * SCREEN_HEIGHT],
    ppu_dmg_fb: [u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3],

    /// CGB-only dot-based framebuffer scaffold.
    ///
    /// This is a minimal background-only pixel pipeline used as a stepping
    /// stone towards a more accurate CGB PPU. It is currently opt-in and is
    /// not used by default rendering.
    ppu_cgb_fb_valid: bool,
    ppu_cgb_fb: [u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
    ppu_cgb_mode3_output_delay: u8,
    ppu_cgb_mode3_x: u8,
    ppu_cgb_mode3_discard: u8,
    ppu_cgb_fifo: [u8; 16],
    ppu_cgb_fifo_head: u8,
    ppu_cgb_fifo_len: u8,
    ppu_cgb_fetch_phase: u8,
    ppu_cgb_fetch_tile_x: u8,
    ppu_cgb_fetch_tile_y: u8,
    ppu_cgb_fetch_tile_line: u8,
    ppu_cgb_fetch_tile_id: u8,
    ppu_cgb_fetch_tile_attr: u8,
    ppu_cgb_fetch_tile_lo: u8,
    ppu_cgb_fetch_tile_hi: u8,
    ppu_cgb_fetch_using_window: bool,
    ppu_cgb_window_line: u8,
    ppu_cgb_window_drawn_this_line: bool,
    ppu_cgb_bg_color_index: [u8; SCREEN_WIDTH * SCREEN_HEIGHT],
    ppu_cgb_bg_priority: [u8; SCREEN_WIDTH * SCREEN_HEIGHT],
    ppu_cgb_sprite_count: u8,
    ppu_cgb_sprites: [PpuCgbSpriteLine; 10],
    ppu_cgb_sprite_order: [u8; 10],
    ppu_dmg_sprite_count: u8,
    ppu_dmg_sprites: [PpuDmgSprite; 10],
    /// Per-scanline register snapshots for the line-based renderer.
    ///
    /// cgb-acid2 (and similar ROMs) perform LCDC/WX/etc. writes during mode 2
    /// and expect those changes to affect only specific scanlines. We latch
    /// the relevant state at the start of mode 3 (line cycle 80) for each
    /// visible line.
    ppu_line_captured: [bool; 154],
    ppu_line_lcdc: [u8; 154],
    ppu_line_scx: [u8; 154],
    ppu_line_scy: [u8; 154],
    ppu_line_wx: [u8; 154],
    ppu_line_wy: [u8; 154],
    ppu_line_bgp: [u8; 154],
    ppu_line_obp0: [u8; 154],
    ppu_line_obp1: [u8; 154],
    ppu_line_bgpd: [[u8; 0x40]; 154],
    ppu_line_obpd: [[u8; 0x40]; 154],
    /// Last completed frame's scanline snapshots, committed on the LY=143->144
    /// vblank edge. The renderer uses these to avoid sampling a partially
    /// completed frame (our CPU stepping is not aligned to vblank boundaries).
    ppu_frame_valid: bool,
    ppu_frame_lcdc: [u8; 154],
    ppu_frame_scx: [u8; 154],
    ppu_frame_scy: [u8; 154],
    ppu_frame_wx: [u8; 154],
    ppu_frame_wy: [u8; 154],
    ppu_frame_bgp: [u8; 154],
    ppu_frame_obp0: [u8; 154],
    ppu_frame_obp1: [u8; 154],
    ppu_frame_bgpd: [[u8; 0x40]; 154],
    ppu_frame_obpd: [[u8; 0x40]; 154],
    /// Internal "STAT interrupt line" latch used to model STAT's edge-triggered
    /// interrupt behaviour. This tracks the logically ORed state of all enabled
    /// STAT interrupt sources between calls to `tick`.
    stat_irq_line: bool,
    // Joypad state: selection bits and button presses. Selection bits
    // correspond to P1 bits 5 (buttons) and 4 (d-pad). The button masks
    // use bit=1 to mean "pressed" for:
    // - joyp_buttons: bit0=A, bit1=B, bit2=Select, bit3=Start
    // - joyp_dpad:    bit0=Right, bit1=Left, bit2=Up, bit3=Down
    joyp_select: u8,
    joyp_buttons: u8,
    joyp_dpad: u8,
}

#[derive(Clone, Copy, Default)]
struct PpuDmgSprite {
    oam_index: u8,
    x: i16,
    y: i16,
    tile: u8,
    attrs: u8,
}

#[derive(Clone, Copy, Default)]
struct PpuCgbSpriteLine {
    oam_index: u8,
    x: i16,
    attrs: u8,
    lo: u8,
    hi: u8,
}

impl Default for GameBoyBus {
    fn default() -> Self {
        let mut bus = Self {
            memory: [0; MEMORY_SIZE],
            serial: Serial::default(),
            if_reg: 0,
            ie_reg: 0,
            model: GameBoyModel::Dmg,
            cartridge: None,
            vbk: 0,
            vram_bank1: [0; 0x2000],
            svbk: 0,
            wram_banks: [[0; 0x1000]; 6],
            cgb_double_speed: false,
            cgb_key1_armed: false,
            cgb_speed_switch_pause_tcycles: 0,
            cgb_bgpi: 0,
            cgb_bgpi_autoinc: false,
            cgb_bgpd: [0; 0x40],
            cgb_obpi: 0,
            cgb_obpi_autoinc: false,
            cgb_obpd: [0; 0x40],
            cgb_opri: 0,
            cgb_hdma1: 0,
            cgb_hdma2: 0,
            cgb_hdma3: 0,
            cgb_hdma4: 0,
            cgb_hdma5: 0x7F,
            cgb_hdma_active: false,
            cpu_stall_tcycles: 0,
            cgb_ppu_subcycle: 0,
            last_ppu_cycles: 0,
            cgb_ff72: 0,
            cgb_ff73: 0,
            cgb_ff74: 0,
            cgb_ff75: 0,
            timer: Timer::new(),
            timer_io_events: 0,
            ppu_cycle_counter: 0,
            ppu_dmg_line0_mode2_extra: 4,
            ppu_dmg_fb_valid: false,
            ppu_dmg_window_line: 0,
            ppu_dmg_window_drawn_this_line: false,
            ppu_dmg_mode3_output_delay: 0,
            ppu_dmg_mode3_len: 172,
            ppu_dmg_mode3_x: 0,
            ppu_dmg_mode3_discard: 0,
            ppu_dmg_bgp_base: 0,
            ppu_dmg_bgp_effective: 0,
            ppu_dmg_bgp_pending: 0,
            ppu_dmg_bgp_restore_delay: 0,
            ppu_dmg_fifo: [0; 16],
            ppu_dmg_fifo_head: 0,
            ppu_dmg_fifo_len: 0,
            ppu_dmg_fetch_phase: 0,
            ppu_dmg_fetch_tile_x: 0,
            ppu_dmg_fetch_tile_y: 0,
            ppu_dmg_fetch_tile_line: 0,
            ppu_dmg_fetch_tile_id: 0,
            ppu_dmg_fetch_tile_lo: 0,
            ppu_dmg_fetch_tile_hi: 0,
            ppu_dmg_lcdc_latched: 0,
            ppu_dmg_scx_latched: 0,
            ppu_dmg_scy_latched: 0,
            ppu_dmg_bg_color_index: [0; SCREEN_WIDTH * SCREEN_HEIGHT],
            ppu_dmg_fb: [0; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
            ppu_cgb_fb_valid: false,
            ppu_cgb_fb: [0; SCREEN_WIDTH * SCREEN_HEIGHT * 3],
            ppu_cgb_mode3_output_delay: 0,
            ppu_cgb_mode3_x: 0,
            ppu_cgb_mode3_discard: 0,
            ppu_cgb_fifo: [0; 16],
            ppu_cgb_fifo_head: 0,
            ppu_cgb_fifo_len: 0,
            ppu_cgb_fetch_phase: 0,
            ppu_cgb_fetch_tile_x: 0,
            ppu_cgb_fetch_tile_y: 0,
            ppu_cgb_fetch_tile_line: 0,
            ppu_cgb_fetch_tile_id: 0,
            ppu_cgb_fetch_tile_attr: 0,
            ppu_cgb_fetch_tile_lo: 0,
            ppu_cgb_fetch_tile_hi: 0,
            ppu_cgb_fetch_using_window: false,
            ppu_cgb_window_line: 0,
            ppu_cgb_window_drawn_this_line: false,
            ppu_cgb_bg_color_index: [0; SCREEN_WIDTH * SCREEN_HEIGHT],
            ppu_cgb_bg_priority: [0; SCREEN_WIDTH * SCREEN_HEIGHT],
            ppu_cgb_sprite_count: 0,
            ppu_cgb_sprites: [PpuCgbSpriteLine::default(); 10],
            ppu_cgb_sprite_order: [0; 10],
            ppu_dmg_sprite_count: 0,
            ppu_dmg_sprites: [PpuDmgSprite::default(); 10],
            ppu_line_captured: [false; 154],
            ppu_line_lcdc: [0; 154],
            ppu_line_scx: [0; 154],
            ppu_line_scy: [0; 154],
            ppu_line_wx: [0; 154],
            ppu_line_wy: [0; 154],
            ppu_line_bgp: [0; 154],
            ppu_line_obp0: [0; 154],
            ppu_line_obp1: [0; 154],
            ppu_line_bgpd: [[0; 0x40]; 154],
            ppu_line_obpd: [[0; 0x40]; 154],
            ppu_frame_valid: false,
            ppu_frame_lcdc: [0; 154],
            ppu_frame_scx: [0; 154],
            ppu_frame_scy: [0; 154],
            ppu_frame_wx: [0; 154],
            ppu_frame_wy: [0; 154],
            ppu_frame_bgp: [0; 154],
            ppu_frame_obp0: [0; 154],
            ppu_frame_obp1: [0; 154],
            ppu_frame_bgpd: [[0; 0x40]; 154],
            ppu_frame_obpd: [[0; 0x40]; 154],
            stat_irq_line: false,
            joyp_select: 0x30, // no group selected, bits 7-6 read back as 1
            joyp_buttons: 0x00,
            joyp_dpad: 0x00,
        };
        bus.apply_dmg_initial_io_state();
        bus.ppu_dmg_bgp_base = bus.memory[0xFF47];
        bus.ppu_dmg_bgp_effective = bus.memory[0xFF47];
        bus.ppu_dmg_bgp_pending = bus.memory[0xFF47];
        bus
    }
}

impl GameBoyBus {
    pub(crate) fn model(&self) -> GameBoyModel {
        self.model
    }

    pub(crate) fn set_model(&mut self, model: GameBoyModel) {
        self.model = model;
        // Reset CGB bank selects to power-on defaults (bank 0 for VRAM, bank 1 for WRAM).
        self.vbk = 0;
        self.svbk = 0;
        self.cgb_double_speed = false;
        self.cgb_key1_armed = false;
        self.cgb_speed_switch_pause_tcycles = 0;
        self.cgb_bgpi = 0;
        self.cgb_bgpi_autoinc = false;
        self.cgb_bgpd.fill(0xFF);
        self.cgb_obpi = 0;
        self.cgb_obpi_autoinc = false;
        self.cgb_obpd.fill(0xFF);
        self.cgb_opri = 0;
        self.cgb_hdma1 = 0;
        self.cgb_hdma2 = 0;
        self.cgb_hdma3 = 0;
        self.cgb_hdma4 = 0;
        self.cgb_hdma5 = 0x7F;
        self.cgb_hdma_active = false;
        self.cpu_stall_tcycles = 0;
        self.cgb_ppu_subcycle = 0;
        self.last_ppu_cycles = 0;
        self.cgb_ff72 = 0;
        self.cgb_ff73 = 0;
        self.cgb_ff74 = 0;
        self.cgb_ff75 = 0;

        // Reset DMG PPU framebuffer state.
        self.ppu_dmg_fb_valid = false;
        self.ppu_dmg_window_line = 0;
        self.ppu_dmg_window_drawn_this_line = false;
        self.ppu_dmg_mode3_output_delay = 0;
        self.ppu_dmg_mode3_x = 0;
        self.ppu_dmg_mode3_discard = 0;
        self.ppu_dmg_bgp_base = self.memory[0xFF47];
        self.ppu_dmg_bgp_effective = self.memory[0xFF47];
        self.ppu_dmg_bgp_restore_delay = 0;
        self.ppu_dmg_fifo_head = 0;
        self.ppu_dmg_fifo_len = 0;
        self.ppu_dmg_fetch_phase = 0;
        self.ppu_dmg_sprite_count = 0;

        // Reset CGB dot-based framebuffer state.
        self.ppu_cgb_fb_valid = false;
        self.ppu_cgb_mode3_output_delay = 0;
        self.ppu_cgb_mode3_x = 0;
        self.ppu_cgb_mode3_discard = 0;
        self.ppu_cgb_fifo_head = 0;
        self.ppu_cgb_fifo_len = 0;
        self.ppu_cgb_fetch_phase = 0;
        self.ppu_cgb_fetch_using_window = false;
        self.ppu_cgb_window_line = 0;
        self.ppu_cgb_window_drawn_this_line = false;
        self.ppu_cgb_sprite_count = 0;
    }

    #[inline]
    fn is_cgb(&self) -> bool {
        matches!(self.model, GameBoyModel::Cgb)
    }

    #[inline]
    fn is_dmg_like(&self) -> bool {
        matches!(self.model, GameBoyModel::Dmg | GameBoyModel::CgbCompat)
    }

    #[inline]
    pub(crate) fn last_ppu_cycles(&self) -> u32 {
        self.last_ppu_cycles
    }

    #[inline]
    fn cgb_vram_bank(&self) -> u8 {
        self.vbk & 0x01
    }

    #[inline]
    fn cgb_wram_bank(&self) -> u8 {
        let raw = self.svbk & 0x07;
        if raw == 0 { 1 } else { raw }
    }

    #[inline]
    fn vram_read(&self, addr: u16) -> u8 {
        if self.is_cgb() && self.cgb_vram_bank() != 0 {
            let idx = (addr - 0x8000) as usize;
            self.vram_bank1[idx]
        } else {
            self.memory[addr as usize]
        }
    }

    #[inline]
    fn vram_write(&mut self, addr: u16, value: u8) {
        if self.is_cgb() && self.cgb_vram_bank() != 0 {
            let idx = (addr - 0x8000) as usize;
            self.vram_bank1[idx] = value;
        } else {
            self.memory[addr as usize] = value;
        }
    }

    /// PPU-side VRAM read that ignores the CPU-visible VBK selection.
    /// Bank 0 is stored in `memory` and bank 1 in `vram_bank1`.
    pub(super) fn ppu_vram_read(&self, bank: u8, addr: u16) -> u8 {
        debug_assert!((0x8000..=0x9FFF).contains(&addr));
        if bank & 0x01 != 0 {
            let idx = (addr - 0x8000) as usize;
            self.vram_bank1[idx]
        } else {
            self.memory[addr as usize]
        }
    }

    pub(super) fn cgb_opri(&self) -> u8 {
        self.cgb_opri & 0x01
    }

    #[inline]
    fn ppu_render_get_u8(&self, ly: u8, table: &[u8; 154], fallback_addr: usize) -> u8 {
        let idx = ly as usize;
        if self.ppu_frame_valid && idx < 144 {
            table[idx]
        } else {
            self.memory[fallback_addr]
        }
    }

    pub(super) fn ppu_line_lcdc(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_lcdc, 0xFF40)
    }

    pub(super) fn ppu_line_scx(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_scx, 0xFF43)
    }

    pub(super) fn ppu_line_scy(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_scy, 0xFF42)
    }

    pub(super) fn ppu_line_wx(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_wx, 0xFF4B)
    }

    pub(super) fn ppu_line_wy(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_wy, 0xFF4A)
    }

    pub(super) fn ppu_line_bgp(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_bgp, 0xFF47)
    }

    pub(super) fn ppu_line_obp0(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_obp0, 0xFF48)
    }

    pub(super) fn ppu_line_obp1(&self, ly: u8) -> u8 {
        self.ppu_render_get_u8(ly, &self.ppu_frame_obp1, 0xFF49)
    }

    pub(super) fn ppu_line_cgb_bgpd(&self, ly: u8) -> &[u8; 0x40] {
        let idx = ly as usize;
        if self.ppu_frame_valid && idx < 144 {
            &self.ppu_frame_bgpd[idx]
        } else {
            &self.cgb_bgpd
        }
    }

    pub(super) fn ppu_line_cgb_obpd(&self, ly: u8) -> &[u8; 0x40] {
        let idx = ly as usize;
        if self.ppu_frame_valid && idx < 144 {
            &self.ppu_frame_obpd[idx]
        } else {
            &self.cgb_obpd
        }
    }

    pub(super) fn dmg_ppu_framebuffer_copy(&self, buffer: &mut [u8]) -> bool {
        if !self.is_dmg_like() {
            return false;
        }
        let enabled = std::env::var("RETROBOY_GB_USE_PPU_FB")
            .map(|v| v != "0" && !v.eq_ignore_ascii_case("false"))
            .unwrap_or(false);
        if !enabled || !self.ppu_dmg_fb_valid {
            return false;
        }
        let n = buffer.len().min(self.ppu_dmg_fb.len());
        buffer[..n].copy_from_slice(&self.ppu_dmg_fb[..n]);
        true
    }

    pub(super) fn cgb_ppu_framebuffer_copy(&self, buffer: &mut [u8]) -> bool {
        if !matches!(self.model, GameBoyModel::Cgb) {
            return false;
        }
        let enabled = std::env::var("RETROBOY_GB_USE_CGB_PPU_FB")
            .map(|v| v != "0" && !v.eq_ignore_ascii_case("false"))
            .unwrap_or(false);
        if !enabled || !self.ppu_cgb_fb_valid {
            return false;
        }
        let n = buffer.len().min(self.ppu_cgb_fb.len());
        buffer[..n].copy_from_slice(&self.ppu_cgb_fb[..n]);
        true
    }

    #[cfg(test)]
    pub(crate) fn cgb_ppu_framebuffer_copy_force(&self, buffer: &mut [u8]) -> bool {
        if !matches!(self.model, GameBoyModel::Cgb) || !self.ppu_cgb_fb_valid {
            return false;
        }
        let n = buffer.len().min(self.ppu_cgb_fb.len());
        buffer[..n].copy_from_slice(&self.ppu_cgb_fb[..n]);
        true
    }

    #[cfg(test)]
    pub(crate) fn dmg_ppu_framebuffer_copy_force(&self, buffer: &mut [u8]) -> bool {
        if !self.is_dmg_like() || !self.ppu_dmg_fb_valid {
            return false;
        }
        let n = buffer.len().min(self.ppu_dmg_fb.len());
        buffer[..n].copy_from_slice(&self.ppu_dmg_fb[..n]);
        true
    }

    #[cfg(test)]
    pub(crate) fn dmg_ppu_bg_color_index_at(&self, x: usize, y: usize) -> Option<u8> {
        if !self.is_dmg_like() || !self.ppu_dmg_fb_valid {
            return None;
        }
        if x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT {
            return None;
        }
        Some(self.ppu_dmg_bg_color_index[y * SCREEN_WIDTH + x])
    }

    #[inline]
    fn wram_read(&self, addr: u16) -> u8 {
        debug_assert!((0xC000..=0xDFFF).contains(&addr));
        if !self.is_cgb() || (0xC000..=0xCFFF).contains(&addr) {
            return self.memory[addr as usize];
        }

        let bank = self.cgb_wram_bank();
        if bank <= 1 {
            return self.memory[addr as usize];
        }

        let idx = (addr - 0xD000) as usize;
        self.wram_banks[(bank - 2) as usize][idx]
    }

    #[inline]
    fn wram_write(&mut self, addr: u16, value: u8) {
        debug_assert!((0xC000..=0xDFFF).contains(&addr));
        if !self.is_cgb() || (0xC000..=0xCFFF).contains(&addr) {
            self.memory[addr as usize] = value;
            return;
        }

        let bank = self.cgb_wram_bank();
        if bank <= 1 {
            self.memory[addr as usize] = value;
            return;
        }

        let idx = (addr - 0xD000) as usize;
        self.wram_banks[(bank - 2) as usize][idx] = value;
    }

    #[inline]
    fn wram_echo_read(&self, addr: u16) -> u8 {
        debug_assert!((0xE000..=0xFDFF).contains(&addr));
        let base = addr.wrapping_sub(0x2000);
        match base {
            0xC000..=0xCFFF => self.memory[base as usize],
            0xD000..=0xDDFF => self.wram_read(base),
            _ => self.memory[base as usize],
        }
    }

    #[inline]
    fn wram_echo_write(&mut self, addr: u16, value: u8) {
        debug_assert!((0xE000..=0xFDFF).contains(&addr));
        self.memory[addr as usize] = value;
        let base = addr.wrapping_sub(0x2000);
        match base {
            0xC000..=0xCFFF => self.memory[base as usize] = value,
            0xD000..=0xDDFF => self.wram_write(base, value),
            _ => self.memory[base as usize] = value,
        }
    }

    #[inline]
    fn cgb_key1_read(&self) -> u8 {
        let mut value = 0x7E;
        if self.cgb_key1_armed {
            value |= 0x01;
        }
        if self.cgb_double_speed {
            value |= 0x80;
        }
        value
    }

    #[inline]
    fn cgb_key1_write(&mut self, value: u8) {
        self.cgb_key1_armed = (value & 0x01) != 0;
    }

    #[inline]
    fn cgb_vbk_read(&self) -> u8 {
        0xFE | (self.vbk & 0x01)
    }

    #[inline]
    fn cgb_vbk_write(&mut self, value: u8) {
        self.vbk = value & 0x01;
    }

    #[inline]
    fn cgb_svbk_read(&self) -> u8 {
        0xF8 | (self.svbk & 0x07)
    }

    #[inline]
    fn cgb_svbk_write(&mut self, value: u8) {
        self.svbk = value & 0x07;
    }

    #[inline]
    fn cgb_bgpi_read(&self) -> u8 {
        let idx = self.cgb_bgpi & 0x3F;
        (if self.cgb_bgpi_autoinc { 0x80 } else { 0x00 }) | idx
    }

    #[inline]
    fn cgb_bgpi_write(&mut self, value: u8) {
        self.cgb_bgpi = value & 0x3F;
        self.cgb_bgpi_autoinc = (value & 0x80) != 0;
    }

    #[inline]
    fn cgb_obpi_read(&self) -> u8 {
        let idx = self.cgb_obpi & 0x3F;
        (if self.cgb_obpi_autoinc { 0x80 } else { 0x00 }) | idx
    }

    #[inline]
    fn cgb_obpi_write(&mut self, value: u8) {
        self.cgb_obpi = value & 0x3F;
        self.cgb_obpi_autoinc = (value & 0x80) != 0;
    }

    #[inline]
    fn cgb_bcpd_read(&self) -> u8 {
        let idx = (self.cgb_bgpi & 0x3F) as usize;
        self.cgb_bgpd[idx]
    }

    #[inline]
    fn cgb_bcpd_write(&mut self, value: u8) {
        let idx = (self.cgb_bgpi & 0x3F) as usize;
        self.cgb_bgpd[idx] = value;
        if self.cgb_bgpi_autoinc {
            self.cgb_bgpi = self.cgb_bgpi.wrapping_add(1) & 0x3F;
        }
    }

    #[inline]
    fn cgb_ocpd_read(&self) -> u8 {
        let idx = (self.cgb_obpi & 0x3F) as usize;
        self.cgb_obpd[idx]
    }

    #[inline]
    fn cgb_ocpd_write(&mut self, value: u8) {
        let idx = (self.cgb_obpi & 0x3F) as usize;
        self.cgb_obpd[idx] = value;
        if self.cgb_obpi_autoinc {
            self.cgb_obpi = self.cgb_obpi.wrapping_add(1) & 0x3F;
        }
    }

    #[inline]
    fn cgb_opri_read(&self) -> u8 {
        0xFE | (self.cgb_opri & 0x01)
    }

    #[inline]
    fn cgb_opri_write(&mut self, value: u8) {
        self.cgb_opri = value & 0x01;
    }

    #[inline]
    fn cgb_ff75_read(&self) -> u8 {
        0x8F | (self.cgb_ff75 & 0x70)
    }

    #[inline]
    fn cgb_ff75_write(&mut self, value: u8) {
        self.cgb_ff75 = value & 0x70;
    }

    #[inline]
    fn cgb_hdma_src_addr(&self) -> u16 {
        ((self.cgb_hdma1 as u16) << 8) | ((self.cgb_hdma2 as u16) & 0x00F0)
    }

    #[inline]
    fn cgb_hdma_dst_addr(&self) -> u16 {
        0x8000u16
            | (((self.cgb_hdma3 as u16) & 0x1F) << 8)
            | ((self.cgb_hdma4 as u16) & 0x00F0)
    }

    #[inline]
    fn cgb_hdma_set_src_addr(&mut self, addr: u16) {
        let addr = addr & 0xFFF0;
        self.cgb_hdma1 = (addr >> 8) as u8;
        self.cgb_hdma2 = (addr & 0x00F0) as u8;
    }

    #[inline]
    fn cgb_hdma_set_dst_addr(&mut self, addr: u16) {
        let offset = addr.wrapping_sub(0x8000) & 0x1FF0;
        self.cgb_hdma3 = ((offset >> 8) as u8) & 0x1F;
        self.cgb_hdma4 = (offset & 0x00F0) as u8;
    }

    #[inline]
    fn cgb_hdma5_read(&self) -> u8 {
        let len = self.cgb_hdma5 & 0x7F;
        if self.cgb_hdma_active {
            len
        } else {
            0x80 | len
        }
    }

    fn cgb_hdma_transfer_block(&mut self) {
        let mut src = self.cgb_hdma_src_addr();
        let mut dst = self.cgb_hdma_dst_addr();

        for _ in 0..0x10u16 {
            let byte = self.read8_mmio(src);
            self.vram_write(dst, byte);
            src = src.wrapping_add(1);
            dst = dst.wrapping_add(1);
        }

        self.cgb_hdma_set_src_addr(src);
        self.cgb_hdma_set_dst_addr(dst);
    }

    fn cgb_hdma_transfer_blocks(&mut self, blocks: u8) {
        for _ in 0..blocks {
            self.cgb_hdma_transfer_block();
        }
    }

    fn cgb_hdma_hblank_tick(&mut self) {
        if !self.cgb_hdma_active {
            return;
        }

        // Each 0x10-byte block takes about 8 μs, regardless of double-speed.
        // Model this as a short CPU stall where the LCD controller keeps
        // running at the normal rate.
        self.cpu_stall_tcycles = self
            .cpu_stall_tcycles
            .saturating_add(self.cgb_hdma_block_stall_tcycles());

        self.cgb_hdma_transfer_block();

        if self.cgb_hdma5 == 0 {
            self.cgb_hdma5 = 0x7F;
            self.cgb_hdma_active = false;
        } else {
            self.cgb_hdma5 = (self.cgb_hdma5.wrapping_sub(1)) & 0x7F;
        }
    }

    fn cgb_hdma5_write(&mut self, value: u8) {
        // Writing bit 7 selects DMA mode:
        // - 0: General DMA (GDMA) transfers all blocks immediately.
        // - 1: HBlank DMA transfers one 0x10-byte block each HBlank.
        //
        // While HBlank DMA is active, writing a value with bit 7 = 0 stops the transfer.
        if !self.is_cgb() {
            return;
        }

        if self.cgb_hdma_active {
            if (value & 0x80) == 0 {
                self.cgb_hdma_active = false;
            }
            return;
        }

        let len = value & 0x7F;
        let blocks = len.wrapping_add(1);
        let hblank = (value & 0x80) != 0;

        self.cgb_hdma5 = len;

        let lcd_enabled = (self.memory[0xFF40] & 0x80) != 0;
        if hblank && lcd_enabled {
            self.cgb_hdma_active = true;
        } else {
            // If the LCD is disabled, treat HBlank DMA as a GDMA transfer so
            // software can still fill VRAM.
            self.cgb_hdma_transfer_blocks(blocks);
            self.cpu_stall_tcycles = self
                .cpu_stall_tcycles
                .saturating_add(self.cgb_hdma_block_stall_tcycles().saturating_mul(blocks as u32));
            self.cgb_hdma5 = 0x7F;
            self.cgb_hdma_active = false;
        }
    }

    #[inline]
    fn cgb_hdma_block_stall_tcycles(&self) -> u32 {
        // Pandocs: ~8 μs per 0x10-byte block.
        // - Normal speed: 8 M-cycles -> 32 T-cycles
        // - Double speed: 16 fast M-cycles -> 64 T-cycles
        if self.cgb_double_speed { 64 } else { 32 }
    }

    pub(super) fn load_rom(&mut self, rom: &[u8]) {
        let cart_type = rom.get(0x147).copied().unwrap_or(0);
        let cgb_flag = rom.get(0x143).copied().unwrap_or(0);
        // Auto-enable CGB mode for CGB-only ROMs (0xC0). For "supports CGB"
        // ROMs (0x80), we keep DMG mode by default until CGB support is more complete,
        // but allow opting in via an environment variable.
        let prefer_cgb = std::env::var("RETROBOY_GB_PREFER_CGB")
            .map(|v| v != "0" && !v.eq_ignore_ascii_case("false"))
            .unwrap_or(false);
        let force_cgb_compat = std::env::var("RETROBOY_GB_FORCE_CGB_COMPAT")
            .map(|v| v != "0" && !v.eq_ignore_ascii_case("false"))
            .unwrap_or(false);

        if force_cgb_compat && cgb_flag != 0xC0 {
            self.set_model(GameBoyModel::CgbCompat);
        } else if cgb_flag == 0xC0 || (cgb_flag == 0x80 && prefer_cgb) {
            self.set_model(GameBoyModel::Cgb);
        } else {
            self.set_model(GameBoyModel::Dmg);
        }
        let rom_size_code = rom.get(0x148).copied().unwrap_or(0);
        let ram_size_code = rom.get(0x149).copied().unwrap_or(0);

        log::info!(
            "GB ROM: bytes={} cart_type=0x{:02X} cgb_flag=0x{:02X} rom_size=0x{:02X} ram_size=0x{:02X} model={:?}",
            rom.len(),
            cart_type,
            cgb_flag,
            rom_size_code,
            ram_size_code,
            self.model
        );

        match cart_type {
            0x00 => {
                // ROM-only (no MBC).
                let len = rom.len().min(MEMORY_SIZE);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = None;
            }
            0x01 | 0x02 | 0x03 => {
                let len = rom.len().min(0x4000);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = Some(Cartridge::new_mbc1(rom));
            }
            0x0F | 0x10 | 0x11 | 0x12 | 0x13 => {
                let len = rom.len().min(0x4000);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = Some(Cartridge::new_mbc3(rom));
            }
            0x19 | 0x1A | 0x1B | 0x1C | 0x1D | 0x1E => {
                let len = rom.len().min(0x4000);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = Some(Cartridge::new_mbc5(rom));
            }
            _ => {
                let len = rom.len().min(MEMORY_SIZE);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = None;
                log::warn!(
                    "Unsupported cartridge type 0x{:02X}; running as ROM-only (no MBC)",
                    cart_type
                );
            }
        }
    }

    pub(super) fn joypad_set_button_bit(&mut self, bit: u8, pressed: bool) {
        let mask = 1u8 << bit;
        if pressed {
            self.joyp_buttons |= mask;
            self.if_reg |= 0x10;
        } else {
            self.joyp_buttons &= !mask;
        }
    }

    pub(super) fn joypad_set_dpad_bit(&mut self, bit: u8, pressed: bool) {
        let mask = 1u8 << bit;
        if pressed {
            self.joyp_dpad |= mask;
            self.if_reg |= 0x10;
        } else {
            self.joyp_dpad &= !mask;
        }
    }
}
