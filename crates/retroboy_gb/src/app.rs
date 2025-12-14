use crate::{GameBoy, SCREEN_HEIGHT, SCREEN_SCALE, SCREEN_WIDTH};
use retroboy_common::app::App;
use retroboy_common::key::Key;

/// SDL-facing application wrapper for the Game Boy machine.
///
/// This type implements the shared `App` trait so that the SDL frontend can
/// drive the emulator in the same way as CHIP-8 and Space Invaders.
#[derive(Default)]
pub struct GameBoyApp {
    should_exit: bool,
    pub gb: GameBoy,
    frame_counter: u64,
    last_pc: u16,
    pc_stagnant_frames: u32,
    lcdc_off_frames: u32,
    last_lcdc: u8,
}

impl App for GameBoyApp {
    fn init(&mut self) {
        log::info!("Game Boy init");
        self.last_pc = self.gb.cpu.regs.pc;
        self.last_lcdc = self.gb.bus.memory[0xFF40];
    }

    fn update(&mut self, screen_state: &mut [u8]) {
        // Step one frame worth of CPU work. The timing here is approximate
        // and will be revisited once we introduce a proper PPU and timers.
        self.gb.step_frame();

        // For now we render a blank (black) screen. Once the PPU is in place
        // this call will forward to the PPU framebuffer.
        self.gb.video_frame(screen_state);

        self.frame_counter = self.frame_counter.wrapping_add(1);

        let regs = &self.gb.cpu.regs;
        let pc = regs.pc;
        if pc == self.last_pc {
            self.pc_stagnant_frames = self.pc_stagnant_frames.saturating_add(1);
        } else {
            self.pc_stagnant_frames = 0;
            self.last_pc = pc;
        }

        let lcdc = self.gb.bus.memory[0xFF40];
        let lcd_enabled = (lcdc & 0x80) != 0;
        let bg_enabled = (lcdc & 0x01) != 0;
        if !lcd_enabled || !bg_enabled {
            self.lcdc_off_frames = self.lcdc_off_frames.saturating_add(1);
        } else {
            self.lcdc_off_frames = 0;
        }

        if lcdc != self.last_lcdc {
            log::debug!("GB LCDC changed: 0x{:02X} -> 0x{:02X}", self.last_lcdc, lcdc);
            self.last_lcdc = lcdc;
        }

        if self.frame_counter == 1 || self.frame_counter % 60 == 0 {
            let stat = self.gb.bus.memory[0xFF41];
            let scy = self.gb.bus.memory[0xFF42];
            let scx = self.gb.bus.memory[0xFF43];
            let ly = self.gb.bus.memory[0xFF44];
            let bgp = self.gb.bus.memory[0xFF47];
            let wy = self.gb.bus.memory[0xFF4A];
            let wx = self.gb.bus.memory[0xFF4B];
            let window_enabled = (lcdc & 0x20) != 0;
            log::info!(
                "GB: frame={} pc=0x{:04X} sp=0x{:04X} af=0x{:04X} bc=0x{:04X} de=0x{:04X} hl=0x{:04X} ime={} halted={} stopped={} locked={} IF=0x{:02X} IE=0x{:02X} LCDC=0x{:02X} STAT=0x{:02X} LY={} SCX={} SCY={} BGP=0x{:02X} WX={} WY={} WIN={}",
                self.frame_counter,
                regs.pc,
                regs.sp,
                regs.af(),
                regs.bc(),
                regs.de(),
                regs.hl(),
                self.gb.cpu.ime,
                self.gb.cpu.halted,
                self.gb.cpu.is_stopped(),
                self.gb.cpu.is_locked(),
                self.gb.bus.if_reg,
                self.gb.bus.ie_reg,
                lcdc,
                stat,
                ly,
                scx,
                scy,
                bgp,
                wx,
                wy,
                window_enabled,
            );

            if log::log_enabled!(log::Level::Debug) {
                let vram = &self.gb.bus.memory[0x8000..0xA000];
                let mut vram_sum: u32 = 0;
                let mut vram_nonzero: u32 = 0;
                for &b in vram {
                    vram_sum = vram_sum.wrapping_add(b as u32);
                    if b != 0 {
                        vram_nonzero += 1;
                    }
                }

                let bg_map_base = if (lcdc & 0x08) != 0 { 0x9C00 } else { 0x9800 };
                let win_map_base = if (lcdc & 0x40) != 0 { 0x9C00 } else { 0x9800 };
                let bg00 = self.gb.bus.memory[bg_map_base as usize];
                let win00 = self.gb.bus.memory[win_map_base as usize];

                log::debug!(
                    "GB video: vram_nonzero={} vram_sum={} bg_map=0x{:04X} bg00=0x{:02X} win_map=0x{:04X} win00=0x{:02X}",
                    vram_nonzero,
                    vram_sum,
                    bg_map_base,
                    bg00,
                    win_map_base,
                    win00,
                );
            }
        }

        if self.lcdc_off_frames == 120 {
            log::warn!(
                "GB: LCD/BG still disabled after ~120 frames (LCDC=0x{:02X}); placeholder renderer will show white",
                lcdc
            );
        }

        if self.pc_stagnant_frames == 600 {
            log::warn!(
                "GB: PC unchanged for ~600 frames at 0x{:04X} (halted={} stopped={} locked={})",
                pc,
                self.gb.cpu.halted,
                self.gb.cpu.is_stopped(),
                self.gb.cpu.is_locked(),
            );
        }
    }

    fn handle_key_event(&mut self, key: Key, is_pressed: bool) {
        log::debug!("GB key event: {:?} pressed={}", key, is_pressed);
        // Forward key events to the Game Boy joypad.
        self.gb.handle_key(key, is_pressed);
    }

    fn should_exit(&self) -> bool {
        self.should_exit
    }

    fn exit(&mut self) {
        log::info!("Game Boy exit");
    }

    fn width(&self) -> u32 {
        SCREEN_WIDTH as u32
    }

    fn height(&self) -> u32 {
        SCREEN_HEIGHT as u32
    }

    fn scale(&self) -> u32 {
        SCREEN_SCALE
    }

    fn title(&self) -> String {
        "RetroBoy Game Boy".to_string()
    }
}
