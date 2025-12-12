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
}

impl App for GameBoyApp {
    fn init(&mut self) {
        log::info!("Game Boy init");
    }

    fn update(&mut self, screen_state: &mut [u8]) {
        // Step one frame worth of CPU work. The timing here is approximate
        // and will be revisited once we introduce a proper PPU and timers.
        self.gb.step_frame();

        // For now we render a blank (black) screen. Once the PPU is in place
        // this call will forward to the PPU framebuffer.
        self.gb.video_frame(screen_state);
    }

    fn handle_key_event(&mut self, key: Key, is_pressed: bool) {
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
