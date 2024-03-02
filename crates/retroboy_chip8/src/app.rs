use crate::emulator::Emulator;
use crate::{SCREEN_HEIGHT, SCREEN_SCALE, SCREEN_WIDTH, TICKS_PER_FRAME};
use retroboy_common::color::Color;
use retroboy_common::key::Key;
use retroboy_sdl2::App;

#[derive(Default)]
pub struct EmulatorApp {
    should_exit: bool,
    pub emulator: Emulator,
}

impl App for EmulatorApp {
    fn init(&mut self) {
        log::info!("Chip8 init");
    }

    fn update(&mut self, screen_state: &mut [u8]) {
        for _ in 0..TICKS_PER_FRAME {
            self.emulator.tick();
        }
        self.emulator.tick_timers();

        let display = self.emulator.get_display();

        for (i, pixel) in display.iter().enumerate() {
            let x = i % SCREEN_WIDTH;
            let y = i / SCREEN_WIDTH;
            let color = if *pixel { Color::BLACK } else { Color::WHITE };
            let index = (y * SCREEN_WIDTH + x) * 3;
            screen_state[index] = color.r;
            screen_state[index + 1] = color.g;
            screen_state[index + 2] = color.b;
        }
    }

    fn handle_key_event(&mut self, key: Key, is_pressed: bool) {
        if let Some(idx) = crate::key2idx(key) {
            self.emulator.set_key(idx, is_pressed);
        }
    }

    fn should_exit(&self) -> bool {
        self.should_exit
    }

    fn exit(&mut self) {
        log::info!("Chip8 exit");
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
        "RetroBoy Chip-8".to_string()
    }
}
