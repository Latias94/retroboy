use crate::emulator::Emulator;
use crate::{SCREEN_HEIGHT, SCREEN_SCALE, SCREEN_WIDTH};
use retroboy_common::Color;
use retroboy_sdl2::sdl2::event::Event;
use retroboy_sdl2::sdl2::keyboard::Keycode;
use retroboy_sdl2::{sdl2, App};

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
        self.emulator.tick();
    }

    fn handle_events(&mut self, event_pump: &mut sdl2::EventPump) {
        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => self.should_exit = true,
                Event::KeyDown {
                    keycode: Some(Keycode::W),
                    ..
                } => {}
                Event::KeyDown {
                    keycode: Some(Keycode::S),
                    ..
                } => {}
                Event::KeyDown {
                    keycode: Some(Keycode::A),
                    ..
                } => {}
                Event::KeyDown {
                    keycode: Some(Keycode::D),
                    ..
                } => {}
                _ => { /* do nothing */ }
            }
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
