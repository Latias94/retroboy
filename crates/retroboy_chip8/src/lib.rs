mod cpu;

use retroboy_sdl2::sdl2::event::Event;
use retroboy_sdl2::sdl2::keyboard::Keycode;
use retroboy_sdl2::{sdl2, App};

pub struct Chip8 {
    should_exit: bool,
}

impl Chip8 {
    pub fn new() -> Self {
        Chip8 { should_exit: false }
    }
}

impl App for Chip8 {
    fn init(&mut self) {
        log::info!("Chip8 init");
    }
    fn update(&mut self, screen_state: &mut [u8]) {}
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
}
