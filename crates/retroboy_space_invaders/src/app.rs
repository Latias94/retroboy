use crate::machine::SpaceInvadersMachine;
use crate::sound::SoundManager;
use crate::{SCREEN_HEIGHT, SCREEN_SCALE, SCREEN_WIDTH};
use retroboy_common::app::App;
use retroboy_common::color::Color;
use retroboy_common::key::Key;

/// SDL2-facing application wrapper for the Space Invaders machine.
///
/// This type implements the shared `App` trait so that the SDL2 frontend
/// (`retroboy_sdl2`) can drive the emulator in the same way as CHIP-8.
#[derive(Default)]
pub struct SpaceInvadersApp {
    should_exit: bool,
    paused: bool,
    pub machine: SpaceInvadersMachine,
    sound: Option<SoundManager>,
}

impl App for SpaceInvadersApp {
    fn init(&mut self) {
        log::info!("Space Invaders init");
        // Try to bring up audio for discrete sounds. If this fails, the
        // game will still run but without sound effects.
        if self.sound.is_none() {
            self.sound = SoundManager::new();
        }
    }

    fn update(&mut self, screen_state: &mut [u8]) {
        if !self.paused {
            self.machine.step_frame();

            if let Some(sound) = &mut self.sound {
                let (out3, out5) = self.machine.outputs();
                sound.update(out3, out5);
            }
        }

        let vram = self.machine.video_ram();
        render_video(vram, screen_state);

        if self.paused {
            overlay_pause_banner(screen_state);
        }
    }

    fn handle_key_event(&mut self, key: Key, is_pressed: bool) {
        if is_pressed {
            match key {
                // Toggle pause on 'P', like the reference emulator.
                Key::P => {
                    self.paused = !self.paused;
                    return;
                }
                // Any other key will unpause if the game is currently paused.
                _ if self.paused => {
                    self.paused = false;
                }
                _ => {}
            }
        }

        self.machine.handle_key(key, is_pressed);
    }

    fn should_exit(&self) -> bool {
        self.should_exit
    }

    fn exit(&mut self) {
        log::info!("Space Invaders exit");
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
        "RetroBoy Space Invaders".to_string()
    }
}

fn render_video(vram: &[u8], screen_state: &mut [u8]) {
    let width = SCREEN_WIDTH;
    let height = SCREEN_HEIGHT;

    // Space Invaders uses 0x1c00 bytes of video RAM starting at 0x2400.
    // Each byte encodes 8 vertical pixels; there are 224 columns and
    // 32 bytes per column (32 * 8 = 256 rows).
    debug_assert_eq!(vram.len(), 0x1c00);
    debug_assert_eq!(screen_state.len(), width * height * 3);

    let mut i = 0usize;
    for x in 0..width {
        for iy in (0..height).step_by(8) {
            let mut byte = vram[i];
            i += 1;
            for b in 0..8 {
                let pixel_on = (byte & 1) != 0;
                byte >>= 1;

                let screen_x = x;
                let screen_y = height as i32 - (iy as i32 + b as i32) - 1;

                let idx = (screen_y as usize * width + screen_x) * 3;
                let color = if !pixel_on {
                    Color::BLACK
                } else if iy > 200 && iy < 220 {
                    Color::RED
                } else if iy < 80 {
                    Color::GREEN
                } else {
                    Color::WHITE
                };

                screen_state[idx] = color.r;
                screen_state[idx + 1] = color.g;
                screen_state[idx + 2] = color.b;
            }
        }
    }
}

/// Draw a very simple pause banner at the top of the screen so it is visually
/// obvious when the emulator is paused.
fn overlay_pause_banner(screen_state: &mut [u8]) {
    let width = SCREEN_WIDTH;
    let height = SCREEN_HEIGHT;
    debug_assert_eq!(screen_state.len(), width * height * 3);

    // Use a small horizontal band at the top of the screen.
    let banner_height = 12usize.min(height);

    for y in 0..banner_height {
        for x in 0..width {
            let idx = (y * width + x) * 3;
            // Alternate dark and bright rows for a simple striped effect.
            let color = if y % 2 == 0 {
                Color::WHITE
            } else {
                Color::BLACK
            };
            screen_state[idx] = color.r;
            screen_state[idx + 1] = color.g;
            screen_state[idx + 2] = color.b;
        }
    }
}
