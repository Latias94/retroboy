use anyhow::{anyhow, Result};
use sdl3::event::Event;
use sdl3::keyboard::Keycode;
use sdl3::pixels::PixelFormat as SdlPixelFormat;
use sdl3::render::Canvas;
use sdl3::video::Window;
use std::time::{Duration, Instant};
use typed_builder::TypedBuilder;

pub use retroboy_common;
pub use retroboy_common::app::App;
pub use sdl3;

use retroboy_common::key::Key;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    RGB24,
}

#[derive(TypedBuilder)]
pub struct SdlInitInfo {
    pub width: u32,
    pub height: u32,
    pub scale: u32,
    pub title: String,
    #[builder(default = PixelFormat::RGB24)]
    pub pixel_format: PixelFormat,
}

pub struct SdlContext {
    pub sdl_context: sdl3::Sdl,
    pub event_pump: sdl3::EventPump,
    pub canvas: Canvas<Window>,
    pub width: u32,
    pub height: u32,
    pub scale: u32,
    pub pixel_format: PixelFormat,
}

impl SdlContext {
    pub fn run(sdl_init_info: SdlInitInfo, mut app: impl App) -> Result<()> {
        let SdlInitInfo {
            width,
            height,
            scale,
            title,
            pixel_format,
        } = sdl_init_info;

        let sdl_context = sdl3::init()?;
        let video_subsystem = sdl_context.video()?;
        let window = video_subsystem
            .window(&title, width * scale, height * scale)
            .position_centered()
            .build()
            .map_err(|e| anyhow!(e.to_string()))?;

        let mut canvas: Canvas<Window> = window.into_canvas();
        canvas
            .set_scale(scale as f32, scale as f32)
            .map_err(|e| anyhow!(e.to_string()))?;

        let texture_creator = canvas.texture_creator();
        let sdl_pixel_format = map_pixel_format(pixel_format);
        let mut texture = texture_creator
            .create_texture_streaming(sdl_pixel_format, width, height)
            .map_err(|e| anyhow!(e.to_string()))?;

        let color_size = map_pixel_format_size(pixel_format);
        let mut screen_state = vec![0u8; (width * 3 * height) as usize];

        app.init();
        let mut event_pump = sdl_context.event_pump()?;

        // 目标帧时间 ~16.67ms，对齐 60 FPS / 2MHz CPU 的设计
        let target_frame = Duration::from_micros(16_667);
        let mut last_frame = Instant::now();

        loop {
            if app.should_exit() {
                app.exit();
                break;
            }

            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. } => {
                        app.exit();
                        return Ok(());
                    }
                    Event::KeyDown {
                        keycode: Some(keycode),
                        repeat: false,
                        ..
                    } => {
                        let key = map_keycode(keycode);
                        app.handle_key_event(key, true);
                    }
                    Event::KeyUp {
                        keycode: Some(keycode),
                        ..
                    } => {
                        let key = map_keycode(keycode);
                        app.handle_key_event(key, false);
                    }
                    _ => {}
                }
            }

            app.update(&mut screen_state);

            texture
                .update(None, &screen_state, (width * color_size) as usize)
                .map_err(|e| anyhow!(e.to_string()))?;
            canvas.clear();
            canvas.copy(&texture, None, None)?;
            canvas.present();

            // 简单的帧率限制：如果本帧耗时少于 16.67ms，就 sleep 剩余时间
            let now = Instant::now();
            let elapsed = now.duration_since(last_frame);
            if elapsed < target_frame {
                std::thread::sleep(target_frame - elapsed);
            }
            last_frame = Instant::now();
        }

        Ok(())
    }
}

pub fn map_pixel_format(pixel_format: PixelFormat) -> SdlPixelFormat {
    match pixel_format {
        PixelFormat::RGB24 => SdlPixelFormat::RGB24,
    }
}

pub fn map_pixel_format_size(pixel_format: PixelFormat) -> u32 {
    match pixel_format {
        PixelFormat::RGB24 => 3,
    }
}

pub fn map_keycode(keycode: Keycode) -> Key {
    match keycode {
        Keycode::_1 | Keycode::Kp1 => Key::Num1,
        Keycode::_2 | Keycode::Kp2 => Key::Num2,
        Keycode::_3 | Keycode::Kp3 => Key::Num3,
        Keycode::_4 | Keycode::Kp4 => Key::Num4,
        Keycode::Q => Key::Q,
        Keycode::W => Key::W,
        Keycode::E => Key::E,
        Keycode::R => Key::R,
        Keycode::A => Key::A,
        Keycode::S => Key::S,
        Keycode::D => Key::D,
        Keycode::F => Key::F,
        Keycode::Z => Key::Z,
        Keycode::X => Key::X,
        Keycode::C => Key::C,
        Keycode::V => Key::V,
        Keycode::P => Key::P,
        Keycode::T => Key::T,
        Keycode::J => Key::J,
        Keycode::K => Key::K,
        Keycode::L => Key::L,
        Keycode::Left => Key::Left,
        Keycode::Right => Key::Right,
        Keycode::Up => Key::Up,
        Keycode::Down => Key::Down,
        Keycode::Space => Key::Space,
        _ => Key::None,
    }
}
