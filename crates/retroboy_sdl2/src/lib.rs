use anyhow::Result;
use sdl2::pixels::PixelFormatEnum;
use sdl2::render::WindowCanvas;
use typed_builder::TypedBuilder;

use retroboy_common::key::Key;
pub use sdl2;
use sdl2::event::Event;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    RGB24,
}

pub trait App {
    fn init(&mut self);
    fn update(&mut self, screen: &mut [u8]);
    fn handle_key_event(&mut self, key: Key, is_down: bool);
    fn should_exit(&self) -> bool;
    fn exit(&mut self);

    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn scale(&self) -> u32;
    fn title(&self) -> String;
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
    pub sdl_context: sdl2::Sdl,
    pub event_pump: sdl2::EventPump,
    pub canvas: WindowCanvas,
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
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();
        let window = video_subsystem
            .window(&title, width * scale, height * scale)
            .position_centered()
            .build()?;
        let mut canvas = window.into_canvas().present_vsync().build()?;
        canvas.set_scale(scale as f32, scale as f32).unwrap();
        let creator = canvas.texture_creator();
        // or use unsafe_texture feature if not aware of lifetime
        let mut texture = creator
            .create_texture_target(map_pixel_format(pixel_format), width, height)
            .unwrap();

        let color_size = map_pixel_format_size(pixel_format);
        let mut screen_state = vec![0u8; (width * 3 * height) as usize];
        app.init();
        loop {
            if app.should_exit() {
                app.exit();
                break;
            }
            let mut event_pump = sdl_context.event_pump().unwrap();

            while let Some(event) = event_pump.poll_event() {
                match event {
                    Event::Quit { .. } => {
                        app.exit();
                        return Ok(());
                    }
                    Event::KeyDown {
                        keycode: Some(keycode),
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
                .unwrap();
            canvas.copy(&texture, None, None).unwrap();
            canvas.present();
            std::thread::sleep(std::time::Duration::new(0, 70_000));
        }

        Ok(())
    }
}

pub fn map_pixel_format(pixel_format: PixelFormat) -> PixelFormatEnum {
    match pixel_format {
        PixelFormat::RGB24 => PixelFormatEnum::RGB24,
    }
}
pub fn map_pixel_format_size(pixel_format: PixelFormat) -> u32 {
    match pixel_format {
        PixelFormat::RGB24 => 3,
    }
}

pub fn map_keycode(keycode: sdl2::keyboard::Keycode) -> Key {
    match keycode {
        sdl2::keyboard::Keycode::Num1 => Key::Num1,
        sdl2::keyboard::Keycode::Num2 => Key::Num2,
        sdl2::keyboard::Keycode::Num3 => Key::Num3,
        sdl2::keyboard::Keycode::Num4 => Key::Num4,
        sdl2::keyboard::Keycode::Q => Key::Q,
        sdl2::keyboard::Keycode::W => Key::W,
        sdl2::keyboard::Keycode::E => Key::E,
        sdl2::keyboard::Keycode::R => Key::R,
        sdl2::keyboard::Keycode::A => Key::A,
        sdl2::keyboard::Keycode::S => Key::S,
        sdl2::keyboard::Keycode::D => Key::D,
        sdl2::keyboard::Keycode::F => Key::F,
        sdl2::keyboard::Keycode::Z => Key::Z,
        sdl2::keyboard::Keycode::X => Key::X,
        sdl2::keyboard::Keycode::C => Key::C,
        sdl2::keyboard::Keycode::V => Key::V,
        _ => Key::None,
    }
}
