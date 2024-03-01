use anyhow::Result;
use sdl2::pixels::PixelFormatEnum;
use sdl2::render::WindowCanvas;
use typed_builder::TypedBuilder;

pub use sdl2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PixelFormat {
    RGB24,
}

pub trait App {
    fn init(&mut self);
    fn update(&mut self, screen: &mut [u8]);
    fn handle_events(&mut self, event_pump: &mut sdl2::EventPump);
    fn should_exit(&self) -> bool;
    fn exit(&mut self);
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
        let mut canvas = window.into_canvas().build()?;
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
            app.update(&mut screen_state);
            app.handle_events(&mut event_pump);

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
