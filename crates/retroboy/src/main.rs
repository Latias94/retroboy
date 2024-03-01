use retroboy_sdl2::{SdlContext, SdlInitInfo};

const TITLE: &str = "RetroBoy";

fn main() {
    env_logger::init();

    let width = 64;
    let height = 32;
    let scale = 10;
    let init_info = SdlInitInfo::builder()
        .width(width)
        .height(height)
        .scale(scale)
        .title(TITLE.to_string())
        .build();
    let app = retroboy_chip8::Chip8::new();
    SdlContext::run(init_info, app).unwrap();
}
