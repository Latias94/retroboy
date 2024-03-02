use retroboy_sdl2::{App, SdlContext, SdlInitInfo};
use std::io;

const TITLE: &str = "RetroBoy";

fn main() {
    env_logger::init();

    let mut app = retroboy_chip8::EmulatorApp::default();
    let rom = load_rom().unwrap();
    app.emulator.load_rom(&rom);
    let width = app.width();
    let height = app.height();
    let scale = app.scale();
    let init_info = SdlInitInfo::builder()
        .width(width)
        .height(height)
        .scale(scale)
        .title(TITLE.to_string())
        .build();
    SdlContext::run(init_info, app).unwrap();
}

// root/assets/roms/chip8/games/Airplane.ch8
fn load_rom() -> io::Result<Vec<u8>> {
    let path = "assets/roms/chip8/games/Pong [Paul Vervalin, 1990].ch8";
    std::fs::read(path)
}
