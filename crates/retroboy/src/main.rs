use retroboy_sdl2::App;
use retroboy_sdl2::{SdlContext, SdlInitInfo};

const TITLE: &str = "RetroBoy";
const DEFAULT_ROM_PATH: &str = "assets/roms/chip8/games/Tetris [Fran Dachille, 1991].ch8";

fn main() {
    env_logger::init();

    let mut rom_path = std::env::args()
        .nth(1)
        .unwrap_or(DEFAULT_ROM_PATH.to_string());
    if rom_path.is_empty() {
        rom_path = DEFAULT_ROM_PATH.to_string();
    }

    let rom = std::fs::read(rom_path).unwrap();

    let mut app = retroboy_chip8::EmulatorApp::default();
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
