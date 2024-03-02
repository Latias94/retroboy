use retroboy::EmulatorType;

const DEFAULT_ROM_PATH: &str = "assets/roms/chip8/games/Tetris [Fran Dachille, 1991].ch8";

fn main() {
    env_logger::init();
    let mut rom_path = std::env::args().nth(1).unwrap_or("".to_string());
    if rom_path.is_empty() {
        rom_path = DEFAULT_ROM_PATH.to_string();
        log::info!("No ROM path provided, using default: {}", rom_path);
    } else {
        log::info!("No ROM path provided, using default: {}", rom_path);
    }
    retroboy::run(EmulatorType::Chip8, &rom_path).unwrap();
}
