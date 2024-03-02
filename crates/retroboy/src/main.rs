use retroboy::EmulatorType;

const DEFAULT_ROM: &[u8] =
    include_bytes!("../../../assets/roms/chip8/games/Tetris [Fran Dachille, 1991].ch8");

fn main() {
    env_logger::init();
    let rom_path = std::env::args().nth(1).unwrap_or("".to_string());
    
    let rom = if rom_path.is_empty() {
        log::info!("No ROM path provided, play bundled game Tetris");
        DEFAULT_ROM.to_vec()
    } else {
        log::info!("Playing ROM path: '{}'", rom_path);
        std::fs::read(rom_path).unwrap()
    };
    retroboy::run(EmulatorType::Chip8, &rom).unwrap();
}
