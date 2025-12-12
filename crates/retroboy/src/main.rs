use retroboy::EmulatorType;

const DEFAULT_CHIP8_ROM: &[u8] =
    include_bytes!("../../../assets/roms/chip8/games/Tetris [Fran Dachille, 1991].ch8");

fn main() {
    env_logger::init();

    let mut args = std::env::args().skip(1);
    let system = args.next().unwrap_or_else(|| "chip8".to_string());
    let rom_path = args.next().unwrap_or_default();

    let emulator = match system.as_str() {
        "chip8" | "CHIP8" => EmulatorType::Chip8,
        "space_invaders" | "space-invaders" | "invaders" => EmulatorType::SpaceInvaders,
        "gb" | "GB" | "gameboy" | "game-boy" => EmulatorType::GameBoy,
        other => {
            eprintln!(
                "Unknown system '{}'. Supported: chip8, space_invaders, gb",
                other
            );
            std::process::exit(1);
        }
    };

    let rom = if rom_path.is_empty() {
        match emulator {
            EmulatorType::Chip8 => {
                log::info!("No ROM path provided, playing bundled CHIP-8 Tetris");
                DEFAULT_CHIP8_ROM.to_vec()
            }
            EmulatorType::SpaceInvaders => {
                eprintln!(
                    "No ROM path provided for Space Invaders.\n\
                     Please specify a path, for example:\n\
                     retroboy space_invaders assets/roms/space_invaders/space-invaders.rom"
                );
                std::process::exit(1);
            }
            EmulatorType::GameBoy => {
                eprintln!(
                    "No ROM path provided for Game Boy.\n\
                     Please specify a path, for example:\n\
                     retroboy gb path/to/your.gb"
                );
                std::process::exit(1);
            }
        }
    } else {
        log::info!("Playing ROM path: '{}'", rom_path);
        std::fs::read(rom_path).expect("Failed to read ROM file")
    };

    retroboy::run(emulator, &rom).unwrap();
}
