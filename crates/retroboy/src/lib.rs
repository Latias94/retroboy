use anyhow::Result;
use retroboy_sdl2::App;
use retroboy_sdl2::{SdlContext, SdlInitInfo};

pub enum EmulatorType {
    Chip8,
}

pub fn run(emulator: EmulatorType, rom_data: &[u8]) -> Result<()> {
    match emulator {
        EmulatorType::Chip8 => {
            run_chip8(rom_data)?;
        }
    }
    Ok(())
}

pub fn run_chip8(rom_data: &[u8]) -> Result<()> {
    let mut app = retroboy_chip8::EmulatorApp::default();
    app.emulator.load_rom(rom_data);
    let width = app.width();
    let height = app.height();
    let scale = app.scale();
    let init_info = SdlInitInfo::builder()
        .width(width)
        .height(height)
        .scale(scale)
        .title("RetroBoy Chip-8".to_string())
        .build();
    SdlContext::run(init_info, app)?;
    Ok(())
}
