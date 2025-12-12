use anyhow::Result;
use retroboy_sdl3::App;
use retroboy_sdl3::{SdlContext, SdlInitInfo};

pub enum EmulatorType {
    Chip8,
    SpaceInvaders,
    GameBoy,
}

pub fn run(emulator: EmulatorType, rom_data: &[u8]) -> Result<()> {
    match emulator {
        EmulatorType::Chip8 => {
            run_chip8(rom_data)?;
        }
        EmulatorType::SpaceInvaders => {
            run_space_invaders(rom_data)?;
        }
        EmulatorType::GameBoy => {
            run_gameboy(rom_data)?;
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

pub fn run_space_invaders(rom_data: &[u8]) -> Result<()> {
    let mut app = retroboy_space_invaders::SpaceInvadersApp::default();
    app.machine.load_rom(rom_data);
    let width = app.width();
    let height = app.height();
    let scale = app.scale();
    let init_info = SdlInitInfo::builder()
        .width(width)
        .height(height)
        .scale(scale)
        .title("RetroBoy Space Invaders".to_string())
        .build();
    SdlContext::run(init_info, app)?;
    Ok(())
}

pub fn run_gameboy(rom_data: &[u8]) -> Result<()> {
    let mut app = retroboy_gb::GameBoyApp::default();
    app.gb.load_rom(rom_data);
    let width = app.width();
    let height = app.height();
    let scale = app.scale();
    let init_info = SdlInitInfo::builder()
        .width(width)
        .height(height)
        .scale(scale)
        .title("RetroBoy Game Boy".to_string())
        .build();
    SdlContext::run(init_info, app)?;
    Ok(())
}
