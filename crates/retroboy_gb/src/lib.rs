pub mod app;
pub mod cpu;
mod cpu_micro;
pub mod machine;

pub use app::GameBoyApp;
pub use machine::GameBoy;

/// Logical screen width in pixels for the Game Boy DMG.
pub const SCREEN_WIDTH: usize = 160;
/// Logical screen height in pixels.
pub const SCREEN_HEIGHT: usize = 144;
/// Default integer scaling factor for the SDL frontend.
pub const SCREEN_SCALE: u32 = 4;
