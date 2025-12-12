pub mod app;
pub mod cpu;
pub mod machine;
pub mod sound;

pub use app::SpaceInvadersApp;
pub use machine::SpaceInvadersMachine;

/// Logical screen width in pixels (Space Invaders is 224x256, rotated).
pub const SCREEN_WIDTH: usize = 224;
/// Logical screen height in pixels.
pub const SCREEN_HEIGHT: usize = 256;
/// Default integer scaling factor for the SDL2 frontend.
pub const SCREEN_SCALE: u32 = 3;
