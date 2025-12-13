mod bus;
mod cartridge;
mod gameboy;
mod serial;
mod timer;
mod video;

pub(crate) use bus::GameBoyBus;
pub use gameboy::GameBoy;

/// Total addressable memory for the Game Boy (64 KiB).
///
/// The real hardware has a more complex memory map with cartridge ROM/RAM,
/// VRAM, WRAM, HRAM, IO registers, etc. We start with a flat array and will
/// evolve it into a structured bus as features are implemented.
const MEMORY_SIZE: usize = 0x10000;

#[cfg(test)]
mod tests;
