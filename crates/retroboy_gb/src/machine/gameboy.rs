use crate::cpu::Cpu;
use crate::cpu::Bus;
use retroboy_common::key::Key;

use super::{video, GameBoyBus};

/// High-level Game Boy machine.
///
/// Holds the CPU core and the bus. This is the main entry point used by the
/// `GameBoyApp` wrapper.
pub struct GameBoy {
    pub cpu: Cpu,
    pub(crate) bus: GameBoyBus,
}

impl Default for GameBoy {
    fn default() -> Self {
        Self::new()
    }
}

impl GameBoy {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            bus: GameBoyBus::default(),
        }
    }

    /// Test-only constructor that initialises the machine with deterministic
    /// internal RAM contents.
    ///
    /// Our production `GameBoyBus` fills WRAM/HRAM with pseudo-random bytes to
    /// approximate DMG power-on behaviour. For lockstep comparison against
    /// reference cores (such as mooneye-gb), it is useful to start from a
    /// zero-initialised RAM state instead so that any divergence is caused by
    /// timing/logic differences rather than random data patterns.
    #[cfg(test)]
    pub fn new_zeroed_internal_ram() -> Self {
        let mut gb = GameBoy::new();

        // Clear WRAM: 0xC000..0xDFFF.
        for addr in 0xC000u16..=0xDFFF {
            gb.bus.memory[addr as usize] = 0;
        }

        // Echo RAM 0xE000..0xFDFF already reads/writes through WRAM in our
        // model, but we clear it as well for completeness.
        for addr in 0xE000u16..=0xFDFF {
            gb.bus.memory[addr as usize] = 0;
        }

        // Clear HRAM: 0xFF80..0xFFFE.
        for addr in 0xFF80u16..=0xFFFE {
            gb.bus.memory[addr as usize] = 0;
        }

        gb
    }

    pub fn reset(&mut self) {
        self.cpu.reset();
        self.bus = GameBoyBus::default();
    }

    /// Load a ROM image into the cartridge ROM area.
    pub fn load_rom(&mut self, rom: &[u8]) {
        self.bus.load_rom(rom);
        // PC is already initialised to 0x0100 by the simulated boot state.
    }

    /// Step the machine for one frame worth of time.
    ///
    /// This advances the CPU until we have consumed roughly one DMG frame's
    /// worth of time (70224 T-cycles).
    pub fn step_frame(&mut self) {
        const PPU_CYCLES_PER_FRAME: u32 = 70_224;
        let mut ppu_cycles = 0u32;

        while ppu_cycles < PPU_CYCLES_PER_FRAME {
            let taken = self.cpu.step(&mut self.bus);
            if taken == 0 {
                // Locked CPU (invalid opcode) or other hard-stop condition.
                break;
            }
            ppu_cycles = ppu_cycles.saturating_add(self.bus.last_ppu_cycles());
        }
    }

    /// Run until the test ROM's "software breakpoint" instruction `LD B,B`
    /// (`0x40`) is about to execute, then execute it and stop.
    ///
    /// This convention is used by the mealybug-tearoom-tests suite to signal
    /// that a screenshot can be taken.
    ///
    /// This uses the CPU micro-step API so that cycle-sensitive PPU tests
    /// (such as mealybug-tearoom) can observe IO writes at the correct
    /// M-cycle.
    ///
    /// Returns `true` if the breakpoint was hit before reaching
    /// `max_mcycles`.
    pub fn step_until_software_breakpoint(&mut self, max_mcycles: u64) -> bool {
        for _ in 0..max_mcycles {
            if self.cpu.micro_is_idle()
                && !self.cpu.halted
                && !self.cpu.is_stopped()
                && !self.bus.cgb_speed_switch_pause_active()
            {
                let pc = self.cpu.regs.pc;
                let opcode = self.bus.read8(pc);
                if opcode == 0x40 {
                    self.cpu.step_mcycle(&mut self.bus);
                    return true;
                }
            }

            let taken = self.cpu.step_mcycle(&mut self.bus);
            if taken == 0 {
                return false;
            }
        }
        false
    }

    /// Update joypad state from a frontend key event.
    ///
    /// This maps a small subset of the shared `Key` enum onto DMG buttons:
    /// - Z => A, X => B
    /// - A => Select, S => Start
    /// - Arrow keys => D-pad
    pub fn handle_key(&mut self, key: Key, pressed: bool) {
        match key {
            // D-pad.
            Key::Right => self.set_dpad_bit(0, pressed),
            Key::Left => self.set_dpad_bit(1, pressed),
            Key::Up => self.set_dpad_bit(2, pressed),
            Key::Down => self.set_dpad_bit(3, pressed),
            // Face / system buttons.
            Key::Z => self.set_button_bit(0, pressed), // A
            Key::X => self.set_button_bit(1, pressed), // B
            Key::A => self.set_button_bit(2, pressed), // Select
            Key::S => self.set_button_bit(3, pressed), // Start
            _ => {}
        }
    }

    fn set_button_bit(&mut self, bit: u8, pressed: bool) {
        self.bus.joypad_set_button_bit(bit, pressed);
    }

    fn set_dpad_bit(&mut self, bit: u8, pressed: bool) {
        self.bus.joypad_set_dpad_bit(bit, pressed);
    }

    /// Placeholder video output.
    ///
    /// The real Game Boy uses a tile-based PPU to render into an internal
    /// framebuffer. We will add a proper PPU module and backing buffer; for
    /// now we generate a very small background-only view directly from VRAM.
    pub fn video_frame<'a>(&'a self, buffer: &'a mut [u8]) {
        if self.bus.cgb_ppu_framebuffer_copy(buffer) {
            return;
        }
        if self.bus.dmg_ppu_framebuffer_copy(buffer) {
            return;
        }
        video::render_video_frame(&self.bus, buffer);
    }
}
