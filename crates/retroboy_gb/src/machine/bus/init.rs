use super::GameBoyBus;

impl GameBoyBus {
    /// Initialize I/O registers to match the DMG/MGB power-on state.
    ///
    /// This is called by `Default` to approximate the boot ROM's post-initialization state.
    ///
    pub(super) fn apply_dmg_initial_io_state(&mut self) {
        // Internal WRAM/HRAM power‑on contents are effectively random on
        // hardware. We approximate that with a small deterministic PRNG so
        // test runs are reproducible while software still cannot rely on
        // specific values.
        self.randomize_internal_ram();

        // Joypad.
        self.memory[0xFF00] = 0xCF;

        // Serial.
        self.serial.sb = 0x00;
        self.serial.sc = 0x7E;

        // Divider / timer.
        self.timer.init_dmg();

        // Interrupt flags and enable.
        //
        // IF's upper 3 bits always read as 1 on DMG; the hardware happens
        // to have bit 0 set at PC=0x0100 as well (VBlank request).
        self.if_reg = 0x01; // lower 5 bits only; read8() ORs with 0xE0.
        self.ie_reg = 0x00;

        // Sound registers (we do not emulate the APU yet, but default values
        // are visible to software).
        self.memory[0xFF10] = 0x80;
        self.memory[0xFF11] = 0xBF;
        self.memory[0xFF12] = 0xF3;
        self.memory[0xFF13] = 0xFF;
        self.memory[0xFF14] = 0xBF;
        self.memory[0xFF16] = 0x3F;
        self.memory[0xFF17] = 0x00;
        self.memory[0xFF18] = 0xFF;
        self.memory[0xFF19] = 0xBF;
        self.memory[0xFF1A] = 0x7F;
        self.memory[0xFF1B] = 0xFF;
        self.memory[0xFF1C] = 0x9F;
        self.memory[0xFF1D] = 0xFF;
        self.memory[0xFF1E] = 0xBF;
        self.memory[0xFF20] = 0xFF;
        self.memory[0xFF21] = 0x00;
        self.memory[0xFF22] = 0x00;
        self.memory[0xFF23] = 0xBF;
        self.memory[0xFF24] = 0x77;
        self.memory[0xFF25] = 0xF3;
        self.memory[0xFF26] = 0xF1;

        // PPU registers.
        self.memory[0xFF40] = 0x91; // LCDC
        self.memory[0xFF41] = 0x85; // STAT (DMG)
        self.memory[0xFF42] = 0x00; // SCY
        self.memory[0xFF43] = 0x00; // SCX
        self.memory[0xFF44] = 0x00; // LY
        self.memory[0xFF45] = 0x00; // LYC
        self.memory[0xFF46] = 0xFF; // DMA
        self.memory[0xFF47] = 0xFC; // BGP
                                    // OBP0/OBP1 are officially "uninitialized"; we leave them at 0.
                                    // Window position.
        self.memory[0xFF4A] = 0x00; // WY
        self.memory[0xFF4B] = 0x00; // WX
    }

    /// Fill internal WRAM and HRAM with pseudo‑random bytes.
    ///
    /// Pandocs notes that these RAM areas contain "random" data at power‑on.
    /// For reproducibility we use a fixed‑seed xorshift PRNG rather than true
    /// randomness. This discourages code from depending on a specific pattern
    /// (it is not all‑zero) while keeping emulator runs deterministic.
    fn randomize_internal_ram(&mut self) {
        // Simple xorshift32 PRNG.
        let mut x: u32 = 0xC0DE_1234;
        let mut next_byte = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x as u8
        };

        // WRAM: 0xC000–0xDFFF.
        for addr in 0xC000..=0xDFFF {
            self.memory[addr] = next_byte();
        }

        // Echo RAM 0xE000–0xFDFF mirrors 0xC000–0xDDFF from the CPU's point
        // of view; we do not need to seed it separately because reads go
        // through the mirrored address, but we fill it for completeness.
        for addr in 0xE000..=0xFDFF {
            self.memory[addr] = 0x00;
        }

        // HRAM: 0xFF80–0xFFFE (IE at 0xFFFF is handled separately).
        for addr in 0xFF80..=0xFFFE {
            self.memory[addr] = next_byte();
        }
    }
}
