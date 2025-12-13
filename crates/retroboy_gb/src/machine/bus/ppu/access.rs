use super::super::GameBoyBus;

impl GameBoyBus {
    /// Check whether VRAM ($8000-$9FFF) is currently accessible to the CPU.
    ///
    /// According to Pandocs ("Accessing VRAM and OAM"), VRAM is accessible in
    /// PPU modes 0, 1 and 2 and becomes inaccessible in mode 3. While the LCD
    /// is disabled (LCDC.7=0), VRAM is always accessible.
    pub(in super::super) fn vram_accessible(&self) -> bool {
        let lcdc = self.memory[0xFF40];
        if (lcdc & 0x80) == 0 {
            // LCD off: VRAM is always accessible.
            return true;
        }
        let stat = self.memory[0xFF41];
        let mode = stat & 0x03;
        mode != 3
    }

    /// Check whether OAM ($FE00-$FE9F) is currently accessible to the CPU.
    ///
    /// Per Pandocs, OAM is accessible in modes 0 and 1 (HBlank / VBlank) and
    /// inaccessible in modes 2 and 3. While the LCD is disabled, OAM is
    /// always accessible.
    pub(in super::super) fn oam_accessible(&self) -> bool {
        let lcdc = self.memory[0xFF40];
        if (lcdc & 0x80) == 0 {
            // LCD off: OAM is always accessible.
            return true;
        }
        let stat = self.memory[0xFF41];
        let mode = stat & 0x03;
        mode == 0 || mode == 1
    }
}
