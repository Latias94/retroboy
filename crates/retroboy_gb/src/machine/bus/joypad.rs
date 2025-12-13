use super::GameBoyBus;

impl GameBoyBus {
    pub(super) fn read_joyp(&self) -> u8 {
        // Bits 7-6 always read as 1 on DMG.
        let mut result = 0xC0;
        // Bits 5 (buttons) and 4 (d-pad) are selection bits; 0 selects.
        let select = self.joyp_select & 0x30;
        result |= select;

        // Lower nibble is read-only. A pressed button is observed as 0.
        let mut low = 0x0F;
        // D-pad group selected?
        if (select & 0x10) == 0 {
            // joyp_dpad bit=1 means pressed; invert into the low nibble.
            low &= !self.joyp_dpad & 0x0F;
        }
        // Buttons group selected?
        if (select & 0x20) == 0 {
            low &= !self.joyp_buttons & 0x0F;
        }
        result | (low & 0x0F)
    }

    pub(super) fn write_joyp(&mut self, value: u8) {
        // Only bits 5 and 4 are writable; lower nibble is read-only.
        // Store selection bits; bits 7-6 are handled on read.
        self.joyp_select = (self.joyp_select & !0x30) | (value & 0x30);
    }
}
