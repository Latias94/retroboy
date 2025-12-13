use super::GameBoyBus;

impl GameBoyBus {
    /// Check whether the APU is currently powered on.
    ///
    /// This is controlled by NR52 (FF26) bit 7 ("Audio on/off"), as described
    /// in Pandocs. When the APU is off, most audio registers become effectively
    /// read-only from the CPU's point of view.
    pub(super) fn apu_enabled(&self) -> bool {
        (self.memory[0xFF26] & 0x80) != 0
    }

    pub(super) fn write_apu_register(&mut self, addr: u16, value: u8) {
        match addr {
            0xFF10..=0xFF14 | 0xFF16..=0xFF1E | 0xFF20..=0xFF25 => {
                // Channel and global audio registers. When the APU is powered
                // off via NR52, writes to these registers are ignored (they are
                // effectively read-only until the APU is turned back on).
                if self.apu_enabled() {
                    self.memory[addr as usize] = value;
                }
            }
            0xFF26 => self.write_nr52(value),
            _ => {
                self.memory[addr as usize] = value;
            }
        }
    }

    fn write_nr52(&mut self, value: u8) {
        // NR52: Audio master control. Only bit 7 (Audio on/off) is
        // writable. Turning audio off clears all APU registers and
        // makes them read-only until turned back on, except NR52.
        let prev = self.memory[0xFF26];
        let was_on = (prev & 0x80) != 0;
        let new_on = (value & 0x80) != 0;

        if was_on && !new_on {
            // Turning the APU off clears channel/global audio registers.
            for a in 0xFF10..=0xFF25 {
                self.memory[a as usize] = 0x00;
            }
            // Lower bits of NR52 (channel status) become 0 as well.
            self.memory[0xFF26] = 0x00;
        }

        // Only bit 7 is writable; other bits are preserved (or were
        // cleared above when powering off).
        let mut reg = self.memory[0xFF26];
        if new_on {
            reg |= 0x80;
        } else {
            reg &= !0x80;
        }
        self.memory[0xFF26] = reg;
    }
}
