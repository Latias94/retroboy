use super::super::GameBoyBus;

impl GameBoyBus {
    pub(in super::super) fn mmio_write_lcdc(&mut self, value: u8) {
        // LCDC: we store it in memory and update LCD/STAT derived state. Toggling LCDC.7 (LCD
        // enable) resets the PPU counters and LY when turning the LCD off; when turning it back
        // on we start from LY=0 again in our simplified model.
        let old = self.memory[0xFF40];
        let was_enabled = (old & 0x80) != 0;
        let now_enabled = (value & 0x80) != 0;
        self.memory[0xFF40] = value;

        if !now_enabled {
            // LCD turned off: reset LY and PPU timing; STAT reports mode 0 and LYC==LY based on
            // the new LY/LYC.
            self.ppu_cycle_counter = 0;
            self.memory[0xFF44] = 0;
            self.stat_irq_line = false;
        } else if !was_enabled && now_enabled {
            // LCD turned on from a disabled state: restart timing from the top of the frame.
            // Allow tweaking the initial PPU phase for cycle-accurate tests.
            // Defaults to 0 (top of frame) when unset.
            let start_offset = std::env::var("RETROBOY_GB_PPU_START_OFFSET")
                .ok()
                .and_then(|v| v.parse::<u32>().ok())
                .unwrap_or(0);
            self.ppu_cycle_counter = start_offset;
            self.memory[0xFF44] = 0;
            self.stat_irq_line = false;
        }

        self.update_lcd_status();
    }

    pub(in super::super) fn mmio_write_stat(&mut self, value: u8) {
        // STAT: only bits 3..6 are writable; bits 0..2 are updated by PPU/LY/LYC logic. We
        // preserve bit 7 and bits 0..2, and update the interrupt select bits from the written
        // value.
        let current = self.memory[0xFF41];
        let new_value = (current & !0x78) | (value & 0x78);
        self.memory[0xFF41] = new_value;
        // Writing to STAT can immediately change which sources are enabled; recompute the STAT
        // interrupt line accordingly.
        self.update_lcd_status();
    }

    pub(in super::super) fn mmio_write_ly(&mut self) {
        // Writing to LY resets it to 0 and restarts the PPU line counter, as on hardware.
        self.memory[0xFF44] = 0;
        self.ppu_cycle_counter = 0;
        self.stat_irq_line = false;
        self.update_lcd_status();
    }

    pub(in super::super) fn mmio_write_lyc(&mut self, value: u8) {
        // LYC: store the compare value and recompute coincidence + potential STAT interrupt.
        self.memory[0xFF45] = value;
        self.update_lcd_status();
    }
}
