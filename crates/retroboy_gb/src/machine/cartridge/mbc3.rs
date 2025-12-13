/// Minimal MBC3 cartridge state.
///
/// We implement basic ROM and RAM banking as described in Pandocs, but
/// intentionally omit RTC behaviour for now. This is enough to support
/// many MBC3 games that do not rely on the real-time clock.
pub(in super::super) struct Mbc3Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    num_rom_banks: u16,
    num_ram_banks: u8,
    rom_bank: u8,
    ram_rtc_select: u8,
    ram_enable: bool,
    latch_clock: u8,
}

impl Mbc3Cartridge {
    pub(super) fn new(rom: &[u8]) -> Self {
        let rom_len = rom.len();
        let num_rom_banks = (rom_len / 0x4000).max(1) as u16;

        // Derive RAM size from header 0x0149. As with MBC1 we round small
        // sizes up to a full 8 KiB bank to keep addressing simple.
        let ram_size_code = rom.get(0x149).copied().unwrap_or(0);
        let num_ram_banks = match ram_size_code {
            0x00 => 0, // no RAM
            0x01 => 1, // 2 KiB -> treat as one 8 KiB bank
            0x02 => 1, // 8 KiB
            0x03 => 4, // 32 KiB
            _ => 0,
        };
        let ram_bytes = (num_ram_banks as usize) * 0x2000;

        Self {
            rom: rom.to_vec(),
            ram: vec![0xFF; ram_bytes],
            num_rom_banks,
            num_ram_banks,
            rom_bank: 1, // bank 1 by default
            ram_rtc_select: 0,
            ram_enable: false,
            latch_clock: 0,
        }
    }

    pub(super) fn rom_read(&self, addr: u16) -> u8 {
        let bank: u16 = if addr < 0x4000 {
            0
        } else {
            let mut b = self.rom_bank & 0x7F;
            if b == 0 {
                b = 1;
            }
            let mut bank_u16 = b as u16;
            if bank_u16 >= self.num_rom_banks {
                bank_u16 %= self.num_rom_banks;
                if bank_u16 == 0 {
                    bank_u16 = 1;
                }
            }
            bank_u16
        };

        let offset = (addr & 0x3FFF) as usize;
        let index = (bank as usize)
            .saturating_mul(0x4000)
            .saturating_add(offset);
        self.rom.get(index).copied().unwrap_or(0xFF)
    }

    pub(super) fn rom_write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // RAM / RTC enable.
                self.ram_enable = (value & 0x0F) == 0x0A;
            }
            0x2000..=0x3FFF => {
                // 7-bit ROM bank number; 0 is remapped to 1.
                self.rom_bank = value & 0x7F;
                if self.rom_bank == 0 {
                    self.rom_bank = 1;
                }
            }
            0x4000..=0x5FFF => {
                // RAM bank number (0-3) or RTC register select (0x08-0x0C).
                self.ram_rtc_select = value;
            }
            0x6000..=0x7FFF => {
                // Latch clock data; for now we just record the last value.
                self.latch_clock = value;
            }
            _ => {}
        }
    }

    pub(super) fn ram_read(&self, addr: u16) -> u8 {
        if !self.ram_enable {
            return 0xFF;
        }

        // Only RAM bank selects 0-3 map to RAM; RTC registers (0x08-0x0C)
        // are not modelled yet and read back as 0xFF.
        let bank_sel = self.ram_rtc_select;
        if bank_sel > 0x03 || self.num_ram_banks == 0 {
            return 0xFF;
        }

        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return 0xFF;
        }
        let bank = bank_sel.min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        self.ram.get(base + offset).copied().unwrap_or(0xFF)
    }

    pub(super) fn ram_write(&mut self, addr: u16, value: u8) {
        if !self.ram_enable {
            return;
        }

        let bank_sel = self.ram_rtc_select;
        if bank_sel > 0x03 || self.num_ram_banks == 0 {
            // RTC registers are not modelled yet.
            return;
        }

        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return;
        }
        let bank = bank_sel.min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        if base + offset < self.ram.len() {
            self.ram[base + offset] = value;
        }
    }
}
