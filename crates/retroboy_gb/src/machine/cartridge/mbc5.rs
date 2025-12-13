/// Minimal MBC5 cartridge state.
///
/// MBC5 supports up to 512 ROM banks (9-bit bank number) and up to 16 RAM
/// banks (4-bit bank number). We implement basic ROM/RAM banking and RAM
/// enable behaviour; rumble and other extensions are intentionally omitted.
pub(in super::super) struct Mbc5Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    num_rom_banks: u16,
    num_ram_banks: u8,
    rom_bank_low8: u8,
    rom_bank_high1: u8,
    ram_bank: u8,
    ram_enable: bool,
}

impl Mbc5Cartridge {
    pub(super) fn new(rom: &[u8]) -> Self {
        let rom_len = rom.len();
        let num_rom_banks = (rom_len / 0x4000).max(1) as u16;

        // Derive RAM size from header 0x0149. We mirror the MBC1/MBC3
        // handling and treat 2 KiB as a single 8 KiB bank for simplicity.
        let ram_size_code = rom.get(0x149).copied().unwrap_or(0);
        let num_ram_banks = match ram_size_code {
            0x00 => 0,  // no RAM
            0x01 => 1,  // 2 KiB -> treat as one 8 KiB bank
            0x02 => 1,  // 8 KiB
            0x03 => 4,  // 32 KiB
            0x04 => 16, // 128 KiB
            0x05 => 8,  // 64 KiB
            _ => 0,
        };
        let ram_bytes = (num_ram_banks as usize) * 0x2000;

        Self {
            rom: rom.to_vec(),
            ram: vec![0xFF; ram_bytes],
            num_rom_banks,
            num_ram_banks,
            rom_bank_low8: 1,
            rom_bank_high1: 0,
            ram_bank: 0,
            ram_enable: false,
        }
    }

    #[inline]
    fn effective_rom_bank(&self, addr: u16) -> u16 {
        if addr < 0x4000 {
            return 0;
        }

        let bank = ((self.rom_bank_high1 as u16) << 8) | (self.rom_bank_low8 as u16);
        if bank >= self.num_rom_banks {
            bank % self.num_rom_banks
        } else {
            bank
        }
    }

    pub(super) fn rom_read(&self, addr: u16) -> u8 {
        let bank = self.effective_rom_bank(addr);
        let offset = (addr & 0x3FFF) as usize;
        let index = (bank as usize)
            .saturating_mul(0x4000)
            .saturating_add(offset);
        self.rom.get(index).copied().unwrap_or(0xFF)
    }

    pub(super) fn rom_write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // RAM enable.
                self.ram_enable = (value & 0x0F) == 0x0A;
            }
            0x2000..=0x2FFF => {
                // ROM bank number lower 8 bits.
                self.rom_bank_low8 = value;
            }
            0x3000..=0x3FFF => {
                // ROM bank number bit 8.
                self.rom_bank_high1 = value & 0x01;
            }
            0x4000..=0x5FFF => {
                // RAM bank number (0-15). Rumble cartridges also use bit 3 for
                // the motor, but we ignore that extension for now.
                self.ram_bank = value & 0x0F;
            }
            _ => {}
        }
    }

    pub(super) fn ram_read(&self, addr: u16) -> u8 {
        if !self.ram_enable || self.num_ram_banks == 0 {
            return 0xFF;
        }
        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return 0xFF;
        }
        let bank = (self.ram_bank & 0x0F).min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        self.ram.get(base + offset).copied().unwrap_or(0xFF)
    }

    pub(super) fn ram_write(&mut self, addr: u16, value: u8) {
        if !self.ram_enable || self.num_ram_banks == 0 {
            return;
        }
        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return;
        }
        let bank = (self.ram_bank & 0x0F).min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        if base + offset < self.ram.len() {
            self.ram[base + offset] = value;
        }
    }
}
