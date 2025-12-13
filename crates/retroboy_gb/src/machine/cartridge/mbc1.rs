/// Minimal MBC1 cartridge state.
///
/// This models a subset of MBC1 behaviour sufficient for many simple
/// ROMs: ROM banking via the standard 5-bit bank register plus two
/// high bits, and optional external RAM with basic banking. Battery
/// persistence is intentionally not modelled.
pub(in super::super) struct Mbc1Cartridge {
    rom: Vec<u8>,
    ram: Vec<u8>,
    num_rom_banks: u16,
    num_ram_banks: u8,
    rom_bank_low5: u8,
    rom_bank_high2: u8,
    ram_enable: bool,
    banking_mode: u8,
}

impl Mbc1Cartridge {
    pub(super) fn new(rom: &[u8]) -> Self {
        let rom_len = rom.len();
        let num_rom_banks = (rom_len / 0x4000).max(1) as u16;

        // Derive RAM size from header 0x0149 when available. We round
        // small sizes up to 8 KiB for simplicity; this wastes memory
        // but keeps addressing straightforward.
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
            rom_bank_low5: 1, // bank 1 by default
            rom_bank_high2: 0,
            ram_enable: false,
            banking_mode: 0,
        }
    }

    /// Compute effective ROM bank for the given address.
    ///
    /// We approximate MBC1 as always being in "ROM banking" mode for
    /// ROM access: bank 0 is fixed at 0x0000-0x3FFF, and 0x4000-0x7FFF
    /// is a switchable bank derived from the 5-bit low register and
    /// the 2-bit high register. This is sufficient for many simple
    /// cartridges that only use mode 0.
    fn effective_rom_bank(&self, addr: u16) -> u16 {
        if addr < 0x4000 {
            return 0;
        }

        let mut bank: u16 = (self.rom_bank_low5 & 0x1F) as u16;
        if bank == 0 {
            bank = 1;
        }
        let high = (self.rom_bank_high2 & 0x03) as u16;
        bank |= high << 5;

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

    pub(super) fn ram_read(&self, addr: u16) -> u8 {
        if !self.ram_enable || self.num_ram_banks == 0 {
            return 0xFF;
        }
        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return 0xFF;
        }
        let bank = (self.rom_bank_high2 & 0x03).min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        self.ram.get(base + offset).copied().unwrap_or(0xFF)
    }

    pub(super) fn rom_write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1FFF => {
                // RAM enable: lower 4 bits must be 0x0A.
                self.ram_enable = (value & 0x0F) == 0x0A;
            }
            0x2000..=0x3FFF => {
                // ROM bank low 5 bits.
                self.rom_bank_low5 = value & 0x1F;
                if self.rom_bank_low5 == 0 {
                    self.rom_bank_low5 = 1;
                }
            }
            0x4000..=0x5FFF => {
                // ROM bank high bits / RAM bank index.
                self.rom_bank_high2 = value & 0x03;
            }
            0x6000..=0x7FFF => {
                // Banking mode: we record it but currently always
                // treat ROM access as "mode 0" for simplicity.
                self.banking_mode = value & 0x01;
            }
            _ => {}
        }
    }

    pub(super) fn ram_write(&mut self, addr: u16, value: u8) {
        if !self.ram_enable || self.num_ram_banks == 0 {
            return;
        }
        let offset = (addr.saturating_sub(0xA000)) as usize;
        if offset >= 0x2000 {
            return;
        }
        let bank = (self.rom_bank_high2 & 0x03).min(self.num_ram_banks.saturating_sub(1));
        let base = (bank as usize) * 0x2000;
        if base + offset < self.ram.len() {
            self.ram[base + offset] = value;
        }
    }
}
