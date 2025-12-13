mod mbc1;
mod mbc3;

pub(super) use mbc1::Mbc1Cartridge;
pub(super) use mbc3::Mbc3Cartridge;

/// Simple wrapper enum for supported cartridge mappers.
pub(super) enum Cartridge {
    Mbc1(Mbc1Cartridge),
    Mbc3(Mbc3Cartridge),
}

impl Cartridge {
    pub(super) fn new_mbc1(rom: &[u8]) -> Self {
        Self::Mbc1(Mbc1Cartridge::new(rom))
    }

    pub(super) fn new_mbc3(rom: &[u8]) -> Self {
        Self::Mbc3(Mbc3Cartridge::new(rom))
    }

    pub(super) fn rom_read(&self, addr: u16) -> u8 {
        match self {
            Cartridge::Mbc1(m) => m.rom_read(addr),
            Cartridge::Mbc3(m) => m.rom_read(addr),
        }
    }

    pub(super) fn rom_write(&mut self, addr: u16, value: u8) {
        match self {
            Cartridge::Mbc1(m) => m.rom_write(addr, value),
            Cartridge::Mbc3(m) => m.rom_write(addr, value),
        }
    }

    pub(super) fn ram_read(&self, addr: u16) -> u8 {
        match self {
            Cartridge::Mbc1(m) => m.ram_read(addr),
            Cartridge::Mbc3(m) => m.ram_read(addr),
        }
    }

    pub(super) fn ram_write(&mut self, addr: u16, value: u8) {
        match self {
            Cartridge::Mbc1(m) => m.ram_write(addr, value),
            Cartridge::Mbc3(m) => m.ram_write(addr, value),
        }
    }
}
