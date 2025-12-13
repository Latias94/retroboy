use super::{cartridge::Cartridge, serial::Serial, timer::Timer, MEMORY_SIZE};

mod apu;
mod dma;
mod init;
mod joypad;
mod mmio;
mod ppu;
mod timer_io;
mod traits;

pub(crate) struct GameBoyBus {
    pub(crate) memory: [u8; MEMORY_SIZE],
    pub(crate) serial: Serial,
    pub(crate) if_reg: u8,
    pub(crate) ie_reg: u8,
    /// Optional cartridge with mapper (currently MBC1 or MBC3).
    cartridge: Option<Cartridge>,
    /// Timer / divider state.
    timer: Timer,
    /// Number of timer register accesses (DIV/TIMA/TMA/TAC reads or
    /// writes) performed during the current CPU instruction.
    ///
    /// Each such access corresponds to one timer "tick" in the DMG
    /// model. We use this counter together with the total instruction
    /// length (in T-cycles) to keep the overall timer tick frequency
    /// aligned with hardware (one tick per machine cycle = 4 T-cycles)
    /// without double-counting ticks in the timer helpers.
    timer_io_events: u8,
    // Very small PPU timing model. We track a running counter of CPU cycles
    // to derive LY and an approximate LCD mode.
    ppu_cycle_counter: u32,
    /// Internal "STAT interrupt line" latch used to model STAT's edge-triggered
    /// interrupt behaviour. This tracks the logically ORed state of all enabled
    /// STAT interrupt sources between calls to `tick`.
    stat_irq_line: bool,
    // Joypad state: selection bits and button presses. Selection bits
    // correspond to P1 bits 5 (buttons) and 4 (d-pad). The button masks
    // use bit=1 to mean "pressed" for:
    // - joyp_buttons: bit0=A, bit1=B, bit2=Select, bit3=Start
    // - joyp_dpad:    bit0=Right, bit1=Left, bit2=Up, bit3=Down
    joyp_select: u8,
    joyp_buttons: u8,
    joyp_dpad: u8,
}

impl Default for GameBoyBus {
    fn default() -> Self {
        let mut bus = Self {
            memory: [0; MEMORY_SIZE],
            serial: Serial::default(),
            if_reg: 0,
            ie_reg: 0,
            cartridge: None,
            timer: Timer::new(),
            timer_io_events: 0,
            ppu_cycle_counter: 0,
            stat_irq_line: false,
            joyp_select: 0x30, // no group selected, bits 7-6 read back as 1
            joyp_buttons: 0x00,
            joyp_dpad: 0x00,
        };
        bus.apply_dmg_initial_io_state();
        bus
    }
}

impl GameBoyBus {
    pub(super) fn load_rom(&mut self, rom: &[u8]) {
        let cart_type = rom.get(0x147).copied().unwrap_or(0);

        match cart_type {
            0x01 | 0x02 | 0x03 => {
                let len = rom.len().min(0x4000);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = Some(Cartridge::new_mbc1(rom));
            }
            0x0F | 0x10 | 0x11 | 0x12 | 0x13 => {
                let len = rom.len().min(0x4000);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = Some(Cartridge::new_mbc3(rom));
            }
            _ => {
                let len = rom.len().min(MEMORY_SIZE);
                self.memory[..len].copy_from_slice(&rom[..len]);
                self.cartridge = None;
            }
        }
    }

    pub(super) fn joypad_set_button_bit(&mut self, bit: u8, pressed: bool) {
        let mask = 1u8 << bit;
        if pressed {
            self.joyp_buttons |= mask;
            self.if_reg |= 0x10;
        } else {
            self.joyp_buttons &= !mask;
        }
    }

    pub(super) fn joypad_set_dpad_bit(&mut self, bit: u8, pressed: bool) {
        let mask = 1u8 << bit;
        if pressed {
            self.joyp_dpad |= mask;
            self.if_reg |= 0x10;
        } else {
            self.joyp_dpad &= !mask;
        }
    }
}
