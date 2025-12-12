use crate::cpu::{Bus8080, Cpu8080};
use retroboy_common::key::Key;

/// Total addressable memory size (64 KiB).
const MEMORY_SIZE: usize = 0x10000;

/// Start of video RAM used by Space Invaders.
///
/// The original hardware maps the frame buffer at 0x2400–0x3fff.
const VRAM_START: usize = 0x2400;
/// Size of video RAM in bytes (0x1c00 = 7168 bytes = 224x256 bits).
const VRAM_SIZE: usize = 0x1c00;

/// 8080 clock and frame timing for Space Invaders.
pub const CPU_CLOCK_HZ: u32 = 2_000_000;
pub const FRAME_RATE_HZ: u32 = 60;
pub const CYCLES_PER_FRAME: u32 = CPU_CLOCK_HZ / FRAME_RATE_HZ;

/// Bit positions for input port 1 (IN 1).
///
/// These constants follow the commonly documented Space Invaders layout.
const IN1_BIT_COIN: u8 = 0;
const IN1_BIT_P2_START: u8 = 1;
const IN1_BIT_P1_START: u8 = 2;
const IN1_BIT_ALWAYS_ONE: u8 = 3;
const IN1_BIT_P1_SHOOT: u8 = 4;
const IN1_BIT_P1_LEFT: u8 = 5;
const IN1_BIT_P1_RIGHT: u8 = 6;

/// Bit positions for input port 2 (IN 2).
///
/// Port 2 is used for player 2 controls, tilt, and DIP switch inputs. We
/// currently wire:
///
/// - bits 0–1: number of ships per credit (DIP)
/// - bit 2:    tilt input
/// - bits 4–6: player 2 controls
/// - bit 7:    "display coin info" DIP (0 = show, 1 = hide)
const IN2_BIT_TILT: u8 = 2;
const IN2_BIT_P2_SHOOT: u8 = 4;
const IN2_BIT_P2_LEFT: u8 = 5;
const IN2_BIT_P2_RIGHT: u8 = 6;
const IN2_BIT_COIN_INFO: u8 = 7;

const IN2_MASK_SHIPS_PER_CREDIT: u8 = 0x03;

/// Simple configuration for the Space Invaders DIP switches that we model.
///
/// The original game uses input port 2 (`IN 2`) to read several DIP switches.
/// We expose a subset that is directly useful for gameplay:
///
/// - `ships_per_credit`: number of ships per game (3–6).
///   Internally this is encoded in bits 0–1 of port 2 as `value - 3`.
/// - `show_coin_info`: whether to show the coin/credit info line on the
///   attract mode. The original ROM treats bit 7 = 1 as "hide coin info".
#[derive(Clone, Copy, Debug)]
pub struct DipConfig {
    pub ships_per_credit: u8,
    pub show_coin_info: bool,
}

impl Default for DipConfig {
    fn default() -> Self {
        Self {
            ships_per_credit: 3,
            show_coin_info: true,
        }
    }
}

impl DipConfig {
    fn apply_to_port2(&self, in_port2: &mut u8) {
        // Clear the DIP bits we control (ships-per-credit and coin info).
        *in_port2 &= !IN2_MASK_SHIPS_PER_CREDIT;
        *in_port2 &= !(1 << IN2_BIT_COIN_INFO);

        // Bits 0–1: number of ships per credit, encoded as (ships - 3).
        let ships = self.ships_per_credit.clamp(3, 6);
        let encoded = ships - 3;
        *in_port2 |= encoded & IN2_MASK_SHIPS_PER_CREDIT;

        // Bit 7: "display coin info" flag.
        // The original ROM uses bit 7 = 1 to *hide* the coin info line.
        if !self.show_coin_info {
            *in_port2 |= 1 << IN2_BIT_COIN_INFO;
        }
    }
}

/// Bus state for the Space Invaders machine (memory and IO ports).
struct SpaceInvadersBusState {
    memory: [u8; MEMORY_SIZE],
    in_port1: u8,
    in_port2: u8,
    out_port3: u8,
    out_port5: u8,
    shift_register: u16,
    shift_offset: u8,
}

impl Default for SpaceInvadersBusState {
    fn default() -> Self {
        let mut in_port1 = 0u8;
        in_port1 |= 1 << IN1_BIT_ALWAYS_ONE;
        Self {
            memory: [0; MEMORY_SIZE],
            in_port1,
            in_port2: 0,
            out_port3: 0,
            out_port5: 0,
            shift_register: 0,
            shift_offset: 0,
        }
    }
}

/// High-level representation of the Space Invaders machine state.
///
/// Holds the Intel 8080 CPU core and the bus state (memory, IO, video).
pub struct SpaceInvadersMachine {
    cpu: Cpu8080,
    bus: SpaceInvadersBusState,
    dip_config: DipConfig,
}

impl SpaceInvadersMachine {
    /// Construct a new machine instance in a powered-up but reset state.
    pub fn new() -> Self {
        let dip_config = DipConfig::default();
        let mut machine = Self {
            cpu: Cpu8080::new(),
            bus: SpaceInvadersBusState::default(),
            dip_config,
        };
        machine.apply_dip_config();
        machine
    }

    /// Construct a new machine with an explicit DIP switch configuration.
    pub fn with_dip_config(dip_config: DipConfig) -> Self {
        let mut machine = Self {
            cpu: Cpu8080::new(),
            bus: SpaceInvadersBusState::default(),
            dip_config,
        };
        machine.apply_dip_config();
        machine
    }

    /// Reset the machine to its initial state, preserving ROM contents.
    pub fn reset(&mut self) {
        self.cpu.reset();
        // Preserve memory (ROM + RAM contents), but reset IO-related fields
        // and re-apply the DIP configuration on port 2.
        let mut new_bus = SpaceInvadersBusState::default();
        // Keep memory contents intact.
        new_bus.memory = self.bus.memory;
        self.bus = new_bus;
        self.apply_dip_config();
    }

    /// Load a Space Invaders ROM image into the machine's memory.
    ///
    /// The common setup uses a single combined ROM image that starts at 0x0000.
    pub fn load_rom(&mut self, rom: &[u8]) {
        let len = rom.len().min(MEMORY_SIZE);
        self.bus.memory[..len].copy_from_slice(&rom[..len]);
        // Execution starts at 0x0000 on Space Invaders.
        self.cpu.pc = 0x0000;
    }

    /// Step the machine for one video frame worth of time.
    ///
    /// We follow the arcade's 2 MHz CPU and 60 Hz frame rate. The original
    /// timing triggers two interrupts per frame (RST 1 and RST 2) at half-frame
    /// intervals.
    pub fn step_frame(&mut self) {
        let half_frame = CYCLES_PER_FRAME / 2;
        let mut cycles: u32 = 0;

        // Run until half a frame worth of cycles, then trigger the first
        // interrupt (RST 1).
        while cycles < half_frame {
            let c = self.cpu.step(&mut self.bus);
            if c == 0 {
                break;
            }
            cycles = cycles.saturating_add(c);
        }
        self.cpu.interrupt(&mut self.bus, 1);

        // Run the second half of the frame, then trigger the second interrupt
        // (RST 2).
        while cycles < CYCLES_PER_FRAME {
            let c = self.cpu.step(&mut self.bus);
            if c == 0 {
                break;
            }
            cycles = cycles.saturating_add(c);
        }
        self.cpu.interrupt(&mut self.bus, 2);
    }

    fn apply_dip_config(&mut self) {
        self.dip_config.apply_to_port2(&mut self.bus.in_port2);
    }

    /// Handle a logical key event mapped from the frontend.
    ///
    /// We currently support the following inputs using the
    /// `retroboy_common::key::Key` set:
    ///
    /// - `C`      → insert coin (port 1, bit 0)
    /// - `Num1`   → start 1 player (port 1, bit 2)
    /// - `Num2`   → start 2 players (port 1, bit 1)
    /// - `A`/`Left`    → player 1 moves left (port 1, bit 5)
    /// - `D`/`Right`   → player 1 moves right (port 1, bit 6)
    /// - `S`/`Space`   → player 1 shoots (port 1, bit 4)
    /// - `P`      → pause/unpause (handled in the app layer)
    /// - `T`      → tilt (port 2, bit 2, latched)
    ///
    /// Player 2 inputs are surfaced on port 2 but the game is turn based,
    /// so they only matter when player 2 is active:
    ///
    /// - `J`      → player 2 moves left (port 2, bit 5)
    /// - `L`      → player 2 moves right (port 2, bit 6)
    /// - `K`      → player 2 shoots (port 2, bit 4)
    pub fn handle_key(&mut self, key: Key, pressed: bool) {
        match key {
            Key::C => set_input_bit(&mut self.bus.in_port1, IN1_BIT_COIN, pressed),
            Key::Num1 => set_input_bit(&mut self.bus.in_port1, IN1_BIT_P1_START, pressed),
            Key::Num2 => set_input_bit(&mut self.bus.in_port1, IN1_BIT_P2_START, pressed),
            Key::A | Key::Left => {
                set_input_bit(&mut self.bus.in_port1, IN1_BIT_P1_LEFT, pressed);
            }
            Key::D | Key::Right => {
                set_input_bit(&mut self.bus.in_port1, IN1_BIT_P1_RIGHT, pressed);
            }
            Key::S | Key::Space => {
                set_input_bit(&mut self.bus.in_port1, IN1_BIT_P1_SHOOT, pressed);
            }
            // Player 2 controls on IN2. These are mostly relevant if you
            // configure the game for two players.
            Key::J => set_input_bit(&mut self.bus.in_port2, IN2_BIT_P2_LEFT, pressed),
            Key::L => set_input_bit(&mut self.bus.in_port2, IN2_BIT_P2_RIGHT, pressed),
            Key::K => set_input_bit(&mut self.bus.in_port2, IN2_BIT_P2_SHOOT, pressed),
            // Tilt input: match the reference emulator and only set the bit
            // on key press (latched until reset by the game).
            Key::T if pressed => {
                set_input_bit(&mut self.bus.in_port2, IN2_BIT_TILT, true);
            }
            _ => {}
        }
    }

    /// Expose the raw video RAM window used by the renderer.
    ///
    /// The layout matches the original hardware and the reference emulator:
    /// 0x1c00 bytes starting at 0x2400.
    pub fn video_ram(&self) -> &[u8] {
        &self.bus.memory[VRAM_START..VRAM_START + VRAM_SIZE]
    }

    /// Expose the current values of the sound output ports (OUT 3 and OUT 5).
    ///
    /// These are used by the audio layer to drive discrete sound effects.
    pub fn outputs(&self) -> (u8, u8) {
        (self.bus.out_port3, self.bus.out_port5)
    }
}

impl Default for SpaceInvadersMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl Bus8080 for SpaceInvadersBusState {
    fn mem_read(&mut self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, value: u8) {
        self.memory[addr as usize] = value;
    }

    fn io_read(&mut self, port: u8) -> u8 {
        match port {
            1 => self.in_port1,
            2 => self.in_port2,
            3 => {
                let shift = 8 - (self.shift_offset & 0x7);
                (self.shift_register >> shift) as u8
            }
            _ => 0,
        }
    }

    fn io_write(&mut self, port: u8, value: u8) {
        match port {
            2 => {
                self.shift_offset = value & 0x7;
            }
            3 => {
                self.out_port3 = value;
            }
            4 => {
                self.shift_register = (self.shift_register >> 8) | ((value as u16) << 8);
            }
            5 => {
                self.out_port5 = value;
            }
            6 => {
                // watchdog, ignore
            }
            _ => {}
        }
    }
}

fn set_input_bit(port: &mut u8, bit: u8, pressed: bool) {
    let mask = 1 << bit;
    if pressed {
        *port |= mask;
    } else {
        *port &= !mask;
    }
}
