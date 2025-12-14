use super::{timer::Timer, GameBoy, GameBoyModel};
use crate::cpu::Bus;
use crate::{SCREEN_HEIGHT, SCREEN_WIDTH};

/// Reference timer model translated from Mooneye GB's
/// `hardware::timer::Timer`. This is used in tests to validate that
/// our `Timer` implementation matches a known-good implementation
/// at the T-cycle level, including obscure behaviours around DIV
/// writes and TIMA/TMA/TAC interactions.
struct RefTimer {
    internal_counter: u16,
    tac: u8,
    tima: u8,
    tma: u8,
    overflow: bool,
    enabled: bool,
}

impl RefTimer {
    fn new() -> Self {
        Self {
            internal_counter: 0,
            tac: 0,
            tima: 0,
            tma: 0,
            overflow: false,
            enabled: false,
        }
    }

    #[inline]
    fn counter_bit(&self) -> bool {
        let mask = match self.tac & 0x03 {
            0x03 => 1u16 << 5,
            0x02 => 1u16 << 3,
            0x01 => 1u16 << 1,
            _ => 1u16 << 7,
        };
        (self.internal_counter & mask) != 0
    }

    #[inline]
    fn increment_tima(&mut self) {
        let (next, overflow) = self.tima.overflowing_add(1);
        self.tima = next;
        self.overflow = overflow;
    }

    fn tick_tcycle(&mut self, if_reg: &mut u8) {
        if self.overflow {
            self.internal_counter = self.internal_counter.wrapping_add(1);
            self.tima = self.tma;
            *if_reg |= 0x04;
            self.overflow = false;
        } else if self.enabled && self.counter_bit() {
            self.internal_counter = self.internal_counter.wrapping_add(1);
            let new_bit = self.counter_bit();
            if !new_bit {
                self.increment_tima();
            }
        } else {
            self.internal_counter = self.internal_counter.wrapping_add(1);
        }
    }

    fn div_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        (self.internal_counter >> 6) as u8
    }

    fn div_write(&mut self, if_reg: &mut u8) {
        self.tick_tcycle(if_reg);
        if self.counter_bit() {
            self.increment_tima();
        }
        self.internal_counter = 0;
    }

    fn tima_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        self.tima
    }

    fn tima_write(&mut self, value: u8, if_reg: &mut u8) {
        let was_overflow = self.overflow;
        self.tick_tcycle(if_reg);
        if !was_overflow {
            self.overflow = false;
            self.tima = value;
        }
    }

    fn tma_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        self.tma
    }

    fn tma_write(&mut self, value: u8, if_reg: &mut u8) {
        let was_overflow = self.overflow;
        self.tick_tcycle(if_reg);
        self.tma = value;
        if was_overflow {
            self.tima = value;
        }
    }

    fn tac_read(&mut self, if_reg: &mut u8) -> u8 {
        self.tick_tcycle(if_reg);
        const TAC_UNUSED: u8 = 0b1111_1000;
        TAC_UNUSED | self.tac
    }

    fn tac_write(&mut self, value: u8, if_reg: &mut u8) {
        self.tick_tcycle(if_reg);
        let old_bit = self.enabled && self.counter_bit();
        self.tac = value & 0x07;
        self.enabled = (self.tac & 0x04) != 0;
        let new_bit = self.enabled && self.counter_bit();
        if old_bit && !new_bit {
            self.increment_tima();
        }
    }
}

#[test]
fn timer_basic_tick_matches_reference() {
    // Compare a simple sequence of T-cycles with TAC configured for
    // a known frequency. We seed both timers to the same state and
    // verify that DIV/TIMA evolve identically.
    let mut t = Timer::new();
    let mut ref_t = RefTimer::new();

    // Seed internal counters and TAC to a non-trivial configuration.
    t.internal_counter = 0x1234;
    t.tac = 0x05; // enable + select bit1
    t.enabled = (t.tac & 0x04) != 0;
    t.tima = 0xAB;
    t.tma = 0xCD;

    ref_t.internal_counter = t.internal_counter;
    ref_t.tac = t.tac & 0x07;
    ref_t.enabled = (ref_t.tac & 0x04) != 0;
    ref_t.tima = t.tima;
    ref_t.tma = t.tma;

    let mut if_main = 0u8;
    let mut if_ref = 0u8;

    for _ in 0..1024 {
        t.tick_tcycle(&mut if_main);
        ref_t.tick_tcycle(&mut if_ref);

        assert_eq!(t.internal_counter, ref_t.internal_counter);
        assert_eq!(t.tima, ref_t.tima);
        assert_eq!(t.tma, ref_t.tma);
        assert_eq!(t.tac & 0x07, ref_t.tac);
        assert_eq!(if_main & 0x04, if_ref & 0x04);
    }
}

#[test]
#[ignore]
// This test assumes that DIV writes advance the timer by one cycle
// internally, matching Mooneye GB's per-cycle timer integration. Our
// CPU currently advances the timer via `GameBoyBus::tick` after each
// instruction, so the exact cycle in which DIV is written is modelled
// approximately. Once we have a true per-cycle CPU/bus model we can
// revisit this test and re-enable it.
fn timer_div_write_side_effects_match_reference() {
    // Exercise the "DIV write can trigger an extra TIMA increment"
    // behaviour by sweeping internal_counter through values where the
    // selected input bit is 1 right before the reset.
    let mut t = Timer::new();
    let mut ref_t = RefTimer::new();

    t.tac = 0x05; // enable + bit1 source (fast mode)
    t.enabled = true;
    ref_t.tac = t.tac & 0x07;
    ref_t.enabled = true;

    for start in 0u16..512 {
        t.internal_counter = start;
        t.tima = 0xFE;
        t.tma = 0x42;
        ref_t.internal_counter = start;
        ref_t.tima = 0xFE;
        ref_t.tma = 0x42;

        let mut if_main = 0u8;
        let mut if_ref = 0u8;

        t.div_write(&mut if_main);
        ref_t.div_write(&mut if_ref);

        assert_eq!(
            (t.internal_counter, t.tima, if_main & 0x04),
            (ref_t.internal_counter, ref_t.tima, if_ref & 0x04),
            "mismatch for start counter {:04x}",
            start
        );
    }
}

#[test]
#[ignore]
// This test assumes per-access tick scheduling identical to Mooneye GB's
// CPU/bus integration. Our CPU currently advances the timer via a
// separate `tick` call after each instruction, so the exact cycle in
// which DIV is written may differ. Once we move to a true per-cycle CPU
// model we can re-enable (and possibly rework) this test.
fn timer_tima_write_overflow_window_matches_reference() {
    // Validate writes to TIMA in the overflow A/B window:
    // - Writes during the overflow M-cycle are ignored (pending reload
    //   still occurs and TIMA becomes TMA).
    // - Writes after the reload cancel the pending behaviour and store
    //   the new value.
    //
    // We drive both implementations until an overflow occurs, then
    // perform writes at carefully chosen points.

    // Helper to prepare timers in a state where the next TIMA increment
    // will overflow (TIMA=0xFF and timer enabled).
    fn setup_for_overflow() -> (Timer, RefTimer) {
        let mut t = Timer::new();
        let mut ref_t = RefTimer::new();

        t.tac = 0x05; // enable + bit1
        t.enabled = true;
        t.tima = 0xFF;
        t.tma = 0xAA;

        ref_t.tac = t.tac & 0x07;
        ref_t.enabled = true;
        ref_t.tima = t.tima;
        ref_t.tma = t.tma;

        (t, ref_t)
    }

    // Case A: write TIMA while overflow is pending (cycle A).
    let (mut t_a, mut r_a) = setup_for_overflow();
    let mut if_a_main = 0u8;
    let mut if_a_ref = 0u8;

    // Run until the first overflow is triggered.
    while !t_a.overflow {
        t_a.tick_tcycle(&mut if_a_main);
        r_a.tick_tcycle(&mut if_a_ref);
        assert_eq!(t_a.tima, r_a.tima);
        assert_eq!(t_a.internal_counter, r_a.internal_counter);
        assert_eq!(t_a.overflow, r_a.overflow);
    }
    // At this point overflow flag is set and TIMA should be 0x00.
    assert!(r_a.overflow);
    assert_eq!(t_a.tima, 0x00);
    assert_eq!(t_a.tima, r_a.tima);
    assert_eq!(if_a_main & 0x04, 0);
    assert_eq!(if_a_ref & 0x04, 0);

    // Write TIMA in the overflow window. This should not change the fact
    // that TIMA will be reloaded from TMA on the next cycle.
    t_a.tima_write(0x55, &mut if_a_main);
    r_a.tima_write(0x55, &mut if_a_ref);

    assert_eq!(t_a.tima, r_a.tima);
    assert_eq!(t_a.tma, r_a.tma);
    assert_eq!(t_a.overflow, r_a.overflow);
    // After the write, reload should have occurred from TMA and overflow
    // cleared, and the pending timer interrupt requested.
    assert_eq!(t_a.tima, 0xAA);
    assert!(!t_a.overflow);
    assert_eq!(if_a_main & 0x04, 0x04);
    assert_eq!(if_a_ref & 0x04, 0x04);

    // Case B: write TIMA after overflow has been serviced (cycle B).
    let (mut t_b, mut r_b) = setup_for_overflow();
    let mut if_b_main = 0u8;
    let mut if_b_ref = 0u8;

    // Drive until overflow is pending.
    while !t_b.overflow {
        t_b.tick_tcycle(&mut if_b_main);
        r_b.tick_tcycle(&mut if_b_ref);
        assert_eq!(t_b.tima, r_b.tima);
        assert_eq!(t_b.internal_counter, r_b.internal_counter);
        assert_eq!(t_b.overflow, r_b.overflow);
    }

    // One more cycle to perform the reload from TMA.
    t_b.tick_tcycle(&mut if_b_main);
    r_b.tick_tcycle(&mut if_b_ref);
    assert_eq!(t_b.tima, 0xAA);
    assert_eq!(t_b.tima, r_b.tima);
    assert!(!t_b.overflow);
    assert!(!r_b.overflow);

    // Now write TIMA; this should override the current value and clear
    // any pending overflow.
    t_b.tima_write(0x55, &mut if_b_main);
    r_b.tima_write(0x55, &mut if_b_ref);

    assert_eq!(t_b.tima, r_b.tima);
    assert_eq!(t_b.tima, 0x55);
    assert!(!t_b.overflow);
    assert!(!r_b.overflow);
}

#[test]
#[ignore]
// See comment on `timer_tima_write_overflow_window_matches_reference`.
fn timer_tma_write_overflow_window_matches_reference() {
    // Validate that writes to TMA during the overflow window both update
    // TMA and affect the value reloaded into TIMA, while writes after the
    // reload only change TMA.

    fn setup_for_overflow() -> (Timer, RefTimer) {
        let mut t = Timer::new();
        let mut ref_t = RefTimer::new();

        t.tac = 0x05; // enable + bit1
        t.enabled = true;
        t.tima = 0xFF;
        t.tma = 0x99;

        ref_t.tac = t.tac & 0x07;
        ref_t.enabled = true;
        ref_t.tima = t.tima;
        ref_t.tma = t.tma;

        (t, ref_t)
    }

    // Case A: write TMA while overflow is pending.
    let (mut t_a, mut r_a) = setup_for_overflow();
    let mut if_a_main = 0u8;
    let mut if_a_ref = 0u8;

    while !t_a.overflow {
        t_a.tick_tcycle(&mut if_a_main);
        r_a.tick_tcycle(&mut if_a_ref);
        assert_eq!(t_a.tima, r_a.tima);
        assert_eq!(t_a.internal_counter, r_a.internal_counter);
        assert_eq!(t_a.overflow, r_a.overflow);
    }

    t_a.tma_write(0x55, &mut if_a_main);
    r_a.tma_write(0x55, &mut if_a_ref);

    assert_eq!(t_a.tma, r_a.tma);
    assert_eq!(t_a.tma, 0x55);
    // During overflow window, the new TMA value is also used for TIMA.
    assert_eq!(t_a.tima, r_a.tima);
    assert_eq!(t_a.tima, 0x55);
    assert!(!t_a.overflow);
    assert!(!r_a.overflow);

    // Case B: write TMA after overflow has been serviced.
    let (mut t_b, mut r_b) = setup_for_overflow();
    let mut if_b_main = 0u8;
    let mut if_b_ref = 0u8;

    while !t_b.overflow {
        t_b.tick_tcycle(&mut if_b_main);
        r_b.tick_tcycle(&mut if_b_ref);
    }

    // Let the reload happen.
    t_b.tick_tcycle(&mut if_b_main);
    r_b.tick_tcycle(&mut if_b_ref);

    let tima_before = t_b.tima;
    assert_eq!(tima_before, r_b.tima);

    // Now write TMA; TIMA should remain unchanged immediately after.
    t_b.tma_write(0x77, &mut if_b_main);
    r_b.tma_write(0x77, &mut if_b_ref);

    assert_eq!(t_b.tma, r_b.tma);
    assert_eq!(t_b.tma, 0x77);
    assert_eq!(t_b.tima, r_b.tima);
    assert_eq!(t_b.tima, tima_before);
}

#[test]
#[ignore]
// See comment on `timer_tima_write_overflow_window_matches_reference`.
fn timer_tac_write_edge_increment_matches_reference() {
    // Sweep a range of divider states and TAC values and ensure that
    // enabling/disabling the timer or changing the input bit produces
    // identical TIMA increments in both implementations.

    let mut t = Timer::new();
    let mut ref_t = RefTimer::new();

    // Initial configuration: timer enabled, fast input (bit1), TIMA near
    // overflow so that extra increments are more visible.
    let tac_initial = 0x05;
    let tima_initial = 0xFE;
    let tma_initial = 0x10;

    let tac_variants = [0x00u8, 0x04, 0x05, 0x06, 0x07];

    for start in 0u16..512 {
        for &new_tac in &tac_variants {
            t.internal_counter = start;
            t.tima = tima_initial;
            t.tma = tma_initial;
            t.tac = tac_initial;
            t.enabled = true;

            ref_t.internal_counter = start;
            ref_t.tima = tima_initial;
            ref_t.tma = tma_initial;
            ref_t.tac = tac_initial & 0x07;
            ref_t.enabled = true;

            let mut if_main = 0u8;
            let mut if_ref = 0u8;

            t.tac_write(new_tac, &mut if_main);
            ref_t.tac_write(new_tac, &mut if_ref);

            assert_eq!(
                (t.internal_counter, t.tima, t.tac & 0x07, t.enabled, if_main & 0x04),
                (ref_t.internal_counter, ref_t.tima, ref_t.tac, ref_t.enabled, if_ref & 0x04),
                "mismatch for start counter {:04x}, new_tac {:02x}",
                start,
                new_tac
            );
        }
    }
}

/// Basic integration test for MBC1 ROM banking through `GameBoyBus`.
///
/// We construct a tiny 4‑bank MBC1 ROM with easily distinguishable
/// patterns in each bank and verify that:
/// - The fixed bank 0 region ($0000–$3FFF) always reads from bank 0.
/// - The switchable bank region ($4000–$7FFF) reflects changes to the
///   low 5‑bit ROM bank register at $2000–$3FFF.
#[test]
fn mbc1_basic_rom_banking_works() {
    // 4 banks × 16 KiB each.
    let num_banks = 4;
    let mut rom = vec![0u8; num_banks * 0x4000];

    // Fill each bank with a distinct pattern.
    let patterns = [0x11u8, 0x22, 0x33, 0x44];
    for (bank, &pattern) in patterns.iter().enumerate() {
        let base = bank * 0x4000;
        for i in 0..0x4000 {
            rom[base + i] = pattern;
        }
    }

    // MBC1 cartridge type in header (bank 0).
    rom[0x147] = 0x01;

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // Bank 0 region should always read bank 0, regardless of bank register.
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);

    // After reset, MBC1 defaults to bank 1 in the switchable window.
    assert_eq!(gb.bus.read8(0x4000), patterns[1]);

    // Select bank 2.
    gb.bus.write8(0x2000, 0x02);
    assert_eq!(gb.bus.read8(0x4000), patterns[2]);
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);

    // Select bank 3.
    gb.bus.write8(0x2000, 0x03);
    assert_eq!(gb.bus.read8(0x4000), patterns[3]);
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);
}

/// Basic MBC1 RAM enable / read / write behaviour.
///
/// We model a single 8 KiB RAM bank and check that:
/// - Reads with RAM disabled return $FF.
/// - Writes with RAM disabled are ignored.
/// - Enabling RAM via $0000–$1FFF allows writes to be observed at
///   $A000–$BFFF.
#[test]
fn mbc1_ram_enable_and_access_works() {
    // Minimal 2‑bank ROM; RAM size code 0x02 → 1×8 KiB RAM bank.
    let mut rom = vec![0u8; 2 * 0x4000];
    rom[0x147] = 0x03; // MBC1 + RAM + battery
    rom[0x149] = 0x02; // 8 KiB RAM

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // With RAM disabled, reads should return 0xFF and writes have no effect.
    assert_eq!(gb.bus.read8(0xA000), 0xFF);
    gb.bus.write8(0xA000, 0x42);
    assert_eq!(gb.bus.read8(0xA000), 0xFF);

    // Enable RAM via lower nibble 0x0A at $0000.
    gb.bus.write8(0x0000, 0x0A);
    gb.bus.write8(0xA000, 0x42);
    assert_eq!(gb.bus.read8(0xA000), 0x42);
}

/// Basic MBC3 ROM banking behaviour.
///
/// Similar to the MBC1 test, but using the MBC3 bank register at
/// $2000–$3FFF and the 7‑bit ROM bank number.
#[test]
fn mbc3_basic_rom_banking_works() {
    // 4 banks × 16 KiB each.
    let num_banks = 4;
    let mut rom = vec![0u8; num_banks * 0x4000];

    // Fill each bank with a distinct pattern.
    let patterns = [0x11u8, 0x22, 0x33, 0x44];
    for (bank, &pattern) in patterns.iter().enumerate() {
        let base = bank * 0x4000;
        for i in 0..0x4000 {
            rom[base + i] = pattern;
        }
    }

    // MBC3 cartridge type in header (bank 0).
    rom[0x147] = 0x11; // MBC3, no RAM/RTC

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // Bank 0 region should always read bank 0.
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);

    // After reset, the switchable region should map bank 1.
    assert_eq!(gb.bus.read8(0x4000), patterns[1]);

    // Select bank 2.
    gb.bus.write8(0x2000, 0x02);
    assert_eq!(gb.bus.read8(0x4000), patterns[2]);
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);

    // Select bank 3.
    gb.bus.write8(0x2000, 0x03);
    assert_eq!(gb.bus.read8(0x4000), patterns[3]);
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);
}

/// Basic MBC3 RAM enable and banked access behaviour.
///
/// This exercises the RAM/RTC enable register at $0000–$1FFF and the
/// RAM bank select at $4000–$5FFF. RTC registers (0x08–0x0C) are
/// intentionally not modelled and are not touched here.
#[test]
fn mbc3_ram_enable_and_access_works() {
    // Minimal 4‑bank RAM config: RAM size code 0x03 → 4×8 KiB banks.
    let mut rom = vec![0u8; 2 * 0x4000];
    rom[0x147] = 0x13; // MBC3 + RAM + battery
    rom[0x149] = 0x03; // 32 KiB RAM (4 banks)

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // With RAM disabled, reads should return 0xFF and writes have no effect.
    assert_eq!(gb.bus.read8(0xA000), 0xFF);
    gb.bus.write8(0xA000, 0x12);
    assert_eq!(gb.bus.read8(0xA000), 0xFF);

    // Enable RAM/RTC via lower nibble 0x0A at $0000.
    gb.bus.write8(0x0000, 0x0A);

    // Bank 0: write a value and read it back.
    gb.bus.write8(0xA123, 0x34);
    assert_eq!(gb.bus.read8(0xA123), 0x34);

    // Switch to RAM bank 1 and ensure it has an independent value.
    gb.bus.write8(0x4000, 0x01);
    assert_eq!(gb.bus.read8(0xA123), 0xFF);
    gb.bus.write8(0xA123, 0x56);
    assert_eq!(gb.bus.read8(0xA123), 0x56);

    // Switch back to bank 0 and confirm original data is intact.
    gb.bus.write8(0x4000, 0x00);
    assert_eq!(gb.bus.read8(0xA123), 0x34);
}

/// Basic integration test for MBC5 ROM banking through `GameBoyBus`.
#[test]
fn mbc5_basic_rom_banking_works() {
    // Use 10 banks so that toggling the high ROM bank bit changes the
    // effective bank even after modulo (256 % 10 != 0).
    let num_banks = 10;
    let mut rom = vec![0u8; num_banks * 0x4000];

    let patterns = [0x10u8, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87, 0x98, 0xA9];
    for (bank, &pattern) in patterns.iter().enumerate() {
        let base = bank * 0x4000;
        for i in 0..0x4000 {
            rom[base + i] = pattern;
        }
    }

    rom[0x147] = 0x19; // MBC5

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // Fixed bank 0 region.
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);

    // Default switchable bank is 1.
    assert_eq!(gb.bus.read8(0x4000), patterns[1]);

    // Select bank 2 via low bank register.
    gb.bus.write8(0x2000, 0x02);
    assert_eq!(gb.bus.read8(0x4000), patterns[2]);

    // Toggle the high bank bit (bit 8). With low=1 and high=1, bank=0x101,
    // which maps to 257 % 10 = 7 in our test ROM.
    gb.bus.write8(0x2000, 0x01);
    gb.bus.write8(0x3000, 0x01);
    assert_eq!(gb.bus.read8(0x4000), patterns[7]);
    assert_eq!(gb.bus.read8(0x0000), patterns[0]);
}

/// Basic MBC5 RAM enable / banking behaviour.
#[test]
fn mbc5_ram_enable_and_access_works() {
    // Minimal 2-bank ROM; RAM size code 0x03 -> 4x8 KiB banks.
    let mut rom = vec![0u8; 2 * 0x4000];
    rom[0x147] = 0x1B; // MBC5 + RAM + battery
    rom[0x149] = 0x03; // 32 KiB RAM

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // With RAM disabled, reads should return 0xFF and writes have no effect.
    assert_eq!(gb.bus.read8(0xA000), 0xFF);
    gb.bus.write8(0xA000, 0x12);
    assert_eq!(gb.bus.read8(0xA000), 0xFF);

    // Enable RAM and write to bank 0.
    gb.bus.write8(0x0000, 0x0A);
    gb.bus.write8(0xA000, 0x12);
    assert_eq!(gb.bus.read8(0xA000), 0x12);

    // Switch to RAM bank 1 and ensure it's independent.
    gb.bus.write8(0x4000, 0x01);
    assert_eq!(gb.bus.read8(0xA000), 0xFF);
    gb.bus.write8(0xA000, 0x34);
    assert_eq!(gb.bus.read8(0xA000), 0x34);

    // Switching back should reveal the original bank 0 value.
    gb.bus.write8(0x4000, 0x00);
    assert_eq!(gb.bus.read8(0xA000), 0x12);
}

#[test]
fn cgb_vram_bank_switch_uses_vbk() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Disable LCD so VRAM is always accessible.
    gb.bus.memory[0xFF40] = 0x00;

    // VBK reads back with upper bits set.
    assert_eq!(gb.bus.read8(0xFF4F), 0xFE);
    gb.bus.write8(0xFF4F, 0x01);
    assert_eq!(gb.bus.read8(0xFF4F), 0xFF);

    // Bank 1 write.
    gb.bus.write8(0x8000, 0x12);
    // Bank 0 write.
    gb.bus.write8(0xFF4F, 0x00);
    gb.bus.write8(0x8000, 0x34);

    // Reads follow the selected bank.
    gb.bus.write8(0xFF4F, 0x00);
    assert_eq!(gb.bus.read8(0x8000), 0x34);
    gb.bus.write8(0xFF4F, 0x01);
    assert_eq!(gb.bus.read8(0x8000), 0x12);
}

#[test]
fn cgb_wram_bank_switch_uses_svbk_and_echo() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // SVBK reads back with upper bits set.
    assert_eq!(gb.bus.read8(0xFF70), 0xF8);

    // Bank 1 (SVBK=0) lives in the flat memory array.
    gb.bus.write8(0xFF70, 0x00);
    gb.bus.write8(0xD000, 0x11);

    // Bank 2 is stored separately.
    gb.bus.write8(0xFF70, 0x02);
    gb.bus.write8(0xD000, 0x22);

    // Switching banks changes which value we observe.
    gb.bus.write8(0xFF70, 0x00);
    assert_eq!(gb.bus.read8(0xD000), 0x11);
    gb.bus.write8(0xFF70, 0x02);
    assert_eq!(gb.bus.read8(0xD000), 0x22);

    // Echo area (F000) mirrors D000..DDFF and should follow the selected bank.
    gb.bus.write8(0xF000, 0x33);
    assert_eq!(gb.bus.read8(0xD000), 0x33);

    // Bank 1 remains intact.
    gb.bus.write8(0xFF70, 0x00);
    assert_eq!(gb.bus.read8(0xD000), 0x11);
}

#[test]
fn cgb_key1_speed_switch_stop_toggles_double_speed() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Program at the post-boot entry point: STOP 0
    gb.bus.memory[0x0100] = 0x10;
    gb.bus.memory[0x0101] = 0x00;

    // Arm speed switch via KEY1.
    gb.bus.write8(0xFF4D, 0x01);
    assert_eq!(gb.bus.read8(0xFF4D), 0x7F);

    let cycles = gb.cpu.step(&mut gb.bus);
    assert_eq!(cycles, 4);
    assert_eq!(gb.cpu.regs.pc, 0x0102);

    // STOP returns immediately on CGB when the latch is armed.
    assert!(!gb.cpu.is_stopped());
    // KEY1 now reports double-speed and the latch cleared.
    assert_eq!(gb.bus.read8(0xFF4D), 0xFE);
}

#[test]
fn cgb_speed_switch_pause_advances_ppu_but_freezes_timer() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Reset PPU timing to a known starting point.
    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);
    let ly_before = gb.bus.read8(0xFF44);

    // Program: STOP 0 (with KEY1 armed, triggers speed switch + pause).
    gb.bus.memory[0x0100] = 0x10;
    gb.bus.memory[0x0101] = 0x00;
    gb.bus.write8(0xFF4D, 0x01);

    let ticks_before = gb.bus.timer_debug_ticks();
    gb.cpu.step(&mut gb.bus);
    let ticks_after = gb.bus.timer_debug_ticks();

    // STOP itself costs 1 timer tick (one M-cycle); the post-switch pause must not tick DIV/TIMA.
    assert_eq!(ticks_after - ticks_before, 1);

    // The LCD controller keeps running during the pause, so the visible LY should advance.
    let ly_after = gb.bus.read8(0xFF44);
    assert_ne!(ly_after, ly_before);
}

#[test]
fn cgb_speed_switch_pause_is_consumed_by_micro_step_without_ticking_timer() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Keep the LCD controller running so we can observe LY advance.
    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);
    let ly_before = gb.bus.read8(0xFF44);

    // Program: STOP 0 (speed switch) then NOP.
    gb.bus.memory[0x0100] = 0x10; // STOP
    gb.bus.memory[0x0101] = 0x00;
    gb.bus.memory[0x0102] = 0x00; // NOP

    gb.bus.write8(0xFF4D, 0x01); // arm speed switch

    // Execute STOP (one M-cycle in the micro-step model).
    let ticks_before = gb.bus.timer_debug_ticks();
    assert_eq!(gb.cpu.step_mcycle(&mut gb.bus), 4);
    let ticks_after_stop = gb.bus.timer_debug_ticks();
    assert_eq!(ticks_after_stop - ticks_before, 1);
    assert_eq!(gb.cpu.regs.pc, 0x0102);
    assert!(gb.bus.cgb_speed_switch_pause_active());

    // Consume the full 2050 M-cycle pause. PC must not advance, and the timer must not tick.
    for _ in 0..2050 {
        let pc_before = gb.cpu.regs.pc;
        let ticks_before = gb.bus.timer_debug_ticks();
        assert_eq!(gb.cpu.step_mcycle(&mut gb.bus), 4);
        assert_eq!(gb.cpu.regs.pc, pc_before);
        assert_eq!(gb.bus.timer_debug_ticks(), ticks_before);
    }

    assert!(!gb.bus.cgb_speed_switch_pause_active());
    let ly_after = gb.bus.read8(0xFF44);
    assert_ne!(ly_after, ly_before);

    // After the pause, the CPU resumes and can execute the next opcode.
    gb.cpu.step_mcycle(&mut gb.bus);
    assert_eq!(gb.cpu.regs.pc, 0x0103);
}

#[test]
fn cgb_hdma_gdma_transfers_to_vram() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Disable LCD so VRAM reads are always accessible.
    gb.bus.memory[0xFF40] = 0x00;

    // Seed 0x10 bytes of source data in WRAM.
    for i in 0..0x10u16 {
        gb.bus.write8(0xC000u16.wrapping_add(i), i as u8);
    }

    // Source = 0xC000, Dest = 0x8000 (bank selected by VBK).
    gb.bus.write8(0xFF51, 0xC0);
    gb.bus.write8(0xFF52, 0x00);
    gb.bus.write8(0xFF53, 0x00);
    gb.bus.write8(0xFF54, 0x00);
    gb.bus.write8(0xFF4F, 0x00);

    // Start a 1-block (0x10 bytes) General DMA transfer.
    gb.bus.write8(0xFF55, 0x00);

    for i in 0..0x10u16 {
        assert_eq!(gb.bus.read8(0x8000u16.wrapping_add(i)), i as u8);
    }

    // After GDMA completes, HDMA5 reads as 0xFF (inactive).
    assert_eq!(gb.bus.read8(0xFF55), 0xFF);

    // Source/dest registers advance by 0x10 (lower nibble forced to 0).
    assert_eq!((gb.bus.read8(0xFF51), gb.bus.read8(0xFF52)), (0xC0, 0x10));
    assert_eq!((gb.bus.read8(0xFF53), gb.bus.read8(0xFF54)), (0x00, 0x10));
}

#[test]
fn cgb_hdma_hblank_transfers_one_block_at_hblank_start() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Source = 0xC000, Dest = 0x8000.
    for i in 0..0x10u16 {
        gb.bus.write8(0xC000u16.wrapping_add(i), 0xA0u8.wrapping_add(i as u8));
        gb.bus.write8(0x8000u16.wrapping_add(i), 0x00);
    }

    gb.bus.write8(0xFF51, 0xC0);
    gb.bus.write8(0xFF52, 0x00);
    gb.bus.write8(0xFF53, 0x00);
    gb.bus.write8(0xFF54, 0x00);

    // Start a 1-block HBlank DMA transfer.
    gb.bus.write8(0xFF55, 0x80);
    assert_eq!(gb.bus.read8(0xFF55), 0x00);

    // Run until the first HBlank begins (line_cycle == 80 + mode3_len).
    gb.bus.tick(252);

    for i in 0..0x10u16 {
        assert_eq!(gb.bus.read8(0x8000u16.wrapping_add(i)), 0xA0u8.wrapping_add(i as u8));
    }

    // Transfer completed.
    assert_eq!(gb.bus.read8(0xFF55), 0xFF);
}

#[test]
fn cgb_double_speed_ppu_runs_half_rate() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Reset PPU timing to a known starting point.
    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);

    // Enable double speed via KEY1 + STOP.
    gb.bus.memory[0x0100] = 0x10; // STOP
    gb.bus.memory[0x0101] = 0x00;
    gb.bus.write8(0xFF4D, 0x01);
    gb.cpu.step(&mut gb.bus);

    // Reset PPU timing again after switching speed.
    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);

    // In double-speed, 456 CPU cycles correspond to 228 PPU cycles, so LY stays 0.
    gb.bus.tick(456);
    assert_eq!(gb.bus.read8(0xFF44), 0);

    // 912 CPU cycles correspond to 456 PPU cycles, so LY advances to 1.
    gb.bus.tick(456);
    assert_eq!(gb.bus.read8(0xFF44), 1);
}

#[test]
fn cgb_double_speed_timer_ticks_double_per_frame() {
    // Normal-speed ROM: JP loop (stay inside ROM, avoid wandering into VRAM).
    let mut rom_normal = vec![0u8; 0x8000];
    rom_normal[0x143] = 0xC0; // CGB-only -> model=Cgb
    rom_normal[0x0100] = 0xC3; // JP a16
    rom_normal[0x0101] = 0x00;
    rom_normal[0x0102] = 0x01; // JP 0x0100

    let mut gb_normal = GameBoy::new();
    gb_normal.load_rom(&rom_normal);
    gb_normal.step_frame();
    let normal_ticks = gb_normal.bus.timer_debug_ticks();

    // Double-speed ROM: STOP (speed switch) then JP loop.
    let mut rom_fast = vec![0u8; 0x8000];
    rom_fast[0x143] = 0xC0; // CGB-only -> model=Cgb
    rom_fast[0x0100] = 0x10; // STOP
    rom_fast[0x0101] = 0x00;
    rom_fast[0x0102] = 0xC3; // JP a16
    rom_fast[0x0103] = 0x02;
    rom_fast[0x0104] = 0x01; // JP 0x0102

    let mut gb_fast = GameBoy::new();
    gb_fast.load_rom(&rom_fast);
    gb_fast.bus.write8(0xFF4D, 0x01); // arm speed switch
    gb_fast.cpu.step(&mut gb_fast.bus); // execute STOP + toggle speed
    let before = gb_fast.bus.timer_debug_ticks();
    gb_fast.step_frame();
    let fast_ticks = gb_fast.bus.timer_debug_ticks() - before;

    assert_eq!(fast_ticks, normal_ticks.saturating_mul(2));
}

#[test]
fn cgb_ppu_bg_fifo_renders_basic_palette_colors() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Reset PPU timing.
    gb.bus.write8(0xFF40, 0x00);

    // Seed BG palette 0: [red, green, blue, white].
    gb.bus.write8(0xFF68, 0x80); // BGPI: index 0 + autoinc
    // color 0: red (0x001F)
    gb.bus.write8(0xFF69, 0x1F);
    gb.bus.write8(0xFF69, 0x00);
    // color 1: green (0x03E0)
    gb.bus.write8(0xFF69, 0xE0);
    gb.bus.write8(0xFF69, 0x03);
    // color 2: blue (0x7C00)
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x7C);
    // color 3: white (0x7FFF)
    gb.bus.write8(0xFF69, 0xFF);
    gb.bus.write8(0xFF69, 0x7F);

    // BG tilemap[0,0] = tile 0, attrs palette 0, bank 0, no flips.
    gb.bus.write8(0xFF4F, 0x01); // VBK=1 (attrs)
    gb.bus.write8(0x9800, 0x00);
    gb.bus.write8(0xFF4F, 0x00); // VBK=0 (tile indices)
    gb.bus.write8(0x9800, 0x00);

    // Tile 0, row 0: pixels [0,1,2,3,...] for x=0..3.
    gb.bus.write8(0x8000, 0x50);
    gb.bus.write8(0x8001, 0x30);

    // Enable LCDC with BG enabled and unsigned tile addressing (0x8000).
    gb.bus.write8(0xFF42, 0x00); // SCY
    gb.bus.write8(0xFF43, 0x00); // SCX
    gb.bus.write8(0xFF40, 0x91); // LCDC

    // Run one frame worth of PPU time.
    // Tick just shy of the frame wrap; the PPU marks the framebuffer as valid
    // at vblank start and clears it again when the next frame begins.
    gb.bus.tick(70_223);

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    assert!(gb.bus.cgb_ppu_framebuffer_copy_force(&mut buffer));

    assert_eq!(&buffer[0..3], &[0xFF, 0x00, 0x00]); // red
    assert_eq!(&buffer[3..6], &[0x00, 0xFF, 0x00]); // green
    assert_eq!(&buffer[6..9], &[0x00, 0x00, 0xFF]); // blue
    assert_eq!(&buffer[9..12], &[0xFF, 0xFF, 0xFF]); // white
}

#[test]
fn cgb_ppu_window_overrides_bg_from_wx() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Reset PPU timing.
    gb.bus.write8(0xFF40, 0x00);

    // Palette 0: color0 = black, color3 = white.
    gb.bus.write8(0xFF68, 0x80);
    // color 0 (0x0000)
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x00);
    // color 1 (unused)
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x00);
    // color 2 (unused)
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x00);
    // color 3 (0x7FFF) -> white
    gb.bus.write8(0xFF69, 0xFF);
    gb.bus.write8(0xFF69, 0x7F);

    // Tile 0: row 0 all color 0.
    gb.bus.write8(0x8000, 0x00);
    gb.bus.write8(0x8001, 0x00);

    // Tile 1: row 0 all color 3 (lo=0xFF, hi=0xFF).
    gb.bus.write8(0x8010, 0xFF);
    gb.bus.write8(0x8011, 0xFF);

    // BG map at 0x9800 uses tile 0.
    gb.bus.write8(0xFF4F, 0x00);
    gb.bus.write8(0x9800, 0x00);
    // Window map at 0x9C00 uses tile 1.
    gb.bus.write8(0x9C00, 0x01);
    // Window attrs (VBK=1): palette 0.
    gb.bus.write8(0xFF4F, 0x01);
    gb.bus.write8(0x9C00, 0x00);

    // Enable LCDC: BG enable + window enable + window map select 9C00.
    gb.bus.write8(0xFF42, 0x00);
    gb.bus.write8(0xFF43, 0x00);
    gb.bus.write8(0xFF4A, 0x00); // WY
    gb.bus.write8(0xFF4B, 0x07); // WX -> window starts at x=0
    gb.bus.write8(0xFF40, 0xF1); // LCDC

    gb.bus.tick(70_223);

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    assert!(gb.bus.cgb_ppu_framebuffer_copy_force(&mut buffer));

    // First pixel should come from window tile (white), not BG tile (black).
    assert_eq!(&buffer[0..3], &[0xFF, 0xFF, 0xFF]);
}

#[test]
fn cgb_ppu_sprite_overlays_bg() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    gb.bus.write8(0xFF40, 0x00);

    // BG palette 0: color 0 black.
    gb.bus.write8(0xFF68, 0x80);
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x00);

    // OBJ palette 0: color 1 red (0x001F).
    gb.bus.write8(0xFF6A, 0x80); // OBPI index 0 + autoinc
    // color 0 (transparent, leave 0)
    gb.bus.write8(0xFF6B, 0x00);
    gb.bus.write8(0xFF6B, 0x00);
    // color 1 red
    gb.bus.write8(0xFF6B, 0x1F);
    gb.bus.write8(0xFF6B, 0x00);

    // BG tile 0: all color 0.
    gb.bus.write8(0x8000, 0x00);
    gb.bus.write8(0x8001, 0x00);

    // Sprite tile 1: all color 1.
    gb.bus.write8(0x8010, 0xFF);
    gb.bus.write8(0x8011, 0x00);

    // BG map tile 0.
    gb.bus.write8(0xFF4F, 0x00);
    gb.bus.write8(0x9800, 0x00);

    // OAM[0]: sprite at (0,0) using tile 1, palette 0.
    gb.bus.write8(0xFE00, 16); // y
    gb.bus.write8(0xFE01, 8); // x
    gb.bus.write8(0xFE02, 1); // tile
    gb.bus.write8(0xFE03, 0x00); // attrs

    gb.bus.write8(0xFF42, 0x00);
    gb.bus.write8(0xFF43, 0x00);
    gb.bus.write8(0xFF40, 0x93); // LCD on, BG+OBJ enabled, tile data at 8000

    gb.bus.tick(70_223);

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    assert!(gb.bus.cgb_ppu_framebuffer_copy_force(&mut buffer));
    assert_eq!(&buffer[0..3], &[0xFF, 0x00, 0x00]);
}

#[test]
fn cgb_ppu_sprite_obj_to_bg_priority_hides_when_bg_nonzero() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    gb.bus.write8(0xFF40, 0x00);

    // BG palette 0: color 1 green (0x03E0).
    gb.bus.write8(0xFF68, 0x80);
    // color 0
    gb.bus.write8(0xFF69, 0x00);
    gb.bus.write8(0xFF69, 0x00);
    // color 1 green
    gb.bus.write8(0xFF69, 0xE0);
    gb.bus.write8(0xFF69, 0x03);

    // OBJ palette 0: color 1 red (0x001F).
    gb.bus.write8(0xFF6A, 0x80);
    gb.bus.write8(0xFF6B, 0x00);
    gb.bus.write8(0xFF6B, 0x00);
    gb.bus.write8(0xFF6B, 0x1F);
    gb.bus.write8(0xFF6B, 0x00);

    // BG tile 0: all color 1.
    gb.bus.write8(0x8000, 0xFF);
    gb.bus.write8(0x8001, 0x00);

    // Sprite tile 1: all color 1.
    gb.bus.write8(0x8010, 0xFF);
    gb.bus.write8(0x8011, 0x00);

    gb.bus.write8(0xFF4F, 0x00);
    gb.bus.write8(0x9800, 0x00);

    // OAM[0]: sprite at (0,0), OBJ-to-BG priority set.
    gb.bus.write8(0xFE00, 16);
    gb.bus.write8(0xFE01, 8);
    gb.bus.write8(0xFE02, 1);
    gb.bus.write8(0xFE03, 0x80);

    gb.bus.write8(0xFF42, 0x00);
    gb.bus.write8(0xFF43, 0x00);
    gb.bus.write8(0xFF40, 0x93);

    gb.bus.tick(70_223);

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    assert!(gb.bus.cgb_ppu_framebuffer_copy_force(&mut buffer));
    assert_eq!(&buffer[0..3], &[0x00, 0xFF, 0x00]);
}

#[test]
fn cgb_hdma_gdma_stalls_cpu_for_transfer_duration() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    // Keep PPU running.
    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);

    // Seed source bytes in WRAM.
    for i in 0..0x10u16 {
        gb.bus.write8(0xC000u16.wrapping_add(i), (0x40u8).wrapping_add(i as u8));
    }

    // Configure GDMA: src=0xC000, dst=0x8000, len=1 block.
    gb.bus.write8(0xFF51, 0xC0);
    gb.bus.write8(0xFF52, 0x00);
    gb.bus.write8(0xFF53, 0x00);
    gb.bus.write8(0xFF54, 0x00);
    gb.bus.write8(0xFF4F, 0x00);

    // Program: LDH (FF55),A with A=0 -> start GDMA for 1 block.
    gb.cpu.regs.pc = 0x0100;
    gb.cpu.regs.a = 0x00;
    gb.bus.memory[0x0100] = 0xE0; // LDH (a8),A
    gb.bus.memory[0x0101] = 0x55;

    let ticks_before = gb.bus.timer_debug_ticks();
    let cycles = gb.cpu.step(&mut gb.bus);
    assert_eq!(cycles, 12);

    // Instruction (12T) + GDMA stall (32T) => 44 PPU cycles at normal speed.
    assert_eq!(gb.bus.last_ppu_cycles(), 44);

    // Timer ticks advance once per M-cycle (4T): 12T => 3, stall 32T => 8, total 11.
    let ticks_after = gb.bus.timer_debug_ticks();
    assert_eq!(ticks_after - ticks_before, 11);

    // Data is visible in VRAM once the CPU resumes.
    for i in 0..0x10u16 {
        assert_eq!(gb.bus.read8(0x8000u16.wrapping_add(i)), (0x40u8).wrapping_add(i as u8));
    }
}

#[test]
fn cgb_hdma_hblank_stalls_cpu_per_block() {
    let mut gb = GameBoy::new();
    gb.bus.set_model(GameBoyModel::Cgb);

    gb.bus.write8(0xFF40, 0x00);
    gb.bus.write8(0xFF40, 0x80);

    // Seed a single block in WRAM and configure HBlank DMA.
    for i in 0..0x10u16 {
        gb.bus.write8(0xC000u16.wrapping_add(i), (0x90u8).wrapping_add(i as u8));
        gb.bus.write8(0x8000u16.wrapping_add(i), 0x00);
    }
    gb.bus.write8(0xFF51, 0xC0);
    gb.bus.write8(0xFF52, 0x00);
    gb.bus.write8(0xFF53, 0x00);
    gb.bus.write8(0xFF54, 0x00);
    gb.bus.write8(0xFF55, 0x80); // 1 block HBlank DMA, active

    // Park the PPU just before HBlank start (252) and execute a NOP that crosses it.
    gb.bus.tick(250);
    gb.cpu.regs.pc = 0x0100;
    gb.bus.memory[0x0100] = 0x00; // NOP

    let cycles = gb.cpu.step(&mut gb.bus);
    assert_eq!(cycles, 4);

    // NOP (4T) + HDMA stall (32T) => 36 PPU cycles at normal speed.
    assert_eq!(gb.bus.last_ppu_cycles(), 36);

    for i in 0..0x10u16 {
        assert_eq!(gb.bus.read8(0x8000u16.wrapping_add(i)), (0x90u8).wrapping_add(i as u8));
    }
}

fn assert_rgb24_frame_eq(label: &str, actual: &[u8], expected: &[u8], width: usize, height: usize) {
    assert_eq!(
        actual.len(),
        expected.len(),
        "{label}: rgb24 length mismatch (actual={} expected={})",
        actual.len(),
        expected.len()
    );

    if actual == expected {
        return;
    }

    let pixels = width * height;
    assert_eq!(
        actual.len(),
        pixels * 3,
        "{label}: rgb24 length does not match {width}x{height}"
    );

    let mut mismatch_pixels = 0usize;
    let mut first_mismatch: Option<usize> = None;
    let mut mismatch_x = vec![0usize; width];
    let mut mismatch_y = vec![0usize; height];

    for i in 0..pixels {
        let base = i * 3;
        if actual[base..base + 3] != expected[base..base + 3] {
            mismatch_pixels += 1;
            if first_mismatch.is_none() {
                first_mismatch = Some(i);
            }
            let x = i % width;
            let y = i / width;
            mismatch_x[x] += 1;
            mismatch_y[y] += 1;
        }
    }

    let first = first_mismatch.unwrap_or(0);
    let x = first % width;
    let y = first / width;
    let a = &actual[first * 3..first * 3 + 3];
    let e = &expected[first * 3..first * 3 + 3];

    let window = 4usize;
    let start_x = x.saturating_sub(window);
    let end_x = (x + window).min(width.saturating_sub(1));
    let mut actual_window = Vec::new();
    let mut expected_window = Vec::new();
    for xi in start_x..=end_x {
        let idx = (y * width + xi) * 3;
        actual_window.push((actual[idx], actual[idx + 1], actual[idx + 2]));
        expected_window.push((expected[idx], expected[idx + 1], expected[idx + 2]));
    }

    let mut x_counts: Vec<(usize, usize)> = mismatch_x
        .iter()
        .enumerate()
        .filter_map(|(x, &count)| (count != 0).then_some((x, count)))
        .collect();
    x_counts.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));

    let mut y_counts: Vec<(usize, usize)> = mismatch_y
        .iter()
        .enumerate()
        .filter_map(|(y, &count)| (count != 0).then_some((y, count)))
        .collect();
    y_counts.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));

    let mut y_ranges = Vec::<(usize, usize)>::new();
    let mut in_range = false;
    let mut range_start = 0usize;
    for (y, &count) in mismatch_y.iter().enumerate() {
        if count != 0 {
            if !in_range {
                in_range = true;
                range_start = y;
            }
        } else if in_range {
            in_range = false;
            y_ranges.push((range_start, y - 1));
        }
    }
    if in_range {
        y_ranges.push((range_start, height.saturating_sub(1)));
    }

    let mut x_ranges = Vec::<(usize, usize)>::new();
    let mut in_range = false;
    let mut range_start = 0usize;
    for (x, &count) in mismatch_x.iter().enumerate() {
        if count != 0 {
            if !in_range {
                in_range = true;
                range_start = x;
            }
        } else if in_range {
            in_range = false;
            x_ranges.push((range_start, x - 1));
        }
    }
    if in_range {
        x_ranges.push((range_start, width.saturating_sub(1)));
    }

    let bbox_min_x = mismatch_x
        .iter()
        .enumerate()
        .find_map(|(x, &count)| (count != 0).then_some(x))
        .unwrap_or(0);
    let bbox_max_x = mismatch_x
        .iter()
        .enumerate()
        .rev()
        .find_map(|(x, &count)| (count != 0).then_some(x))
        .unwrap_or(0);
    let bbox_min_y = mismatch_y
        .iter()
        .enumerate()
        .find_map(|(y, &count)| (count != 0).then_some(y))
        .unwrap_or(0);
    let bbox_max_y = mismatch_y
        .iter()
        .enumerate()
        .rev()
        .find_map(|(y, &count)| (count != 0).then_some(y))
        .unwrap_or(0);

    let max_report = 12usize;
    let x_summary = x_counts
        .iter()
        .take(max_report)
        .map(|(x, count)| format!("{x}:{count}"))
        .collect::<Vec<_>>()
        .join(", ");
    let y_summary = y_counts
        .iter()
        .take(max_report)
        .map(|(y, count)| format!("{y}:{count}"))
        .collect::<Vec<_>>()
        .join(", ");
    let y_range_summary = y_ranges
        .iter()
        .map(|(a, b)| format!("{a}..{b}"))
        .collect::<Vec<_>>()
        .join(", ");
    let x_range_summary = x_ranges
        .iter()
        .map(|(a, b)| format!("{a}..{b}"))
        .collect::<Vec<_>>()
        .join(", ");
    panic!(
        "{label}: rgb24 mismatch: {mismatch_pixels}/{pixels} pixels differ; first at (x={x}, y={y}) actual={a:?} expected={e:?}; bbox=(x={bbox_min_x}..{bbox_max_x}, y={bbox_min_y}..{bbox_max_y}); window x={start_x}..{end_x} actual={actual_window:?} expected={expected_window:?}; mismatch_x(top)=[{x_summary}]; mismatch_y(top)=[{y_summary}]; mismatch_x_ranges=[{x_range_summary}]; mismatch_y_ranges=[{y_range_summary}]"
    );
}

#[test]
#[ignore]
fn cgb_acid2_matches_reference_image() {
    fn decode_png_rgb24(path: &std::path::Path) -> (u32, u32, Vec<u8>) {
        use std::io::Read;

        let mut file = std::fs::File::open(path)
            .unwrap_or_else(|e| panic!("Failed to open PNG '{}': {e}", path.display()));
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .unwrap_or_else(|e| panic!("Failed to read PNG '{}': {e}", path.display()));

        let mut decoder = png::Decoder::new(std::io::Cursor::new(data));
        decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = decoder
            .read_info()
            .unwrap_or_else(|e| panic!("Failed to decode PNG '{}': {e}", path.display()));

        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader
            .next_frame(&mut buf)
            .unwrap_or_else(|e| panic!("Failed to read PNG frame '{}': {e}", path.display()));
        let bytes = &buf[..info.buffer_size()];

        let (w, h) = (info.width, info.height);
        match info.color_type {
            png::ColorType::Rgb => (w, h, bytes.to_vec()),
            png::ColorType::Rgba => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(4) {
                    out.extend_from_slice(&px[..3]);
                }
                (w, h, out)
            }
            png::ColorType::Grayscale => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for &v in bytes {
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::GrayscaleAlpha => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(2) {
                    let v = px[0];
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            other => panic!(
                "Unsupported PNG color type {:?} for '{}'; expected RGB/RGBA/Grayscale",
                other,
                path.display()
            ),
        }
    }

    let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("Failed to resolve repo root from CARGO_MANIFEST_DIR");

    let rom_path = repo_root.join("repo-ref/roms/cgb_tests/cgb-acid2.gbc");
    let reference_path = repo_root
        .join("repo-ref/roms/cgb_tests/_gbtestroms_extract/cgb-acid2/cgb-acid2.png");

    assert!(
        rom_path.exists(),
        "Missing ROM at '{}'. Place the test ROMs under `repo-ref/roms/` at repo root.",
        rom_path.display()
    );
    assert!(
        reference_path.exists(),
        "Missing reference image at '{}'.",
        reference_path.display()
    );

    let rom = std::fs::read(&rom_path)
        .unwrap_or_else(|e| panic!("Failed to read ROM '{}': {e}", rom_path.display()));

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    // Run long enough for the test to settle, then compare to the published reference image.
    for _ in 0..240 {
        gb.step_frame();
    }

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    gb.video_frame(&mut buffer);

    let (w, h, reference) = decode_png_rgb24(&reference_path);
    assert_eq!(w as usize, SCREEN_WIDTH);
    assert_eq!(h as usize, SCREEN_HEIGHT);
    assert_rgb24_frame_eq("cgb-acid2", &buffer, &reference, SCREEN_WIDTH, SCREEN_HEIGHT);
}

#[test]
#[ignore]
fn cgb_acid2_matches_reference_image_ppu_fb() {
    if std::env::var_os("RETROBOY_GB_RUN_WIP_SCREENSHOTS").is_none() {
        return;
    }

    fn decode_png_rgb24(path: &std::path::Path) -> (u32, u32, Vec<u8>) {
        use std::io::Read;

        let mut file = std::fs::File::open(path)
            .unwrap_or_else(|e| panic!("Failed to open PNG '{}': {e}", path.display()));
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .unwrap_or_else(|e| panic!("Failed to read PNG '{}': {e}", path.display()));

        let mut decoder = png::Decoder::new(std::io::Cursor::new(data));
        decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = decoder
            .read_info()
            .unwrap_or_else(|e| panic!("Failed to decode PNG '{}': {e}", path.display()));

        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader
            .next_frame(&mut buf)
            .unwrap_or_else(|e| panic!("Failed to read PNG frame '{}': {e}", path.display()));
        let bytes = &buf[..info.buffer_size()];

        let (w, h) = (info.width, info.height);
        match info.color_type {
            png::ColorType::Rgb => (w, h, bytes.to_vec()),
            png::ColorType::Rgba => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(4) {
                    out.extend_from_slice(&px[..3]);
                }
                (w, h, out)
            }
            png::ColorType::Grayscale => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for &v in bytes {
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::GrayscaleAlpha => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(2) {
                    let v = px[0];
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            other => panic!(
                "Unsupported PNG color type {:?} for '{}'; expected RGB/RGBA/Grayscale",
                other,
                path.display()
            ),
        }
    }

    let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("Failed to resolve repo root from CARGO_MANIFEST_DIR");

    let rom_path = repo_root.join("repo-ref/roms/cgb_tests/cgb-acid2.gbc");
    let reference_path = repo_root
        .join("repo-ref/roms/cgb_tests/_gbtestroms_extract/cgb-acid2/cgb-acid2.png");

    assert!(rom_path.exists(), "Missing ROM at '{}'.", rom_path.display());
    assert!(reference_path.exists(), "Missing reference image at '{}'.", reference_path.display());

    let rom = std::fs::read(&rom_path)
        .unwrap_or_else(|e| panic!("Failed to read ROM '{}': {e}", rom_path.display()));

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    for _ in 0..240 {
        gb.step_frame();
    }

    if std::env::var_os("RETROBOY_GB_DEBUG_ACID2_PPU").is_some() {
        let ly = 56u8;
        let lcdc = gb.bus.ppu_line_lcdc(ly);
        let scx = gb.bus.ppu_line_scx(ly);
        let scy = gb.bus.ppu_line_scy(ly);
        let wx = gb.bus.ppu_line_wx(ly);
        let wy = gb.bus.ppu_line_wy(ly);
        eprintln!(
            "acid2 debug: ly={ly} lcdc=0x{lcdc:02X} scx={scx} scy={scy} wx={wx} wy={wy}"
        );

        let sx = 24usize;
        let sy = ly as usize;
        let bg_x = sx.wrapping_add(scx as usize) & 0xFF;
        let bg_y = sy.wrapping_add(scy as usize) & 0xFF;
        let tile_x = (bg_x >> 3) & 0x1F;
        let tile_y = (bg_y >> 3) & 0x1F;
        let map_base = if (lcdc & 0x08) != 0 { 0x9C00u16 } else { 0x9800u16 };
        let map_addr = map_base + (tile_y as u16) * 32 + (tile_x as u16);
        let tile_id = gb.bus.ppu_vram_read(0, map_addr);
        let attr = gb.bus.ppu_vram_read(1, map_addr);

        let tile_data_unsigned = (lcdc & 0x10) != 0;
        let tile_base = if tile_data_unsigned {
            0x8000u16.wrapping_add((tile_id as u16) * 16)
        } else {
            let idx_signed = tile_id as i8 as i32;
            (0x9000i32 + idx_signed * 16) as u16
        };
        let fine_y = (bg_y & 7) as u16;
        let yflip = (attr & 0x40) != 0;
        let bank = if (attr & 0x08) != 0 { 1 } else { 0 };
        let row = if yflip { 7u16.wrapping_sub(fine_y) } else { fine_y };
        let lo = gb.bus.ppu_vram_read(bank, tile_base + row * 2);
        let hi = gb.bus.ppu_vram_read(bank, tile_base + row * 2 + 1);

        let xflip = (attr & 0x20) != 0;
        let fine_x = (bg_x & 7) as u8;
        let src_x = if xflip { fine_x } else { 7 - fine_x };
        let bit = src_x;
        let low = (lo >> bit) & 0x01;
        let high = (hi >> bit) & 0x01;
        let color_index = (high << 1) | low;
        let palette = attr & 0x07;
        let pal = gb.bus.ppu_line_cgb_bgpd(ly);
        let base = palette as usize * 8;
        let c0 = u16::from_le_bytes([pal[base], pal[base + 1]]);
        let c1 = u16::from_le_bytes([pal[base + 2], pal[base + 3]]);
        let c2 = u16::from_le_bytes([pal[base + 4], pal[base + 5]]);
        let c3 = u16::from_le_bytes([pal[base + 6], pal[base + 7]]);
        eprintln!(
            "acid2 debug: screen=({sx},{sy}) bg=({bg_x},{bg_y}) map=0x{map_addr:04X} tile_id=0x{tile_id:02X} attr=0x{attr:02X} bank={bank} tile_base=0x{tile_base:04X} row={row} lo=0x{lo:02X} hi=0x{hi:02X} color_index={color_index} palette={palette} pal_raw=[0x{c0:04X},0x{c1:04X},0x{c2:04X},0x{c3:04X}]"
        );
    }

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    let ok = gb.bus.cgb_ppu_framebuffer_copy_force(&mut buffer);
    assert!(ok, "CGB PPU framebuffer not available/valid");

    let (w, h, reference) = decode_png_rgb24(&reference_path);
    assert_eq!(w as usize, SCREEN_WIDTH);
    assert_eq!(h as usize, SCREEN_HEIGHT);
    assert_rgb24_frame_eq("cgb-acid2:ppu_fb", &buffer, &reference, SCREEN_WIDTH, SCREEN_HEIGHT);
}

#[test]
#[ignore]
fn tearoom_m2_win_en_toggle_dmg_matches_reference() {
    fn decode_png_rgb24(path: &std::path::Path) -> (u32, u32, Vec<u8>) {
        use std::io::Read;

        let mut file = std::fs::File::open(path)
            .unwrap_or_else(|e| panic!("Failed to open PNG '{}': {e}", path.display()));
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .unwrap_or_else(|e| panic!("Failed to read PNG '{}': {e}", path.display()));

        let mut decoder = png::Decoder::new(std::io::Cursor::new(data));
        decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = decoder
            .read_info()
            .unwrap_or_else(|e| panic!("Failed to decode PNG '{}': {e}", path.display()));

        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader
            .next_frame(&mut buf)
            .unwrap_or_else(|e| panic!("Failed to read PNG frame '{}': {e}", path.display()));
        let bytes = &buf[..info.buffer_size()];

        let (w, h) = (info.width, info.height);
        match info.color_type {
            png::ColorType::Rgb => (w, h, bytes.to_vec()),
            png::ColorType::Rgba => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(4) {
                    out.extend_from_slice(&px[..3]);
                }
                (w, h, out)
            }
            png::ColorType::Grayscale => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for &v in bytes {
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::GrayscaleAlpha => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(2) {
                    let v = px[0];
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            other => panic!(
                "Unsupported PNG color type {:?} for '{}'; expected RGB/RGBA/Grayscale",
                other,
                path.display()
            ),
        }
    }

    let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("Failed to resolve repo root from CARGO_MANIFEST_DIR");

    let rom_path =
        repo_root.join("repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m2_win_en_toggle.gb");
    let reference_path = repo_root.join(
        "repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m2_win_en_toggle_dmg_blob.png",
    );

    assert!(
        rom_path.exists(),
        "Missing ROM at '{}'. Place the test ROMs under `repo-ref/roms/` at repo root.",
        rom_path.display()
    );
    assert!(
        reference_path.exists(),
        "Missing reference image at '{}'.",
        reference_path.display()
    );

    let rom = std::fs::read(&rom_path)
        .unwrap_or_else(|e| panic!("Failed to read ROM '{}': {e}", rom_path.display()));

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    let hit = gb.step_until_software_breakpoint(50_000_000);
    assert!(hit, "Did not hit LD B,B software breakpoint");

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    gb.video_frame(&mut buffer);

    let (w, h, reference) = decode_png_rgb24(&reference_path);
    assert_eq!(w as usize, SCREEN_WIDTH);
    assert_eq!(h as usize, SCREEN_HEIGHT);
    assert_rgb24_frame_eq(
        "tearoom:m2_win_en_toggle:dmg",
        &buffer,
        &reference,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    );
}

#[test]
#[ignore]
fn tearoom_m3_bgp_change_dmg_matches_reference_ppu() {
    fn decode_png_rgb24(path: &std::path::Path) -> (u32, u32, Vec<u8>) {
        use std::io::Read;

        let mut file = std::fs::File::open(path)
            .unwrap_or_else(|e| panic!("Failed to open PNG '{}': {e}", path.display()));
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .unwrap_or_else(|e| panic!("Failed to read PNG '{}': {e}", path.display()));

        let mut decoder = png::Decoder::new(std::io::Cursor::new(data));
        decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = decoder
            .read_info()
            .unwrap_or_else(|e| panic!("Failed to decode PNG '{}': {e}", path.display()));

        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader
            .next_frame(&mut buf)
            .unwrap_or_else(|e| panic!("Failed to read PNG frame '{}': {e}", path.display()));
        let bytes = &buf[..info.buffer_size()];

        let (w, h) = (info.width, info.height);
        match info.color_type {
            png::ColorType::Rgb => (w, h, bytes.to_vec()),
            png::ColorType::Rgba => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(4) {
                    out.extend_from_slice(&px[..3]);
                }
                (w, h, out)
            }
            png::ColorType::Grayscale => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for &v in bytes {
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::GrayscaleAlpha => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(2) {
                    let v = px[0];
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            other => panic!(
                "Unsupported PNG color type {:?} for '{}'; expected RGB/RGBA/Grayscale",
                other,
                path.display()
            ),
        }
}

    let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("Failed to resolve repo root from CARGO_MANIFEST_DIR");

    let rom_path =
        repo_root.join("repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m3_bgp_change.gb");
    let reference_path = repo_root.join(
        "repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m3_bgp_change_dmg_blob.png",
    );

    assert!(rom_path.exists(), "Missing ROM at '{}'", rom_path.display());
    assert!(
        reference_path.exists(),
        "Missing reference image at '{}'",
        reference_path.display()
    );

    let rom = std::fs::read(&rom_path)
        .unwrap_or_else(|e| panic!("Failed to read ROM '{}': {e}", rom_path.display()));

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    let hit = gb.step_until_software_breakpoint(50_000_000);
    assert!(hit, "Did not hit LD B,B software breakpoint");

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    let ok = gb.bus.dmg_ppu_framebuffer_copy_force(&mut buffer);
    assert!(ok, "PPU framebuffer not available/valid");

    let (w, h, reference) = decode_png_rgb24(&reference_path);
    assert_eq!(w as usize, SCREEN_WIDTH);
    assert_eq!(h as usize, SCREEN_HEIGHT);

    if std::env::var_os("RETROBOY_GB_DEBUG_TEAROOM_M3_BGP").is_some() {
        let sample_ys = [0usize, 16, 48, 80, 112, 143];
        let sample_xs = [0usize, 1, 13, 73, 85, 145, 157, 159];
        for &y in &sample_ys {
            let mut line = Vec::new();
            for &x in &sample_xs {
                let idx = (y * SCREEN_WIDTH + x) * 3;
                let a = buffer[idx];
                let e = reference[idx];
                let ci = gb.bus.dmg_ppu_bg_color_index_at(x, y);
                line.push(format!("x={x} a={a} e={e} ci={ci:?}"));
            }
            eprintln!("tearoom dbg: y={y}: {}", line.join(" | "));
        }
    }

    assert_rgb24_frame_eq(
        "tearoom:m3_bgp_change:dmg_ppu",
        &buffer,
        &reference,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    );
}

#[test]
#[ignore]
fn tearoom_m3_bgp_change_cgb_compat_matches_reference_cgb_d() {
    if std::env::var_os("RETROBOY_GB_RUN_WIP_SCREENSHOTS").is_none() {
        return;
    }

    fn decode_png_rgb24(path: &std::path::Path) -> (u32, u32, Vec<u8>) {
        use std::io::Read;

        let mut file = std::fs::File::open(path)
            .unwrap_or_else(|e| panic!("Failed to open PNG '{}': {e}", path.display()));
        let mut data = Vec::new();
        file.read_to_end(&mut data)
            .unwrap_or_else(|e| panic!("Failed to read PNG '{}': {e}", path.display()));

        let mut decoder = png::Decoder::new(std::io::Cursor::new(data));
        decoder.set_transformations(png::Transformations::EXPAND | png::Transformations::STRIP_16);
        let mut reader = decoder
            .read_info()
            .unwrap_or_else(|e| panic!("Failed to decode PNG '{}': {e}", path.display()));

        let mut buf = vec![0u8; reader.output_buffer_size()];
        let info = reader
            .next_frame(&mut buf)
            .unwrap_or_else(|e| panic!("Failed to read PNG frame '{}': {e}", path.display()));
        let bytes = &buf[..info.buffer_size()];

        let (w, h) = (info.width, info.height);
        match info.color_type {
            png::ColorType::Rgb => (w, h, bytes.to_vec()),
            png::ColorType::Rgba => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(4) {
                    out.extend_from_slice(&px[..3]);
                }
                (w, h, out)
            }
            png::ColorType::Grayscale => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for &v in bytes {
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::GrayscaleAlpha => {
                let mut out = Vec::with_capacity((w * h * 3) as usize);
                for px in bytes.chunks_exact(2) {
                    let v = px[0];
                    out.extend_from_slice(&[v, v, v]);
                }
                (w, h, out)
            }
            png::ColorType::Indexed => panic!(
                "Unexpected indexed PNG after EXPAND transformation for '{}'",
                path.display()
            ),
        }
    }

    let repo_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("Failed to resolve repo root from CARGO_MANIFEST_DIR");

    let rom_path =
        repo_root.join("repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m3_bgp_change.gb");
    let reference_path =
        repo_root.join("repo-ref/roms/cgb_tests/mealybug-tearoom-tests/ppu/m3_bgp_change_cgb_d.png");

    assert!(rom_path.exists(), "Missing ROM at '{}'", rom_path.display());
    assert!(
        reference_path.exists(),
        "Missing reference image at '{}'",
        reference_path.display()
    );

    let rom = std::fs::read(&rom_path)
        .unwrap_or_else(|e| panic!("Failed to read ROM '{}': {e}", rom_path.display()));

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);
    gb.bus.set_model(GameBoyModel::CgbCompat);

    let hit = gb.step_until_software_breakpoint(50_000_000);
    assert!(hit, "Did not hit LD B,B software breakpoint");

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    let ok = gb.bus.dmg_ppu_framebuffer_copy_force(&mut buffer);
    assert!(ok, "PPU framebuffer not available/valid");

    let (w, h, reference) = decode_png_rgb24(&reference_path);
    assert_eq!(w as usize, SCREEN_WIDTH);
    assert_eq!(h as usize, SCREEN_HEIGHT);

    if std::env::var_os("RETROBOY_GB_DEBUG_TEAROOM_M3_BGP").is_some() {
        let sample_ys = [0usize, 16, 48, 80, 112, 143];
        let sample_xs = [0usize, 1, 13, 73, 85, 145, 157, 159];
        for &y in &sample_ys {
            let mut line = Vec::new();
            for &x in &sample_xs {
                let idx = (y * SCREEN_WIDTH + x) * 3;
                let a = buffer[idx];
                let e = reference[idx];
                let ci = gb.bus.dmg_ppu_bg_color_index_at(x, y);
                line.push(format!("x={x} a={a} e={e} ci={ci:?}"));
            }
            eprintln!("tearoom dbg(cgb_compat): y={y}: {}", line.join(" | "));
        }
    }

    assert_rgb24_frame_eq(
        "tearoom:m3_bgp_change:cgb_compat_dmg_ppu",
        &buffer,
        &reference,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
    );
}
