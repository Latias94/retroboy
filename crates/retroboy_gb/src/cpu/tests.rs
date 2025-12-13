use super::*;
use crate::machine::GameBoy;
use once_cell::sync::OnceCell;

struct TestBus {
    memory: [u8; 0x10000],
}

impl Default for TestBus {
    fn default() -> Self {
        Self {
            memory: [0; 0x10000],
        }
    }
}

impl Bus for TestBus {
    fn read8(&mut self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn write8(&mut self, addr: u16, value: u8) {
        self.memory[addr as usize] = value;
    }
}

static CPU_INSTRS_ROM: OnceCell<Vec<u8>> = OnceCell::new();

// Expected serial output from blargg's cpu_instrs.gb when all tests pass.
// Taken from the reference implementation (rboy).
const CPU_SERIAL_EXPECT: &[u8] = b"cpu_instrs\n\n\
01:ok  02:ok  03:ok  04:ok  05:ok  06:ok  07:ok  08:ok  09:ok  10:ok  11:ok  \n\
\nPassed all tests\n";

fn load_cpu_instrs_rom() -> &'static [u8] {
    CPU_INSTRS_ROM.get_or_init(|| {
        use std::path::PathBuf;

        // Support both workspace-root and crate-relative layouts so that the
        // test continues to work when run from different working directories.
        let candidates = [
            PathBuf::from("assets/roms/gb_tests/blargg/cpu_instrs.gb"),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../assets/roms/gb_tests/blargg/cpu_instrs.gb"),
        ];

        for path in &candidates {
            if let Ok(data) = std::fs::read(path) {
                return data;
            }
        }

        panic!("cpu_instrs.gb not found. Tried: {:?}", candidates)
    })
}

fn load_mooneye_timer_rom(filename: &str) -> Vec<u8> {
    use std::path::PathBuf;

    let candidates = [
        // In-repo copy that can be committed alongside the emulator.
        PathBuf::from("assets/mooneye/acceptance/timer").join(filename),
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/mooneye/acceptance/timer")
            .join(filename),
    ];

    for path in &candidates {
        if let Ok(data) = std::fs::read(path) {
            return data;
        }
    }

    panic!(
        "failed to read mooneye timer ROM {:?} (tried {:?})",
        filename, candidates
    );
}

/// Load a mooneye "acceptance/ppu" ROM from the workspace.
///
/// This mirrors `load_mooneye_timer_rom` but targets the `ppu`
/// subdirectory. Callers typically pass bare filenames such as
/// `"vblank_stat_intr-GS.gb"`.
fn load_mooneye_ppu_rom(filename: &str) -> Vec<u8> {
    use std::path::PathBuf;

    let candidates = [
        PathBuf::from("assets/mooneye/acceptance/ppu").join(filename),
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/mooneye/acceptance/ppu")
            .join(filename),
    ];

    for path in &candidates {
        if let Ok(data) = std::fs::read(path) {
            return data;
        }
    }

    panic!(
        "failed to read mooneye PPU ROM {:?} (tried {:?})",
        filename, candidates
    );
}

/// Load a mooneye "acceptance/interrupts" ROM (e.g. ie_push.gb) from
/// the workspace. This mirrors `load_mooneye_timer_rom` but targets
/// the interrupts subdirectory.
fn load_mooneye_interrupts_rom(filename: &str) -> Vec<u8> {
    use std::path::PathBuf;

    let candidates = [
        // In-repo copy mirroring the `acceptance` layout so callers
        // can pass `"interrupts/ie_push.gb"` or `"ei_timing.gb"`.
        PathBuf::from("assets/mooneye/acceptance").join(filename),
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/mooneye/acceptance")
            .join(filename),
    ];

    for path in &candidates {
        if let Ok(data) = std::fs::read(path) {
            return data;
        }
    }

    panic!(
        "failed to read mooneye interrupts ROM {:?} (tried {:?})",
        filename, candidates
    );
}

#[test]
fn microstep_interrupt_entry_matches_step() {
    // Prepare a CPU and bus with a simple pending VBlank interrupt.
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();

    cpu_step.regs.pc = 0x1234;
    cpu_step.regs.sp = 0xFFFE;
    cpu_step.ime = true;
    cpu_step.halted = false;
    cpu_step.stopped = false;

    // Enable all interrupts and request VBlank only.
    bus_step.memory[0xFFFF] = 0x1F; // IE
    bus_step.memory[0xFF0F] = 0x01; // IF: VBlank pending

    // Clone CPU/bus state for the micro‑step path.
    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    // Path 1: instruction‑level interrupt handling via `step`.
    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 20);

    // Path 2: micro‑step interrupt handling via repeated `step_mcycle`
    // calls until the 20 T‑cycle entry sequence completes.
    let mut total_cycles = 0;
    // First call will detect the pending interrupt and start the
    // micro‑sequence; subsequent calls walk through the 5 M‑cycles.
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        // Safety net: interrupt entry must finish within 5 M‑cycles.
        assert!(total_cycles <= 20);
    }
    assert_eq!(total_cycles, 20);

    // Both paths should agree on PC/SP/IME/HALTED and the stack/IF/IE
    // state after the interrupt entry.
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
    assert_eq!(cpu_step.regs.sp, cpu_micro.regs.sp);
    assert_eq!(cpu_step.ime, cpu_micro.ime);
    assert_eq!(cpu_step.halted, cpu_micro.halted);

    let sp = cpu_step.regs.sp;
    assert_eq!(bus_step.memory[sp as usize], bus_micro.memory[sp as usize]);
    assert_eq!(
        bus_step.memory[sp.wrapping_add(1) as usize],
        bus_micro.memory[sp.wrapping_add(1) as usize]
    );

    // IF bit for the interrupt should be cleared; IE should be unchanged.
    assert_eq!(bus_step.memory[0xFF0F], bus_micro.memory[0xFF0F]);
    assert_eq!(bus_step.memory[0xFFFF], bus_micro.memory[0xFFFF]);
}

#[test]
fn call_a16_microstep_matches_step() {
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();

    // Program:
    // 0x0000: CALL 0x1234
    bus_step.memory[0x0000] = 0xCD;
    bus_step.memory[0x0001] = 0x34;
    bus_step.memory[0x0002] = 0x12;

    cpu_step.regs.pc = 0x0000;
    cpu_step.regs.sp = 0xFFFE;

    // Instruction-level path.
    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 24);

    // Clone for micro-step path.
    let mut cpu_micro = Cpu::new();
    let mut bus_micro = TestBus::default();
    cpu_micro.regs = cpu_step.regs;
    cpu_micro.regs.pc = 0x0000;
    cpu_micro.regs.sp = 0xFFFE;
    bus_micro.memory.copy_from_slice(&bus_step.memory);
    // Overwrite program bytes again (stack contents may differ).
    bus_micro.memory[0x0000] = 0xCD;
    bus_micro.memory[0x0001] = 0x34;
    bus_micro.memory[0x0002] = 0x12;

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= 24);
    }
    assert_eq!(total_cycles, 24);

    // Compare core effects: PC, SP, and stack contents at new SP.
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
    assert_eq!(cpu_step.regs.sp, cpu_micro.regs.sp);

    let sp = cpu_step.regs.sp;
    assert_eq!(bus_step.memory[sp as usize], bus_micro.memory[sp as usize]);
    assert_eq!(
        bus_step.memory[sp.wrapping_add(1) as usize],
        bus_micro.memory[sp.wrapping_add(1) as usize]
    );
}

#[test]
fn jr_microstep_matches_step() {
    // Program: JR +2 at 0x0000.
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();
    bus_step.memory[0x0000] = 0x18; // JR r8
    bus_step.memory[0x0001] = 0x02;

    cpu_step.regs.pc = 0x0000;
    cpu_step.regs.sp = 0xFFFE;

    // Clone CPU/bus for micro-step path.
    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    let cycles_step = cpu_step.step(&mut bus_step);

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= cycles_step);
    }

    assert_eq!(total_cycles, cycles_step);
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
    assert_eq!(cpu_step.regs.sp, cpu_micro.regs.sp);
}

#[test]
fn jr_nz_microstep_condition_matches_step() {
    // Program: JR NZ,+2 at 0x0000.
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();
    bus_step.memory[0x0000] = 0x20; // JR NZ, r8
    bus_step.memory[0x0001] = 0x02;

    // Case 1: condition not taken (Z=1).
    cpu_step.regs.pc = 0x0000;
    cpu_step.set_flag(Flag::Z, true);

    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 8);

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= cycles_step);
    }

    assert_eq!(total_cycles, cycles_step);
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);

    // Case 2: condition taken (Z=0).
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();
    bus_step.memory[0x0000] = 0x20;
    bus_step.memory[0x0001] = 0x02;
    cpu_step.regs.pc = 0x0000;
    cpu_step.set_flag(Flag::Z, false);

    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 12);

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= cycles_step);
    }

    assert_eq!(total_cycles, cycles_step);
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
}

#[test]
fn call_nz_microstep_matches_step() {
    // Program: CALL NZ,0x1234 at 0x0000.
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();
    bus_step.memory[0x0000] = 0xC4; // CALL NZ,a16
    bus_step.memory[0x0001] = 0x34;
    bus_step.memory[0x0002] = 0x12;

    // Case 1: condition not taken (Z=1).
    cpu_step.regs.pc = 0x0000;
    cpu_step.regs.sp = 0xFFFE;
    cpu_step.set_flag(Flag::Z, true);

    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 12);

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= cycles_step);
    }

    assert_eq!(total_cycles, cycles_step);
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
    assert_eq!(cpu_step.regs.sp, cpu_micro.regs.sp);

    // Case 2: condition taken (Z=0).
    let mut cpu_step = Cpu::new();
    let mut bus_step = TestBus::default();
    bus_step.memory[0x0000] = 0xC4;
    bus_step.memory[0x0001] = 0x34;
    bus_step.memory[0x0002] = 0x12;

    cpu_step.regs.pc = 0x0000;
    cpu_step.regs.sp = 0xFFFE;
    cpu_step.set_flag(Flag::Z, false);

    let mut cpu_micro = cpu_step.clone();
    let mut bus_micro = TestBus::default();
    bus_micro.memory.copy_from_slice(&bus_step.memory);

    let cycles_step = cpu_step.step(&mut bus_step);
    assert_eq!(cycles_step, 24);

    let mut total_cycles = 0;
    while cpu_micro.micro_cycles_remaining == 0 || cpu_micro.micro_cycles_remaining > 0 {
        let c = cpu_micro.step_mcycle(&mut bus_micro);
        total_cycles += c;
        if cpu_micro.micro_cycles_remaining == 0 {
            break;
        }
        assert!(total_cycles <= cycles_step);
    }

    assert_eq!(total_cycles, cycles_step);
    assert_eq!(cpu_step.regs.pc, cpu_micro.regs.pc);
    assert_eq!(cpu_step.regs.sp, cpu_micro.regs.sp);

    // Also compare stack contents when the call is taken.
    let sp = cpu_step.regs.sp;
    assert_eq!(bus_step.memory[sp as usize], bus_micro.memory[sp as usize]);
    assert_eq!(
        bus_step.memory[sp.wrapping_add(1) as usize],
        bus_micro.memory[sp.wrapping_add(1) as usize]
    );
}

#[test]
fn nop_advances_pc() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();
    // 0x00: NOP
    bus.memory[0x0000] = 0x00;

    cpu.regs.pc = 0x0000;
    let cycles = cpu.step(&mut bus);

    assert_eq!(cpu.regs.pc, 0x0001);
    assert_eq!(cycles, 4);
}

#[test]
fn ld_16bit_and_basic_ld_indirect_work() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: LD BC, 0x1234
    // 0x0003: LD (BC), A
    // 0x0004: LD A, (BC)
    bus.memory[0x0000] = 0x01; // LD BC, d16
    bus.memory[0x0001] = 0x34;
    bus.memory[0x0002] = 0x12;
    bus.memory[0x0003] = 0x02; // LD (BC), A
    bus.memory[0x0004] = 0x0A; // LD A, (BC)

    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0xAB;

    // LD BC, 0x1234
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 12);
    assert_eq!(cpu.regs.bc(), 0x1234);
    assert_eq!(cpu.regs.pc, 0x0003);

    // LD (BC), A
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 8);
    assert_eq!(bus.memory[0x1234], 0xAB);
    assert_eq!(cpu.regs.pc, 0x0004);

    // Clear A then reload from (BC).
    cpu.regs.a = 0x00;
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 8);
    assert_eq!(cpu.regs.a, 0xAB);
}

#[test]
fn ld_r_r_and_hl_inc_dec_forms_work() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: LD HL, 0xC000
    // 0x0003: LD B, 0x12
    // 0x0005: LD (HL), B
    // 0x0006: LD A, (HL+)
    // 0x0007: LD (HL-), A
    // 0x0008: LD C, A
    bus.memory[0x0000] = 0x21; // LD HL, d16
    bus.memory[0x0001] = 0x00;
    bus.memory[0x0002] = 0xC0;
    bus.memory[0x0003] = 0x06; // LD B, d8
    bus.memory[0x0004] = 0x12;
    bus.memory[0x0005] = 0x70; // LD (HL), B   (via LD r,r matrix)
    bus.memory[0x0006] = 0x2A; // LD A, (HL+)
    bus.memory[0x0007] = 0x32; // LD (HL-), A
    bus.memory[0x0008] = 0x4F; // LD C, A

    cpu.regs.pc = 0x0000;

    // LD HL, 0xC000
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 12);
    assert_eq!(cpu.regs.hl(), 0xC000);

    // LD B, 0x12
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.b, 0x12);

    // LD (HL), B  => writes 0x12 to 0xC000
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 8);
    assert_eq!(bus.memory[0xC000], 0x12);

    // LD A, (HL+) => loads from 0xC000, then HL becomes 0xC001
    let c4 = cpu.step(&mut bus);
    assert_eq!(c4, 8);
    assert_eq!(cpu.regs.a, 0x12);
    assert_eq!(cpu.regs.hl(), 0xC001);

    // LD (HL-), A => writes to 0xC001, then HL becomes 0xC000
    let c5 = cpu.step(&mut bus);
    assert_eq!(c5, 8);
    assert_eq!(bus.memory[0xC001], 0x12);
    assert_eq!(cpu.regs.hl(), 0xC000);

    // LD C, A via LD r,r matrix.
    let c6 = cpu.step(&mut bus);
    assert_eq!(c6, 4);
    assert_eq!(cpu.regs.c, 0x12);
}

#[test]
fn inc_dec_8bit_update_flags_and_preserve_c() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: INC B
    // 0x0001: DEC B
    // 0x0002: INC A
    // 0x0003: DEC A
    bus.memory[0x0000] = 0x04;
    bus.memory[0x0001] = 0x05;
    bus.memory[0x0002] = 0x3C;
    bus.memory[0x0003] = 0x3D;

    cpu.regs.pc = 0x0000;
    cpu.regs.b = 0x0F;
    cpu.set_flag(Flag::C, true);

    // INC B: 0x0F -> 0x10, H=1, Z=0, N=0, C unchanged.
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 4);
    assert_eq!(cpu.regs.b, 0x10);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // DEC B: 0x10 -> 0x0F, H=1 (borrow), N=1, Z=0, C still unchanged.
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 4);
    assert_eq!(cpu.regs.b, 0x0F);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // Now test INC/DEC A wrapping through zero.
    cpu.regs.pc = 0x0002;
    cpu.regs.a = 0xFF;
    cpu.set_flag(Flag::C, false);

    let c3 = cpu.step(&mut bus); // INC A
    assert_eq!(c3, 4);
    assert_eq!(cpu.regs.a, 0x00);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);

    let c4 = cpu.step(&mut bus); // DEC A
    assert_eq!(c4, 4);
    assert_eq!(cpu.regs.a, 0xFF);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
}

#[test]
fn inc_dec_on_hl_memory() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program: INC (HL); DEC (HL)
    bus.memory[0x0000] = 0x34;
    bus.memory[0x0001] = 0x35;

    cpu.regs.pc = 0x0000;
    cpu.regs.h = 0xC0;
    cpu.regs.l = 0x00;
    bus.memory[0xC000] = 0x00;

    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 12);
    assert_eq!(bus.memory[0xC000], 0x01);

    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 12);
    assert_eq!(bus.memory[0xC000], 0x00);
}

#[test]
fn inc_dec_16bit_and_add_hl_rr_behaviour() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: INC BC
    // 0x0001: DEC BC
    // 0x0002: ADD HL,BC
    // 0x0003: ADD HL,SP
    bus.memory[0x0000] = 0x03;
    bus.memory[0x0001] = 0x0B;
    bus.memory[0x0002] = 0x09;
    bus.memory[0x0003] = 0x39;

    cpu.regs.pc = 0x0000;
    cpu.regs.set_bc(0x1234);
    cpu.regs.set_hl(0x0FFF);
    cpu.regs.sp = 0x0001;

    // Set all flags to 1 to verify INC/DEC do not touch them.
    cpu.regs.f = 0xF0;

    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 8);
    assert_eq!(cpu.regs.bc(), 0x1235);
    assert_eq!(cpu.regs.f, 0xF0);

    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 8);
    assert_eq!(cpu.regs.bc(), 0x1234);
    assert_eq!(cpu.regs.f, 0xF0);

    // ADD HL,BC: 0x0FFF + 0x1234 = 0x2233; carry from bit 11 but not 15.
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 8);
    assert_eq!(cpu.regs.hl(), 0x2233);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // For ADD HL,SP, verify that Z is preserved.
    cpu.set_flag(Flag::Z, true);
    let c4 = cpu.step(&mut bus);
    assert_eq!(c4, 8);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    // HL should have advanced by SP (0x0001).
    assert_eq!(cpu.regs.hl(), 0x2234);
}

#[test]
fn add_sp_r8_signed_and_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: ADD SP, +1
    // 0x0002: ADD SP, -1
    bus.memory[0x0000] = 0xE8;
    bus.memory[0x0001] = 0x01;
    bus.memory[0x0002] = 0xE8;
    bus.memory[0x0003] = 0xFF;

    cpu.regs.pc = 0x0000;
    cpu.regs.sp = 0x0FFF;

    // ADD SP, +1 -> 0x1000, H=1, C=1, Z=0, N=0.
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 16);
    assert_eq!(cpu.regs.sp, 0x1000);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // ADD SP, -1 -> back to 0x0FFF, Z and N remain 0.
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 16);
    assert_eq!(cpu.regs.sp, 0x0FFF);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
}

#[test]
fn ld_hl_sp_plus_r8_and_ld_sp_hl() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: LD HL,SP+1
    // 0x0002: LD SP,HL
    bus.memory[0x0000] = 0xF8;
    bus.memory[0x0001] = 0x01;
    bus.memory[0x0002] = 0xF9;

    cpu.regs.pc = 0x0000;
    cpu.regs.sp = 0x0FFF;

    // LD HL,SP+1 -> HL=0x1000; flags like ADD SP,+1.
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 12);
    assert_eq!(cpu.regs.hl(), 0x1000);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // Set some flags and ensure LD SP,HL does not change them.
    cpu.set_flag(Flag::Z, true);
    cpu.set_flag(Flag::N, true);
    cpu.set_flag(Flag::H, false);
    cpu.set_flag(Flag::C, false);

    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 8);
    assert_eq!(cpu.regs.sp, 0x1000);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), false);
}

#[test]
fn push_and_pop_roundtrip_and_pop_af_masks_low_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: PUSH BC
    // 0x0001: POP DE
    // 0x0002: POP AF   (after manually preparing stack contents)
    bus.memory[0x0000] = 0xC5;
    bus.memory[0x0001] = 0xD1;
    bus.memory[0x0002] = 0xF1;

    cpu.regs.pc = 0x0000;
    cpu.regs.sp = 0xFFFE;
    cpu.regs.set_bc(0x1234);

    // PUSH BC
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 16);
    assert_eq!(cpu.regs.sp, 0xFFFC);

    // POP DE
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 12);
    assert_eq!(cpu.regs.de(), 0x1234);
    assert_eq!(cpu.regs.sp, 0xFFFE);

    // Now test POP AF masking of low flag bits.
    // Prepare stack so that AF would be 0x12 0x3F (low nibble non-zero).
    cpu.regs.sp = 0xFFFC;
    bus.memory[0xFFFC] = 0x3F; // low byte (F)
    bus.memory[0xFFFD] = 0x12; // high byte (A)

    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 12);
    assert_eq!(cpu.regs.a, 0x12);
    // Low 4 bits of F must be cleared.
    assert_eq!(cpu.regs.f & 0x0F, 0x00);
    assert_eq!(cpu.regs.f & 0xF0, 0x30);
}

/// Run a single mooneye "acceptance/interrupts" ROM until it reaches the
/// standard success register pattern used by the test suite, or we hit a
/// large cycle budget. This reuses the same Fibonacci register pattern
/// as the timer tests.
fn run_mooneye_interrupts_rom(filename: &str) {
    let rom = load_mooneye_interrupts_rom(filename);
    // For mooneye acceptance ROMs we try to mirror the reference core's
    // power-on environment rather than the DMG post-boot state used by
    // our high-level emulator. In particular we:
    // - start from zeroed internal RAM
    // - start with LCDC disabled
    // - start the Timer/IF in a "mooneye-like" power-on state where
    //   DIV/TIMA/TMA/TAC/IF are all zeroed.
    let mut gb = crate::machine::GameBoy::new_zeroed_internal_ram();
    gb.load_rom(&rom);
    gb.bus.memory[0xFF40] = 0x00; // LCDC off
    gb.bus.reset_timer_for_mooneye_tests();
    gb.bus.if_reg = 0x00;

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    // Basic instrumentation for interrupt behaviour debugging. We log
    // changes to IE/IME and the first few HALT entries to understand
    // how the ROM configures the interrupt controller.
    let mut prev_ie = gb.bus.read8(0xFFFF);
    let mut prev_ime = gb.cpu.ime;
    let mut halted_entries: u64 = 0;
    let mut prev_halted = gb.cpu.halted;

    while cycles < MAX_CYCLES {
        let c = gb.cpu.step(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        // Track HALT transitions.
        let halted = gb.cpu.halted;
        if !prev_halted && halted {
            halted_entries = halted_entries.saturating_add(1);
            let r = &gb.cpu.regs;
            println!(
                "[mooneye intr dbg] ROM={} HALT entered at cycle={} PC={:04X} SP={:04X} \
                 A={:02X} F={:02X} IF={:02X} IE={:02X} IME={}",
                filename,
                cycles,
                r.pc,
                r.sp,
                r.a,
                r.f & 0xF0,
                gb.bus.read8(0xFF0F),
                gb.bus.read8(0xFFFF),
                gb.cpu.ime,
            );
        }
        prev_halted = halted;

        // Log IE/IME changes; these often mark important configuration
        // points in the ROM's interrupt setup sequence.
        let ie_now = gb.bus.read8(0xFFFF);
        if ie_now != prev_ie {
            let r = &gb.cpu.regs;
            println!(
                "[mooneye intr dbg] ROM={} cycle={} IE change: {:02X} -> {:02X} \
                 PC={:04X} A={:02X} F={:02X} IME={}",
                filename,
                cycles,
                prev_ie,
                ie_now,
                r.pc,
                r.a,
                r.f & 0xF0,
                gb.cpu.ime,
            );
            prev_ie = ie_now;
        }

        let ime_now = gb.cpu.ime;
        if ime_now != prev_ime {
            let r = &gb.cpu.regs;
            println!(
                "[mooneye intr dbg] ROM={} cycle={} IME change: {} -> {} \
                 PC={:04X} A={:02X} F={:02X} IE={:02X}",
                filename,
                cycles,
                prev_ime,
                ime_now,
                r.pc,
                r.a,
                r.f & 0xF0,
                gb.bus.read8(0xFFFF),
            );
            prev_ime = ime_now;
        }

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34
        {
            success = true;
            break;
        }
    }

    if !success {
        panic!(
            "mooneye interrupts ROM '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}\n\
HALT entries: {}",
            filename, MAX_CYCLES, gb.cpu.regs,
            halted_entries,
        );
    }
}

/// Run a single mooneye "acceptance/ppu" ROM until it reaches the
/// standard success register pattern (A,B,C,D,E,H,L = Fibonacci
/// sequence) or we hit a large cycle budget.
///
/// PPU-focused ROMs exercise LCD/STAT/LY timing and related
/// interrupts. Compared to the timer/interrupt helpers, we keep
/// the LCD enabled so that the minimal PPU model in `GameBoyBus`
/// can drive LY/mode and STAT/VBlank interrupts.
fn run_mooneye_ppu_rom(filename: &str) {
    let rom = load_mooneye_ppu_rom(filename);

    // As with other mooneye acceptance helpers, start from a
    // deterministic RAM state and a "mooneye-like" Timer/IF
    // power-on configuration.
    let mut gb = crate::machine::GameBoy::new_zeroed_internal_ram();
    gb.load_rom(&rom);
    gb.bus.reset_timer_for_mooneye_tests();
    gb.bus.if_reg = 0x00;
    // Do not touch LCDC here: we want the DMG default (LCD on)
    // so that PPU timing and STAT/VBlank interrupts are active.

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    while cycles < MAX_CYCLES {
        let c = gb.cpu.step(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34 {
            success = true;
            break;
        }
    }

    if !success {
        panic!(
            "mooneye PPU ROM '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}",
            filename, MAX_CYCLES, gb.cpu.regs,
        );
    }
}

/// Micro-step variant of `run_mooneye_ppu_rom` that drives the CPU
/// using `step_mcycle`. This exercises the per-M-cycle integration
/// between our PPU timing model and the CPU core more directly.
fn run_mooneye_ppu_rom_micro(filename: &str) {
    let rom = load_mooneye_ppu_rom(filename);
    let mut gb = crate::machine::GameBoy::new_zeroed_internal_ram();
    gb.load_rom(&rom);
    gb.bus.reset_timer_for_mooneye_tests();
    gb.bus.if_reg = 0x00;

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    while cycles < MAX_CYCLES {
        let c = gb.cpu.step_mcycle(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34
        {
            success = true;
            break;
        }
    }

    if !success {
        panic!(
            "mooneye PPU ROM (micro) '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}",
            filename, MAX_CYCLES, gb.cpu.regs,
        );
    }
}

/// Micro-step variant of `run_mooneye_interrupts_rom` that drives the
/// CPU using `step_mcycle` instead of the instruction-level `step`.
///
/// This is useful for validating that our per-M-cycle integration
/// (especially around Timer and interrupt entry) behaves sensibly on
/// interrupt-focused acceptance ROMs such as `ie_push.gb`.
fn run_mooneye_interrupts_rom_micro(filename: &str) {
    let rom = load_mooneye_interrupts_rom(filename);
    let mut gb = crate::machine::GameBoy::new_zeroed_internal_ram();
    gb.load_rom(&rom);
    gb.bus.memory[0xFF40] = 0x00;
    gb.bus.reset_timer_for_mooneye_tests();
    gb.bus.if_reg = 0x00;

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    let mut halted_entries: u64 = 0;
    let mut prev_halted = gb.cpu.halted;

    // Basic instrumentation: track when we jump into any interrupt
    // vector and when HALT is entered.
    let mut intr_vector_entries: u64 = 0;
    let mut last_intr_pc: Option<u16> = None;
    // Limit the amount of detailed logging so that particularly
    // chatty ROMs (or bugs) do not spam the output.
    const MAX_INTR_LOGS: u64 = 16;
    let mut detailed_intr_logs: u64 = 0;

    while cycles < MAX_CYCLES {
        let prev_pc = gb.cpu.regs.pc;

        let c = gb.cpu.step_mcycle(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let new_pc = gb.cpu.regs.pc;

        // Track HALT transitions.
        let halted = gb.cpu.halted;
        if !prev_halted && halted {
            halted_entries = halted_entries.saturating_add(1);
            let r = &gb.cpu.regs;
            println!(
                "[mooneye intr micro dbg] ROM={} HALT entered at cycle={} PC={:04X} SP={:04X} \
                 A={:02X} F={:02X} IF={:02X} IE={:02X} IME={}",
                filename,
                cycles,
                r.pc,
                r.sp,
                r.a,
                r.f & 0xF0,
                gb.bus.read8(0xFF0F),
                gb.bus.read8(0xFFFF),
                gb.cpu.ime,
            );
        }
        prev_halted = halted;

        // Count entries into any interrupt vector in the 0x0040–0x0060
        // range. This gives us a coarse picture of how often interrupts
        // fire under the micro-step timing model.
        if prev_pc != new_pc && (0x0040..=0x0060).contains(&new_pc) {
            intr_vector_entries = intr_vector_entries.saturating_add(1);
            last_intr_pc = Some(new_pc);

            if detailed_intr_logs < MAX_INTR_LOGS {
                detailed_intr_logs = detailed_intr_logs.saturating_add(1);
                let r = &gb.cpu.regs;
                let if_reg = gb.bus.read8(0xFF0F);
                let ie_reg = gb.bus.read8(0xFFFF);
                let sp = r.sp;

                // Peek a few bytes at the top of the stack for IE/PC
                // debugging. Out-of-bounds reads fall back to 0.
                let byte_at = |addr: u16, mem: &[u8; 0x10000]| -> u8 {
                    *mem.get(addr as usize).unwrap_or(&0)
                };
                let s0 = byte_at(sp, &gb.bus.memory);
                let s1 = byte_at(sp.wrapping_add(1), &gb.bus.memory);
                let s2 = byte_at(sp.wrapping_add(2), &gb.bus.memory);

                println!(
                    "[mooneye intr micro dbg] ROM={} intr_entry={} cycle={} vec_pc={:04X} prev_pc={:04X} \
                     SP={:04X} stack[0]={:02X} stack[1]={:02X} stack[2]={:02X} \
                     IF={:02X} IE={:02X} IME={} A={:02X} F={:02X}",
                    filename,
                    intr_vector_entries,
                    cycles,
                    new_pc,
                    prev_pc,
                    sp,
                    s0,
                    s1,
                    s2,
                    if_reg,
                    ie_reg,
                    gb.cpu.ime,
                    r.a,
                    r.f & 0xF0,
                );
            }
        }

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34 {
            success = true;
            break;
        }
    }

    if !success {
        panic!(
            "mooneye interrupts ROM (micro) '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}\n\
Interrupt vector entries: {}\n\
Last interrupt PC: {:?}\n\
HALT entries: {}",
            filename,
            MAX_CYCLES,
            gb.cpu.regs,
            intr_vector_entries,
            last_intr_pc,
            halted_entries,
        );
    }
}

/// Run a single mooneye "acceptance/timer" ROM until it reaches the
/// standard success register pattern used by the test suite, or we hit a
/// large cycle budget.
///
/// Mooneye's harness treats a test as successful when it triggers a
/// special breakpoint and the CPU registers contain:
///   A = 0 (no assertion failures)
///   B = 3, C = 5, D = 8, E = 13, H = 21, L = 34
///
/// We approximate that behaviour by running the ROM and periodically
/// checking the registers for this pattern.
fn run_mooneye_timer_rom(filename: &str) {
    let rom = load_mooneye_timer_rom(filename);
    let mut gb = crate::machine::GameBoy::new();
    gb.load_rom(&rom);

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    // Basic instrumentation to help debug timer/interrupt timing issues on
    // mooneye ROMs. We track how many times execution jumps to the timer
    // ISR vector ($0050) and how often HALT is entered. In test builds we
    // also expose the total number of timer T-cycles.
    let mut timer_irq_count: u64 = 0;
    let mut last_timer_irq_cycle: Option<u64> = None;
    let mut last_timer_irq_interval: Option<u64> = None;
    let mut halted_entries: u64 = 0;
    let mut prev_halted = gb.cpu.halted;

    // Optional extra debug for tricky timer tests such as tim00 and
    // tim00_div_trigger: log CPU/Timer/interrupt state around the first
    // few TIMA overflows so we can see whether IE/IME/IF are configured
    // as expected when the timer starts requesting interrupts.
    let log_overflow_debug = matches!(filename, "tim00.gb" | "tim00_div_trigger.gb");
    let mut prev_overflows: u64 = 0;
    let mut prev_ie_reg: u8 = gb.bus.read8(0xFFFF);
    let mut prev_ime: bool = gb.cpu.ime;

    while cycles < MAX_CYCLES {
        let prev_pc = gb.cpu.regs.pc;

        let c = gb.cpu.step(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let new_pc = gb.cpu.regs.pc;

        // Count transitions into HALT.
        let halted = gb.cpu.halted;
        if !prev_halted && halted {
            halted_entries = halted_entries.saturating_add(1);
        }
        prev_halted = halted;

        // Count entries into the timer ISR at $0050 (from any other PC).
        if prev_pc != 0x0050 && new_pc == 0x0050 {
            timer_irq_count = timer_irq_count.saturating_add(1);
            if let Some(prev_cycle) = last_timer_irq_cycle {
                last_timer_irq_interval = Some(cycles.saturating_sub(prev_cycle));
            }
            last_timer_irq_cycle = Some(cycles);
        }

        // For selected ROMs, log the environment around the first few
        // TIMA overflows so we can correlate timer behaviour with the
        // interrupt controller state.
        if log_overflow_debug {
            // Track when IE or IME change, as this often marks important
            // points in the ROM's timer/interrupt setup sequence.
            let ie_now = gb.bus.read8(0xFFFF);
            if ie_now != prev_ie_reg {
                let regs = &gb.cpu.regs;
                println!(
                    "[mooneye timer dbg] ROM={} cycle={} IE change: {:02X} -> {:02X} \
                     PC={:04X} A={:02X} F={:02X} IME={}",
                    filename,
                    cycles,
                    prev_ie_reg,
                    ie_now,
                    regs.pc,
                    regs.a,
                    regs.f & 0xF0,
                    gb.cpu.ime,
                );
                prev_ie_reg = ie_now;
            }

            let ime_now = gb.cpu.ime;
            if ime_now != prev_ime {
                let regs = &gb.cpu.regs;
                println!(
                    "[mooneye timer dbg] ROM={} cycle={} IME change: {} -> {} \
                     PC={:04X} A={:02X} F={:02X} IE={:02X}",
                    filename,
                    cycles,
                    prev_ime,
                    ime_now,
                    regs.pc,
                    regs.a,
                    regs.f & 0xF0,
                    gb.bus.read8(0xFFFF),
                );
                prev_ime = ime_now;
            }

            let overflows = gb.bus.timer_debug_overflows();
            if overflows != prev_overflows {
                // Only print the first few overflows to keep the log
                // manageable when running manually.
                if overflows <= 4 {
                    let div = gb.bus.read8(0xFF04);
                    let tima = gb.bus.read8(0xFF05);
                    let tma = gb.bus.read8(0xFF06);
                    let tac = gb.bus.read8(0xFF07);
                    let if_reg = gb.bus.read8(0xFF0F);
                    let ie_reg = gb.bus.read8(0xFFFF);
                    let regs = &gb.cpu.regs;
                    println!(
                        "[mooneye timer dbg] ROM={} cycle={} overflows={} \
                         PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} \
                         D={:02X} E={:02X} H={:02X} L={:02X} IME={} HALT={} \
                         DIV={:02X} TIMA={:02X} TMA={:02X} TAC={:02X} IF={:02X} IE={:02X}",
                        filename,
                        cycles,
                        overflows,
                        regs.pc,
                        regs.sp,
                        regs.a,
                        regs.f & 0xF0,
                        regs.b,
                        regs.c,
                        regs.d,
                        regs.e,
                        regs.h,
                        regs.l,
                        gb.cpu.ime,
                        gb.cpu.halted,
                        div,
                        tima,
                        tma,
                        tac,
                        if_reg,
                        ie_reg,
                    );
                }
                prev_overflows = overflows;
            }
        }

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34 {
            success = true;
            break;
        }
    }

    if !success {
        #[cfg(test)]
        let (timer_ticks, tima_incs, overflows) = (
            gb.bus.timer_debug_ticks(),
            gb.bus.timer_debug_tima_increments(),
            gb.bus.timer_debug_overflows(),
        );
        #[cfg(not(test))]
        let (timer_ticks, tima_incs, overflows) = (0, 0, 0);

        panic!(
            "mooneye timer ROM '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}\n\
Timer ticks (timer cycles): {}\n\
TIMA increments: {}\n\
Overflow events serviced: {}\n\
Timer IRQs taken: {}\n\
Last timer IRQ at cycle: {:?}\n\
Last timer IRQ interval: {:?}\n\
HALT entries: {}",
            filename,
            MAX_CYCLES,
            gb.cpu.regs,
            timer_ticks,
            tima_incs,
            overflows,
            timer_irq_count,
            last_timer_irq_cycle,
            last_timer_irq_interval,
            halted_entries,
        );
    }
}

/// Micro-step variant of `run_mooneye_timer_rom` that drives the CPU
/// using `step_mcycle` instead of instruction-level `step`. This is
/// useful to validate that the per-M-cycle micro-op sequencing remains
/// compatible with the existing timer model and Mooneye's acceptance
/// tests.
fn run_mooneye_timer_rom_micro(filename: &str) {
    let rom = load_mooneye_timer_rom(filename);
    let mut gb = crate::machine::GameBoy::new();
    gb.load_rom(&rom);

    const MAX_CYCLES: u64 = 50_000_000;
    let mut cycles: u64 = 0;
    let mut success = false;

    let mut timer_irq_count: u64 = 0;
    let mut last_timer_irq_cycle: Option<u64> = None;
    let mut last_timer_irq_interval: Option<u64> = None;
    let mut halted_entries: u64 = 0;
    let mut prev_halted = gb.cpu.halted;

    while cycles < MAX_CYCLES {
        let prev_pc = gb.cpu.regs.pc;

        let c = gb.cpu.step_mcycle(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let new_pc = gb.cpu.regs.pc;

        let halted = gb.cpu.halted;
        if !prev_halted && halted {
            halted_entries = halted_entries.saturating_add(1);
        }
        prev_halted = halted;

        if prev_pc != 0x0050 && new_pc == 0x0050 {
            timer_irq_count = timer_irq_count.saturating_add(1);
            if let Some(prev_cycle) = last_timer_irq_cycle {
                last_timer_irq_interval = Some(cycles.saturating_sub(prev_cycle));
            }
            last_timer_irq_cycle = Some(cycles);
        }

        let r = &gb.cpu.regs;
        if r.a == 0 && r.b == 3 && r.c == 5 && r.d == 8 && r.e == 13 && r.h == 21 && r.l == 34 {
            success = true;
            break;
        }
    }

    if !success {
        #[cfg(test)]
        let (timer_ticks, tima_incs, overflows) = (
            gb.bus.timer_debug_ticks(),
            gb.bus.timer_debug_tima_increments(),
            gb.bus.timer_debug_overflows(),
        );
        #[cfg(not(test))]
        let (timer_ticks, tima_incs, overflows) = (0, 0, 0);

        panic!(
            "mooneye timer ROM (micro) '{}' did not reach success register pattern within {} cycles.\n\
Final regs: {:?}\n\
Timer ticks (timer cycles): {}\n\
TIMA increments: {}\n\
Overflow events serviced: {}\n\
Timer IRQs taken: {}\n\
Last timer IRQ at cycle: {:?}\n\
Last timer IRQ interval: {:?}\n\
HALT entries: {}",
            filename,
            MAX_CYCLES,
            gb.cpu.regs,
            timer_ticks,
            tima_incs,
            overflows,
            timer_irq_count,
            last_timer_irq_cycle,
            last_timer_irq_interval,
            halted_entries,
        );
    }
}

#[test]
fn daa_cpl_scf_ccf_behaviour() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: ADD A, 0x15  (A starts at 0x27 -> 0x3C)
    // 0x0002: DAA         (adjust A; here we only check flag semantics)
    // 0x0003: CPL         (invert A)
    // 0x0004: SCF         (set carry)
    // 0x0005: CCF         (invert carry)
    bus.memory[0x0000] = 0xC6; // ADD A, d8
    bus.memory[0x0001] = 0x15;
    bus.memory[0x0002] = 0x27; // DAA
    bus.memory[0x0003] = 0x2F; // CPL
    bus.memory[0x0004] = 0x37; // SCF
    bus.memory[0x0005] = 0x3F; // CCF

    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0x27;

    // ADD A,0x15 => 0x3C, no carry; H flag may or may not be set,
    // but Z=0, N=0 is guaranteed.
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 8);
    assert_eq!(cpu.regs.a, 0x3C);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);

    // DAA: 0x3C (BCD 3.12) adjusted to 0x42 (BCD 4.2), C=0.
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 4);
    assert_eq!(cpu.regs.a, 0x42);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // CPL: bitwise invert A, set H=1,N=1, do not touch Z/C.
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 4);
    assert_eq!(cpu.regs.a, !0x42);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::N), true);

    // SCF: set C=1, clear H and N, leave Z untouched.
    let c4 = cpu.step(&mut bus);
    assert_eq!(c4, 4);
    assert_eq!(cpu.get_flag(Flag::C), true);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);

    // CCF: toggle C, clear H and N again.
    let c5 = cpu.step(&mut bus);
    assert_eq!(c5, 4);
    assert_eq!(cpu.get_flag(Flag::C), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
}

#[test]
fn rlca_rrca_rla_rra_behaviour() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: RLCA
    // 0x0001: RRCA
    // 0x0002: RLA
    // 0x0003: RRA
    bus.memory[0x0000] = 0x07;
    bus.memory[0x0001] = 0x0F;
    bus.memory[0x0002] = 0x17;
    bus.memory[0x0003] = 0x1F;

    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0b1000_0001;
    cpu.set_flag(Flag::C, false);

    // RLCA: 1000_0001 -> 0000_0011, C=1, Z=0,N=0,H=0.
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 4);
    assert_eq!(cpu.regs.a, 0b0000_0011);
    assert_eq!(cpu.get_flag(Flag::C), true);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);

    // RRCA: 0000_0011 -> 1000_0001, C=1.
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 4);
    assert_eq!(cpu.regs.a, 0b1000_0001);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // RLA with C=1: 1000_0001 -> 0000_0011, C=1.
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 4);
    assert_eq!(cpu.regs.a, 0b0000_0011);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // Clear carry, then RRA: 0000_0011, C=0 -> 0000_0001, C=1.
    cpu.set_flag(Flag::C, false);
    let c4 = cpu.step(&mut bus);
    assert_eq!(c4, 4);
    assert_eq!(cpu.regs.a, 0b0000_0001);
    assert_eq!(cpu.get_flag(Flag::C), true);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
}

#[test]
fn cb_rlc_b_and_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // 0x0000: CB 00 => RLC B
    bus.memory[0x0000] = 0xCB;
    bus.memory[0x0001] = 0x00;

    cpu.regs.pc = 0x0000;
    cpu.regs.b = 0b1000_0001;

    let cycles = cpu.step(&mut bus);
    assert_eq!(cycles, 8);
    // 1000_0001 -> 0000_0011, C=1
    assert_eq!(cpu.regs.b, 0b0000_0011);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), true);
}

#[test]
fn ld_and_inc_a_work() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: LD A, 0x0F
    // 0x0002: INC A
    bus.memory[0x0000] = 0x3E;
    bus.memory[0x0001] = 0x0F;
    bus.memory[0x0002] = 0x3C;

    cpu.regs.pc = 0x0000;

    // Make flags and A start from a neutral state.
    cpu.regs.a = 0x00;
    cpu.regs.f = 0x00;
    cpu.regs.pc = 0x0000;

    // LD A, 0x0F
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 8);
    assert_eq!(cpu.regs.a, 0x0F);
    assert_eq!(cpu.regs.pc, 0x0002);

    // INC A
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 4);
    assert_eq!(cpu.regs.a, 0x10);

    // After incrementing 0x0F -> 0x10:
    // Z = 0, N = 0, H = 1, C unchanged (0)
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), false);
}

#[test]
fn cb_bit_res_set_on_hl() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Place value at HL.
    cpu.regs.h = 0xC0;
    cpu.regs.l = 0x00;
    bus.memory[0xC000] = 0xFF;

    // BIT 7,(HL): CB 7E
    bus.memory[0x0000] = 0xCB;
    bus.memory[0x0001] = 0x7E;
    cpu.regs.pc = 0x0000;
    cpu.set_flag(Flag::C, true);
    let cycles_bit = cpu.step(&mut bus);
    assert_eq!(cycles_bit, 12);
    // Bit 7 set -> Z=0, H=1, N=0, C preserved.
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);

    // RES 0,(HL): CB 86
    bus.memory[0x0002] = 0xCB;
    bus.memory[0x0003] = 0x86;
    cpu.regs.pc = 0x0002;
    let cycles_res = cpu.step(&mut bus);
    assert_eq!(cycles_res, 16);
    assert_eq!(bus.memory[0xC000], 0xFE);

    // SET 0,(HL): CB C6
    bus.memory[0x0004] = 0xCB;
    bus.memory[0x0005] = 0xC6;
    cpu.regs.pc = 0x0004;
    let cycles_set = cpu.step(&mut bus);
    assert_eq!(cycles_set, 16);
    assert_eq!(bus.memory[0xC000], 0xFF);
}

#[test]
fn jp_absolute_sets_pc() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // 0x0000: JP 0x1234
    bus.memory[0x0000] = 0xC3;
    bus.memory[0x0001] = 0x34; // low byte
    bus.memory[0x0002] = 0x12; // high byte

    cpu.regs.pc = 0x0000;
    let cycles = cpu.step(&mut bus);

    assert_eq!(cycles, 16);
    assert_eq!(cpu.regs.pc, 0x1234);
}

#[test]
fn add_immediate_updates_a_and_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program: LD A, 0x01; ADD A, 0x01
    bus.memory[0x0000] = 0x3E; // LD A, d8
    bus.memory[0x0001] = 0x01;
    bus.memory[0x0002] = 0xC6; // ADD A, d8
    bus.memory[0x0003] = 0x01;

    cpu.regs.pc = 0x0000;

    let _ = cpu.step(&mut bus); // LD
    let cycles = cpu.step(&mut bus); // ADD

    assert_eq!(cycles, 8);
    assert_eq!(cpu.regs.a, 0x02);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), false);
}

#[test]
fn add_register_sets_half_carry_and_carry() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Single instruction: ADD A, B
    bus.memory[0x0000] = 0x80;
    cpu.regs.pc = 0x0000;

    // Case 1: 0x0F + 0x01 = 0x10, H=1, C=0
    cpu.regs.a = 0x0F;
    cpu.regs.b = 0x01;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x10);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // Reset PC and run again for a different case.
    cpu.regs.pc = 0x0000;

    // Case 2: 0xFF + 0x01 = 0x00, Z=1, H=1, C=1
    cpu.regs.a = 0xFF;
    cpu.regs.b = 0x01;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x00);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);
}

#[test]
fn jr_relative_forward_and_backward() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // 0x0000: JR +2
    bus.memory[0x0000] = 0x18;
    bus.memory[0x0001] = 0x02;
    // NOPs / padding
    bus.memory[0x0002] = 0x00;
    bus.memory[0x0003] = 0x00;

    cpu.regs.pc = 0x0000;
    let cycles = cpu.step(&mut bus);
    assert_eq!(cycles, 12);
    // PC after JR should be 0x0004 (0x0002 + 2)
    assert_eq!(cpu.regs.pc, 0x0004);

    // Now test a backward jump: place JR -2 at 0x0010
    cpu.regs.pc = 0x0010;
    bus.memory[0x0010] = 0x18;
    bus.memory[0x0011] = 0xFE; // -2
    let cycles2 = cpu.step(&mut bus);
    assert_eq!(cycles2, 12);
    // After fetch of offset, PC = 0x0012, then +(-2) => 0x0010
    assert_eq!(cpu.regs.pc, 0x0010);
}

#[test]
fn jr_nz_condition() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // JR NZ, +2
    bus.memory[0x0000] = 0x20;
    bus.memory[0x0001] = 0x02;

    // Case 1: Z = 1, branch not taken
    cpu.regs.pc = 0x0000;
    cpu.set_flag(Flag::Z, true);
    let cycles = cpu.step(&mut bus);
    assert_eq!(cycles, 8);
    assert_eq!(cpu.regs.pc, 0x0002);

    // Case 2: Z = 0, branch taken
    cpu.regs.pc = 0x0000;
    cpu.set_flag(Flag::Z, false);
    let cycles2 = cpu.step(&mut bus);
    assert_eq!(cycles2, 12);
    assert_eq!(cpu.regs.pc, 0x0004);
}

#[test]
fn call_and_ret_roundtrip() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program:
    // 0x0000: CALL 0x1234
    bus.memory[0x0000] = 0xCD;
    bus.memory[0x0001] = 0x34;
    bus.memory[0x0002] = 0x12;
    // Subroutine at 0x1234: RET
    bus.memory[0x1234] = 0xC9;

    cpu.regs.pc = 0x0000;
    cpu.regs.sp = 0xFFFE;

    // CALL
    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 24);
    assert_eq!(cpu.regs.pc, 0x1234);
    assert_eq!(cpu.regs.sp, 0xFFFC);
    // Return address (0x0003) should be on stack at 0xFFFC/0xFFFD
    let lo = bus.memory[0xFFFC];
    let hi = bus.memory[0xFFFD];
    assert_eq!(lo, 0x03);
    assert_eq!(hi, 0x00);

    // RET
    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 16);
    assert_eq!(cpu.regs.pc, 0x0003);
    assert_eq!(cpu.regs.sp, 0xFFFE);
}

#[test]
fn jp_cc_and_ret_cc_behaviour() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // JP NZ, a16 at 0x0000
    bus.memory[0x0000] = 0xC2;
    bus.memory[0x0001] = 0x34;
    bus.memory[0x0002] = 0x12;

    // Case 1: Z = 1 -> not taken
    cpu.regs.pc = 0x0000;
    cpu.set_flag(Flag::Z, true);
    let cycles = cpu.step(&mut bus);
    assert_eq!(cycles, 12);
    assert_eq!(cpu.regs.pc, 0x0003);

    // Case 2: Z = 0 -> taken
    cpu.regs.pc = 0x0000;
    cpu.set_flag(Flag::Z, false);
    let cycles2 = cpu.step(&mut bus);
    assert_eq!(cycles2, 16);
    assert_eq!(cpu.regs.pc, 0x1234);

    // Now test RET Z
    // Prepare stack with return address 0x00FF.
    cpu.regs.sp = 0xFFFC;
    bus.memory[0xFFFC] = 0xFF;
    bus.memory[0xFFFD] = 0x00;

    // Place RET Z at 0x2000 (only opcode matters).
    cpu.regs.pc = 0x2000;
    bus.memory[0x2000] = 0xC8; // RET Z

    // Case 3: Z = 0 -> not taken
    cpu.set_flag(Flag::Z, false);
    let c3 = cpu.step(&mut bus);
    assert_eq!(c3, 8);
    assert_eq!(cpu.regs.pc, 0x2001);
    assert_eq!(cpu.regs.sp, 0xFFFC); // stack unchanged

    // Case 4: Z = 1 -> taken
    cpu.regs.pc = 0x2000;
    cpu.set_flag(Flag::Z, true);
    let c4 = cpu.step(&mut bus);
    assert_eq!(c4, 20);
    assert_eq!(cpu.regs.pc, 0x00FF);
    assert_eq!(cpu.regs.sp, 0xFFFE);
}

#[test]
fn rst_and_reti_behaviour() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Place RST 0x38 at 0x0000.
    cpu.regs.pc = 0x0000;
    cpu.regs.sp = 0xFFFE;
    bus.memory[0x0000] = 0xFF; // RST 38H

    let c1 = cpu.step(&mut bus);
    assert_eq!(c1, 16);
    assert_eq!(cpu.regs.pc, 0x0038);
    assert_eq!(cpu.regs.sp, 0xFFFC);

    // Return address should be 0x0001 on stack.
    assert_eq!(bus.memory[0xFFFC], 0x01);
    assert_eq!(bus.memory[0xFFFD], 0x00);

    // Now test RETI: put it at 0x0038 and clear IME first.
    cpu.regs.pc = 0x0038;
    cpu.ime = false;
    bus.memory[0x0038] = 0xD9; // RETI

    let c2 = cpu.step(&mut bus);
    assert_eq!(c2, 16);
    assert_eq!(cpu.regs.pc, 0x0001);
    assert_eq!(cpu.regs.sp, 0xFFFE);
    assert_eq!(cpu.ime, true);
}

#[test]
fn ei_sets_ime_after_next_instruction() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // 0x0000: EI
    // 0x0001: NOP
    bus.memory[0x0000] = 0xFB;
    bus.memory[0x0001] = 0x00;
    cpu.regs.pc = 0x0000;

    cpu.ime = false;

    // Execute EI
    let _ = cpu.step(&mut bus);
    // EI 所在这条指令之后 IME 仍为 0
    assert_eq!(cpu.ime, false);

    // Execute NOP
    let _ = cpu.step(&mut bus);
    // NOP 执行完成后，IME 才变为 1
    assert_eq!(cpu.ime, true);
}

#[test]
fn di_clears_ime_immediately() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // 0x0000: DI
    bus.memory[0x0000] = 0xF3;
    cpu.regs.pc = 0x0000;

    cpu.ime = true;
    cpu.step(&mut bus);
    assert_eq!(cpu.ime, false);
}

/// Integration test: when the timer is configured and both IE/IME
/// enable the timer interrupt, a TIMA overflow should eventually
/// cause the CPU to jump to the timer ISR vector at $0050.
///
/// This is marked as `#[ignore]` because it is primarily a debugging
/// aid while working on timer/interrupt timing, and it relies on the
/// current (approximate) CPU/Timer integration.
#[test]
#[ignore]
fn timer_overflow_triggers_timer_interrupt() {
    let mut gb = GameBoy::new();

    // Program placed at the DMG post-boot entry point 0x0100:
    //   0x0100: EI         ; enable IME after next instruction
    //   0x0101: NOP        ; IME becomes 1 after this
    //   0x0102: JR -2      ; tight loop at 0x0102
    gb.bus.memory[0x0100] = 0xFB; // EI
    gb.bus.memory[0x0101] = 0x00; // NOP
    gb.bus.memory[0x0102] = 0x18; // JR r8
    gb.bus.memory[0x0103] = 0xFE; // offset -2

    // Configure the timer for a fast overflow:
    // - TIMA starts near overflow (0xFE)
    // - TMA=0x00 (reload value)
    // - TAC: enable timer + fast input (bits 1:0 = 01)
    gb.bus.write8(0xFF06, 0x00); // TMA
    gb.bus.write8(0xFF05, 0xFE); // TIMA
    gb.bus.write8(0xFF07, 0b101); // TAC: enable + freq=01

    // Enable only the timer interrupt in IE.
    gb.bus.write8(0xFFFF, 0x04);

    // Sanity check boot state.
    assert_eq!(gb.cpu.regs.pc, 0x0100);
    assert_eq!(gb.cpu.ime, false);

    const MAX_CYCLES: u64 = 1_000_000;
    let mut cycles: u64 = 0;
    let mut seen_overflow = false;
    let mut seen_timer_irq = false;

    while cycles < MAX_CYCLES {
        let prev_pc = gb.cpu.regs.pc;
        let c = gb.cpu.step(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        if gb.bus.timer_debug_overflows() > 0 {
            seen_overflow = true;
        }

        let new_pc = gb.cpu.regs.pc;
        if prev_pc != 0x0050 && new_pc == 0x0050 {
            seen_timer_irq = true;
            break;
        }
    }

    assert!(
        seen_overflow,
        "Timer did not overflow within {} cycles (timer_ticks={})",
        MAX_CYCLES,
        gb.bus.timer_debug_ticks()
    );

    assert!(
        seen_timer_irq,
        "Timer interrupt was not taken despite overflow.\n\
timer_ticks={} tima_incs={} overflows={} IME={} IF={:02X} IE={:02X} PC={:04X}",
        gb.bus.timer_debug_ticks(),
        gb.bus.timer_debug_tima_increments(),
        gb.bus.timer_debug_overflows(),
        gb.cpu.ime,
        gb.bus.read8(0xFF0F),
        gb.bus.read8(0xFFFF),
        gb.cpu.regs.pc,
    );
}

/// Run blargg's cpu_instrs.gb until either it produces the expected serial
/// output or we hit a large cycle budget.
///
/// This test is ignored by default because it is relatively slow and our
/// CPU/MMU implementation is not yet complete. Once the core becomes more
/// accurate and stable, we can enable it regularly.
#[test]
#[ignore]
fn run_blargg_cpu_instrs() {
    // Large safety budget to keep the test bounded even if the emulator
    // misbehaves. The reference implementation finishes well before this.
    const MAX_CYCLES: u64 = 2_000_000_000;

    let rom = load_cpu_instrs_rom();
    let mut gb = crate::machine::GameBoy::new();
    gb.load_rom(rom);

    let mut cycles: u64 = 0;
    loop {
        let c = gb.cpu.step(&mut gb.bus) as u64;
        // Avoid an infinite loop if we ever return 0 cycles.
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let output = &gb.bus.serial.output;

        // If we've produced at least as many bytes as the expected output,
        // we can stop and compare exactly.
        if output.len() >= CPU_SERIAL_EXPECT.len() {
            break;
        }

        if cycles >= MAX_CYCLES {
            break;
        }
    }

    let output = &gb.bus.serial.output;
    let output_str = String::from_utf8_lossy(output);
    assert_eq!(
        output, CPU_SERIAL_EXPECT,
        "cpu_instrs serial output mismatch.\nCaptured output:\n{}\n(total cycles {})\n",
        output_str, cycles
    );
}

/// Debug helper for the combined cpu_instrs.gb ROM.
///
/// This does not assert anything; it just runs the ROM for a bounded
/// number of cycles and prints CPU/bus state whenever new serial output
/// is produced. Use this to understand where the ROM gets "stuck"
/// between sub‑tests (e.g. after "03").
#[test]
#[ignore]
fn debug_blargg_cpu_instrs_progress() {
    const MAX_CYCLES: u64 = 400_000_000;

    let rom = load_cpu_instrs_rom();
    let mut gb = crate::machine::GameBoy::new();
    gb.load_rom(rom);

    let mut cycles: u64 = 0;
    let mut last_output_len: usize = 0;
    let mut last_halted: bool = false;

    while cycles < MAX_CYCLES {
        let c = gb.cpu.step(&mut gb.bus) as u64;
        if c == 0 {
            break;
        }
        cycles = cycles.saturating_add(c);

        let out_len = gb.bus.serial.output.len();
        if out_len != last_output_len {
            // Take a snapshot of the serial buffer so that we can release
            // the immutable borrow on `gb.bus` before calling `read8`.
            let out_snapshot = gb.bus.serial.output.clone();
            last_output_len = out_snapshot.len();
            let out_str = String::from_utf8_lossy(&out_snapshot);

            // Snapshot a subset of CPU and bus state that is useful for
            // diagnosing where the ROM is spending time.
            println!(
                "[serial len={len} cycles={cycles}] output_so_far = {text:?}\n\
                 PC={pc:04X} SP={sp:04X} AF={af:04X} BC={bc:04X} DE={de:04X} HL={hl:04X} \
                 IME={ime} halted={halted} halt_bug={halt_bug}\n\
                 IF={if_reg:02X} IE={ie_reg:02X} DIV={div:02X} TIMA={tima:02X} \
                 TMA={tma:02X} TAC={tac:02X}\n",
                len = out_snapshot.len(),
                cycles = cycles,
                text = out_str,
                pc = gb.cpu.regs.pc,
                sp = gb.cpu.regs.sp,
                af = gb.cpu.regs.af(),
                bc = gb.cpu.regs.bc(),
                de = gb.cpu.regs.de(),
                hl = gb.cpu.regs.hl(),
                ime = gb.cpu.ime,
                halted = gb.cpu.halted,
                halt_bug = gb.cpu.halt_bug,
                if_reg = gb.bus.read8(0xFF0F),
                ie_reg = gb.bus.read8(0xFFFF),
                div = gb.bus.read8(0xFF04),
                tima = gb.bus.read8(0xFF05),
                tma = gb.bus.read8(0xFF06),
                tac = gb.bus.read8(0xFF07),
            );
        }

        // Log the first time we enter HALT so we can see the context in
        // which the ROM decides to stop the CPU.
        if gb.cpu.halted && !last_halted {
            last_halted = true;
            println!(
                "[halt-enter] cycles={cycles} \
                 PC={pc:04X} SP={sp:04X} AF={af:04X} BC={bc:04X} DE={de:04X} HL={hl:04X} \
                 IME={ime} IF={if_reg:02X} IE={ie_reg:02X}",
                cycles = cycles,
                pc = gb.cpu.regs.pc,
                sp = gb.cpu.regs.sp,
                af = gb.cpu.regs.af(),
                bc = gb.cpu.regs.bc(),
                de = gb.cpu.regs.de(),
                hl = gb.cpu.regs.hl(),
                ime = gb.cpu.ime,
                if_reg = gb.bus.read8(0xFF0F),
                ie_reg = gb.bus.read8(0xFFFF),
            );
        } else if !gb.cpu.halted {
            last_halted = false;
        }
    }

    // Final snapshot so we can see where the CPU/bus ended up even if
    // no further serial output was produced.
    println!(
        "[end] cycles={cycles} \
         PC={pc:04X} SP={sp:04X} AF={af:04X} BC={bc:04X} DE={de:04X} HL={hl:04X} \
         IME={ime} halted={halted} halt_bug={halt_bug} \
         IF={if_reg:02X} IE={ie_reg:02X} DIV={div:02X} TIMA={tima:02X} \
         TMA={tma:02X} TAC={tac:02X}",
        cycles = cycles,
        pc = gb.cpu.regs.pc,
        sp = gb.cpu.regs.sp,
        af = gb.cpu.regs.af(),
        bc = gb.cpu.regs.bc(),
        de = gb.cpu.regs.de(),
        hl = gb.cpu.regs.hl(),
        ime = gb.cpu.ime,
        halted = gb.cpu.halted,
        halt_bug = gb.cpu.halt_bug,
        if_reg = gb.bus.read8(0xFF0F),
        ie_reg = gb.bus.read8(0xFFFF),
        div = gb.bus.read8(0xFF04),
        tima = gb.bus.read8(0xFF05),
        tma = gb.bus.read8(0xFF06),
        tac = gb.bus.read8(0xFF07),
    );

    // Dump a small window around the current PC in WRAM/ROM to see what
    // the CPU is trying to execute when it appears stuck.
    let pc = gb.cpu.regs.pc;
    let start = pc.wrapping_sub(0x10);
    let end = pc.wrapping_add(0x10);
    println!("Memory around PC ({} ..={}):", start, end);
    for addr in (start..=end).step_by(0x10) {
        let mut line = String::new();
        for offset in 0..0x10 {
            let a = addr.wrapping_add(offset);
            let byte = gb.bus.read8(a);
            line.push_str(&format!("{:02X} ", byte));
        }
        println!("{:04X}: {}", addr, line);
    }
}

#[test]
fn sub_and_sbc_update_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // Program: SUB A, d8 (0xD6 imm)
    bus.memory[0x0000] = 0xD6;
    bus.memory[0x0001] = 0x01;
    cpu.regs.pc = 0x0000;

    // Case 1: 0x10 - 0x01 = 0x0F => Z=0, N=1, H=1 (borrow from bit 4), C=0
    cpu.regs.a = 0x10;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x0F);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // Reset PC and run again for SBC case
    cpu.regs.pc = 0x0000;

    // Case 2: set carry, then SBC 0x01 from 0x00: 0x00 - 0x01 - 1 = 0xFE
    cpu.regs.a = 0x00;
    cpu.set_flag(Flag::C, true);
    let _ = cpu.step(&mut bus); // still using 0xD6 (SUB), but since carry is ignored it's 0xFF
                                // Now test SBC explicit: SBC A, d8
    bus.memory[0x0000] = 0xDE; // SBC A, d8
    bus.memory[0x0001] = 0x01;
    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0x00;
    cpu.set_flag(Flag::C, true);
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0xFE);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), true);
}

#[test]
fn logical_ops_and_cp_flags() {
    let mut cpu = Cpu::new();
    let mut bus = TestBus::default();

    // AND A, d8 (0xE6)
    bus.memory[0x0000] = 0xE6;
    bus.memory[0x0001] = 0x0F;
    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0xF0;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x00);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // OR A, d8 (0xF6)
    bus.memory[0x0000] = 0xF6;
    bus.memory[0x0001] = 0x0F;
    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0xF0;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0xFF);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // XOR A, d8 (0xEE)
    bus.memory[0x0000] = 0xEE;
    bus.memory[0x0001] = 0xFF;
    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0xFF;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x00);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), false);

    // CP d8 (0xFE) — compare 0x10 with 0x10 => Z=1, N=1, H=0, C=0, A unchanged
    bus.memory[0x0000] = 0xFE;
    bus.memory[0x0001] = 0x10;
    cpu.regs.pc = 0x0000;
    cpu.regs.a = 0x10;
    let _ = cpu.step(&mut bus);
    assert_eq!(cpu.regs.a, 0x10);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::C), false);
}

// --- Mooneye timer acceptance tests (optional, run manually) ---

#[test]
fn mooneye_timer_tim00() {
    run_mooneye_timer_rom("tim00.gb");
    run_mooneye_timer_rom_micro("tim00.gb");
}

/// Mooneye acceptance `ppu/vblank_stat_intr-GS` ROM: exercises the
/// relationship between VBlank and STAT mode-2 interrupts at LY=144.
#[test]
#[ignore]
fn mooneye_ppu_vblank_stat_intr() {
    run_mooneye_ppu_rom("vblank_stat_intr-GS.gb");
}

/// Micro-step variant of the `vblank_stat_intr-GS` PPU test.
#[test]
#[ignore]
fn mooneye_ppu_vblank_stat_intr_micro() {
    run_mooneye_ppu_rom_micro("vblank_stat_intr-GS.gb");
}

/// Mooneye acceptance `ppu/intr_2_0_timing` ROM: measures the timing
/// between STAT mode-2 and mode-0 interrupts on a mid-screen line.
#[test]
#[ignore]
fn mooneye_ppu_intr_2_0_timing() {
    run_mooneye_ppu_rom("intr_2_0_timing.gb");
}

/// Micro-step variant of the `intr_2_0_timing` PPU test.
#[test]
#[ignore]
fn mooneye_ppu_intr_2_0_timing_micro() {
    run_mooneye_ppu_rom_micro("intr_2_0_timing.gb");
}

#[test]
fn mooneye_timer_tim01() {
    run_mooneye_timer_rom("tim01.gb");
    run_mooneye_timer_rom_micro("tim01.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim10() {
    run_mooneye_timer_rom("tim10.gb");
    run_mooneye_timer_rom_micro("tim10.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim11() {
    run_mooneye_timer_rom("tim11.gb");
    run_mooneye_timer_rom_micro("tim11.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim00_div_trigger() {
    run_mooneye_timer_rom("tim00_div_trigger.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim01_div_trigger() {
    run_mooneye_timer_rom("tim01_div_trigger.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim10_div_trigger() {
    run_mooneye_timer_rom("tim10_div_trigger.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tim11_div_trigger() {
    run_mooneye_timer_rom("tim11_div_trigger.gb");
}

#[test]
#[ignore]
fn mooneye_timer_div_write() {
    run_mooneye_timer_rom("div_write.gb");
}

#[test]
#[ignore]
fn mooneye_timer_rapid_toggle() {
    run_mooneye_timer_rom("rapid_toggle.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tima_reload() {
    run_mooneye_timer_rom("tima_reload.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tima_write_reloading() {
    run_mooneye_timer_rom("tima_write_reloading.gb");
}

#[test]
#[ignore]
fn mooneye_timer_tma_write_reloading() {
    run_mooneye_timer_rom("tma_write_reloading.gb");
}

/// Mooneye acceptance test for interrupt/IE stack interaction.
///
/// This uses the `ie_push.gb` ROM from mooneye's
/// `acceptance/interrupts` suite and relies on the standard Fibonacci
/// register pattern to signal success.
#[test]
#[ignore]
fn mooneye_interrupt_ie_push() {
    run_mooneye_interrupts_rom("interrupts/ie_push.gb");
}

/// Micro-step variant of the `ie_push` interrupt acceptance test.
///
/// This drives the ROM using `step_mcycle` so that we can observe how
/// our per-M-cycle Timer/interrupt integration behaves compared to the
/// instruction-level path.
#[test]
#[ignore]
fn mooneye_interrupt_ie_push_micro() {
    run_mooneye_interrupts_rom_micro("interrupts/ie_push.gb");
}

/// Mooneye acceptance `ei_sequence` ROM: exercises EI delay semantics and
/// basic interrupt sequencing.
#[test]
#[ignore]
fn mooneye_interrupt_ei_sequence() {
    run_mooneye_interrupts_rom("ei_sequence.gb");
}

/// Micro-step variant of the `ei_sequence` interrupt test.
#[test]
#[ignore]
fn mooneye_interrupt_ei_sequence_micro() {
    run_mooneye_interrupts_rom_micro("ei_sequence.gb");
}

/// Mooneye acceptance `ei_timing` ROM: focuses on the exact timing of EI
/// enabling IME relative to subsequent instructions.
#[test]
#[ignore]
fn mooneye_interrupt_ei_timing() {
    run_mooneye_interrupts_rom("ei_timing.gb");
}

/// Micro-step variant of the `ei_timing` interrupt test.
#[test]
#[ignore]
fn mooneye_interrupt_ei_timing_micro() {
    run_mooneye_interrupts_rom_micro("ei_timing.gb");
}

/// Mooneye acceptance `intr_timing` ROM: probes general interrupt entry
/// timing relative to instruction boundaries.
#[test]
#[ignore]
fn mooneye_interrupt_intr_timing() {
    run_mooneye_interrupts_rom("intr_timing.gb");
}

/// Micro-step variant of the `intr_timing` interrupt test.
#[test]
#[ignore]
fn mooneye_interrupt_intr_timing_micro() {
    run_mooneye_interrupts_rom_micro("intr_timing.gb");
}

/// Mooneye acceptance `if_ie_registers` ROM: validates IF/IE observable
/// register behaviour.
#[test]
#[ignore]
fn mooneye_interrupt_if_ie_registers() {
    run_mooneye_interrupts_rom("if_ie_registers.gb");
}

/// Micro-step variant of the `if_ie_registers` interrupt test.
#[test]
#[ignore]
fn mooneye_interrupt_if_ie_registers_micro() {
    run_mooneye_interrupts_rom_micro("if_ie_registers.gb");
}
