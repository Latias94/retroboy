use crate::cpu_micro::MicroInstrKind;

use super::{Cpu, Registers};

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Cpu {
    pub fn new() -> Self {
        let mut cpu = Self {
            regs: Registers::default(),
            ime: false,
            halted: false,
            stopped: false,
            halt_bug: false,
            ime_enable_pending: false,
            ime_enable_delay: false,
            locked: false,
            micro_cycles_remaining: 0,
            micro_prefetched_opcode: None,
            micro_from_interrupt: false,
            interrupt_pc: 0,
            interrupt_vector: 0,
            interrupt_stage: 0,
            interrupt_index_latched: None,
            interrupt_iflags_latched: 0,
            micro_instr: MicroInstrKind::None,
            micro_stage: 0,
            micro_imm16: 0,
            micro_cond_taken: false,
        };
        cpu.apply_dmg_boot_state();
        cpu
    }

    /// Reset the CPU to its power-on state.
    ///
    /// We start with a neutral state and will later update this to match the
    /// actual DMG power-on/register values.
    pub fn reset(&mut self) {
        self.regs = Registers::default();
        self.ime = false;
        self.halted = false;
        self.stopped = false;
        self.ime_enable_pending = false;
        self.ime_enable_delay = false;
        self.locked = false;
        self.micro_cycles_remaining = 0;
        self.micro_prefetched_opcode = None;
        self.micro_from_interrupt = false;
        self.micro_instr = MicroInstrKind::None;
        self.micro_stage = 0;
        self.interrupt_pc = 0;
        self.interrupt_vector = 0;
        self.interrupt_stage = 0;
        self.interrupt_index_latched = None;
        self.interrupt_iflags_latched = 0;
        self.micro_instr = MicroInstrKind::None;
        self.micro_stage = 0;
        self.micro_imm16 = 0;
        self.micro_cond_taken = false;
        self.apply_dmg_boot_state();
    }

    /// Initialize registers to match the DMG boot ROM's state after it
    /// hands control to cartridge code.
    ///
    /// These values follow common emulator conventions and are based on
    /// hardware tests (e.g. as used by the `rboy` project and documented
    /// in Pan Docs).
    fn apply_dmg_boot_state(&mut self) {
        // Registers (DMG mode).
        self.regs.a = 0x01;
        self.regs.f = 0xB0; // Z, N, H, C = 1,1,1,0 (upper nibble 1011_0000)
        self.regs.b = 0x00;
        self.regs.c = 0x13;
        self.regs.d = 0x00;
        self.regs.e = 0xD8;
        self.regs.h = 0x01;
        self.regs.l = 0x4D;
        self.regs.sp = 0xFFFE;
        self.regs.pc = 0x0100;

        // Per Pan Docs, IME is clear when control is handed to the cartridge
        // at 0x0100. The game (or test ROM) is responsible for enabling
        // interrupts via EI/RETI as needed.
        self.ime = false;
    }

}
