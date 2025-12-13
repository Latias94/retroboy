mod bus;
mod alu;
mod cb;
mod exec;
mod helpers;
mod interrupts;
mod regs;
mod init;
mod step;

pub use bus::Bus;
pub use regs::{Flag, Registers};

impl Cpu {
    #[inline]
    pub fn get_flag(&self, flag: Flag) -> bool {
        let bit = flag as u8;
        (self.regs.f & (1 << bit)) != 0
    }

    #[inline]
    pub fn set_flag(&mut self, flag: Flag, value: bool) {
        let bit = flag as u8;
        if value {
            self.regs.f |= 1 << bit;
        } else {
            self.regs.f &= !(1 << bit);
        }
    }

    #[inline]
    pub fn clear_flags(&mut self) {
        self.regs.f = 0;
    }

    #[inline]
    pub(crate) fn is_locked(&self) -> bool {
        self.locked
    }

    #[inline]
    pub(crate) fn is_stopped(&self) -> bool {
        self.stopped
    }
}

use crate::cpu_micro::MicroInstrKind;

/// Game Boy CPU core skeleton.
///
/// The initial implementation only models the register set and control
/// flags (`ime`, `halted`, `stopped`). The decode/execute logic and timing
/// will be filled in incrementally, guided by test ROMs.
#[derive(Clone, Debug)]
pub struct Cpu {
    pub regs: Registers,
    pub ime: bool,
    pub halted: bool,
    /// STOP low‑power state. While `stopped` is true, the CPU ignores maskable
    /// interrupts and only resumes execution when a joypad input line goes
    /// low (modelled approximately via reads from P1/$FF00).
    stopped: bool,
    halt_bug: bool,
    ime_enable_pending: bool,
    ime_enable_delay: bool,
    /// When true, the CPU has executed an invalid opcode that hard‑locks
    /// the machine on real hardware. We approximate this as a permanent
    /// stop where `step()` returns 0 cycles until reset.
    locked: bool,
    /// Remaining cycles (in T‑cycles) for the current instruction or
    /// interrupt handler when running via the micro‑step API
    /// (`step_mcycle`). This is always a multiple of 4 while non‑zero.
    micro_cycles_remaining: u32,
    /// Whether the current micro‑stepped sequence was started by taking
    /// an interrupt (as opposed to executing a normal opcode). This is
    /// used to decide when to apply the delayed IME change from EI.
    micro_from_interrupt: bool,
    /// Latched PC value that will be pushed when servicing an interrupt
    /// via the micro‑step API.
    interrupt_pc: u16,
    /// Interrupt vector address that the CPU will jump to at the end of
    /// the micro‑stepped interrupt entry sequence.
    interrupt_vector: u16,
    /// Current M‑cycle index within the interrupt entry sequence when
    /// using the micro‑step API. Ranges from 0 to 4 while an interrupt
    /// is being serviced.
    interrupt_stage: u8,
    /// Pending interrupt index (0–4) and IF value latched when starting
    /// a micro‑stepped interrupt entry. This allows us to clear the IF
    /// bit at a precise point in the entry sequence rather than
    /// immediately.
    interrupt_index_latched: Option<u8>,
    interrupt_iflags_latched: u8,
    /// Kind of instruction currently being executed via the micro-step
    /// API. This is only used by `step_mcycle`; the instruction-level
    /// `step` entry point always goes through `exec_opcode`.
    pub(crate) micro_instr: MicroInstrKind,
    /// Per-instruction micro-op stage index used by `step_mcycle`.
    pub(crate) micro_stage: u8,
    /// Latched 16-bit immediate used by certain micro-coded instructions
    /// (e.g. CALL a16).
    pub(crate) micro_imm16: u16,
    /// Condition flag used by certain conditional micro-coded instructions
    /// (e.g. RET cc, JP cc,a16). When false, the instruction behaves as a
    /// no-op apart from its timing.
    pub(crate) micro_cond_taken: bool,
}

#[cfg(test)]
mod tests;
