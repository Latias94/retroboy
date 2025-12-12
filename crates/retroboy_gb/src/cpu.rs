/// Registers for the Game Boy CPU (LR35902).
///
/// The core is Z80-like with an 8-bit ALU and a 16-bit address space.
/// We start with a straightforward representation and refine as we go.
#[derive(Clone, Copy, Debug, Default)]
pub struct Registers {
    pub a: u8,
    pub f: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub sp: u16,
    pub pc: u16,
}

impl Registers {
    #[inline]
    pub fn af(&self) -> u16 {
        u16::from_be_bytes([self.a, self.f & 0xF0])
    }

    #[inline]
    pub fn set_af(&mut self, value: u16) {
        let [a, f] = value.to_be_bytes();
        self.a = a;
        // Lower 4 bits of F are always zero.
        self.f = f & 0xF0;
    }

    #[inline]
    pub fn bc(&self) -> u16 {
        u16::from_be_bytes([self.b, self.c])
    }

    #[inline]
    pub fn set_bc(&mut self, value: u16) {
        let [b, c] = value.to_be_bytes();
        self.b = b;
        self.c = c;
    }

    #[inline]
    pub fn de(&self) -> u16 {
        u16::from_be_bytes([self.d, self.e])
    }

    #[inline]
    pub fn set_de(&mut self, value: u16) {
        let [d, e] = value.to_be_bytes();
        self.d = d;
        self.e = e;
    }

    #[inline]
    pub fn hl(&self) -> u16 {
        u16::from_be_bytes([self.h, self.l])
    }

    #[inline]
    pub fn set_hl(&mut self, value: u16) {
        let [h, l] = value.to_be_bytes();
        self.h = h;
        self.l = l;
    }
}

/// Flag bits in the F register.
///
/// Layout (bit index in the byte, from MSB to LSB):
/// - bit 7: Z (zero)
/// - bit 6: N (subtract)
/// - bit 5: H (half carry)
/// - bit 4: C (carry)
/// - bits 0–3 are always zero.
#[derive(Clone, Copy, Debug)]
pub enum Flag {
    Z = 7,
    N = 6,
    H = 5,
    C = 4,
}

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
}

/// Abstraction over the Game Boy bus (memory and IO).
///
/// We keep this intentionally small at first; it may later grow to include
/// separate methods for VRAM, IO registers, and cartridge space.
pub trait Bus {
    fn read8(&mut self, addr: u16) -> u8;
    fn write8(&mut self, addr: u16, value: u8);
    /// Advance bus-side peripherals by a given number of CPU cycles.
    ///
    /// Default implementation does nothing; system buses can override this
    /// to drive timers, PPU, APU, etc.
    fn tick(&mut self, _cycles: u32) {}

    /// Advance non‑timer peripherals (PPU/APU/etc.) by a single CPU
    /// M‑cycle (4 T‑cycles).
    ///
    /// Micro‑step CPU paths (`step_mcycle`) use this to model generic
    /// bus activity while keeping timer advancement under explicit
    /// control via `timer_tick_mcycle` and the Timer IO helpers.
    fn tick_mcycle_generic(&mut self) {
        self.tick(4);
    }

    /// Advance the timer by one M‑cycle worth of time when no explicit
    /// Timer IO (DIV/TIMA/TMA/TAC) access took place in the current
    /// CPU cycle.
    ///
    /// Game Boy‑specific buses can override this to call into their
    /// Timer core; generic buses can leave it as a no‑op.
    fn timer_tick_mcycle(&mut self) {}

    /// Hook that marks the beginning of a single CPU instruction (or
    /// interrupt entry) from the bus's point of view.
    ///
    /// Instruction‑level stepping (`Cpu::step`) calls this once per
    /// instruction before any memory accesses. Buses that need to track
    /// per‑instruction state (e.g. timer IO events) can override this;
    /// the default implementation is a no‑op.
    fn begin_instruction(&mut self) {}

    /// Hook that finalises a single CPU instruction (or interrupt entry)
    /// from the bus's point of view.
    ///
    /// The `cycles` argument is the total number of CPU T‑cycles consumed
    /// by the instruction. The default implementation simply forwards to
    /// `tick(cycles)` so existing buses keep their current behaviour.
    fn end_instruction(&mut self, cycles: u32) {
        self.tick(cycles);
    }

    /// Single T‑cycle that performs a Timer register read at `addr`.
    ///
    /// The default implementation simply forwards to `read8` so that
    /// non‑Game Boy buses do not need to care about Timer semantics.
    /// Game Boy‑specific buses (such as `GameBoyBus`) can override this
    /// to model per‑cycle Timer behaviour for DIV/TIMA/TMA/TAC accesses.
    fn timer_cycle_read(&mut self, addr: u16) -> u8 {
        self.read8(addr)
    }

    /// Single T‑cycle that performs a Timer register write at `addr`.
    ///
    /// The default implementation forwards to `write8`. Game Boy‑specific
    /// buses can override this to call into their Timer read/write helpers
    /// instead, so that each Timer IO access corresponds to exactly one
    /// T‑cycle.
    fn timer_cycle_write(&mut self, addr: u16, value: u8) {
        self.write8(addr, value)
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

    /// Core 8-bit ADD/ADC operation on A.
    ///
    /// `use_carry` selects between ADD (false) and ADC (true).
    fn alu_add(&mut self, value: u8, use_carry: bool) {
        let a = self.regs.a;
        let carry_in = if use_carry && self.get_flag(Flag::C) {
            1u8
        } else {
            0
        };

        let half = (a & 0x0F) + (value & 0x0F) + carry_in;
        let full = (a as u16) + (value as u16) + (carry_in as u16);
        let result = full as u8;

        self.regs.a = result;

        // Flags: Z N H C
        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, (half & 0x10) != 0);
        self.set_flag(Flag::C, full > 0xFF);
    }

    /// Core 8-bit SUB/SBC operation on A.
    ///
    /// `use_carry` selects between SUB (false) and SBC (true).
    fn alu_sub(&mut self, value: u8, use_carry: bool) {
        let a = self.regs.a;
        let carry_in = if use_carry && self.get_flag(Flag::C) {
            1i16
        } else {
            0
        };

        let half = (a & 0x0F) as i16 - (value & 0x0F) as i16 - carry_in;
        let full = a as i16 - value as i16 - carry_in;
        let result = full as u8;

        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, half < 0);
        self.set_flag(Flag::C, full < 0);
    }

    #[inline]
    fn alu_and(&mut self, value: u8) {
        let result = self.regs.a & value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::H, true);
        // N and C are already cleared.
    }

    #[inline]
    fn alu_or(&mut self, value: u8) {
        let result = self.regs.a | value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
    }

    #[inline]
    fn alu_xor(&mut self, value: u8) {
        let result = self.regs.a ^ value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
    }

    /// Compare A with `value`, setting flags as if `A - value` was performed.
    /// A itself is not modified.
    #[inline]
    fn alu_cp(&mut self, value: u8) {
        let a = self.regs.a;
        let half = (a & 0x0F) as i16 - (value & 0x0F) as i16;
        let full = a as i16 - value as i16;
        let result = full as u8;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, half < 0);
        self.set_flag(Flag::C, full < 0);
    }

    /// Decimal adjust accumulator after BCD addition/subtraction.
    ///
    /// This follows the standard Game Boy DAA behaviour:
    /// - Uses C, H, N, and A to compute a correction value.
    /// - Updates A, Z, H, C; leaves N unchanged.
    fn alu_daa(&mut self) {
        let mut a = self.regs.a;
        let mut adjust: u8 = if self.get_flag(Flag::C) { 0x60 } else { 0x00 };
        if self.get_flag(Flag::H) {
            adjust |= 0x06;
        }

        if !self.get_flag(Flag::N) {
            // After an addition.
            if (a & 0x0F) > 0x09 {
                adjust |= 0x06;
            }
            if a > 0x99 {
                adjust |= 0x60;
            }
            a = a.wrapping_add(adjust);
        } else {
            // After a subtraction.
            a = a.wrapping_sub(adjust);
        }

        self.set_flag(Flag::C, adjust >= 0x60);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::Z, a == 0);
        self.regs.a = a;
    }

    /// 8-bit increment helper used by INC r and INC (HL).
    ///
    /// Updates Z, N, H while leaving C unchanged.
    #[inline]
    fn alu_inc8(&mut self, value: u8) -> u8 {
        let result = value.wrapping_add(1);
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, (value & 0x0F) + 1 > 0x0F);
        result
    }

    /// 8-bit decrement helper used by DEC r and DEC (HL).
    ///
    /// Updates Z, N, H while leaving C unchanged.
    #[inline]
    fn alu_dec8(&mut self, value: u8) -> u8 {
        let result = value.wrapping_sub(1);
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::N, true);
        self.set_flag(Flag::H, (value & 0x0F) == 0);
        result
    }

    /// 16-bit add helper for `ADD HL,rr`.
    ///
    /// Z is unaffected; N is cleared; H and C are updated based on the
    /// 16-bit addition.
    #[inline]
    fn alu_add16_hl(&mut self, value: u16) {
        let hl = self.regs.hl();
        let result = hl.wrapping_add(value);

        self.set_flag(Flag::N, false);
        self.set_flag(Flag::H, (hl & 0x0FFF) + (value & 0x0FFF) > 0x0FFF);
        self.set_flag(Flag::C, (hl as u32) + (value as u32) > 0xFFFF);

        self.regs.set_hl(result);
    }

    /// 16-bit add helper for instructions that add a signed 8-bit immediate
    /// to a 16-bit base (e.g. ADD SP, r8 and LD HL, SP+r8).
    ///
    /// Z is cleared; N is cleared; H and C are computed from the low byte.
    #[inline]
    fn alu_add16_signed(&mut self, base: u16, imm8: u8) -> u16 {
        let offset = imm8 as i8 as i16 as u16;
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::Z, false);
        self.set_flag(Flag::H, (base & 0x000F) + (offset & 0x000F) > 0x000F);
        self.set_flag(Flag::C, (base & 0x00FF) + (offset & 0x00FF) > 0x00FF);
        base.wrapping_add(offset)
    }

    /// Helper to read an 8-bit register or (HL) by index.
    ///
    /// The encoding matches the standard Game Boy register order used by
    /// opcode tables:
    /// 0=B, 1=C, 2=D, 3=E, 4=H, 5=L, 6=(HL), 7=A.
    #[inline]
    fn read_reg8<B: Bus>(&mut self, bus: &mut B, index: u8) -> u8 {
        match index {
            0 => self.regs.b,
            1 => self.regs.c,
            2 => self.regs.d,
            3 => self.regs.e,
            4 => self.regs.h,
            5 => self.regs.l,
            6 => bus.read8(self.regs.hl()),
            7 => self.regs.a,
            _ => 0,
        }
    }

    /// Helper to write an 8-bit register or (HL) by index.
    ///
    /// The encoding matches `read_reg8`.
    #[inline]
    fn write_reg8<B: Bus>(&mut self, bus: &mut B, index: u8, value: u8) {
        match index {
            0 => self.regs.b = value,
            1 => self.regs.c = value,
            2 => self.regs.d = value,
            3 => self.regs.e = value,
            4 => self.regs.h = value,
            5 => self.regs.l = value,
            6 => bus.write8(self.regs.hl(), value),
            7 => self.regs.a = value,
            _ => {}
        }
    }

    #[inline]
    fn fetch8<B: Bus>(&mut self, bus: &mut B) -> u8 {
        let value = bus.read8(self.regs.pc);
        if self.halt_bug {
            // HALT bug: the first opcode fetch after the bug does not
            // increment PC. We consume the bug here.
            self.halt_bug = false;
        } else {
            self.regs.pc = self.regs.pc.wrapping_add(1);
        }
        value
    }

    #[inline]
    fn fetch16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus) as u16;
        let hi = self.fetch8(bus) as u16;
        (hi << 8) | lo
    }

    #[inline]
    fn push_u16<B: Bus>(&mut self, bus: &mut B, value: u16) {
        let lo = value as u8;
        let hi = (value >> 8) as u8;
        // Stack grows downward. We want memory[SP] = low, memory[SP+1] = high.
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, hi);
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, lo);
    }

    #[inline]
    fn pop_u16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = bus.read8(self.regs.sp) as u16;
        let hi = bus.read8(self.regs.sp.wrapping_add(1)) as u16;
        self.regs.sp = self.regs.sp.wrapping_add(2);
        (hi << 8) | lo
    }

    /// Perform a single bus read cycle at the given address and advance the
    /// rest of the machine by one CPU T‑cycle.
    ///
    /// This helper is the building block for a future per‑cycle CPU/bus
    /// integration model. The current instruction‑level `step` API still
    /// uses `bus.read8` + `bus.tick(total_cycles)`; new micro‑step paths
    /// can gradually migrate to these helpers instead.
    #[inline]
    #[allow(dead_code)]
    fn read_cycle<B: Bus>(&mut self, bus: &mut B, addr: u16) -> u8 {
        let value = bus.read8(addr);
        bus.tick(1);
        value
    }

    /// Perform a single bus write cycle at the given address and advance the
    /// rest of the machine by one CPU T‑cycle.
    #[inline]
    #[allow(dead_code)]
    fn write_cycle<B: Bus>(&mut self, bus: &mut B, addr: u16, value: u8) {
        bus.write8(addr, value);
        bus.tick(1);
    }

    /// Idle for one CPU T‑cycle without performing a memory access.
    #[inline]
    #[allow(dead_code)]
    fn idle_cycle<B: Bus>(&mut self, bus: &mut B) {
        bus.tick(1);
    }

    /// Relative jump helper used by JR/JR cc.
    ///
    /// The displacement is a signed 8-bit offset relative to the address
    /// following the operand.
    fn jr<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        let offset = self.fetch8(bus) as i8;
        if cond {
            let pc = self.regs.pc as i16 + offset as i16;
            self.regs.pc = pc as u16;
            12
        } else {
            8
        }
    }

    /// Absolute jump helper used by JP cc,a16.
    fn jp_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        let addr = self.fetch16(bus);
        if cond {
            self.regs.pc = addr;
            16
        } else {
            12
        }
    }

    /// Conditional call helper used by CALL cc,a16.
    fn call_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        let addr = self.fetch16(bus);
        if cond {
            let ret = self.regs.pc;
            self.push_u16(bus, ret);
            self.regs.pc = addr;
            24
        } else {
            12
        }
    }

    /// Conditional return helper used by RET cc.
    fn ret_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        if cond {
            let addr = self.pop_u16(bus);
            self.regs.pc = addr;
            20
        } else {
            8
        }
    }

	    /// Poll the interrupt controller and determine whether a maskable
	    /// interrupt should be serviced in the current state.
    ///
    /// On success, returns the interrupt index (0–4, corresponding to
    /// VBlank/STAT/Timer/Serial/Joypad) together with the current IF
    /// value. This helper also implements the "HALT bug" wake‑up case:
    /// if the CPU is halted with IME=0 and an interrupt becomes pending,
    /// HALT is cleared but the interrupt is not serviced yet.
 	    fn poll_pending_interrupt<B: Bus>(&mut self, bus: &mut B) -> Option<(u8, u8)> {
 	        let ie = bus.read8(0xFFFF);
 	        let iflags = bus.read8(0xFF0F);
 	        let pending = ie & iflags & 0x1F;
        if pending == 0 {
            return None;
        }

        // If the CPU is halted and an interrupt becomes pending while IME is
        // disabled, the CPU wakes up without servicing the interrupt. This is
        // part of the so‑called "HALT bug" behaviour.
        if self.halted && !self.ime {
            self.halted = false;
            return None;
        }

        if !self.ime {
            return None;
        }

        // Find lowest-numbered pending interrupt (VBlank > LCD STAT > Timer > Serial > Joypad).
        let index = pending.trailing_zeros();
        if index >= 5 {
            return None;
        }
 	        Some((index as u8, iflags))
 	    }

	    /// Select the highest‑priority pending interrupt *after* the high byte
	    /// of PC has been pushed, but *before* the low byte is written.
	    ///
	    /// This mirrors mooneye‑gb's `write_cycle_intr` semantics used by the
	    /// `ie_push` acceptance test:
	    ///
	    /// - `IE` is evaluated after any effects of the high‑byte push
	    ///   (e.g. when `SP` was 0 and the high byte was written to `$FFFF`).
	    /// - `IF` is sampled before the low‑byte push.
	    /// - The interrupt line is chosen from `IE & IF` at this point;
	    ///   writes to `IE` performed by the low‑byte push are "too late" to
	    ///   affect the current dispatch.
	    ///
	    /// Returns `Some((index, new_if))` when an interrupt should be
	    /// dispatched (index 0–4 and IF with that bit cleared), or `None`
	    /// when the dispatch should be cancelled and PC should fall back to
	    /// `0x0000`.
	    fn select_interrupt_after_high_push<B: Bus>(&mut self, bus: &mut B) -> Option<(u8, u8)> {
	        let ie_now = bus.read8(0xFFFF);
	        let if_now = bus.read8(0xFF0F);
	        let pending = ie_now & if_now & 0x1F;
	        if pending == 0 {
	            return None;
	        }
	        let index = pending.trailing_zeros();
	        if index >= 5 {
	            return None;
	        }
	        let new_if = if_now & !(1 << index);
	        Some((index as u8, new_if))
	    }

 	    /// Handle maskable interrupts if IME is set and a pending interrupt exists.
    ///
 	    /// Returns `Some(cycles)` if an interrupt was taken, or `None` otherwise.
 	    fn handle_interrupts<B: Bus>(&mut self, bus: &mut B) -> Option<u32> {
	        // First check whether a maskable interrupt should be serviced at
	        // all in the current state (including HALT‑bug behaviour). We
	        // intentionally ignore the specific index here and instead select
	        // the final interrupt after the high‑byte push so that writes to
	        // IE during that push (as in mooneye's `ie_push` test) can cancel
	        // or retarget the dispatch.
	        if self.poll_pending_interrupt(bus).is_none() {
	            return None;
	        }

        // Standard interrupt entry sequence (instruction‑level model).
        //
        // We broadly follow the same ordering as the micro‑step path and
        // mooneye‑gb's CPU: IME is cleared immediately, then the current PC
        // is pushed to the stack, and only after that is the corresponding
        // IF bit cleared and the PC redirected to the interrupt vector. This
        // makes the timing of IF/IE observable state closer to real hardware
        // while still charging the canonical cost of 20 T‑cycles as a single
        // logical "instruction".
	        self.ime = false;
	        self.halted = false;

	        let pc = self.regs.pc;
	        let hi = (pc >> 8) as u8;
	        let lo = pc as u8;

	        // Push high byte of PC. This may write to `$FFFF` (IE) when SP was
	        // 0, so the IE value used for interrupt selection must be sampled
	        // *after* this write.
	        self.regs.sp = self.regs.sp.wrapping_sub(1);
	        bus.write8(self.regs.sp, hi);

	        // Decide which interrupt is still pending after the high‑byte
	        // push. If none remain, the dispatch is cancelled and PC falls
	        // back to 0x0000; otherwise we clear that IF bit and jump to the
	        // corresponding vector.
	        let selection = self.select_interrupt_after_high_push(bus);

	        // Push low byte of PC. Writes performed by this store (e.g. to
	        // `$FFFF` when SP was 1) must not affect the interrupt selection
	        // for the current dispatch.
	        self.regs.sp = self.regs.sp.wrapping_sub(1);
	        bus.write8(self.regs.sp, lo);

	        match selection {
	            Some((index, new_if)) => {
	                bus.write8(0xFF0F, new_if);
	                self.regs.pc = 0x0040 + (index as u16) * 8;
	            }
	            None => {
	                // No interrupt line remained pending after the high‑byte
	                // push: treat this as a cancelled dispatch and jump to
	                // address 0x0000 (see mooneye `ie_push` Round 1).
	                self.regs.pc = 0x0000;
	            }
	        }

        Some(20)
    }

	    /// Begin servicing a maskable interrupt using the micro‑step API.
    ///
    /// This mirrors `handle_interrupts` in terms of which interrupt is
 	    /// chosen and when IF/IME/`halted` are updated, but it defers the
 	    /// actual stack push and PC jump to `step_interrupt_mcycle` so that
 	    /// the entry sequence can be spread across multiple M‑cycles.
 	    fn begin_interrupt_micro<B: Bus>(&mut self, bus: &mut B) -> bool {
	        // As in the instruction‑level path, first check whether any
	        // maskable interrupt should be serviced (including HALT handling).
	        if self.poll_pending_interrupt(bus).is_none() {
	            return false;
	        }

        // Latch state needed for the interrupt entry micro‑sequence. We
        // defer clearing the IF bit until a later M‑cycle to better match
        // hardware behaviour.
	        self.ime = false;
	        self.halted = false;
	        self.interrupt_pc = self.regs.pc;
	        // The final interrupt vector (or cancellation to 0x0000) will be
	        // decided after the high‑byte push in `step_interrupt_mcycle`.
	        self.interrupt_vector = 0x0000;
 	        self.interrupt_stage = 0;
	        self.interrupt_index_latched = None;
	        self.interrupt_iflags_latched = 0;

        // Total cost is 20 T‑cycles (5 M‑cycles).
        self.micro_cycles_remaining = 20;
        self.micro_from_interrupt = true;
        true
    }

    /// Execute a single 4‑T‑cycle slice of the interrupt entry sequence.
    ///
    /// This helper is used by the micro‑step API (`step_mcycle`) once
    /// `begin_interrupt_micro` has latched the interrupt metadata. It
    /// spreads the PC push and vector jump across five M‑cycles while
    /// still charging the canonical total of 20 T‑cycles.
    fn step_interrupt_mcycle<B: Bus>(&mut self, bus: &mut B) {
        match self.interrupt_stage {
            0 => {
                // Initial idle M‑cycle after accepting the interrupt.
            }
            1 => {
                // Second idle M‑cycle (approximates internal wait states).
            }
            2 => {
                // Push high byte of PC.
                let pc = self.interrupt_pc;
                let hi = (pc >> 8) as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, hi);
            }
            3 => {
                // Push low byte of PC and, based on the IE/IF state *after* the
                // high‑byte push but *before* this write, decide which interrupt
                // (if any) should actually be dispatched. Writes to IE performed
                // by this low‑byte push are therefore too late to affect the
                // current dispatch.
                let pc = self.interrupt_pc;
                let lo = pc as u8;

                // Sample IE/IF after the high‑byte push (stage 2).
                let selection = self.select_interrupt_after_high_push(bus);

                // Perform the low‑byte push. This may write to `$FFFF` (IE) when
                // SP was 1, but the selection above must not observe those changes.
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, lo);

                match selection {
                    Some((index, new_if)) => {
                        bus.write8(0xFF0F, new_if);
                        self.interrupt_vector = 0x0040 + (index as u16) * 8;
                    }
                    None => {
                        // No interrupt line remained pending after the high‑byte
                        // push: treat this as a cancelled dispatch and jump to
                        // address 0x0000 (see mooneye `ie_push` Round 1).
                        self.interrupt_vector = 0x0000;
                    }
                }

                // Any previously latched index/IF are no longer needed.
                self.interrupt_index_latched = None;
                self.interrupt_iflags_latched = 0;
            }
            4 => {
                // Jump to interrupt vector.
                self.regs.pc = self.interrupt_vector;
            }
            _ => {}
        }

        // One M‑cycle worth of time: generic peripherals plus a single
        // Timer tick (interrupt entry does not perform explicit Timer IO).
        bus.tick_mcycle_generic();
        bus.timer_tick_mcycle();
        // Saturating add to avoid wrapping if something goes wrong.
        self.interrupt_stage = self.interrupt_stage.saturating_add(1);
    }

    /// Apply delayed IME change requested by EI.
    #[inline]
    fn apply_ime_delay(&mut self) {
        if self.ime_enable_delay {
            // Second step after EI: actually enable IME.
            self.ime = true;
            self.ime_enable_delay = false;
        } else if self.ime_enable_pending {
            // First step after EI: arm the delayed enable.
            self.ime_enable_pending = false;
            self.ime_enable_delay = true;
        }
    }

    /// Handle CB-prefixed instructions (bit operations, shifts, and rotates).
    fn step_cb<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let cb = self.fetch8(bus);
        let x = cb >> 6;
        let y = (cb >> 3) & 0x07;
        let z = cb & 0x07;

        match x {
            0 => {
                // Rotates and shifts.
                let mut value = self.read_reg8(bus, z);
                // Base cycles: 8 for register, 16 for (HL).
                let cycles = if z == 6 { 16 } else { 8 };

                match y {
                    // RLC r
                    0 => {
                        let carry = (value & 0x80) != 0;
                        value = value.rotate_left(1);
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry);
                    }
                    // RRC r
                    1 => {
                        let carry = (value & 0x01) != 0;
                        value = value.rotate_right(1);
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry);
                    }
                    // RL r
                    2 => {
                        let carry_out = (value & 0x80) != 0;
                        let carry_in = if self.get_flag(Flag::C) { 1 } else { 0 };
                        value = (value << 1) | carry_in;
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry_out);
                    }
                    // RR r
                    3 => {
                        let carry_out = (value & 0x01) != 0;
                        let carry_in = if self.get_flag(Flag::C) { 0x80 } else { 0 };
                        value = (value >> 1) | carry_in;
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry_out);
                    }
                    // SLA r
                    4 => {
                        let carry = (value & 0x80) != 0;
                        value <<= 1;
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry);
                    }
                    // SRA r
                    5 => {
                        let carry = (value & 0x01) != 0;
                        let msb = value & 0x80;
                        value = (value >> 1) | msb;
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry);
                    }
                    // SWAP r
                    6 => {
                        value = (value << 4) | (value >> 4);
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                    }
                    // SRL r
                    7 => {
                        let carry = (value & 0x01) != 0;
                        value >>= 1;
                        self.clear_flags();
                        self.set_flag(Flag::Z, value == 0);
                        self.set_flag(Flag::C, carry);
                    }
                    _ => {}
                }

                self.write_reg8(bus, z, value);
                cycles
            }
            1 => {
                // BIT b, r
                let bit = y;
                let value = self.read_reg8(bus, z);
                let bit_set = (value & (1 << bit)) != 0;
                // Preserve C, set H=1, N=0.
                let carry = self.get_flag(Flag::C);
                self.clear_flags();
                self.set_flag(Flag::Z, !bit_set);
                self.set_flag(Flag::H, true);
                self.set_flag(Flag::C, carry);

                if z == 6 {
                    12
                } else {
                    8
                }
            }
            2 => {
                // RES b, r
                let bit = y;
                let mut value = self.read_reg8(bus, z);
                value &= !(1 << bit);
                self.write_reg8(bus, z, value);
                if z == 6 {
                    16
                } else {
                    8
                }
            }
            3 => {
                // SET b, r
                let bit = y;
                let mut value = self.read_reg8(bus, z);
                value |= 1 << bit;
                self.write_reg8(bus, z, value);
                if z == 6 {
                    16
                } else {
                    8
                }
            }
            _ => 4,
        }
    }

    /// Decode and execute a single opcode and return the number of cycles.
    ///
    /// This method contains the full instruction table and is factored out of
    /// `step` so that we can later introduce a more fine‑grained, cycle‑level
    /// stepping API while reusing the same semantics.
    fn exec_opcode<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        if opcode == 0xCB {
            self.step_cb(bus)
        } else {
            match opcode {
                // 0x00: NOP
                0x00 => 4,

                // 16-bit immediate loads.
                0x01 => {
                    // LD BC, d16
                    let value = self.fetch16(bus);
                    self.regs.set_bc(value);
                    12
                }
                0x11 => {
                    // LD DE, d16
                    let value = self.fetch16(bus);
                    self.regs.set_de(value);
                    12
                }
                0x21 => {
                    // LD HL, d16
                    let value = self.fetch16(bus);
                    self.regs.set_hl(value);
                    12
                }
                0x31 => {
                    // LD SP, d16
                    let value = self.fetch16(bus);
                    self.regs.sp = value;
                    12
                }

                // Rotate A instructions (unprefixed).
                //
                // These are similar to the CB-prefixed rotates but always operate
                // on A and have slightly different flag semantics (Z is always 0).
                0x07 => {
                    // RLCA: rotate A left. Bit 7 to Carry and bit 0.
                    let a = self.regs.a;
                    let result = a.rotate_left(1);
                    self.regs.a = result;
                    self.set_flag(Flag::C, (a & 0x80) != 0);
                    self.set_flag(Flag::Z, false);
                    self.set_flag(Flag::N, false);
                    self.set_flag(Flag::H, false);
                    4
                }
                0x0F => {
                    // RRCA: rotate A right. Bit 0 to Carry and bit 7.
                    let a = self.regs.a;
                    let result = a.rotate_right(1);
                    self.regs.a = result;
                    self.set_flag(Flag::C, (a & 0x01) != 0);
                    self.set_flag(Flag::Z, false);
                    self.set_flag(Flag::N, false);
                    self.set_flag(Flag::H, false);
                    4
                }
                0x17 => {
                    // RLA: rotate A left through Carry.
                    let a = self.regs.a;
                    let carry_in = if self.get_flag(Flag::C) { 1 } else { 0 };
                    let carry_out = (a & 0x80) != 0;
                    let result = (a << 1) | carry_in;
                    self.regs.a = result;
                    self.set_flag(Flag::C, carry_out);
                    self.set_flag(Flag::Z, false);
                    self.set_flag(Flag::N, false);
                    self.set_flag(Flag::H, false);
                    4
                }
                0x1F => {
                    // RRA: rotate A right through Carry.
                    let a = self.regs.a;
                    let carry_in = if self.get_flag(Flag::C) { 0x80 } else { 0 };
                    let carry_out = (a & 0x01) != 0;
                    let result = (a >> 1) | carry_in;
                    self.regs.a = result;
                    self.set_flag(Flag::C, carry_out);
                    self.set_flag(Flag::Z, false);
                    self.set_flag(Flag::N, false);
                    self.set_flag(Flag::H, false);
                    4
                }

                // 16-bit INC rr
                0x03 => {
                    // INC BC
                    let value = self.regs.bc().wrapping_add(1);
                    self.regs.set_bc(value);
                    8
                }
                0x13 => {
                    // INC DE
                    let value = self.regs.de().wrapping_add(1);
                    self.regs.set_de(value);
                    8
                }
                0x23 => {
                    // INC HL
                    let value = self.regs.hl().wrapping_add(1);
                    self.regs.set_hl(value);
                    8
                }
                0x33 => {
                    // INC SP
                    self.regs.sp = self.regs.sp.wrapping_add(1);
                    8
                }

                // 16-bit DEC rr
                0x0B => {
                    // DEC BC
                    let value = self.regs.bc().wrapping_sub(1);
                    self.regs.set_bc(value);
                    8
                }
                0x1B => {
                    // DEC DE
                    let value = self.regs.de().wrapping_sub(1);
                    self.regs.set_de(value);
                    8
                }
                0x2B => {
                    // DEC HL
                    let value = self.regs.hl().wrapping_sub(1);
                    self.regs.set_hl(value);
                    8
                }
                0x3B => {
                    // DEC SP
                    self.regs.sp = self.regs.sp.wrapping_sub(1);
                    8
                }

                // LD r, d8
                0x06 => {
                    // LD B, d8
                    self.regs.b = self.fetch8(bus);
                    8
                }
                0x0E => {
                    // LD C, d8
                    self.regs.c = self.fetch8(bus);
                    8
                }
                0x16 => {
                    // LD D, d8
                    self.regs.d = self.fetch8(bus);
                    8
                }
                0x1E => {
                    // LD E, d8
                    self.regs.e = self.fetch8(bus);
                    8
                }
                0x26 => {
                    // LD H, d8
                    self.regs.h = self.fetch8(bus);
                    8
                }
                0x2E => {
                    // LD L, d8
                    self.regs.l = self.fetch8(bus);
                    8
                }
                0x3E => {
                    // LD A, d8
                    self.regs.a = self.fetch8(bus);
                    8
                }

                // 8-bit register/memory transfers: LD r1, r2
                //
                // This covers opcodes 0x40–0x7F (except 0x76, which is HALT).
                0x40..=0x7F => {
                    if opcode == 0x76 {
                        // HALT
                        if !self.ime {
                            // Possible HALT bug: if interrupts are pending while
                            // IME is clear, the CPU does not actually halt, and
                            // instead the next opcode fetch does not increment PC.
                            let ie = bus.read8(0xFFFF);
                            let iflags = bus.read8(0xFF0F);
                            let pending = ie & iflags & 0x1F;
                            if pending != 0 {
                                self.halt_bug = true;
                                // Do not enter HALT state in this case.
                                return 4;
                            }
                        }
                        self.halted = true;
                        4
                    } else {
                        let dst = (opcode >> 3) & 0x07;
                        let src = opcode & 0x07;
                        let value = self.read_reg8(bus, src);
                        self.write_reg8(bus, dst, value);
                        if dst == 6 || src == 6 {
                            8
                        } else {
                            4
                        }
                    }
                }

                // LD (BC), A
                0x02 => {
                    let addr = self.regs.bc();
                    bus.write8(addr, self.regs.a);
                    8
                }
                // LD (DE), A
                0x12 => {
                    let addr = self.regs.de();
                    bus.write8(addr, self.regs.a);
                    8
                }
                // LD (HL+), A
                0x22 => {
                    let addr = self.regs.hl();
                    bus.write8(addr, self.regs.a);
                    let new_hl = addr.wrapping_add(1);
                    self.regs.set_hl(new_hl);
                    8
                }
                // LD (HL-), A
                0x32 => {
                    let addr = self.regs.hl();
                    bus.write8(addr, self.regs.a);
                    let new_hl = addr.wrapping_sub(1);
                    self.regs.set_hl(new_hl);
                    8
                }

                // LD A, (BC)
                0x0A => {
                    let addr = self.regs.bc();
                    self.regs.a = bus.read8(addr);
                    8
                }
                // LD A, (DE)
                0x1A => {
                    let addr = self.regs.de();
                    self.regs.a = bus.read8(addr);
                    8
                }
                // LD A, (HL+)
                0x2A => {
                    let addr = self.regs.hl();
                    self.regs.a = bus.read8(addr);
                    let new_hl = addr.wrapping_add(1);
                    self.regs.set_hl(new_hl);
                    8
                }
                // LD A, (HL-)
                0x3A => {
                    let addr = self.regs.hl();
                    self.regs.a = bus.read8(addr);
                    let new_hl = addr.wrapping_sub(1);
                    self.regs.set_hl(new_hl);
                    8
                }

                // LD (HL), d8
                0x36 => {
                    let value = self.fetch8(bus);
                    let addr = self.regs.hl();
                    bus.write8(addr, value);
                    12
                }

                // LD (a16), SP
                0x08 => {
                    let addr = self.fetch16(bus);
                    let sp = self.regs.sp;
                    let lo = sp as u8;
                    let hi = (sp >> 8) as u8;
                    bus.write8(addr, lo);
                    bus.write8(addr.wrapping_add(1), hi);
                    20
                }

                // STOP
                0x10 => {
                    // STOP is officially a 2‑byte instruction; the second byte is
                    // often 0 and ignored. We always fetch and discard the padding
                    // byte so that PC matches hardware.
                    let _padding = self.fetch8(bus);
                    // Enter STOP low‑power mode. In this state the CPU ignores
                    // maskable interrupts and remains idle until a joypad input
                    // line goes low (approximated in `step` by polling P1/$FF00).
                    self.stopped = true;
                    self.halted = false;
                    4
                }

                // LDH (a8), A  — write A to 0xFF00 + imm8
                0xE0 => {
                    let offset = self.fetch8(bus) as u16;
                    let addr = 0xFF00u16.wrapping_add(offset);
                    bus.write8(addr, self.regs.a);
                    12
                }

                // LDH A, (a8)  — read from 0xFF00 + imm8 into A
                0xF0 => {
                    let offset = self.fetch8(bus) as u16;
                    let addr = 0xFF00u16.wrapping_add(offset);
                    self.regs.a = bus.read8(addr);
                    12
                }

                // LDH (C), A  — write A to 0xFF00 + C
                0xE2 => {
                    let addr = 0xFF00u16.wrapping_add(self.regs.c as u16);
                    bus.write8(addr, self.regs.a);
                    8
                }

                // LDH A, (C)  — read from 0xFF00 + C into A
                0xF2 => {
                    let addr = 0xFF00u16.wrapping_add(self.regs.c as u16);
                    self.regs.a = bus.read8(addr);
                    8
                }

                // LD (a16), A
                0xEA => {
                    let addr = self.fetch16(bus);
                    bus.write8(addr, self.regs.a);
                    16
                }

                // LD A, (a16)
                0xFA => {
                    let addr = self.fetch16(bus);
                    self.regs.a = bus.read8(addr);
                    16
                }

                // ADD SP, r8
                0xE8 => {
                    let imm = self.fetch8(bus);
                    let result = self.alu_add16_signed(self.regs.sp, imm);
                    self.regs.sp = result;
                    16
                }

                // LD HL, SP+r8
                0xF8 => {
                    let imm = self.fetch8(bus);
                    let base = self.regs.sp;
                    let result = self.alu_add16_signed(base, imm);
                    self.regs.set_hl(result);
                    12
                }

                // LD SP, HL
                0xF9 => {
                    let hl = self.regs.hl();
                    self.regs.sp = hl;
                    8
                }

                // JR r8 (relative)
                0x18 => self.jr(bus, true),

                // JR cc, r8
                0x20 => {
                    // JR NZ, r8
                    let cond = !self.get_flag(Flag::Z);
                    self.jr(bus, cond)
                }
                0x28 => {
                    // JR Z, r8
                    let cond = self.get_flag(Flag::Z);
                    self.jr(bus, cond)
                }
                0x30 => {
                    // JR NC, r8
                    let cond = !self.get_flag(Flag::C);
                    self.jr(bus, cond)
                }
                0x38 => {
                    // JR C, r8
                    let cond = self.get_flag(Flag::C);
                    self.jr(bus, cond)
                }

                // JP cc, a16
                0xC2 => {
                    // JP NZ, a16
                    let cond = !self.get_flag(Flag::Z);
                    self.jp_cond(bus, cond)
                }
                0xCA => {
                    // JP Z, a16
                    let cond = self.get_flag(Flag::Z);
                    self.jp_cond(bus, cond)
                }
                0xD2 => {
                    // JP NC, a16
                    let cond = !self.get_flag(Flag::C);
                    self.jp_cond(bus, cond)
                }
                0xDA => {
                    // JP C, a16
                    let cond = self.get_flag(Flag::C);
                    self.jp_cond(bus, cond)
                }

                // ADD HL, rr (16-bit)
                0x09 => {
                    // ADD HL, BC
                    let value = self.regs.bc();
                    self.alu_add16_hl(value);
                    8
                }
                0x19 => {
                    // ADD HL, DE
                    let value = self.regs.de();
                    self.alu_add16_hl(value);
                    8
                }
                0x29 => {
                    // ADD HL, HL
                    let value = self.regs.hl();
                    self.alu_add16_hl(value);
                    8
                }
                0x39 => {
                    // ADD HL, SP
                    let value = self.regs.sp;
                    self.alu_add16_hl(value);
                    8
                }

                // DAA
                0x27 => {
                    self.alu_daa();
                    4
                }

                // CPL
                0x2F => {
                    self.regs.a = !self.regs.a;
                    self.set_flag(Flag::H, true);
                    self.set_flag(Flag::N, true);
                    4
                }

                // SCF
                0x37 => {
                    self.set_flag(Flag::C, true);
                    self.set_flag(Flag::H, false);
                    self.set_flag(Flag::N, false);
                    4
                }

                // CCF
                0x3F => {
                    let carry = self.get_flag(Flag::C);
                    self.set_flag(Flag::C, !carry);
                    self.set_flag(Flag::H, false);
                    self.set_flag(Flag::N, false);
                    4
                }

                // ADD A, r
                0x87 => {
                    // ADD A, A
                    let value = self.regs.a;
                    self.alu_add(value, false);
                    4
                }
                0x80 => {
                    // ADD A, B
                    let value = self.regs.b;
                    self.alu_add(value, false);
                    4
                }
                0x81 => {
                    // ADD A, C
                    let value = self.regs.c;
                    self.alu_add(value, false);
                    4
                }
                0x82 => {
                    // ADD A, D
                    let value = self.regs.d;
                    self.alu_add(value, false);
                    4
                }
                0x83 => {
                    // ADD A, E
                    let value = self.regs.e;
                    self.alu_add(value, false);
                    4
                }
                0x84 => {
                    // ADD A, H
                    let value = self.regs.h;
                    self.alu_add(value, false);
                    4
                }
                0x85 => {
                    // ADD A, L
                    let value = self.regs.l;
                    self.alu_add(value, false);
                    4
                }
                0x86 => {
                    // ADD A, (HL)
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_add(value, false);
                    8
                }

                // ADD A, d8
                0xC6 => {
                    let value = self.fetch8(bus);
                    self.alu_add(value, false);
                    8
                }

                // ADC A, r
                0x8F => {
                    let value = self.regs.a;
                    self.alu_add(value, true);
                    4
                }
                0x88 => {
                    let value = self.regs.b;
                    self.alu_add(value, true);
                    4
                }
                0x89 => {
                    let value = self.regs.c;
                    self.alu_add(value, true);
                    4
                }
                0x8A => {
                    let value = self.regs.d;
                    self.alu_add(value, true);
                    4
                }
                0x8B => {
                    let value = self.regs.e;
                    self.alu_add(value, true);
                    4
                }
                0x8C => {
                    let value = self.regs.h;
                    self.alu_add(value, true);
                    4
                }
                0x8D => {
                    let value = self.regs.l;
                    self.alu_add(value, true);
                    4
                }
                0x8E => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_add(value, true);
                    8
                }
                0xCE => {
                    let value = self.fetch8(bus);
                    self.alu_add(value, true);
                    8
                }

                // SUB A, r
                0x97 => {
                    let value = self.regs.a;
                    self.alu_sub(value, false);
                    4
                }
                0x90 => {
                    let value = self.regs.b;
                    self.alu_sub(value, false);
                    4
                }
                0x91 => {
                    let value = self.regs.c;
                    self.alu_sub(value, false);
                    4
                }
                0x92 => {
                    let value = self.regs.d;
                    self.alu_sub(value, false);
                    4
                }
                0x93 => {
                    let value = self.regs.e;
                    self.alu_sub(value, false);
                    4
                }
                0x94 => {
                    let value = self.regs.h;
                    self.alu_sub(value, false);
                    4
                }
                0x95 => {
                    let value = self.regs.l;
                    self.alu_sub(value, false);
                    4
                }
                0x96 => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_sub(value, false);
                    8
                }
                0xD6 => {
                    let value = self.fetch8(bus);
                    self.alu_sub(value, false);
                    8
                }

                // SBC A, r
                0x9F => {
                    let value = self.regs.a;
                    self.alu_sub(value, true);
                    4
                }
                0x98 => {
                    let value = self.regs.b;
                    self.alu_sub(value, true);
                    4
                }
                0x99 => {
                    let value = self.regs.c;
                    self.alu_sub(value, true);
                    4
                }
                0x9A => {
                    let value = self.regs.d;
                    self.alu_sub(value, true);
                    4
                }
                0x9B => {
                    let value = self.regs.e;
                    self.alu_sub(value, true);
                    4
                }
                0x9C => {
                    let value = self.regs.h;
                    self.alu_sub(value, true);
                    4
                }
                0x9D => {
                    let value = self.regs.l;
                    self.alu_sub(value, true);
                    4
                }
                0x9E => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_sub(value, true);
                    8
                }
                0xDE => {
                    let value = self.fetch8(bus);
                    self.alu_sub(value, true);
                    8
                }

                // AND A, r
                0xA7 => {
                    let value = self.regs.a;
                    self.alu_and(value);
                    4
                }
                0xA0 => {
                    let value = self.regs.b;
                    self.alu_and(value);
                    4
                }
                0xA1 => {
                    let value = self.regs.c;
                    self.alu_and(value);
                    4
                }
                0xA2 => {
                    let value = self.regs.d;
                    self.alu_and(value);
                    4
                }
                0xA3 => {
                    let value = self.regs.e;
                    self.alu_and(value);
                    4
                }
                0xA4 => {
                    let value = self.regs.h;
                    self.alu_and(value);
                    4
                }
                0xA5 => {
                    let value = self.regs.l;
                    self.alu_and(value);
                    4
                }
                0xA6 => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_and(value);
                    8
                }
                0xE6 => {
                    let value = self.fetch8(bus);
                    self.alu_and(value);
                    8
                }

                // OR A, r
                0xB7 => {
                    let value = self.regs.a;
                    self.alu_or(value);
                    4
                }
                0xB0 => {
                    let value = self.regs.b;
                    self.alu_or(value);
                    4
                }
                0xB1 => {
                    let value = self.regs.c;
                    self.alu_or(value);
                    4
                }
                0xB2 => {
                    let value = self.regs.d;
                    self.alu_or(value);
                    4
                }
                0xB3 => {
                    let value = self.regs.e;
                    self.alu_or(value);
                    4
                }
                0xB4 => {
                    let value = self.regs.h;
                    self.alu_or(value);
                    4
                }
                0xB5 => {
                    let value = self.regs.l;
                    self.alu_or(value);
                    4
                }
                0xB6 => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_or(value);
                    8
                }
                0xF6 => {
                    let value = self.fetch8(bus);
                    self.alu_or(value);
                    8
                }

                // XOR A, r
                0xAF => {
                    let value = self.regs.a;
                    self.alu_xor(value);
                    4
                }
                0xA8 => {
                    let value = self.regs.b;
                    self.alu_xor(value);
                    4
                }
                0xA9 => {
                    let value = self.regs.c;
                    self.alu_xor(value);
                    4
                }
                0xAA => {
                    let value = self.regs.d;
                    self.alu_xor(value);
                    4
                }
                0xAB => {
                    let value = self.regs.e;
                    self.alu_xor(value);
                    4
                }
                0xAC => {
                    let value = self.regs.h;
                    self.alu_xor(value);
                    4
                }
                0xAD => {
                    let value = self.regs.l;
                    self.alu_xor(value);
                    4
                }
                0xAE => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_xor(value);
                    8
                }
                0xEE => {
                    let value = self.fetch8(bus);
                    self.alu_xor(value);
                    8
                }

                // CP A, r
                0xBF => {
                    let value = self.regs.a;
                    self.alu_cp(value);
                    4
                }
                0xB8 => {
                    let value = self.regs.b;
                    self.alu_cp(value);
                    4
                }
                0xB9 => {
                    let value = self.regs.c;
                    self.alu_cp(value);
                    4
                }
                0xBA => {
                    let value = self.regs.d;
                    self.alu_cp(value);
                    4
                }
                0xBB => {
                    let value = self.regs.e;
                    self.alu_cp(value);
                    4
                }
                0xBC => {
                    let value = self.regs.h;
                    self.alu_cp(value);
                    4
                }
                0xBD => {
                    let value = self.regs.l;
                    self.alu_cp(value);
                    4
                }
                0xBE => {
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    self.alu_cp(value);
                    8
                }
                0xFE => {
                    let value = self.fetch8(bus);
                    self.alu_cp(value);
                    8
                }

                // INC r
                0x04 => {
                    // INC B
                    let value = self.regs.b;
                    let result = self.alu_inc8(value);
                    self.regs.b = result;
                    4
                }
                0x0C => {
                    // INC C
                    let value = self.regs.c;
                    let result = self.alu_inc8(value);
                    self.regs.c = result;
                    4
                }
                0x14 => {
                    // INC D
                    let value = self.regs.d;
                    let result = self.alu_inc8(value);
                    self.regs.d = result;
                    4
                }
                0x1C => {
                    // INC E
                    let value = self.regs.e;
                    let result = self.alu_inc8(value);
                    self.regs.e = result;
                    4
                }
                0x24 => {
                    // INC H
                    let value = self.regs.h;
                    let result = self.alu_inc8(value);
                    self.regs.h = result;
                    4
                }
                0x2C => {
                    // INC L
                    let value = self.regs.l;
                    let result = self.alu_inc8(value);
                    self.regs.l = result;
                    4
                }
                0x3C => {
                    // INC A
                    let value = self.regs.a;
                    let result = self.alu_inc8(value);
                    self.regs.a = result;
                    4
                }
                0x34 => {
                    // INC (HL)
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    let result = self.alu_inc8(value);
                    bus.write8(addr, result);
                    12
                }

                // DEC r
                0x05 => {
                    // DEC B
                    let value = self.regs.b;
                    let result = self.alu_dec8(value);
                    self.regs.b = result;
                    4
                }
                0x0D => {
                    // DEC C
                    let value = self.regs.c;
                    let result = self.alu_dec8(value);
                    self.regs.c = result;
                    4
                }
                0x15 => {
                    // DEC D
                    let value = self.regs.d;
                    let result = self.alu_dec8(value);
                    self.regs.d = result;
                    4
                }
                0x1D => {
                    // DEC E
                    let value = self.regs.e;
                    let result = self.alu_dec8(value);
                    self.regs.e = result;
                    4
                }
                0x25 => {
                    // DEC H
                    let value = self.regs.h;
                    let result = self.alu_dec8(value);
                    self.regs.h = result;
                    4
                }
                0x2D => {
                    // DEC L
                    let value = self.regs.l;
                    let result = self.alu_dec8(value);
                    self.regs.l = result;
                    4
                }
                0x3D => {
                    // DEC A
                    let value = self.regs.a;
                    let result = self.alu_dec8(value);
                    self.regs.a = result;
                    4
                }
                0x35 => {
                    // DEC (HL)
                    let addr = self.regs.hl();
                    let value = bus.read8(addr);
                    let result = self.alu_dec8(value);
                    bus.write8(addr, result);
                    12
                }

                // DI
                0xF3 => {
                    self.ime = false;
                    self.ime_enable_pending = false;
                    self.ime_enable_delay = false;
                    4
                }

                // EI
                0xFB => {
                    // IME becomes 1 after the *next* instruction completes.
                    self.ime_enable_pending = true;
                    4
                }

                // 0xC3: JP a16
                0xC3 => {
                    let addr = self.fetch16(bus);
                    self.regs.pc = addr;
                    16
                }

                // JP (HL)
                0xE9 => {
                    let addr = self.regs.hl();
                    self.regs.pc = addr;
                    4
                }

                // CALL a16
                0xCD => {
                    let addr = self.fetch16(bus);
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = addr;
                    24
                }

                // PUSH rr
                0xC5 => {
                    // PUSH BC
                    let value = self.regs.bc();
                    self.push_u16(bus, value);
                    16
                }
                0xD5 => {
                    // PUSH DE
                    let value = self.regs.de();
                    self.push_u16(bus, value);
                    16
                }
                0xE5 => {
                    // PUSH HL
                    let value = self.regs.hl();
                    self.push_u16(bus, value);
                    16
                }
                0xF5 => {
                    // PUSH AF
                    let value = self.regs.af();
                    self.push_u16(bus, value);
                    16
                }

                // POP rr
                0xC1 => {
                    // POP BC
                    let value = self.pop_u16(bus);
                    self.regs.set_bc(value);
                    12
                }
                0xD1 => {
                    // POP DE
                    let value = self.pop_u16(bus);
                    self.regs.set_de(value);
                    12
                }
                0xE1 => {
                    // POP HL
                    let value = self.pop_u16(bus);
                    self.regs.set_hl(value);
                    12
                }
                0xF1 => {
                    // POP AF (low 4 bits of F are always zero)
                    let value = self.pop_u16(bus);
                    self.regs.set_af(value);
                    12
                }

                // CALL cc, a16
                0xC4 => {
                    // CALL NZ, a16
                    let cond = !self.get_flag(Flag::Z);
                    self.call_cond(bus, cond)
                }
                0xCC => {
                    // CALL Z, a16
                    let cond = self.get_flag(Flag::Z);
                    self.call_cond(bus, cond)
                }
                0xD4 => {
                    // CALL NC, a16
                    let cond = !self.get_flag(Flag::C);
                    self.call_cond(bus, cond)
                }
                0xDC => {
                    // CALL C, a16
                    let cond = self.get_flag(Flag::C);
                    self.call_cond(bus, cond)
                }

                // RET
                0xC9 => {
                    let addr = self.pop_u16(bus);
                    self.regs.pc = addr;
                    16
                }

                // RET cc
                0xC0 => {
                    // RET NZ
                    let cond = !self.get_flag(Flag::Z);
                    self.ret_cond(bus, cond)
                }
                0xC8 => {
                    // RET Z
                    let cond = self.get_flag(Flag::Z);
                    self.ret_cond(bus, cond)
                }
                0xD0 => {
                    // RET NC
                    let cond = !self.get_flag(Flag::C);
                    self.ret_cond(bus, cond)
                }
                0xD8 => {
                    // RET C
                    let cond = self.get_flag(Flag::C);
                    self.ret_cond(bus, cond)
                }

                // RETI
                0xD9 => {
                    let addr = self.pop_u16(bus);
                    self.regs.pc = addr;
                    self.ime = true;
                    16
                }

                // RST nn
                0xC7 => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x00;
                    16
                }
                0xCF => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x08;
                    16
                }
                0xD7 => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x10;
                    16
                }
                0xDF => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x18;
                    16
                }
                0xE7 => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x20;
                    16
                }
                0xEF => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x28;
                    16
                }
                0xF7 => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x30;
                    16
                }
                0xFF => {
                    let ret = self.regs.pc;
                    self.push_u16(bus, ret);
                    self.regs.pc = 0x38;
                    16
                }

                // Invalid / unimplemented opcodes.
                //
                // Pandocs documents a set of "opcode holes" (D3, DB, DD, E3, E4,
                // EB, EC, ED, F4, FC, FD) that hard‑lock the CPU until power‑off.
                // We approximate that by marking the CPU as locked so subsequent
                // `step` calls return 0 cycles.
                _ => {
                    self.locked = true;
                    0
                }
            }
        }
    }

    /// Legacy instruction-level stepping entry point.
    ///
    /// This implementation predates the micro-step (`step_mcycle`) API
    /// and drives the bus using a single `tick(cycles)` call per
    /// instruction via `end_instruction`. It is kept around for
    /// reference and as a fallback, but the main `step` entry point
    /// below now delegates to `step_mcycle` so that both APIs share the
    /// same per-cycle timing model.
    #[allow(dead_code)]
    fn step_instruction_legacy<B: Bus>(&mut self, bus: &mut B) -> u32 {
        // Instruction‑level stepping ignores any in‑flight micro‑step state
        // and always executes a full instruction in one go.
        self.micro_cycles_remaining = 0;
        self.micro_from_interrupt = false;
        self.micro_instr = MicroInstrKind::None;
        self.micro_stage = 0;
        self.micro_imm16 = 0;
        self.micro_cond_taken = false;

        if self.locked {
            // CPU has executed an invalid opcode (see Pandocs CPU opcode
            // holes). On hardware the CPU is effectively dead until power‑off.
            // We approximate that by returning 0 cycles so higher‑level loops
            // can detect the condition and stop.
            return 0;
        }

        // Mark the beginning of a single instruction (or interrupt entry)
        // so that the bus can track per‑instruction state such as timer
        // IO events.
        bus.begin_instruction();

        // In STOP mode the CPU enters a deeper low‑power state than HALT. On
        // real hardware STOP is exited when a joypad input line (P10–P13)
        // goes low. We approximate this by polling P1 ($FF00) on each step
        // and clearing `stopped` once any of the lower four bits becomes 0.
        if self.stopped {
            let p1 = bus.read8(0xFF00);
            if (p1 & 0x0F) != 0x0F {
                // A button/d‑pad line is low: resume normal execution.
                self.stopped = false;
            }
            // In STOP mode the system counter used by DIV/TIMA does not
            // advance (see Pandocs "System counter"). We therefore avoid
            // calling `bus.tick` here so that timers/PPU remain frozen until
            // STOP is exited. We still report a small non‑zero cycle cost so
            // that callers that rely on forward progress do not spin forever.
            bus.end_instruction(0);
            return 4;
        }

        if let Some(cycles) = self.handle_interrupts(bus) {
            bus.end_instruction(cycles);
            return cycles;
        }

        if self.halted {
            // In HALT, the CPU effectively performs a NOP each cycle until
            // an interrupt occurs. For now we just return a fixed cost.
            bus.end_instruction(4);
            return 4;
        }

        let opcode = self.fetch8(bus);
        let cycles = self.exec_opcode(bus, opcode);

        bus.end_instruction(cycles);
        self.apply_ime_delay();
        cycles
    }

    /// Advance the CPU and bus by (up to) a single M‑cycle (4 T‑cycles).
    ///
    /// This experimental micro‑step API preserves the same instruction‑level
    /// semantics as `step`, but spreads `bus.tick` and the IME delay pipeline
    /// across individual 4‑cycle chunks. It is currently unused by the
    /// higher‑level machine and exists as a scaffold for future
    /// cycle‑accurate work.
    ///
    /// The return value is the number of T‑cycles consumed (usually 4, but
    /// it can be 0 when the CPU is locked or an invalid opcode has just
    /// hard‑locked the core).
    pub fn step_mcycle<B: Bus>(&mut self, bus: &mut B) -> u32 {
        if self.locked {
            // Hard‑locked cores never advance time.
            return 0;
        }

        // STOP low‑power mode: identical to the `step` implementation, we
        // poll P1 ($FF00) on each call and do not tick the bus so that
        // timers/PPU remain frozen.
        if self.stopped {
            let p1 = bus.read8(0xFF00);
            if (p1 & 0x0F) != 0x0F {
                self.stopped = false;
            }
            return 4;
        }

        // If no instruction/interrupt sequence is currently in flight, start
        // one. We mirror the control‑flow ordering in `step`:
        //   1) service interrupts
        //   2) HALT low‑power state
        //   3) normal opcode fetch/execute
        if self.micro_cycles_remaining == 0 {
            if self.begin_interrupt_micro(bus) {
                // Interrupt entry has been latched; `micro_cycles_remaining`
                // now tracks the remaining 20 T‑cycles, and individual
                // M‑cycles are executed by `step_interrupt_mcycle` below.
            } else if self.halted {
                // In HALT, the CPU issues a stream of NOP‑like cycles until
                // an interrupt is taken. From the bus's perspective this is
                // just a fixed cost.
                bus.tick(4);
                return 4;
            } else {
                // Normal instruction path: fetch the opcode and either
                // hand it off to a dedicated micro-op sequence or fall
                // back to the monolithic `exec_opcode` semantics.
                let opcode = self.fetch8(bus);
                match opcode {
                    // 0xCD: CALL a16 (24 T‑cycles).
                    0xCD => {
                        self.micro_instr = MicroInstrKind::CallA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 24;
                        self.micro_from_interrupt = false;
                    }
                    // 0xC9: RET (16 T‑cycles).
                    0xC9 => {
                        self.micro_instr = MicroInstrKind::Ret;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 16;
                        self.micro_from_interrupt = false;
                    }
                    // 0xC3: JP a16 (16 T‑cycles).
                    0xC3 => {
                        self.micro_instr = MicroInstrKind::JpA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 16;
                        self.micro_from_interrupt = false;
                    }
                    // 0x08: LD (a16),SP (20 T‑cycles).
                    0x08 => {
                        self.micro_instr = MicroInstrKind::LdSpA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 20;
                        self.micro_from_interrupt = false;
                    }
                    // RST nn (16 T‑cycles).
                    0xC7 | 0xCF | 0xD7 | 0xDF | 0xE7 | 0xEF | 0xF7 | 0xFF => {
                        let vector = match opcode {
                            0xC7 => 0x00,
                            0xCF => 0x08,
                            0xD7 => 0x10,
                            0xDF => 0x18,
                            0xE7 => 0x20,
                            0xEF => 0x28,
                            0xF7 => 0x30,
                            0xFF => 0x38,
                            _ => 0x00,
                        };
                        self.micro_instr = MicroInstrKind::Rst;
                        self.micro_stage = 0;
                        self.micro_imm16 = vector;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 16;
                        self.micro_from_interrupt = false;
                    }
                    // JR r8 (relative, 12 T-cycles).
                    0x18 => {
                        self.micro_instr = MicroInstrKind::Jr;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 12;
                        self.micro_from_interrupt = false;
                    }
                    // JR cc, r8 (12/8 T-cycles).
                    0x20 | 0x28 | 0x30 | 0x38 => {
                        let cond = match opcode {
                            0x20 => !self.get_flag(Flag::Z), // JR NZ
                            0x28 => self.get_flag(Flag::Z),  // JR Z
                            0x30 => !self.get_flag(Flag::C), // JR NC
                            0x38 => self.get_flag(Flag::C),  // JR C
                            _ => false,
                        };
                        self.micro_instr = MicroInstrKind::JrCond;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = cond;
                        self.micro_cycles_remaining = if cond { 12 } else { 8 };
                        self.micro_from_interrupt = false;
                    }
                    // RET cc (20/8 T‑cycles).
                    0xC0 | 0xC8 | 0xD0 | 0xD8 => {
                        let cond = match opcode {
                            0xC0 => !self.get_flag(Flag::Z), // RET NZ
                            0xC8 => self.get_flag(Flag::Z),  // RET Z
                            0xD0 => !self.get_flag(Flag::C), // RET NC
                            0xD8 => self.get_flag(Flag::C),  // RET C
                            _ => false,
                        };
                        self.micro_instr = MicroInstrKind::RetCond;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = cond;
                        self.micro_cycles_remaining = if cond { 20 } else { 8 };
                        self.micro_from_interrupt = false;
                    }
                    // JP cc,a16 (16/12 T‑cycles).
                    0xC2 | 0xCA | 0xD2 | 0xDA => {
                        let cond = match opcode {
                            0xC2 => !self.get_flag(Flag::Z), // JP NZ
                            0xCA => self.get_flag(Flag::Z),  // JP Z
                            0xD2 => !self.get_flag(Flag::C), // JP NC
                            0xDA => self.get_flag(Flag::C),  // JP C
                            _ => false,
                        };
                        self.micro_instr = MicroInstrKind::JpCondA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = cond;
                        self.micro_cycles_remaining = if cond { 16 } else { 12 };
                        self.micro_from_interrupt = false;
                    }
                    // CALL cc,a16 (24/12 T-cycles).
                    0xC4 | 0xCC | 0xD4 | 0xDC => {
                        let cond = match opcode {
                            0xC4 => !self.get_flag(Flag::Z), // CALL NZ
                            0xCC => self.get_flag(Flag::Z),  // CALL Z
                            0xD4 => !self.get_flag(Flag::C), // CALL NC
                            0xDC => self.get_flag(Flag::C),  // CALL C
                            _ => false,
                        };
                        self.micro_instr = MicroInstrKind::CallCond;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = cond;
                        self.micro_cycles_remaining = if cond { 24 } else { 12 };
                        self.micro_from_interrupt = false;
                    }
                    // 0xF0: LDH A,(a8) (12 T-cycles).
                    0xF0 => {
                        self.micro_instr = MicroInstrKind::LdhAFromA8;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 12;
                        self.micro_from_interrupt = false;
                    }
                    // 0xE0: LDH (a8),A (12 T-cycles).
                    0xE0 => {
                        self.micro_instr = MicroInstrKind::LdhAToA8;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 12;
                        self.micro_from_interrupt = false;
                    }
                    _ => {
                        let cycles = self.exec_opcode(bus, opcode);
                        self.micro_cycles_remaining = cycles;
                        self.micro_from_interrupt = false;

                        // Some invalid opcodes hard‑lock the CPU and return 0
                        // cycles. We still need to advance the IME delay
                        // pipeline once to match `step`'s behaviour.
                        if self.micro_cycles_remaining == 0 {
                            self.apply_ime_delay();
                            return 0;
                        }
                    }
                }
            }
        }

        // At this point we have a non‑zero cycle budget for the current
        // instruction or interrupt handler. Consume (up to) one M‑cycle.
        if self.micro_cycles_remaining >= 4 {
            if self.micro_from_interrupt {
                self.step_interrupt_mcycle(bus);
            } else if self.micro_instr != MicroInstrKind::None {
                self.step_instruction_mcycle(bus);
            } else {
                // Generic M‑cycle without explicit Timer IO: advance both
                // non‑timer peripherals and the Timer once.
                bus.tick_mcycle_generic();
                bus.timer_tick_mcycle();
            }
            self.micro_cycles_remaining -= 4;

            if self.micro_cycles_remaining == 0 {
                // Instruction boundary: for regular opcodes we advance the
                // delayed IME state; interrupt entries do not participate in
                // the EI delay pipeline in our current model.
                if !self.micro_from_interrupt {
                    self.apply_ime_delay();
                }
                self.micro_from_interrupt = false;
                // Reset interrupt micro‑state for the next potential entry.
                self.interrupt_stage = 0;
                self.interrupt_index_latched = None;
                self.micro_instr = MicroInstrKind::None;
                self.micro_stage = 0;
                self.micro_imm16 = 0;
                self.micro_cond_taken = false;
            }

            4
        } else {
            // Defensive fallback for non‑aligned cycle counts. Game Boy
            // instructions are multiples of 4 T‑cycles, so this arm should
            // never be hit, but we handle it to keep the API robust.
            let remaining = self.micro_cycles_remaining;
            if remaining > 0 {
                bus.tick(remaining);
                self.micro_cycles_remaining = 0;
                if !self.micro_from_interrupt {
                    self.apply_ime_delay();
                }
                self.micro_from_interrupt = false;
            }
            remaining
        }
    }

    /// Execute a single instruction and return the number of T-cycles taken.
    ///
    /// For now this entry point delegates to the legacy instruction-level
    /// implementation so that the higher-level machine continues to use
    /// the original `begin_instruction`/`end_instruction` integration.
    /// The micro-step API (`step_mcycle`) is used separately by dedicated
    /// tests and experiments.
    pub fn step<B: Bus>(&mut self, bus: &mut B) -> u32 {
        self.step_instruction_legacy(bus)
    }
}

#[cfg(test)]
mod tests {
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

    fn load_individual_rom(filename: &str) -> Vec<u8> {
        use std::path::PathBuf;

        let candidates = [
            PathBuf::from("repo-ref/game-boy-test-roms/artifacts/blargg/cpu_instrs/individual")
                .join(filename),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../repo-ref/game-boy-test-roms/artifacts/blargg/cpu_instrs/individual")
                .join(filename),
        ];

        for path in &candidates {
            if let Ok(data) = std::fs::read(path) {
                return data;
            }
        }

        panic!(
            "failed to read individual ROM {:?} (tried {:?})",
            filename, candidates
        );
    }

    fn load_mooneye_timer_rom(filename: &str) -> Vec<u8> {
        use std::path::PathBuf;

        let candidates = [
            // In-repo copy that can be committed alongside the emulator.
            PathBuf::from("assets/mooneye/acceptance/timer").join(filename),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../assets/mooneye/acceptance/timer")
                .join(filename),
            // Fallback to the external reference checkout under `repo-ref`.
            PathBuf::from("repo-ref/mooneye-test-roms/acceptance/timer").join(filename),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../repo-ref/mooneye-test-roms/acceptance/timer")
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
            PathBuf::from("repo-ref/mooneye-test-roms/acceptance/ppu").join(filename),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../repo-ref/mooneye-test-roms/acceptance/ppu")
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
            // External reference checkout under `repo-ref`.
            PathBuf::from("repo-ref/mooneye-test-roms/acceptance").join(filename),
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../repo-ref/mooneye-test-roms/acceptance")
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

    /// Helper for blargg's 02-interrupts test ROM.
    ///
    /// This is useful when debugging interrupt / HALT / IME behaviour in
    /// isolation instead of via the combined cpu_instrs.gb harness.
    #[test]
    #[ignore]
    fn run_blargg_02_interrupts_individual() {
        run_individual_rom_and_check("02-interrupts.gb", "02-interrupts");
    }

    /// Helper to debug blargg's 03-op sp,hl individual test ROM.
    ///
    /// This is ignored by default and used for manual inspection of the
    /// serial output while bringing up SP/HL-related instructions.
    #[test]
    #[ignore]
    fn run_blargg_03_op_sp_hl_individual() {
        run_individual_rom_and_check("03-op sp,hl.gb", "03-op sp,hl");
    }

    /// Helper for blargg's 04-op r,imm test ROM.
    #[test]
    #[ignore]
    fn run_blargg_04_op_r_imm_individual() {
        run_individual_rom_and_check("04-op r,imm.gb", "04-op r,imm");
    }

    /// Helper for blargg's 05-op rp test ROM.
    #[test]
    #[ignore]
    fn run_blargg_05_op_rp_individual() {
        run_individual_rom_and_check("05-op rp.gb", "05-op rp");
    }

    /// Helper for blargg's 06-ld r,r test ROM.
    #[test]
    #[ignore]
    fn run_blargg_06_ld_r_r_individual() {
        run_individual_rom_and_check("06-ld r,r.gb", "06-ld r,r");
    }

    /// Helper for blargg's 07-jr,jp,call,ret,rst test ROM.
    #[test]
    #[ignore]
    fn run_blargg_07_jr_jp_call_ret_rst_individual() {
        run_individual_rom_and_check("07-jr,jp,call,ret,rst.gb", "07-jr,jp,call,ret,rst");
    }

    /// Helper for blargg's 08-misc instrs test ROM.
    #[test]
    #[ignore]
    fn run_blargg_08_misc_instrs_individual() {
        run_individual_rom_and_check("08-misc instrs.gb", "08-misc instrs");
    }

    /// Helper for blargg's 09-op r,r test ROM.
    #[test]
    #[ignore]
    fn run_blargg_09_op_r_r_individual() {
        run_individual_rom_and_check("09-op r,r.gb", "09-op r,r");
    }

    /// Helper for blargg's 10-bit ops test ROM.
    #[test]
    #[ignore]
    fn run_blargg_10_bit_ops_individual() {
        run_individual_rom_and_check("10-bit ops.gb", "10-bit ops");
    }

    /// Helper for blargg's 11-op a,(hl) test ROM.
    #[test]
    #[ignore]
    fn run_blargg_11_op_a_hl_individual() {
        run_individual_rom_and_check("11-op a,(hl).gb", "11-op a,(hl)");
    }

    /// Run a single blargg cpu_instrs *individual* ROM until it reports either
    /// "Passed" or "Failed" on the serial port, or we hit a large cycle budget.
    ///
    /// These helpers are marked `#[ignore]` on the tests so that they are only
    /// run manually. On success we print the ROM label, its output and the
    /// total cycle count; on failure we panic with the captured output to aid
    /// debugging.
    fn run_individual_rom_and_check(filename: &str, label: &str) {
        let rom = load_individual_rom(filename);
        let mut gb = crate::machine::GameBoy::new();
        gb.load_rom(&rom);

        const MAX_CYCLES: u64 = 200_000_000;
        let mut cycles: u64 = 0;
        loop {
            let c = gb.cpu.step(&mut gb.bus) as u64;
            if c == 0 {
                break;
            }
            cycles = cycles.saturating_add(c);

            let output = &gb.bus.serial.output;
            if output.windows(b"Passed".len()).any(|w| w == b"Passed")
                || output.windows(b"Failed".len()).any(|w| w == b"Failed")
            {
                break;
            }

            if cycles >= MAX_CYCLES {
                break;
            }
        }

        let output_str = String::from_utf8_lossy(&gb.bus.serial.output);
        let has_passed = output_str.contains("Passed");
        let has_failed = output_str.contains("Failed");

        if has_passed && !has_failed {
            println!("{label} output:\n{output_str}\n(total cycles {cycles})");
        } else {
            panic!(
                "{label} did not report success.\nOutput:\n{output_str}\n(total cycles {cycles})"
            );
        }
    }

    /// Reference run of a mooneye timer ROM using the mooneye-gb-core
    /// implementation. This is used by some debugging tests to compare our
    /// behaviour against the reference core at a coarse granularity.
    ///
    /// Note: this helper is only compiled in tests.
    #[cfg(test)]
    fn run_mooneye_timer_rom_reference(filename: &str) {
        use mooneye_gb_core::config::{Cartridge, HardwareConfig, Model};
        use mooneye_gb_core::emulation::EmuTime;
        use mooneye_gb_core::machine::Machine as MooneyeMachine;
        use std::sync::Arc;

        let rom = load_mooneye_timer_rom(filename);
        let data: Arc<[u8]> = Arc::from(rom);
        let cart = Cartridge::from_data(data).expect("failed to construct mooneye cartridge");
        let config = HardwareConfig {
            model: Model::Dmg,
            bootrom: None,
            cartridge: cart,
        };
        let mut m = MooneyeMachine::new(config);

        // Run the reference core until it reaches the standard success
        // register pattern or a large cycle budget.
        const MAX_MCYCLES: u64 = 12_500_000; // 50M T-cycles / 4
        let mut success = false;

        while m.emu_time().machine_cycles < MAX_MCYCLES {
            let (_events, _time) = m.emulate_step();
            let regs = m.regs();
            if regs.a == 0
                && regs.b == 3
                && regs.c == 5
                && regs.d == 8
                && regs.e == 13
                && regs.h == 21
                && regs.l == 34
            {
                success = true;
                break;
            }
        }

        assert!(
            success,
            "reference mooneye core did not reach success pattern for ROM {}",
            filename
        );
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

    /// Lockstep comparison between our CPU core and mooneye-gb-core on the
    /// `tim00` timer acceptance ROM.
    ///
    /// This helper advances both machines roughly one instruction at a time
    /// (using our `step` and mooneye's `emulate_step`) and checks CPU
    /// registers after each step. On the first mismatch it panics with a
    /// detailed dump of both cores, including our visible timer/interrupt
    /// registers and cumulative cycle counts.
    ///
    /// The goal is to identify the earliest point where our timing or
    /// semantics diverge from the reference core, so that we can iterate on
    /// micro-ops and timer integration with concrete evidence.
    #[cfg(test)]
    fn run_tim00_lockstep_against_mooneye() {
        use mooneye_gb_core::config::{Cartridge, HardwareConfig, Model};
        use mooneye_gb_core::machine::Machine as MooneyeMachine;
        use std::sync::Arc;

        let rom = load_mooneye_timer_rom("tim00.gb");

        // Our machine: use a deterministic RAM initialisation so that any
        // divergence comes from logic/timing differences rather than random
        // WRAM/HRAM contents.
        let mut ours = crate::machine::GameBoy::new_zeroed_internal_ram();
        ours.load_rom(&rom);

        // Reference machine: mooneye-gb-core in DMG mode without a boot ROM.
        let data: Arc<[u8]> = Arc::from(rom);
        let cart = Cartridge::from_data(data).expect("failed to construct mooneye cartridge");
        let config = HardwareConfig {
            model: Model::Dmg,
            bootrom: None,
            cartridge: cart,
        };
        let mut reference = MooneyeMachine::new(config);

        // Initialise the reference core's registers to the same DMG
        // post-boot state that our CPU uses. This avoids pulling in the
        // real boot ROM sequencing and keeps the comparison focused on
        // timer/CPU behaviour in the test ROM itself.
        {
            let regs = reference.regs_mut();
            regs.pc = 0x0100;
            regs.sp = 0xFFFE;
            regs.a = 0x01;
            regs.set_zf(true);
            regs.set_nf(false);
            regs.set_hf(true);
            regs.set_cf(true);
            regs.b = 0x00;
            regs.c = 0x13;
            regs.d = 0x00;
            regs.e = 0xD8;
            regs.h = 0x01;
            regs.l = 0x4D;
        }

        // Optional early check: if the very first state already differs, treat
        // that as a divergence at "instruction 0".
        {
            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                panic!(
                    "tim00 lockstep: initial CPU state differs before executing any instructions.\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}",
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                );
            }
        }

        const MAX_INSTRUCTIONS: u64 = 5_000_000;
        // Allow a generous warm-up window so that any pipeline /
        // prefetch differences between the two cores settle before we
        // start asserting on exact CPU state.
        const WARMUP_INSTRUCTIONS: u64 = 32;
        const DEBUG_INSTRUCTIONS: u64 = 32;
        let mut instr_count: u64 = 0;
        let mut cycles_ours: u64 = 0;
        let mut cycles_ref: u64 = 0;

        while instr_count < MAX_INSTRUCTIONS {
            instr_count = instr_count.saturating_add(1);

            // Step our core once.
            let c_ours = ours.cpu.step(&mut ours.bus) as u64;
            if c_ours == 0 {
                panic!(
                    "tim00 lockstep: our CPU returned 0 cycles at instruction {} (possible lock-up)",
                    instr_count
                );
            }
            cycles_ours = cycles_ours.saturating_add(c_ours);

            // Step mooneye once and derive the T-cycle delta from its
            // machine-cycle counter. We treat each `emulate_step` call as
            // roughly analogous to one `step` call on our core; differences
            // in how instructions are internally chunked will show up as
            // PC/cycle mismatches in the comparison below.
            let before = reference.emu_time().machine_cycles;
            let (_events, _time) = reference.emulate_step();
            let after = reference.emu_time().machine_cycles;
            let delta_mc = after.saturating_sub(before);
            cycles_ref = cycles_ref.saturating_add(delta_mc.saturating_mul(4));

            // Skip the initial few instructions where the reference core's
            // internal prefetch pipeline and our simplified model are not
            // directly comparable. After this warm-up window we expect both
            // cores to be executing the same ROM code in a steady state.
            if instr_count <= WARMUP_INSTRUCTIONS {
                continue;
            }

            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            // Optional early-debug window: for the first few instructions
            // after the warm-up, log both cores' CPU state plus our timer
            // registers so that we can see how DIV/TIMA/TMA/TAC evolve
            // around the timer initialisation sequence in `tim00`.
            //
            // We use a special snapshot helper rather than MMIO reads to
            // avoid advancing the Timer while collecting debug data.
            if instr_count <= DEBUG_INSTRUCTIONS {
                let (div_my, tima_my, tma_my, tac_my, if_my, ie_my) =
                    ours.bus.debug_timer_snapshot();

                println!(
                    "[tim00 dbg] instr={instr} ours: PC={pc_my:04X} A={a_my:02X} F={f_my:02X} \
                     DIV={div_my:02X} TIMA={tima_my:02X} TMA={tma_my:02X} TAC={tac_my:02X} \
                     IF={if_my:02X} IE={ie_my:02X} \
                     | ref: PC={pc_ref:04X} A={a_ref:02X} F={f_ref:02X}",
                    instr = instr_count,
                    pc_my = r_my.pc,
                    a_my = r_my.a,
                    pc_ref = r_ref.pc,
                    a_ref = r_ref.a,
                );
            }

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                // Capture our visible timer/interrupt state for additional
                // context around the divergence.
                let (div, tima, tma, tac, if_reg, ie_reg) = ours.bus.debug_timer_snapshot();

                panic!(
                    "tim00 lockstep diverged at instruction {}.\n\
Our cycles (T): {}\n\
Ref cycles (T): {}\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Our timer regs: DIV={:02X} TIMA={:02X} TMA={:02X} TAC={:02X}, IF={:02X}, IE={:02X}",
                    instr_count,
                    cycles_ours,
                    cycles_ref,
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                    div,
                    tima,
                    tma,
                    tac,
                    if_reg,
                    ie_reg,
                );
            }

            // Stop once we hit the standard "success" Fibonacci register
            // pattern used by mooneye timer acceptance tests.
            if r_my.a == 0
                && r_my.b == 3
                && r_my.c == 5
                && r_my.d == 8
                && r_my.e == 13
                && r_my.h == 21
                && r_my.l == 34
            {
                break;
            }
        }

        if instr_count >= MAX_INSTRUCTIONS {
            panic!(
                "tim00 lockstep: reached instruction budget ({}) without divergence or success.\n\
Final our cycles (T): {}\n\
Final ref cycles (T): {}\n\
Final our regs: {:?}\n\
Final ref regs: {:?}",
                MAX_INSTRUCTIONS,
                cycles_ours,
                cycles_ref,
                ours.cpu.regs,
                    reference.regs(),
            );
        }
    }

    /// Lockstep comparison between our CPU core and mooneye-gb-core on the
    /// `interrupts/ie_push` acceptance ROM.
    ///
    /// This mirrors `run_tim00_lockstep_against_mooneye` but drives the
    /// interrupt-focused ROM instead of the timer one. The goal is to
    /// pinpoint the earliest divergence in CPU state so that we can refine
    /// our interrupt/IME/IF behaviour.
    #[cfg(test)]
    fn run_ie_push_lockstep_against_mooneye() {
        use mooneye_gb_core::config::{Cartridge, HardwareConfig, Model};
        use mooneye_gb_core::machine::Machine as MooneyeMachine;
        use std::sync::Arc;

        let rom = load_mooneye_interrupts_rom("interrupts/ie_push.gb");

        // Our machine: deterministic internal RAM so differences come from
        // logic/timing rather than random data. For this lockstep we also
        // start from a "mooneye-like" LCDC state (LCDC off) and timer
        // power-on state so that tests that probe FF40 and the Timer
        // (e.g. `ie_push.gb`) observe the same initial values as the
        // reference core, which does not emulate the DMG boot ROM's LCD
        // enable or DIV seeding.
        let mut ours = crate::machine::GameBoy::new_zeroed_internal_ram();
        ours.load_rom(&rom);
        // Mooneye starts with LCDC=0; override our DMG post-boot default.
        ours.bus.memory[0xFF40] = 0x00;
        // Align Timer/interrupt controller power-on state with mooneye:
        // DIV/TIMA/TMA/TAC start from 0 and no interrupt is pending.
        ours.bus.reset_timer_for_mooneye_tests();
        ours.bus.if_reg = 0x00;

        // Reference machine.
        let data: Arc<[u8]> = Arc::from(rom);
        let cart = Cartridge::from_data(data).expect("failed to construct mooneye cartridge");
        let config = HardwareConfig {
            model: Model::Dmg,
            bootrom: None,
            cartridge: cart,
        };
        let mut reference = MooneyeMachine::new(config);

        // Align reference CPU registers with our DMG post-boot snapshot.
        {
            let regs = reference.regs_mut();
            regs.pc = 0x0100;
            regs.sp = 0xFFFE;
            regs.a = 0x01;
            regs.set_zf(true);
            regs.set_nf(false);
            regs.set_hf(true);
            regs.set_cf(true);
            regs.b = 0x00;
            regs.c = 0x13;
            regs.d = 0x00;
            regs.e = 0xD8;
            regs.h = 0x01;
            regs.l = 0x4D;
        }

        // Early sanity check before executing anything.
        {
            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                panic!(
                    "ie_push lockstep: initial CPU state differs before executing any instructions.\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}",
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                );
            }
        }

        const MAX_INSTRUCTIONS: u64 = 5_000_000;
        // Keep the same warm-up window as the instruction-level
        // lockstep so that both helpers start comparing from a
        // consistent point in the ROM.
        const WARMUP_INSTRUCTIONS: u64 = 32;
        const DEBUG_INSTRUCTIONS: u64 = 32;
        let mut instr_count: u64 = 0;
        let mut cycles_ours: u64 = 0;
        let mut cycles_ref: u64 = 0;

        while instr_count < MAX_INSTRUCTIONS {
            instr_count = instr_count.saturating_add(1);

            let c_ours = ours.cpu.step(&mut ours.bus) as u64;
            if c_ours == 0 {
                panic!(
                    "ie_push lockstep: our CPU returned 0 cycles at instruction {} (possible lock-up)",
                    instr_count
                );
            }
            cycles_ours = cycles_ours.saturating_add(c_ours);

            let before = reference.emu_time().machine_cycles;
            let (_events, _time) = reference.emulate_step();
            let after = reference.emu_time().machine_cycles;
            let delta_mc = after.saturating_sub(before);
            cycles_ref = cycles_ref.saturating_add(delta_mc.saturating_mul(4));

            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            // For the first few instructions (including the warm-up
            // window) log both cores' CPU + timer/interrupt state and
            // our stack view so that we can understand how
            // DIV/TIMA/TMA/TAC/IF/IE and IME evolve around the initial
            // interrupt setup in `ie_push.gb`.
            if instr_count <= DEBUG_INSTRUCTIONS {
                let (div_my, tima_my, tma_my, tac_my, if_my, ie_my) =
                    ours.bus.debug_timer_snapshot();
                let ovf_my = ours.bus.timer_debug_overflows();
                let (div_ref, tima_ref, tma_ref, tac_ref, if_ref, ie_ref) =
                    reference.hardware_mut().debug_timer_snapshot_for_tests();

                // Peek the byte at the current stack pointer and the
                // following byte for return-address debugging. Out of
                // bounds reads default to 0.
                let sp = r_my.sp;
                let peek = |addr: u16, mem: &[u8; 0x10000]| -> u8 {
                    *mem.get(addr as usize).unwrap_or(&0)
                };
                let s0 = peek(sp, &ours.bus.memory);
                let s1 = peek(sp.wrapping_add(1), &ours.bus.memory);

                let opcode_my = ours.bus.read8(r_my.pc);
                let opcode_ref = ours.bus.read8(r_ref.pc);

                println!(
                    "[ie_push dbg] instr={instr} \
ours: PC={pc_my:04X} SP={sp_my:04X} A={a_my:02X} F={f_my:02X} \
DIV={div_my:02X} TIMA={tima_my:02X} TMA={tma_my:02X} TAC={tac_my:02X} \
IF={if_my:02X} IE={ie_my:02X} IME={ime_my} OVF={ovf_my} \
STACK[0]={s0:02X} STACK[1]={s1:02X} OPC={op_my:02X} \
ref: PC={pc_ref:04X} SP={sp_ref:04X} A={a_ref:02X} F={f_ref:02X} \
DIV={div_ref:02X} TIMA={tima_ref:02X} TMA={tma_ref:02X} TAC={tac_ref:02X} \
IF={if_ref:02X} IE={ie_ref:02X} OPC={op_ref:02X}",
                    instr = instr_count,
                    pc_my = r_my.pc,
                    sp_my = r_my.sp,
                    a_my = r_my.a,
                    f_my = f_my,
                    div_my = div_my,
                    tima_my = tima_my,
                    tma_my = tma_my,
                    tac_my = tac_my,
                    if_my = if_my,
                    ie_my = ie_my,
                    ime_my = ours.cpu.ime,
                    ovf_my = ovf_my,
                    s0 = s0,
                    s1 = s1,
                    op_my = opcode_my,
                    pc_ref = r_ref.pc,
                    sp_ref = r_ref.sp,
                    a_ref = r_ref.a,
                    f_ref = f_ref,
                    div_ref = div_ref,
                    tima_ref = tima_ref,
                    tma_ref = tma_ref,
                    tac_ref = tac_ref,
                    if_ref = if_ref,
                    ie_ref = ie_ref,
                    op_ref = opcode_ref,
                );
            }

            if instr_count <= WARMUP_INSTRUCTIONS {
                continue;
            }

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                let if_reg = ours.bus.read8(0xFF0F);
                let ie_reg = ours.bus.read8(0xFFFF);

                panic!(
                    "ie_push lockstep diverged at instruction {}.\n\
Our cycles (T): {}\n\
Ref cycles (T): {}\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X} IME={}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Our IF={:02X}, IE={:02X}",
                    instr_count,
                    cycles_ours,
                    cycles_ref,
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    ours.cpu.ime,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                    if_reg,
                    ie_reg,
                );
            }

            // Stop early once both cores reach the Fibonacci success pattern.
            if r_my.a == 0
                && r_my.b == 3
                && r_my.c == 5
                && r_my.d == 8
                && r_my.e == 13
                && r_my.h == 21
                && r_my.l == 34
            {
                break;
            }
        }

        if instr_count >= MAX_INSTRUCTIONS {
            panic!(
                "ie_push lockstep: reached instruction budget ({}) without divergence or success.\n\
Final our cycles (T): {}\n\
Final ref cycles (T): {}\n\
Final our regs: {:?}\n\
Final ref regs: {:?}",
                MAX_INSTRUCTIONS,
                cycles_ours,
                cycles_ref,
                ours.cpu.regs,
                reference.regs(),
            );
        }
    }

    /// Micro-step variant of the `ie_push` lockstep comparison.
    ///
    /// This drives our core using `step_mcycle` (micro-step API) while the
    /// reference core still uses its instruction-level `emulate_step`.
    /// The goal is to validate that the per-M-cycle Timer/interrupt
    /// integration behaves consistently with the instruction-level model
    /// and to pinpoint any timing-related divergences.
    #[cfg(test)]
    fn run_ie_push_lockstep_against_mooneye_micro() {
        use mooneye_gb_core::config::{Cartridge, HardwareConfig, Model};
        use mooneye_gb_core::machine::Machine as MooneyeMachine;
        use std::sync::Arc;

        let rom = load_mooneye_interrupts_rom("interrupts/ie_push.gb");

        // Our machine: deterministic internal RAM so differences come from
        // logic/timing rather than random data. For this lockstep we also
        // start from a "mooneye-like" LCDC state (LCDC off) and timer
        // power-on state so that tests that probe FF40 and the Timer
        // (e.g. `ie_push.gb`) observe the same initial values as the
        // reference core, which does not emulate the DMG boot ROM's LCD
        // enable or DIV seeding.
        let mut ours = crate::machine::GameBoy::new_zeroed_internal_ram();
        ours.load_rom(&rom);
        // Mooneye starts with LCDC=0; override our DMG post-boot default.
        ours.bus.memory[0xFF40] = 0x00;
        // Align Timer/interrupt controller power-on state with mooneye.
        ours.bus.reset_timer_for_mooneye_tests();
        ours.bus.if_reg = 0x00;

        // Reference machine.
        let data: Arc<[u8]> = Arc::from(rom);
        let cart = Cartridge::from_data(data).expect("failed to construct mooneye cartridge");
        let config = HardwareConfig {
            model: Model::Dmg,
            bootrom: None,
            cartridge: cart,
        };
        let mut reference = MooneyeMachine::new(config);

        // Align reference CPU registers with our DMG post-boot snapshot.
        {
            let regs = reference.regs_mut();
            regs.pc = 0x0100;
            regs.sp = 0xFFFE;
            regs.a = 0x01;
            regs.set_zf(true);
            regs.set_nf(false);
            regs.set_hf(true);
            regs.set_cf(true);
            regs.b = 0x00;
            regs.c = 0x13;
            regs.d = 0x00;
            regs.e = 0xD8;
            regs.h = 0x01;
            regs.l = 0x4D;
        }

        // Early sanity check before executing anything.
        {
            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                panic!(
                    "ie_push micro lockstep: initial CPU state differs before executing any instructions.\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}",
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                );
            }
        }

        const MAX_INSTRUCTIONS: u64 = 5_000_000;
        // Keep a short warm-up window to hide initial pipeline/pre-fetch
        // differences between our core and mooneye's.
        const WARMUP_INSTRUCTIONS: u64 = 8;
        // For the first few instructions, dump detailed CPU + timer/interrupt
        // state for both cores so we can understand how IE/IF/IME evolve
        // around the early interrupt setup in `ie_push.gb`.
        const DEBUG_INSTRUCTIONS: u64 = 32;
        let mut instr_count: u64 = 0;
        let mut cycles_ours: u64 = 0;
        let mut cycles_ref: u64 = 0;

        while instr_count < MAX_INSTRUCTIONS {
            instr_count = instr_count.saturating_add(1);

            // Step our core for one instruction's worth of work using
            // the micro-step API. We keep calling `step_mcycle` until
            // the CPU reports an instruction boundary via
            // `micro_cycles_remaining == 0`.
            let mut instr_cycles_ours: u64 = 0;
            loop {
                let c = ours.cpu.step_mcycle(&mut ours.bus) as u64;
                if c == 0 {
                    panic!(
                        "ie_push micro lockstep: our CPU returned 0 cycles at instruction {} (possible lock-up)",
                        instr_count
                    );
                }
                instr_cycles_ours = instr_cycles_ours.saturating_add(c);

                if ours.cpu.micro_cycles_remaining == 0 {
                    break;
                }

                // Safety net: no single instruction should exceed a
                // reasonable number of cycles on DMG hardware.
                assert!(
                    instr_cycles_ours <= 80,
                    "ie_push micro lockstep: instruction {} consumed {} T-cycles without completing",
                    instr_count,
                    instr_cycles_ours,
                );
            }
            cycles_ours = cycles_ours.saturating_add(instr_cycles_ours);

            // Step reference core once and accumulate its T-cycle count.
            let before = reference.emu_time().machine_cycles;
            let (_events, _time) = reference.emulate_step();
            let after = reference.emu_time().machine_cycles;
            let delta_mc = after.saturating_sub(before);
            cycles_ref = cycles_ref.saturating_add(delta_mc.saturating_mul(4));

            // Early debug window: mirror the instruction-level lockstep and
            // print both cores' CPU + timer/interrupt state for the initial
            // few instructions. This helps pinpoint how IE/IF/IME and the
            // stack evolve around the first interrupt(s).
            if instr_count <= DEBUG_INSTRUCTIONS {
                let r_my = &ours.cpu.regs;
                let r_ref = reference.regs();
                let f_my = r_my.f & 0xF0;
                let f_ref: u8 = r_ref.f.bits() & 0xF0;

                let (div_my, tima_my, tma_my, tac_my, if_my, ie_my) =
                    ours.bus.debug_timer_snapshot();
                let ovf_my = ours.bus.timer_debug_overflows();
                let (div_ref, tima_ref, tma_ref, tac_ref, if_ref, ie_ref) =
                    reference.hardware_mut().debug_timer_snapshot_for_tests();

                // Peek a couple of bytes at the top of our stack for
                // return-address / IE debugging.
                let sp = r_my.sp;
                let peek = |addr: u16, mem: &[u8; 0x10000]| -> u8 {
                    *mem.get(addr as usize).unwrap_or(&0)
                };
                let s0 = peek(sp, &ours.bus.memory);
                let s1 = peek(sp.wrapping_add(1), &ours.bus.memory);

                let opcode_my = ours.bus.read8(r_my.pc);
                let opcode_ref = ours.bus.read8(r_ref.pc);

                println!(
                    "[ie_push micro dbg] instr={instr} \
ours: PC={pc_my:04X} SP={sp_my:04X} A={a_my:02X} F={f_my:02X} \
DIV={div_my:02X} TIMA={tima_my:02X} TMA={tma_my:02X} TAC={tac_my:02X} \
IF={if_my:02X} IE={ie_my:02X} IME={ime_my} OVF={ovf_my} \
STACK[0]={s0:02X} STACK[1]={s1:02X} OPC={op_my:02X} \
ref: PC={pc_ref:04X} SP={sp_ref:04X} A={a_ref:02X} F={f_ref:02X} \
DIV={div_ref:02X} TIMA={tima_ref:02X} TMA={tma_ref:02X} TAC={tac_ref:02X} \
IF={if_ref:02X} IE={ie_ref:02X} OPC={op_ref:02X}",
                    instr = instr_count,
                    pc_my = r_my.pc,
                    sp_my = r_my.sp,
                    a_my = r_my.a,
                    f_my = f_my,
                    div_my = div_my,
                    tima_my = tima_my,
                    tma_my = tma_my,
                    tac_my = tac_my,
                    if_my = if_my,
                    ie_my = ie_my,
                    ime_my = ours.cpu.ime,
                    ovf_my = ovf_my,
                    s0 = s0,
                    s1 = s1,
                    op_my = opcode_my,
                    pc_ref = r_ref.pc,
                    sp_ref = r_ref.sp,
                    a_ref = r_ref.a,
                    f_ref = f_ref,
                    div_ref = div_ref,
                    tima_ref = tima_ref,
                    tma_ref = tma_ref,
                    tac_ref = tac_ref,
                    if_ref = if_ref,
                    ie_ref = ie_ref,
                    op_ref = opcode_ref,
                );
            }

            // Skip the initial few instructions to allow for pipeline
            // differences, as in the instruction-level lockstep.
            if instr_count <= WARMUP_INSTRUCTIONS {
                continue;
            }

            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            if r_my.pc != r_ref.pc
                || r_my.sp != r_ref.sp
                || r_my.a != r_ref.a
                || f_my != f_ref
                || r_my.b != r_ref.b
                || r_my.c != r_ref.c
                || r_my.d != r_ref.d
                || r_my.e != r_ref.e
                || r_my.h != r_ref.h
                || r_my.l != r_ref.l
            {
                let if_reg = ours.bus.read8(0xFF0F);
                let ie_reg = ours.bus.read8(0xFFFF);

                panic!(
                    "ie_push micro lockstep diverged at instruction {}.\n\
Our cycles (T): {}\n\
Ref cycles (T): {}\n\
Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X} IME={}\n\
Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
Our IF={:02X}, IE={:02X}",
                    instr_count,
                    cycles_ours,
                    cycles_ref,
                    r_my.pc,
                    r_my.sp,
                    r_my.a,
                    f_my,
                    r_my.b,
                    r_my.c,
                    r_my.d,
                    r_my.e,
                    r_my.h,
                    r_my.l,
                    ours.cpu.ime,
                    r_ref.pc,
                    r_ref.sp,
                    r_ref.a,
                    f_ref,
                    r_ref.b,
                    r_ref.c,
                    r_ref.d,
                    r_ref.e,
                    r_ref.h,
                    r_ref.l,
                    if_reg,
                    ie_reg,
                );
            }

            // Stop early once both cores reach the Fibonacci success pattern.
            if r_my.a == 0
                && r_my.b == 3
                && r_my.c == 5
                && r_my.d == 8
                && r_my.e == 13
                && r_my.h == 21
                && r_my.l == 34
            {
                break;
            }
        }

	        if instr_count >= MAX_INSTRUCTIONS {
	            panic!(
	                "ie_push micro lockstep: reached instruction budget ({}) without divergence or success.\n\
	Final our cycles (T): {}\n\
	Final ref cycles (T): {}\n\
	Final our regs: {:?}\n\
	Final ref regs: {:?}",
	                MAX_INSTRUCTIONS,
	                cycles_ours,
	                cycles_ref,
	                ours.cpu.regs,
	                reference.regs(),
	            );
	        }
	    }

	    /// Lockstep comparison between our CPU core and mooneye-gb-core on the
	    /// `ei_timing` interrupt acceptance ROM.
	    ///
	    /// Similar in spirit to the `ie_push` lockstep, but focused on the EI /
	    /// IME enable delay semantics. This helper is intended for manual
	    /// debugging and is marked `#[ignore]` at the test call-site.
	    #[cfg(test)]
	    fn run_ei_timing_lockstep_against_mooneye() {
	        use mooneye_gb_core::config::{Cartridge, HardwareConfig, Model};
	        use mooneye_gb_core::machine::Machine as MooneyeMachine;
	        use std::sync::Arc;

	        let rom = load_mooneye_interrupts_rom("ei_timing.gb");

	        // Our machine: deterministic internal RAM plus a "mooneye-like"
	        // power-on environment (LCDC off, timer/IF cleared).
	        let mut ours = crate::machine::GameBoy::new_zeroed_internal_ram();
	        ours.load_rom(&rom);
	        ours.bus.memory[0xFF40] = 0x00;
	        ours.bus.reset_timer_for_mooneye_tests();
	        ours.bus.if_reg = 0x00;

	        // Reference machine.
	        let data: Arc<[u8]> = Arc::from(rom);
	        let cart = Cartridge::from_data(data).expect("failed to construct mooneye cartridge");
	        let config = HardwareConfig {
	            model: Model::Dmg,
	            bootrom: None,
	            cartridge: cart,
	        };
	        let mut reference = MooneyeMachine::new(config);

	        // Align reference CPU registers with our DMG post-boot snapshot.
	        {
	            let regs = reference.regs_mut();
	            regs.pc = 0x0100;
	            regs.sp = 0xFFFE;
	            regs.a = 0x01;
	            regs.set_zf(true);
	            regs.set_nf(false);
	            regs.set_hf(true);
	            regs.set_cf(true);
	            regs.b = 0x00;
	            regs.c = 0x13;
	            regs.d = 0x00;
	            regs.e = 0xD8;
	            regs.h = 0x01;
	            regs.l = 0x4D;
	        }

	        // Early sanity check before executing anything.
	        {
	            let r_my = &ours.cpu.regs;
	            let r_ref = reference.regs();
	            let f_my = r_my.f & 0xF0;
	            let f_ref: u8 = r_ref.f.bits() & 0xF0;

	            if r_my.pc != r_ref.pc
	                || r_my.sp != r_ref.sp
	                || r_my.a != r_ref.a
	                || f_my != f_ref
	                || r_my.b != r_ref.b
	                || r_my.c != r_ref.c
	                || r_my.d != r_ref.d
	                || r_my.e != r_ref.e
	                || r_my.h != r_ref.h
	                || r_my.l != r_ref.l
	            {
	                panic!(
	                    "ei_timing lockstep: initial CPU state differs before executing any instructions.\n\
	Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
	Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}",
	                    r_my.pc,
	                    r_my.sp,
	                    r_my.a,
	                    f_my,
	                    r_my.b,
	                    r_my.c,
	                    r_my.d,
	                    r_my.e,
	                    r_my.h,
	                    r_my.l,
	                    r_ref.pc,
	                    r_ref.sp,
	                    r_ref.a,
	                    f_ref,
	                    r_ref.b,
	                    r_ref.c,
	                    r_ref.d,
	                    r_ref.e,
	                    r_ref.h,
	                    r_ref.l,
	                );
	            }
	        }

	        const MAX_INSTRUCTIONS: u64 = 200_000;
	        const WARMUP_INSTRUCTIONS: u64 = 32;
	        let mut instr_count: u64 = 0;
	        let mut cycles_ours: u64 = 0;
	        let mut cycles_ref: u64 = 0;

        while instr_count < MAX_INSTRUCTIONS {
            instr_count = instr_count.saturating_add(1);

            // Capture PCs before executing the next instruction so that we
            // can attribute any divergence to a specific opcode.
            let our_pc_before = ours.cpu.regs.pc;
            let ref_pc_before = reference.regs().pc;

            let c_ours = ours.cpu.step(&mut ours.bus) as u64;
            if c_ours == 0 {
                panic!(
                    "ei_timing lockstep: our CPU returned 0 cycles at instruction {} (possible lock-up)",
                    instr_count
                );
	            }
	            cycles_ours = cycles_ours.saturating_add(c_ours);

            let before = reference.emu_time().machine_cycles;
            let (_events, _time) = reference.emulate_step();
            let after = reference.emu_time().machine_cycles;
            let delta_mc = after.saturating_sub(before);
            cycles_ref = cycles_ref.saturating_add(delta_mc.saturating_mul(4));

            let r_my = &ours.cpu.regs;
            let r_ref = reference.regs();
            let f_my = r_my.f & 0xF0;
            let f_ref: u8 = r_ref.f.bits() & 0xF0;

            // Light-weight trace around the beginning of the run to aid
            // debugging. We dump the opcode and PC delta for both cores for
            // the first few dozen instructions so that we can see where
            // their control flow diverges.
            if instr_count <= WARMUP_INSTRUCTIONS.saturating_add(8) {
                let our_opcode = ours.bus.memory[our_pc_before as usize];
                let ref_opcode = ours.bus.memory[ref_pc_before as usize];
                println!(
                    "ei_timing [instr {:03}] our: PC={:04X}->{:04X} OP={:02X}  ref: PC={:04X}->{:04X} OP={:02X}",
                    instr_count,
                    our_pc_before,
                    r_my.pc,
                    our_opcode,
                    ref_pc_before,
                    r_ref.pc,
                    ref_opcode,
                );
            }

            if instr_count <= WARMUP_INSTRUCTIONS {
                continue;
            }

	            if r_my.pc != r_ref.pc
	                || r_my.sp != r_ref.sp
	                || r_my.a != r_ref.a
	                || f_my != f_ref
	                || r_my.b != r_ref.b
	                || r_my.c != r_ref.c
	                || r_my.d != r_ref.d
	                || r_my.e != r_ref.e
	                || r_my.h != r_ref.h
	                || r_my.l != r_ref.l
	            {
	                let (div, tima, tma, tac, if_reg, ie_reg) = ours.bus.debug_timer_snapshot();

	                panic!(
	                    "ei_timing lockstep diverged at instruction {}.\n\
	Our cycles (T): {}\n\
	Ref cycles (T): {}\n\
	Our regs:  PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
	Ref regs: PC={:04X} SP={:04X} A={:02X} F={:02X} B={:02X} C={:02X} D={:02X} E={:02X} H={:02X} L={:02X}\n\
	Our timer regs: DIV={:02X} TIMA={:02X} TMA={:02X} TAC={:02X}, IF={:02X}, IE={:02X}",
	                    instr_count,
	                    cycles_ours,
	                    cycles_ref,
	                    r_my.pc,
	                    r_my.sp,
	                    r_my.a,
	                    f_my,
	                    r_my.b,
	                    r_my.c,
	                    r_my.d,
	                    r_my.e,
	                    r_my.h,
	                    r_my.l,
	                    r_ref.pc,
	                    r_ref.sp,
	                    r_ref.a,
	                    f_ref,
	                    r_ref.b,
	                    r_ref.c,
	                    r_ref.d,
	                    r_ref.e,
	                    r_ref.h,
	                    r_ref.l,
	                    div,
	                    tima,
	                    tma,
	                    tac,
	                    if_reg,
	                    ie_reg,
	                );
	            }
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
        // Optional reference run; this currently serves as a smoke test for
        // wiring the mooneye core, but the timer acceptance ROMs use a
        // different success signalling mechanism than our Fibonacci pattern
        // check, so we do not assert on it here.
        let _ = std::panic::catch_unwind(|| run_mooneye_timer_rom_reference("tim00.gb"));
        run_mooneye_timer_rom("tim00.gb");
        run_mooneye_timer_rom_micro("tim00.gb");
    }

    /// Lockstep variant of the `tim00` test that pinpoints the earliest
    /// divergence against mooneye-gb-core. This is intentionally marked as
    /// `#[ignore]` because it is relatively slow and primarily intended for
    /// manual debugging when working on timer/CPU timing.
    #[test]
    #[ignore]
    fn mooneye_timer_tim00_lockstep() {
        run_tim00_lockstep_against_mooneye();
    }

    /// Optional lockstep debug helper for the `acceptance/interrupts/ie_push`
    /// ROM. This is marked `#[ignore]` because it is relatively slow and is
    /// intended for manual investigation when working on interrupt timing.
    #[test]
    #[ignore]
    fn mooneye_interrupt_ie_push_lockstep() {
        run_ie_push_lockstep_against_mooneye();
    }

    /// Micro-step lockstep variant of the `ie_push` interrupt test.
    ///
    /// This uses `step_mcycle` on our core and compares against
    /// mooneye-gb-core's instruction-level execution.
    #[test]
    #[ignore]
    fn mooneye_interrupt_ie_push_lockstep_micro() {
        run_ie_push_lockstep_against_mooneye_micro();
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

    /// Lockstep debug helper for the `acceptance/interrupts/ei_timing` ROM.
    ///
    /// Like the `ie_push` lockstep, this is marked `#[ignore]` and is meant
    /// for manual use when working on EI/IME enable timing; it compares our
    /// CPU against mooneye-gb-core at instruction granularity.
    #[test]
    #[ignore]
    fn mooneye_interrupt_ei_timing_lockstep() {
        run_ei_timing_lockstep_against_mooneye();
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
}
