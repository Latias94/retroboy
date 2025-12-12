//! Micro-op helpers for cycle-level CPU stepping.
//!
//! This module contains small state machines used by `step_mcycle` to
//! execute selected instructions (e.g. CALL/RET) across multiple
//! M-cycles while preserving the same semantics as the monolithic
//! `exec_opcode` implementation used by `step`.

use crate::cpu::{Bus, Cpu};

/// Kind of instruction currently being executed via the micro-step API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MicroInstrKind {
    None,
    CallA16,
    Ret,
    RetCond,
    JpA16,
    JpCondA16,
    LdSpA16,
    Rst,
    Jr,
    JrCond,
    CallCond,
    /// LDH A,(a8)
    LdhAFromA8,
    /// LDH (a8),A
    LdhAToA8,
}

impl Cpu {
    /// Helper used by micro-ops to advance one M-cycle worth of time.
    ///
    /// `timer_used_this_cycle` should be set to `true` when the current
    /// micro-op M-cycle has already called into a Timer IO helper
    /// (`timer_cycle_read`/`timer_cycle_write`), which internally ticks
    /// the Timer once. In that case we avoid ticking the Timer a second
    /// time here.
    #[inline]
    fn tick_mcycle_with_timer<B: Bus>(&mut self, bus: &mut B, timer_used_this_cycle: bool) {
        bus.tick_mcycle_generic();
        if !timer_used_this_cycle {
            bus.timer_tick_mcycle();
        }
    }

    /// Execute one M-cycle (4 T-cycles) worth of work for the current
    /// micro-stepped instruction, if any.
    pub(crate) fn step_instruction_mcycle<B: Bus>(&mut self, bus: &mut B) {
        match self.micro_instr {
            MicroInstrKind::CallA16 => self.step_call_a16_mcycle(bus),
            MicroInstrKind::Ret => self.step_ret_mcycle(bus),
            MicroInstrKind::RetCond => self.step_ret_cond_mcycle(bus),
            MicroInstrKind::JpA16 | MicroInstrKind::JpCondA16 => self.step_jp_a16_mcycle(bus),
            MicroInstrKind::LdSpA16 => self.step_ld_sp_a16_mcycle(bus),
            MicroInstrKind::Rst => self.step_rst_mcycle(bus),
            MicroInstrKind::Jr | MicroInstrKind::JrCond => self.step_jr_mcycle(bus),
            MicroInstrKind::CallCond => self.step_call_cond_mcycle(bus),
             MicroInstrKind::LdhAFromA8 => self.step_ldh_a_from_a8_mcycle(bus),
             MicroInstrKind::LdhAToA8 => self.step_ldh_a_to_a8_mcycle(bus),
            MicroInstrKind::None => {
                // Fallback: no instruction-specific micro-ops; just tick
                // the bus for one M-cycle.
                bus.tick(4);
                return;
            }
        }

        self.micro_stage = self.micro_stage.saturating_add(1);
    }

    fn step_call_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch was performed by `fetch8` in
                // `step_mcycle` before we switched into the CALL
                // micro-sequence. Nothing more to do in this M-cycle.
            }
            1 => {
                // M2: read low byte of target address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of target address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: internal delay M-cycle before stack write. No bus
                // access here; we just account for time via `bus.tick`.
            }
            4 => {
                // M5: push high byte of return address.
                let ret = self.regs.pc;
                let hi = (ret >> 8) as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, hi);
            }
            5 => {
                // M6: push low byte of return address and jump.
                let ret = self.regs.pc;
                let lo = ret as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, lo);
                self.regs.pc = self.micro_imm16;
            }
            _ => {}
        }
        // Account for one M-cycle worth of time.
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_ret_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: read low byte of return address from stack.
                let lo = bus.read8(self.regs.sp);
                self.regs.sp = self.regs.sp.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of return address from stack.
                let hi = bus.read8(self.regs.sp);
                self.regs.sp = self.regs.sp.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: load PC from the popped address.
                self.regs.pc = self.micro_imm16;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_jp_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: read low byte of target address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of target address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: jump to the loaded address.
                self.regs.pc = self.micro_imm16;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_ret_cond_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: condition evaluation / internal delay.
            }
            2 => {
                // M3: read low byte of return address from stack (if taken).
                if self.micro_cond_taken {
                    let lo = bus.read8(self.regs.sp);
                    self.regs.sp = self.regs.sp.wrapping_add(1);
                    self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
                }
            }
            3 => {
                // M4: read high byte of return address from stack (if taken).
                if self.micro_cond_taken {
                    let hi = bus.read8(self.regs.sp);
                    self.regs.sp = self.regs.sp.wrapping_add(1);
                    self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
                }
            }
            4 => {
                // M5: load PC from the popped address (if taken).
                if self.micro_cond_taken {
                    self.regs.pc = self.micro_imm16;
                }
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_ld_sp_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: read low byte of destination address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of destination address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: write low byte of SP to (a16).
                let addr = self.micro_imm16;
                let sp = self.regs.sp;
                let lo = sp as u8;
                bus.write8(addr, lo);
            }
            4 => {
                // M5: write high byte of SP to (a16+1).
                let addr = self.micro_imm16.wrapping_add(1);
                let sp = self.regs.sp;
                let hi = (sp >> 8) as u8;
                bus.write8(addr, hi);
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_rst_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: internal delay.
            }
            2 => {
                // M3: push high byte of return address.
                let ret = self.regs.pc;
                let hi = (ret >> 8) as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, hi);
            }
            3 => {
                // M4: push low byte and jump to vector.
                let ret = self.regs.pc;
                let lo = ret as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, lo);
                self.regs.pc = self.micro_imm16;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_jr_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: read 8-bit signed offset.
                let off = bus.read8(self.regs.pc) as i8;
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = off as i16 as u16;
            }
            2 => {
                // M3: apply offset if the condition is taken.
                if self.micro_cond_taken {
                    let pc = self.regs.pc.wrapping_add(self.micro_imm16);
                    self.regs.pc = pc;
                }
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    fn step_call_cond_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled in `step_mcycle`.
            }
            1 => {
                // M2: read low byte of target address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of target address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: internal delay before potential stack write.
            }
            4 => {
                // M5: if taken, push high byte of return address.
                if self.micro_cond_taken {
                    let ret = self.regs.pc;
                    let hi = (ret >> 8) as u8;
                    self.regs.sp = self.regs.sp.wrapping_sub(1);
                    bus.write8(self.regs.sp, hi);
                }
            }
            5 => {
                // M6: if taken, push low byte and jump.
                if self.micro_cond_taken {
                    let ret = self.regs.pc;
                    let lo = ret as u8;
                    self.regs.sp = self.regs.sp.wrapping_sub(1);
                    bus.write8(self.regs.sp, lo);
                    self.regs.pc = self.micro_imm16;
                }
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LDH A,(a8)` (opcode 0xF0, 12 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: fetch 8-bit offset (a8)
    ///   M3: read from 0xFF00 + a8 into A
    fn step_ldh_a_from_a8_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done by `step_mcycle`.
            }
            1 => {
                // M2: read 8-bit offset.
                let off = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = off as u16;
            }
            2 => {
                // M3: read from high memory 0xFF00 + a8.
                let addr = 0xFF00u16.wrapping_add(self.micro_imm16);
                let value = if (0xFF04..=0xFF07).contains(&addr) {
                    // Timer IO: model as a dedicated timer cycle. The Timer
                    // core is advanced inside `timer_cycle_read`, so we skip
                    // the generic timer tick for this M-cycle.
                    timer_used = true;
                    bus.timer_cycle_read(addr)
                } else {
                    bus.read8(addr)
                };
                self.regs.a = value;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LDH (a8),A` (opcode 0xE0, 12 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: fetch 8-bit offset (a8)
    ///   M3: write A to 0xFF00 + a8
    fn step_ldh_a_to_a8_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done.
            }
            1 => {
                // M2: read 8-bit offset.
                let off = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = off as u16;
            }
            2 => {
                // M3: write A to high memory 0xFF00 + a8.
                let addr = 0xFF00u16.wrapping_add(self.micro_imm16);
                if (0xFF04..=0xFF07).contains(&addr) {
                    // Timer IO cycle; Timer core is advanced internally.
                    timer_used = true;
                    bus.timer_cycle_write(addr, self.regs.a);
                } else {
                    bus.write8(addr, self.regs.a);
                }
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }
}
