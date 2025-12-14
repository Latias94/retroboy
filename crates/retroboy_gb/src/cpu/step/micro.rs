use crate::cpu_micro::MicroInstrKind;

use super::super::{Bus, Cpu, Flag};

impl Cpu {
    /// Advance the CPU and bus by (up to) a single M-cycle (4 T-cycles).
    ///
    /// This experimental micro-step API preserves the same instruction-level
    /// semantics as `step`, but spreads `bus.tick` and the IME delay pipeline
    /// across individual 4-cycle chunks. It is currently unused by the
    /// higher-level machine and exists as a scaffold for future
    /// cycle-accurate work.
    ///
    /// The return value is the number of T-cycles consumed (usually 4, but
    /// it can be 0 when the CPU is locked or an invalid opcode has just
    /// hard-locked the core).
    pub fn step_mcycle<B: Bus>(&mut self, bus: &mut B) -> u32 {
        if self.locked {
            // Hard‑locked cores never advance time.
            return 0;
        }

        // STOP low‑power mode: identical to the `step` implementation, we
        // poll P1 ($FF00) on each call and do not tick the bus so that
        // timers/PPU remain frozen.
        // CGB KEY1+STOP speed switch pause: the CPU is paused while the LCD
        // controller continues to advance, and DIV/TIMA remain frozen.
        if bus.cgb_speed_switch_pause_mcycle() {
            return 4;
        }

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
                bus.tick_mcycle_generic();
                bus.timer_tick_mcycle();
                return 4;
            } else {
                // Normal instruction path: fetch the opcode and either
                // hand it off to a dedicated micro-op sequence or fall
                // back to the monolithic `exec_opcode` semantics.
                let (opcode, prefetched_m1) = match self.micro_prefetched_opcode.take() {
                    Some(opcode) => {
                        // The opcode was already read during the final
                        // interrupt-entry M-cycle. Mirror `fetch8` by
                        // advancing PC here so subsequent immediate reads
                        // start at the correct address.
                        if self.halt_bug {
                            self.halt_bug = false;
                        } else {
                            self.regs.pc = self.regs.pc.wrapping_add(1);
                        }
                        (opcode, true)
                    },
                    None => (self.fetch8(bus), false),
                };
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
                    // LD (HL),r where r != (HL) (8 T-cycles).
                    0x70..=0x75 | 0x77 => {
                        let src = opcode & 0x07;
                        self.micro_instr = MicroInstrKind::LdHlFromR;
                        self.micro_stage = 0;
                        self.micro_imm16 = src as u16;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 8;
                        self.micro_from_interrupt = false;
                    }
                    // LD r,(HL) where r != (HL) (8 T-cycles).
                    0x46 | 0x4E | 0x56 | 0x5E | 0x66 | 0x6E | 0x7E => {
                        let dst = (opcode >> 3) & 0x07;
                        self.micro_instr = MicroInstrKind::LdRFromHl;
                        self.micro_stage = 0;
                        self.micro_imm16 = dst as u16;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 8;
                        self.micro_from_interrupt = false;
                    }
                    // 0xEA: LD (a16),A (16 T-cycles).
                    0xEA => {
                        self.micro_instr = MicroInstrKind::LdAToA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 16;
                        self.micro_from_interrupt = false;
                    }
                    // 0xFA: LD A,(a16) (16 T-cycles).
                    0xFA => {
                        self.micro_instr = MicroInstrKind::LdAFromA16;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 16;
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
                    // 0xF2: LD A,(C) (8 T-cycles).
                    0xF2 => {
                        self.micro_instr = MicroInstrKind::LdhAFromC;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 8;
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
                    // 0xE2: LD (C),A (8 T-cycles).
                    0xE2 => {
                        self.micro_instr = MicroInstrKind::LdhAToC;
                        self.micro_stage = 0;
                        self.micro_imm16 = 0;
                        self.micro_cond_taken = true;
                        self.micro_cycles_remaining = 8;
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

                // If the opcode fetch (M1) was already performed during the
                // final M-cycle of interrupt entry, start the instruction at
                // M2 (skip the stage-0 tick) and reduce the remaining cycle
                // budget accordingly.
                if prefetched_m1 && self.micro_cycles_remaining >= 8 {
                    self.micro_cycles_remaining = self.micro_cycles_remaining.saturating_sub(4);
                    if self.micro_instr != MicroInstrKind::None {
                        self.micro_stage = 1;
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
}
