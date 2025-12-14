use crate::cpu::{Bus, Cpu};

fn mcycle_io_offset(env_key: &str, default: u8) -> u32 {
    std::env::var(env_key)
        .ok()
        .and_then(|v| v.parse::<u8>().ok())
        .unwrap_or(default)
        .min(3) as u32
}

impl Cpu {
    /// Micro-ops for `LD (HL),r` (8 T-cycles when r != (HL)).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: write register value to (HL)
    pub(super) fn step_ld_hl_from_r_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done.
            }
            1 => {
                // M2: write register to (HL).
                let addr = self.regs.hl();
                let src = (self.micro_imm16 & 0x00FF) as u8;
                let value = match src {
                    0 => self.regs.b,
                    1 => self.regs.c,
                    2 => self.regs.d,
                    3 => self.regs.e,
                    4 => self.regs.h,
                    5 => self.regs.l,
                    7 => self.regs.a,
                    _ => 0,
                };

                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    bus.timer_cycle_write(addr, value);
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                // Memory write occurs during M2; allow an offset within the
                // M-cycle so cycle-sensitive tests can tune the edge.
                let offset = mcycle_io_offset("RETROBOY_GB_LD_HL_WRITE_OFFSET", 0);
                bus.tick(offset);
                bus.write8(addr, value);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LD r,(HL)` (8 T-cycles when r != (HL)).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: read (HL) into register
    pub(super) fn step_ld_r_from_hl_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done.
            }
            1 => {
                // M2: read (HL) into destination register.
                let addr = self.regs.hl();
                let dst = (self.micro_imm16 & 0x00FF) as u8;

                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    let v = bus.timer_cycle_read(addr);
                    match dst {
                        0 => self.regs.b = v,
                        1 => self.regs.c = v,
                        2 => self.regs.d = v,
                        3 => self.regs.e = v,
                        4 => self.regs.h = v,
                        5 => self.regs.l = v,
                        7 => self.regs.a = v,
                        _ => {}
                    }
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LD_HL_READ_OFFSET", 3);
                bus.tick(offset);
                let v = bus.read8(addr);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                match dst {
                    0 => self.regs.b = v,
                    1 => self.regs.c = v,
                    2 => self.regs.d = v,
                    3 => self.regs.e = v,
                    4 => self.regs.h = v,
                    5 => self.regs.l = v,
                    7 => self.regs.a = v,
                    _ => {}
                }
                return;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    pub(super) fn step_call_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    pub(super) fn step_ret_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_jp_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_ret_cond_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_ld_sp_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_rst_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_jr_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    pub(super) fn step_call_cond_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let timer_used = false;
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

    /// Micro-ops for `LD (a16),A` (opcode 0xEA, 16 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: fetch low byte of address
    ///   M3: fetch high byte of address
    ///   M4: write A to (a16)
    pub(super) fn step_ld_a_to_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled by `step_mcycle`.
            }
            1 => {
                // M2: read low byte of address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: write A to address.
                let addr = self.micro_imm16;
                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    bus.timer_cycle_write(addr, self.regs.a);
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LD_A16_WRITE_OFFSET", 0);
                bus.tick(offset);
                bus.write8(addr, self.regs.a);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LD A,(a16)` (opcode 0xFA, 16 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: fetch low byte of address
    ///   M3: fetch high byte of address
    ///   M4: read from (a16) into A
    pub(super) fn step_ld_a_from_a16_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch handled by `step_mcycle`.
            }
            1 => {
                // M2: read low byte of address.
                let lo = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = (self.micro_imm16 & 0xFF00) | (lo as u16);
            }
            2 => {
                // M3: read high byte of address.
                let hi = bus.read8(self.regs.pc);
                self.regs.pc = self.regs.pc.wrapping_add(1);
                self.micro_imm16 = ((hi as u16) << 8) | (self.micro_imm16 & 0x00FF);
            }
            3 => {
                // M4: read from address.
                let addr = self.micro_imm16;
                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    self.regs.a = bus.timer_cycle_read(addr);
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LD_A16_READ_OFFSET", 3);
                bus.tick(offset);
                self.regs.a = bus.read8(addr);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
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
    pub(super) fn step_ldh_a_from_a8_mcycle<B: Bus>(&mut self, bus: &mut B) {
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
                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    self.regs.a = bus.timer_cycle_read(addr);
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LDH_A8_READ_OFFSET", 3);
                bus.tick(offset);
                self.regs.a = bus.read8(addr);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
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
    pub(super) fn step_ldh_a_to_a8_mcycle<B: Bus>(&mut self, bus: &mut B) {
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
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LDH_A8_WRITE_OFFSET", 0);
                bus.tick(offset);
                bus.write8(addr, self.regs.a);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LD A,(C)` (opcode 0xF2, 8 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: read from 0xFF00 + C into A
    pub(super) fn step_ldh_a_from_c_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done.
            }
            1 => {
                // M2: read from high memory 0xFF00 + C.
                let addr = 0xFF00u16.wrapping_add(self.regs.c as u16);
                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    self.regs.a = bus.timer_cycle_read(addr);
                    self.tick_mcycle_with_timer(bus, timer_used);
                    return;
                }

                let offset = mcycle_io_offset("RETROBOY_GB_LDH_C_READ_OFFSET", 3);
                bus.tick(offset);
                self.regs.a = bus.read8(addr);
                bus.tick(4 - offset);
                bus.timer_tick_mcycle();
                return;
            }
            _ => {}
        }
        self.tick_mcycle_with_timer(bus, timer_used);
    }

    /// Micro-ops for `LD (C),A` (opcode 0xE2, 8 T-cycles).
    ///
    /// Timing (M-cycles):
    ///   M1: opcode fetch (handled in `step_mcycle`)
    ///   M2: write A to 0xFF00 + C
    pub(super) fn step_ldh_a_to_c_mcycle<B: Bus>(&mut self, bus: &mut B) {
        let mut timer_used = false;
        match self.micro_stage {
            0 => {
                // M1: opcode fetch already done.
                self.tick_mcycle_with_timer(bus, false);
                return;
            }
            1 => {
                // M2: write A to high memory 0xFF00 + C.
                let offset = std::env::var("RETROBOY_GB_LDH_C_WRITE_OFFSET")
                    .ok()
                    .and_then(|v| v.parse::<u8>().ok())
                    .unwrap_or(0)
                    .min(3) as u32;

                bus.tick(offset);
                let addr = 0xFF00u16.wrapping_add(self.regs.c as u16);
                if (0xFF04..=0xFF07).contains(&addr) {
                    timer_used = true;
                    bus.timer_cycle_write(addr, self.regs.a);
                } else {
                    bus.write8(addr, self.regs.a);
                }
                bus.tick(4 - offset);
            }
            _ => return,
        }
        if !timer_used {
            bus.timer_tick_mcycle();
        }
    }
}
