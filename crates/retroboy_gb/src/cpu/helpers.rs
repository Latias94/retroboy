use super::{Bus, Cpu};

impl Cpu {
    /// Helper to read an 8-bit register or (HL) by index.
    ///
    /// The encoding matches the standard Game Boy register order used by
    /// opcode tables:
    /// 0=B, 1=C, 2=D, 3=E, 4=H, 5=L, 6=(HL), 7=A.
    #[inline]
    pub(super) fn read_reg8<B: Bus>(&mut self, bus: &mut B, index: u8) -> u8 {
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
    pub(super) fn write_reg8<B: Bus>(&mut self, bus: &mut B, index: u8, value: u8) {
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
    pub(super) fn fetch8<B: Bus>(&mut self, bus: &mut B) -> u8 {
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
    pub(super) fn fetch16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = self.fetch8(bus) as u16;
        let hi = self.fetch8(bus) as u16;
        (hi << 8) | lo
    }

    #[inline]
    pub(super) fn push_u16<B: Bus>(&mut self, bus: &mut B, value: u16) {
        let lo = value as u8;
        let hi = (value >> 8) as u8;
        // Stack grows downward. We want memory[SP] = low, memory[SP+1] = high.
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, hi);
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, lo);
    }

    #[inline]
    pub(super) fn pop_u16<B: Bus>(&mut self, bus: &mut B) -> u16 {
        let lo = bus.read8(self.regs.sp) as u16;
        let hi = bus.read8(self.regs.sp.wrapping_add(1)) as u16;
        self.regs.sp = self.regs.sp.wrapping_add(2);
        (hi << 8) | lo
    }

    /// Perform a single bus read cycle at the given address and advance the
    /// rest of the machine by one CPU T-cycle.
    ///
    /// This helper is the building block for a future per-cycle CPU/bus
    /// integration model. The current instruction-level `step` API still
    /// uses `bus.read8` + `bus.tick(total_cycles)`; new micro-step paths
    /// can gradually migrate to these helpers instead.
    #[inline]
    #[allow(dead_code)]
    pub(super) fn read_cycle<B: Bus>(&mut self, bus: &mut B, addr: u16) -> u8 {
        let value = bus.read8(addr);
        bus.tick(1);
        value
    }

    /// Perform a single bus write cycle at the given address and advance the
    /// rest of the machine by one CPU T-cycle.
    #[inline]
    #[allow(dead_code)]
    pub(super) fn write_cycle<B: Bus>(&mut self, bus: &mut B, addr: u16, value: u8) {
        bus.write8(addr, value);
        bus.tick(1);
    }

    /// Idle for one CPU T-cycle without performing a memory access.
    #[inline]
    #[allow(dead_code)]
    pub(super) fn idle_cycle<B: Bus>(&mut self, bus: &mut B) {
        bus.tick(1);
    }

    /// Relative jump helper used by JR/JR cc.
    ///
    /// The displacement is a signed 8-bit offset relative to the address
    /// following the operand.
    pub(super) fn jr<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
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
    pub(super) fn jp_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        let addr = self.fetch16(bus);
        if cond {
            self.regs.pc = addr;
            16
        } else {
            12
        }
    }

    /// Conditional call helper used by CALL cc,a16.
    pub(super) fn call_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
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
    pub(super) fn ret_cond<B: Bus>(&mut self, bus: &mut B, cond: bool) -> u32 {
        if cond {
            let addr = self.pop_u16(bus);
            self.regs.pc = addr;
            20
        } else {
            8
        }
    }
}

