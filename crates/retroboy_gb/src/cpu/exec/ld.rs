use crate::cpu::{Bus, Cpu};

impl Cpu {
    pub(super) fn exec_ld_rr_d16<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x01 | 0x11 | 0x21 | 0x31));

        let value = self.fetch16(bus);
        let rp = (opcode >> 4) & 0x03;
        match rp {
            0 => self.regs.set_bc(value),
            1 => self.regs.set_de(value),
            2 => self.regs.set_hl(value),
            3 => self.regs.sp = value,
            _ => unreachable!(),
        }

        12
    }

    pub(super) fn exec_ld_r_d8<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(
            opcode,
            0x06 | 0x0E | 0x16 | 0x1E | 0x26 | 0x2E | 0x36 | 0x3E
        ));

        let reg = (opcode >> 3) & 0x07;
        let value = self.fetch8(bus);
        self.write_reg8(bus, reg, value);

        if reg == 6 { 12 } else { 8 }
    }

    pub(super) fn exec_ld_a16_sp<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let addr = self.fetch16(bus);
        let sp = self.regs.sp;
        let lo = sp as u8;
        let hi = (sp >> 8) as u8;
        bus.write8(addr, lo);
        bus.write8(addr.wrapping_add(1), hi);
        20
    }

    pub(super) fn exec_ldh_a8<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xE0 | 0xF0));

        let offset = self.fetch8(bus) as u16;
        let addr = 0xFF00u16.wrapping_add(offset);
        match opcode {
            0xE0 => bus.write8(addr, self.regs.a),
            0xF0 => self.regs.a = bus.read8(addr),
            _ => unreachable!(),
        }
        12
    }

    pub(super) fn exec_ldh_c<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xE2 | 0xF2));

        let addr = 0xFF00u16.wrapping_add(self.regs.c as u16);
        match opcode {
            0xE2 => bus.write8(addr, self.regs.a),
            0xF2 => self.regs.a = bus.read8(addr),
            _ => unreachable!(),
        }
        8
    }

    pub(super) fn exec_ld_a16_a<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xEA | 0xFA));

        let addr = self.fetch16(bus);
        match opcode {
            0xEA => bus.write8(addr, self.regs.a),
            0xFA => self.regs.a = bus.read8(addr),
            _ => unreachable!(),
        }
        16
    }

    pub(super) fn exec_ld_indirect_a<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x02 | 0x12 | 0x22 | 0x32));

        let rp = (opcode >> 4) & 0x03;
        let addr = match rp {
            0 => self.regs.bc(),
            1 => self.regs.de(),
            2 | 3 => self.regs.hl(),
            _ => unreachable!(),
        };

        bus.write8(addr, self.regs.a);

        match rp {
            2 => self.regs.set_hl(addr.wrapping_add(1)),
            3 => self.regs.set_hl(addr.wrapping_sub(1)),
            _ => {}
        }

        8
    }

    pub(super) fn exec_ld_a_indirect<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x0A | 0x1A | 0x2A | 0x3A));

        let rp = (opcode >> 4) & 0x03;
        let addr = match rp {
            0 => self.regs.bc(),
            1 => self.regs.de(),
            2 | 3 => self.regs.hl(),
            _ => unreachable!(),
        };

        self.regs.a = bus.read8(addr);

        match rp {
            2 => self.regs.set_hl(addr.wrapping_add(1)),
            3 => self.regs.set_hl(addr.wrapping_sub(1)),
            _ => {}
        }

        8
    }

    pub(super) fn exec_ld_rr_or_halt<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!((0x40..=0x7F).contains(&opcode));

        if opcode == 0x76 {
            // HALT
            if !self.ime {
                // Possible HALT bug: if interrupts are pending while IME is
                // clear, the CPU does not actually halt, and instead the next
                // opcode fetch does not increment PC.
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
            return 4;
        }

        let dst = (opcode >> 3) & 0x07;
        let src = opcode & 0x07;
        let value = self.read_reg8(bus, src);
        self.write_reg8(bus, dst, value);

        if dst == 6 || src == 6 { 8 } else { 4 }
    }

    pub(super) fn exec_ld_sp_hl(&mut self) -> u32 {
        let hl = self.regs.hl();
        self.regs.sp = hl;
        8
    }
}
