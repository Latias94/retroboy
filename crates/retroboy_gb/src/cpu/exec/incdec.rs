use crate::cpu::{Bus, Cpu};

impl Cpu {
    pub(super) fn exec_inc8_reg<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(
            matches!(opcode, 0x04 | 0x0C | 0x14 | 0x1C | 0x24 | 0x2C | 0x34 | 0x3C),
            "unexpected INC r opcode {opcode:#04x}"
        );

        let reg = (opcode >> 3) & 0x07;
        let value = self.read_reg8(bus, reg);
        let result = self.alu_inc8(value);
        self.write_reg8(bus, reg, result);

        if reg == 6 { 12 } else { 4 }
    }

    pub(super) fn exec_dec8_reg<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(
            matches!(opcode, 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D),
            "unexpected DEC r opcode {opcode:#04x}"
        );

        let reg = (opcode >> 3) & 0x07;
        let value = self.read_reg8(bus, reg);
        let result = self.alu_dec8(value);
        self.write_reg8(bus, reg, result);

        if reg == 6 { 12 } else { 4 }
    }

    pub(super) fn exec_inc16_rr(&mut self, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x03 | 0x13 | 0x23 | 0x33));
        match opcode {
            0x03 => {
                let value = self.regs.bc().wrapping_add(1);
                self.regs.set_bc(value);
            }
            0x13 => {
                let value = self.regs.de().wrapping_add(1);
                self.regs.set_de(value);
            }
            0x23 => {
                let value = self.regs.hl().wrapping_add(1);
                self.regs.set_hl(value);
            }
            0x33 => {
                self.regs.sp = self.regs.sp.wrapping_add(1);
            }
            _ => unreachable!(),
        }
        8
    }

    pub(super) fn exec_dec16_rr(&mut self, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x0B | 0x1B | 0x2B | 0x3B));
        match opcode {
            0x0B => {
                let value = self.regs.bc().wrapping_sub(1);
                self.regs.set_bc(value);
            }
            0x1B => {
                let value = self.regs.de().wrapping_sub(1);
                self.regs.set_de(value);
            }
            0x2B => {
                let value = self.regs.hl().wrapping_sub(1);
                self.regs.set_hl(value);
            }
            0x3B => {
                self.regs.sp = self.regs.sp.wrapping_sub(1);
            }
            _ => unreachable!(),
        }
        8
    }
}
