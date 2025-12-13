use crate::cpu::{Bus, Cpu, Flag};

impl Cpu {
    pub(super) fn exec_alu_reg_group<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!((0x80..=0xBF).contains(&opcode));
        let operation = (opcode >> 3) & 0x07;
        let src = opcode & 0x07;
        let value = self.read_reg8(bus, src);

        match operation {
            0 => self.alu_add(value, false),
            1 => self.alu_add(value, true),
            2 => self.alu_sub(value, false),
            3 => self.alu_sub(value, true),
            4 => self.alu_and(value),
            5 => self.alu_xor(value),
            6 => self.alu_or(value),
            7 => self.alu_cp(value),
            _ => unreachable!(),
        }

        if src == 6 { 8 } else { 4 }
    }

    pub(super) fn exec_alu_imm<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        let value = self.fetch8(bus);

        match opcode {
            0xC6 => self.alu_add(value, false),
            0xCE => self.alu_add(value, true),
            0xD6 => self.alu_sub(value, false),
            0xDE => self.alu_sub(value, true),
            0xE6 => self.alu_and(value),
            0xEE => self.alu_xor(value),
            0xF6 => self.alu_or(value),
            0xFE => self.alu_cp(value),
            _ => unreachable!(),
        }

        8
    }

    pub(super) fn exec_rotate_a(&mut self, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x07 | 0x0F | 0x17 | 0x1F));

        match opcode {
            0x07 => {
                // RLCA: rotate A left. Bit 7 to Carry and bit 0.
                let a = self.regs.a;
                let result = a.rotate_left(1);
                self.regs.a = result;
                self.set_flag(Flag::C, (a & 0x80) != 0);
                self.set_flag(Flag::Z, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
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
            }
            _ => unreachable!(),
        }

        4
    }

    pub(super) fn exec_add_hl_rr(&mut self, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x09 | 0x19 | 0x29 | 0x39));
        let value = match opcode {
            0x09 => self.regs.bc(),
            0x19 => self.regs.de(),
            0x29 => self.regs.hl(),
            0x39 => self.regs.sp,
            _ => unreachable!(),
        };
        self.alu_add16_hl(value);
        8
    }

    pub(super) fn exec_add_sp_r8<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let imm = self.fetch8(bus);
        let result = self.alu_add16_signed(self.regs.sp, imm);
        self.regs.sp = result;
        16
    }

    pub(super) fn exec_ld_hl_sp_r8<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let imm = self.fetch8(bus);
        let base = self.regs.sp;
        let result = self.alu_add16_signed(base, imm);
        self.regs.set_hl(result);
        12
    }

    pub(super) fn exec_daa(&mut self) -> u32 {
        self.alu_daa();
        4
    }

    pub(super) fn exec_cpl(&mut self) -> u32 {
        self.regs.a = !self.regs.a;
        self.set_flag(Flag::H, true);
        self.set_flag(Flag::N, true);
        4
    }

    pub(super) fn exec_scf(&mut self) -> u32 {
        self.set_flag(Flag::C, true);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::N, false);
        4
    }

    pub(super) fn exec_ccf(&mut self) -> u32 {
        let carry = self.get_flag(Flag::C);
        self.set_flag(Flag::C, !carry);
        self.set_flag(Flag::H, false);
        self.set_flag(Flag::N, false);
        4
    }
}
