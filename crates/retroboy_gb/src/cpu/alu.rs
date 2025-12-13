use super::{Cpu, Flag};

impl Cpu {
    /// Core 8-bit ADD/ADC operation on A.
    ///
    /// `use_carry` selects between ADD (false) and ADC (true).
    pub(super) fn alu_add(&mut self, value: u8, use_carry: bool) {
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
    pub(super) fn alu_sub(&mut self, value: u8, use_carry: bool) {
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
    pub(super) fn alu_and(&mut self, value: u8) {
        let result = self.regs.a & value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
        self.set_flag(Flag::H, true);
        // N and C are already cleared.
    }

    #[inline]
    pub(super) fn alu_or(&mut self, value: u8) {
        let result = self.regs.a | value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
    }

    #[inline]
    pub(super) fn alu_xor(&mut self, value: u8) {
        let result = self.regs.a ^ value;
        self.regs.a = result;

        self.clear_flags();
        self.set_flag(Flag::Z, result == 0);
    }

    /// Compare A with `value`, setting flags as if `A - value` was performed.
    /// A itself is not modified.
    #[inline]
    pub(super) fn alu_cp(&mut self, value: u8) {
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
    pub(super) fn alu_daa(&mut self) {
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
    pub(super) fn alu_inc8(&mut self, value: u8) -> u8 {
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
    pub(super) fn alu_dec8(&mut self, value: u8) -> u8 {
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
    pub(super) fn alu_add16_hl(&mut self, value: u16) {
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
    pub(super) fn alu_add16_signed(&mut self, base: u16, imm8: u8) -> u16 {
        let offset = imm8 as i8 as i16 as u16;
        self.set_flag(Flag::N, false);
        self.set_flag(Flag::Z, false);
        self.set_flag(Flag::H, (base & 0x000F) + (offset & 0x000F) > 0x000F);
        self.set_flag(Flag::C, (base & 0x00FF) + (offset & 0x00FF) > 0x00FF);
        base.wrapping_add(offset)
    }
}

