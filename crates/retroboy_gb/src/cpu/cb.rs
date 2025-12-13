use super::{Bus, Cpu, Flag};

impl Cpu {
    /// Handle CB-prefixed instructions (bit operations, shifts, and rotates).
    pub(super) fn step_cb<B: Bus>(&mut self, bus: &mut B) -> u32 {
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
}

