use crate::cpu::{Bus, Cpu, Flag};

impl Cpu {
    #[inline]
    fn cc_condition(&self, cc: u8) -> bool {
        match cc {
            0 => !self.get_flag(Flag::Z), // NZ
            1 => self.get_flag(Flag::Z),  // Z
            2 => !self.get_flag(Flag::C), // NC
            3 => self.get_flag(Flag::C),  // C
            _ => false,
        }
    }

    pub(super) fn exec_jr_cc<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0x20 | 0x28 | 0x30 | 0x38));
        let cc = (opcode >> 3) & 0x03;
        self.jr(bus, self.cc_condition(cc))
    }

    pub(super) fn exec_jp_cc<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xC2 | 0xCA | 0xD2 | 0xDA));
        let cc = (opcode >> 3) & 0x03;
        self.jp_cond(bus, self.cc_condition(cc))
    }

    pub(super) fn exec_jp_a16<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let addr = self.fetch16(bus);
        self.regs.pc = addr;
        16
    }

    pub(super) fn exec_jp_hl(&mut self) -> u32 {
        let addr = self.regs.hl();
        self.regs.pc = addr;
        4
    }

    pub(super) fn exec_call_cc<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xC4 | 0xCC | 0xD4 | 0xDC));
        let cc = (opcode >> 3) & 0x03;
        self.call_cond(bus, self.cc_condition(cc))
    }

    pub(super) fn exec_ret_cc<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xC0 | 0xC8 | 0xD0 | 0xD8));
        let cc = (opcode >> 3) & 0x03;
        self.ret_cond(bus, self.cc_condition(cc))
    }
}
