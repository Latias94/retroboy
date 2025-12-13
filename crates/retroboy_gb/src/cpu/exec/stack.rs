use crate::cpu::{Bus, Cpu};

impl Cpu {
    pub(super) fn exec_push_rr<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xC5 | 0xD5 | 0xE5 | 0xF5));

        let rp2 = (opcode >> 4) & 0x03;
        let value = match rp2 {
            0 => self.regs.bc(),
            1 => self.regs.de(),
            2 => self.regs.hl(),
            3 => self.regs.af(),
            _ => unreachable!(),
        };

        self.push_u16(bus, value);
        16
    }

    pub(super) fn exec_pop_rr<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(opcode, 0xC1 | 0xD1 | 0xE1 | 0xF1));

        let value = self.pop_u16(bus);
        let rp2 = (opcode >> 4) & 0x03;
        match rp2 {
            0 => self.regs.set_bc(value),
            1 => self.regs.set_de(value),
            2 => self.regs.set_hl(value),
            3 => self.regs.set_af(value),
            _ => unreachable!(),
        }

        12
    }

    pub(super) fn exec_rst<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        debug_assert!(matches!(
            opcode,
            0xC7 | 0xCF | 0xD7 | 0xDF | 0xE7 | 0xEF | 0xF7 | 0xFF
        ));

        let ret = self.regs.pc;
        self.push_u16(bus, ret);
        self.regs.pc = (opcode & 0x38) as u16;
        16
    }

    pub(super) fn exec_call_a16<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let addr = self.fetch16(bus);
        let ret = self.regs.pc;
        self.push_u16(bus, ret);
        self.regs.pc = addr;
        24
    }

    pub(super) fn exec_ret<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let addr = self.pop_u16(bus);
        self.regs.pc = addr;
        16
    }

    pub(super) fn exec_reti<B: Bus>(&mut self, bus: &mut B) -> u32 {
        let addr = self.pop_u16(bus);
        self.regs.pc = addr;
        self.ime = true;
        16
    }
}
