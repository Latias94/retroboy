mod alu;
mod control;
mod ld;
mod incdec;
mod stack;
mod system;

use super::{Bus, Cpu};

impl Cpu {
    /// Decode and execute a single opcode and return the number of cycles.
    ///
    /// This method contains the full instruction table and is factored out of
    /// `step` so that we can later introduce a more fine‑grained, cycle‑level
    /// stepping API while reusing the same semantics.
    pub(super) fn exec_opcode<B: Bus>(&mut self, bus: &mut B, opcode: u8) -> u32 {
        if opcode == 0xCB {
            self.step_cb(bus)
        } else {
            match opcode {
                // 0x00: NOP
                0x00 => 4,

                // 16-bit immediate loads.
                0x01 | 0x11 | 0x21 | 0x31 => self.exec_ld_rr_d16(bus, opcode),

                // Rotate A instructions (unprefixed).
                0x07 | 0x0F | 0x17 | 0x1F => self.exec_rotate_a(opcode),

                // 16-bit INC rr
                0x03 | 0x13 | 0x23 | 0x33 => self.exec_inc16_rr(opcode),

                // 16-bit DEC rr
                0x0B | 0x1B | 0x2B | 0x3B => self.exec_dec16_rr(opcode),

                // LD r, d8 (and LD (HL), d8)
                0x06 | 0x0E | 0x16 | 0x1E | 0x26 | 0x2E | 0x36 | 0x3E => {
                    self.exec_ld_r_d8(bus, opcode)
                }

                // 8-bit register/memory transfers: LD r1, r2 (and HALT)
                //
                // This covers opcodes 0x40–0x7F (including 0x76, HALT).
                0x40..=0x7F => self.exec_ld_rr_or_halt(bus, opcode),

                // LD (BC/DE/HL±), A
                0x02 | 0x12 | 0x22 | 0x32 => self.exec_ld_indirect_a(bus, opcode),

                // LD A, (BC/DE/HL±)
                0x0A | 0x1A | 0x2A | 0x3A => self.exec_ld_a_indirect(bus, opcode),

                // LD (a16), SP
                0x08 => self.exec_ld_a16_sp(bus),

                // STOP
                0x10 => self.exec_stop(bus),

                // LDH (a8),A / LDH A,(a8)
                0xE0 | 0xF0 => self.exec_ldh_a8(bus, opcode),

                // LDH (C),A / LDH A,(C)
                0xE2 | 0xF2 => self.exec_ldh_c(bus, opcode),

                // LD (a16),A / LD A,(a16)
                0xEA | 0xFA => self.exec_ld_a16_a(bus, opcode),

                // ADD SP, r8
                0xE8 => self.exec_add_sp_r8(bus),

                // LD HL, SP+r8
                0xF8 => self.exec_ld_hl_sp_r8(bus),

                // LD SP, HL
                0xF9 => self.exec_ld_sp_hl(),

                // JR r8 (relative)
                0x18 => self.jr(bus, true),

                // JR cc, r8
                0x20 | 0x28 | 0x30 | 0x38 => self.exec_jr_cc(bus, opcode),

                // JP cc, a16
                0xC2 | 0xCA | 0xD2 | 0xDA => self.exec_jp_cc(bus, opcode),

                // ADD HL, rr (16-bit)
                0x09 | 0x19 | 0x29 | 0x39 => self.exec_add_hl_rr(opcode),

                // DAA
                0x27 => self.exec_daa(),

                // CPL
                0x2F => self.exec_cpl(),

                // SCF
                0x37 => self.exec_scf(),

                // CCF
                0x3F => self.exec_ccf(),

                // 8-bit ALU operations on A: ADD/ADC/SUB/SBC/AND/XOR/OR/CP r,(HL)
                0x80..=0xBF => self.exec_alu_reg_group(bus, opcode),

                // 8-bit ALU immediate operations on A: ADD/ADC/SUB/SBC/AND/XOR/OR/CP d8
                0xC6 | 0xCE | 0xD6 | 0xDE | 0xE6 | 0xEE | 0xF6 | 0xFE => {
                    self.exec_alu_imm(bus, opcode)
                }

                // INC r
                0x04 | 0x0C | 0x14 | 0x1C | 0x24 | 0x2C | 0x34 | 0x3C => {
                    self.exec_inc8_reg(bus, opcode)
                }

                // DEC r
                0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D => {
                    self.exec_dec8_reg(bus, opcode)
                }

                // DI
                0xF3 => self.exec_di(),

                // EI
                0xFB => self.exec_ei(),

                // JP a16
                0xC3 => self.exec_jp_a16(bus),

                // JP (HL)
                0xE9 => self.exec_jp_hl(),

                // CALL a16
                0xCD => self.exec_call_a16(bus),

                // PUSH rr
                0xC5 | 0xD5 | 0xE5 | 0xF5 => self.exec_push_rr(bus, opcode),

                // POP rr
                0xC1 | 0xD1 | 0xE1 | 0xF1 => self.exec_pop_rr(bus, opcode),

                // CALL cc, a16
                0xC4 | 0xCC | 0xD4 | 0xDC => self.exec_call_cc(bus, opcode),

                // RET
                0xC9 => self.exec_ret(bus),

                // RET cc
                0xC0 | 0xC8 | 0xD0 | 0xD8 => self.exec_ret_cc(bus, opcode),

                // RETI
                0xD9 => self.exec_reti(bus),

                // RST nn
                0xC7 | 0xCF | 0xD7 | 0xDF | 0xE7 | 0xEF | 0xF7 | 0xFF => {
                    self.exec_rst(bus, opcode)
                }

                // Invalid / unimplemented opcodes.
                //
                // Pandocs documents a set of "opcode holes" (D3, DB, DD, E3, E4,
                // EB, EC, ED, F4, FC, FD) that hard‑lock the CPU until power‑off.
                // We approximate that by marking the CPU as locked so subsequent
                // `step` calls return 0 cycles.
                _ => {
                    if !self.locked {
                        let opcode_addr = self.regs.pc.wrapping_sub(1);
                        log::error!(
                            "GB CPU locked: invalid opcode 0x{opcode:02X} at PC=0x{pc:04X} (SP=0x{sp:04X} AF=0x{af:04X} BC=0x{bc:04X} DE=0x{de:04X} HL=0x{hl:04X})",
                            opcode = opcode,
                            pc = opcode_addr,
                            sp = self.regs.sp,
                            af = self.regs.af(),
                            bc = self.regs.bc(),
                            de = self.regs.de(),
                            hl = self.regs.hl(),
                        );
                    }
                    self.locked = true;
                    0
                }
            }
        }
    }

}
