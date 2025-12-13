use super::super::{Bus, Cpu};

impl Cpu {
    /// Handle maskable interrupts if IME is set and a pending interrupt exists.
    ///
    /// Returns `Some(cycles)` if an interrupt was taken, or `None` otherwise.
    pub(in crate::cpu) fn handle_interrupts<B: Bus>(&mut self, bus: &mut B) -> Option<u32> {
        // First check whether a maskable interrupt should be serviced at
        // all in the current state (including HALT-bug behaviour). We
        // intentionally ignore the specific index here and instead select
        // the final interrupt after the high-byte push so that writes to
        // IE during that push (as in mooneye's `ie_push` test) can cancel
        // or retarget the dispatch.
        if self.poll_pending_interrupt(bus).is_none() {
            return None;
        }

        // Standard interrupt entry sequence (instruction-level model).
        //
        // We broadly follow the same ordering as the micro-step path and
        // mooneye-gb's CPU: IME is cleared immediately, then the current PC
        // is pushed to the stack, and only after that is the corresponding
        // IF bit cleared and the PC redirected to the interrupt vector. This
        // makes the timing of IF/IE observable state closer to real hardware
        // while still charging the canonical cost of 20 T-cycles as a single
        // logical "instruction".
        self.ime = false;
        self.halted = false;

        let pc = self.regs.pc;
        let hi = (pc >> 8) as u8;
        let lo = pc as u8;

        // Push high byte of PC. This may write to `$FFFF` (IE) when SP was
        // 0, so the IE value used for interrupt selection must be sampled
        // *after* this write.
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, hi);

        // Decide which interrupt is still pending after the high-byte
        // push. If none remain, the dispatch is cancelled and PC falls
        // back to 0x0000; otherwise we clear that IF bit and jump to the
        // corresponding vector.
        let selection = self.select_interrupt_after_high_push(bus);

        // Push low byte of PC. Writes performed by this store (e.g. to
        // `$FFFF` when SP was 1) must not affect the interrupt selection
        // for the current dispatch.
        self.regs.sp = self.regs.sp.wrapping_sub(1);
        bus.write8(self.regs.sp, lo);

        match selection {
            Some((index, new_if)) => {
                let vector = 0x0040 + (index as u16) * 8;
                let ie = bus.read8(0xFFFF);
                let iflags = bus.read8(0xFF0F) & 0x1F;
                log::debug!(
                    "GB CPU interrupt: idx={} vector=0x{:04X} pc=0x{:04X} sp=0x{:04X} IF=0x{:02X} IE=0x{:02X}",
                    index,
                    vector,
                    pc,
                    self.regs.sp,
                    iflags,
                    ie,
                );
                bus.write8(0xFF0F, new_if);
                self.regs.pc = 0x0040 + (index as u16) * 8;
            }
            None => {
                // No interrupt line remained pending after the high-byte
                // push: treat this as a cancelled dispatch and jump to
                // address 0x0000 (see mooneye `ie_push` Round 1).
                self.regs.pc = 0x0000;
            }
        }

        Some(20)
    }
}
