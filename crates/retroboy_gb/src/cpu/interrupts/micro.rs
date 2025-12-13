use super::super::{Bus, Cpu};

impl Cpu {
    /// Begin servicing a maskable interrupt using the micro-step API.
    ///
    /// This mirrors `handle_interrupts` in terms of which interrupt is
    /// chosen and when IF/IME/`halted` are updated, but it defers the
    /// actual stack push and PC jump to `step_interrupt_mcycle` so that
    /// the entry sequence can be spread across multiple M-cycles.
    pub(in crate::cpu) fn begin_interrupt_micro<B: Bus>(&mut self, bus: &mut B) -> bool {
        // As in the instruction-level path, first check whether any
        // maskable interrupt should be serviced (including HALT handling).
        if self.poll_pending_interrupt(bus).is_none() {
            return false;
        }

        // Latch state needed for the interrupt entry micro‑sequence. We
        // defer clearing the IF bit until a later M‑cycle to better match
        // hardware behaviour.
        self.ime = false;
        self.halted = false;
        self.interrupt_pc = self.regs.pc;
        // The final interrupt vector (or cancellation to 0x0000) will be
        // decided after the high‑byte push in `step_interrupt_mcycle`.
        self.interrupt_vector = 0x0000;
        self.interrupt_stage = 0;
        self.interrupt_index_latched = None;
        self.interrupt_iflags_latched = 0;

        // Total cost is 20 T‑cycles (5 M‑cycles).
        self.micro_cycles_remaining = 20;
        self.micro_from_interrupt = true;
        true
    }

    /// Execute a single 4‑T‑cycle slice of the interrupt entry sequence.
    ///
    /// This helper is used by the micro‑step API (`step_mcycle`) once
    /// `begin_interrupt_micro` has latched the interrupt metadata. It
    /// spreads the PC push and vector jump across five M‑cycles while
    /// still charging the canonical total of 20 T‑cycles.

    pub(in crate::cpu) fn step_interrupt_mcycle<B: Bus>(&mut self, bus: &mut B) {
        match self.interrupt_stage {
            0 => {
                // Initial idle M‑cycle after accepting the interrupt.
            }
            1 => {
                // Second idle M‑cycle (approximates internal wait states).
            }
            2 => {
                // Push high byte of PC.
                let pc = self.interrupt_pc;
                let hi = (pc >> 8) as u8;
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, hi);
            }
            3 => {
                // Push low byte of PC and, based on the IE/IF state *after* the
                // high‑byte push but *before* this write, decide which interrupt
                // (if any) should actually be dispatched. Writes to IE performed
                // by this low‑byte push are therefore too late to affect the
                // current dispatch.
                let pc = self.interrupt_pc;
                let lo = pc as u8;

                // Sample IE/IF after the high‑byte push (stage 2).
                let selection = self.select_interrupt_after_high_push(bus);

                // Perform the low‑byte push. This may write to `$FFFF` (IE) when
                // SP was 1, but the selection above must not observe those changes.
                self.regs.sp = self.regs.sp.wrapping_sub(1);
                bus.write8(self.regs.sp, lo);

                match selection {
                    Some((index, new_if)) => {
                        bus.write8(0xFF0F, new_if);
                        self.interrupt_vector = 0x0040 + (index as u16) * 8;
                    }
                    None => {
                        // No interrupt line remained pending after the high‑byte
                        // push: treat this as a cancelled dispatch and jump to
                        // address 0x0000 (see mooneye `ie_push` Round 1).
                        self.interrupt_vector = 0x0000;
                    }
                }

                // Any previously latched index/IF are no longer needed.
                self.interrupt_index_latched = None;
                self.interrupt_iflags_latched = 0;
            }
            4 => {
                // Jump to interrupt vector.
                self.regs.pc = self.interrupt_vector;
            }
            _ => {}
        }

        // One M‑cycle worth of time: generic peripherals plus a single
        // Timer tick (interrupt entry does not perform explicit Timer IO).
        bus.tick_mcycle_generic();
        bus.timer_tick_mcycle();
        // Saturating add to avoid wrapping if something goes wrong.
        self.interrupt_stage = self.interrupt_stage.saturating_add(1);
    }

}
