use super::super::{Bus, Cpu};

impl Cpu {
    pub(in crate::cpu) fn poll_pending_interrupt<B: Bus>(&mut self, bus: &mut B) -> Option<(u8, u8)> {
        let ie = bus.read8(0xFFFF);
        let iflags = bus.read8(0xFF0F);
        let pending = ie & iflags & 0x1F;
        if pending == 0 {
            return None;
        }

        // If the CPU is halted and an interrupt becomes pending while IME is
        // disabled, the CPU wakes up without servicing the interrupt. This is
        // part of the so‑called "HALT bug" behaviour.
        if self.halted && !self.ime {
            self.halted = false;
            return None;
        }

        if !self.ime {
            return None;
        }

        // Find lowest-numbered pending interrupt (VBlank > LCD STAT > Timer > Serial > Joypad).
        let index = pending.trailing_zeros();
        if index >= 5 {
            return None;
        }

        Some((index as u8, iflags))
    }

    /// Select the highest‑priority pending interrupt *after* the high byte
    /// of PC has been pushed, but *before* the low byte is written.
    ///
    /// This mirrors mooneye‑gb's `write_cycle_intr` semantics used by the
    /// `ie_push` acceptance test:
    ///
    /// - `IE` is evaluated after any effects of the high‑byte push
    ///   (e.g. when `SP` was 0 and the high byte was written to `$FFFF`).
    /// - `IF` is sampled before the low‑byte push.
    /// - The interrupt line is chosen from `IE & IF` at this point;
    ///   writes to `IE` performed by the low‑byte push are "too late" to
    ///   affect the current dispatch.
    ///
    pub(in crate::cpu) fn select_interrupt_after_high_push<B: Bus>(
        &mut self,
        bus: &mut B,
    ) -> Option<(u8, u8)> {
        let ie_now = bus.read8(0xFFFF);
        let if_now = bus.read8(0xFF0F);
        let pending = ie_now & if_now & 0x1F;
        if pending == 0 {
            return None;
        }

        let index = pending.trailing_zeros();
        if index >= 5 {
            return None;
        }

        let new_if = if_now & !(1 << index);
        Some((index as u8, new_if))
    }

}
