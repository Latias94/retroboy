use super::super::Cpu;

impl Cpu {
    /// Apply delayed IME change requested by EI.
    #[inline]
    pub(in crate::cpu) fn apply_ime_delay(&mut self) {
        if self.ime_enable_delay {
            // Second step after EI: actually enable IME.
            self.ime = true;
            self.ime_enable_delay = false;
        } else if self.ime_enable_pending {
            // First step after EI: arm the delayed enable.
            self.ime_enable_pending = false;
            self.ime_enable_delay = true;
        }
    }
}
