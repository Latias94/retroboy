use crate::cpu::{Bus, Cpu};

impl Cpu {
    pub(super) fn exec_stop<B: Bus>(&mut self, bus: &mut B) -> u32 {
        // STOP is officially a 2‑byte instruction; the second byte is
        // often 0 and ignored. We always fetch and discard the padding
        // byte so that PC matches hardware.
        let _padding = self.fetch8(bus);

        // CGB speed switch: when KEY1's "prepare speed switch" latch is set,
        // STOP toggles double-speed and returns immediately (it does not
        // enter the low-power STOP state).
        if bus.cgb_speed_switch() {
            self.stopped = false;
            self.halted = false;
            return 4;
        }
        // Enter STOP low‑power mode. In this state the CPU ignores
        // maskable interrupts and remains idle until a joypad input
        // line goes low (approximated in `step` by polling P1/$FF00).
        self.stopped = true;
        self.halted = false;
        4
    }

    pub(super) fn exec_di(&mut self) -> u32 {
        self.ime = false;
        self.ime_enable_pending = false;
        self.ime_enable_delay = false;
        4
    }

    pub(super) fn exec_ei(&mut self) -> u32 {
        // IME becomes 1 after the *next* instruction completes.
        self.ime_enable_pending = true;
        4
    }
}
