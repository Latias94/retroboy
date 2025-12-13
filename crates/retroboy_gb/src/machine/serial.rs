/// Very small serial interface modelled via SB/SC.
///
/// This is intentionally minimal: we only care about the CPU test ROMs
/// writing bytes over the serial port. When a transfer is started on
/// SC (bit 7 set while bit 0 is 1), the current SB value is appended
/// to `output` and the transfer-complete flag is cleared.
#[derive(Default)]
pub(crate) struct Serial {
    pub(crate) sb: u8,
    pub(crate) sc: u8,
    pub(crate) output: Vec<u8>,
}

impl Serial {
    pub(super) fn write_sb(&mut self, value: u8) {
        self.sb = value;
    }

    pub(super) fn write_sc(&mut self, value: u8) {
        self.sc = value;
        // Internal clock & start bit set?
        if (self.sc & 0x81) == 0x81 {
            self.output.push(self.sb);
            // Clear transfer start bit.
            self.sc &= !0x80;
        }
    }
}

