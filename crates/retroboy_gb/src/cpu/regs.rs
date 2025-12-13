/// Registers for the Game Boy CPU (LR35902).
///
/// The core is Z80-like with an 8-bit ALU and a 16-bit address space.
/// We start with a straightforward representation and refine as we go.
#[derive(Clone, Copy, Debug, Default)]
pub struct Registers {
    pub a: u8,
    pub f: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub sp: u16,
    pub pc: u16,
}

impl Registers {
    #[inline]
    pub fn af(&self) -> u16 {
        u16::from_be_bytes([self.a, self.f & 0xF0])
    }

    #[inline]
    pub fn set_af(&mut self, value: u16) {
        let [a, f] = value.to_be_bytes();
        self.a = a;
        // Lower 4 bits of F are always zero.
        self.f = f & 0xF0;
    }

    #[inline]
    pub fn bc(&self) -> u16 {
        u16::from_be_bytes([self.b, self.c])
    }

    #[inline]
    pub fn set_bc(&mut self, value: u16) {
        let [b, c] = value.to_be_bytes();
        self.b = b;
        self.c = c;
    }

    #[inline]
    pub fn de(&self) -> u16 {
        u16::from_be_bytes([self.d, self.e])
    }

    #[inline]
    pub fn set_de(&mut self, value: u16) {
        let [d, e] = value.to_be_bytes();
        self.d = d;
        self.e = e;
    }

    #[inline]
    pub fn hl(&self) -> u16 {
        u16::from_be_bytes([self.h, self.l])
    }

    #[inline]
    pub fn set_hl(&mut self, value: u16) {
        let [h, l] = value.to_be_bytes();
        self.h = h;
        self.l = l;
    }
}

/// Flag bits in the F register.
///
/// Layout (bit index in the byte, from MSB to LSB):
/// - bit 7: Z (zero)
/// - bit 6: N (subtract)
/// - bit 5: H (half carry)
/// - bit 4: C (carry)
/// - bits 0–3 are always zero.
#[derive(Clone, Copy, Debug)]
pub enum Flag {
    Z = 7,
    N = 6,
    H = 5,
    C = 4,
}

