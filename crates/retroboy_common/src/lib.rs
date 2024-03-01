#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    pub const BLACK: Color = Color::new_rgb(0, 0, 0);
    pub const WHITE: Color = Color::new_rgb(255, 255, 255);
    pub const RED: Color = Color::new_rgb(255, 0, 0);
    pub const GREEN: Color = Color::new_rgb(0, 255, 0);
    pub const BLUE: Color = Color::new_rgb(0, 0, 255);

    pub const GRAY: Color = Color::new_rgb(128, 128, 128);
    pub const GREY: Color = Color::GRAY;
    pub const MAGENTA: Color = Color::new_rgb(255, 0, 255);
    pub const YELLOW: Color = Color::new_rgb(255, 255, 0);
    pub const CYAN: Color = Color::new_rgb(0, 255, 255);

    #[inline]
    pub const fn new_rgb(r: u8, g: u8, b: u8) -> Color {
        Color { r, g, b, a: 0xff }
    }

    #[inline]
    pub const fn new_rgba(r: u8, g: u8, b: u8, a: u8) -> Color {
        Color { r, g, b, a }
    }

    #[inline]
    pub const fn rgb(&self) -> (u8, u8, u8) {
        (self.r, self.g, self.b)
    }

    #[inline]
    pub const fn rgba(&self) -> (u8, u8, u8, u8) {
        (self.r, self.g, self.b, self.a)
    }

    pub fn to_u32(&self) -> u32 {
        u32::from_le_bytes([self.r, self.g, self.b, self.a])
    }
}
