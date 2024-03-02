use crate::{
    FONTSET, FONTSET_SIZE, NUM_KEYS, NUM_REGS, RAM_SIZE, SCREEN_HEIGHT, SCREEN_WIDTH, STACK_SIZE,
    START_ADDRESS,
};

pub struct Emulator {
    /// program counter
    pc: u16,
    ram: [u8; RAM_SIZE],
    /// display
    screen: [bool; SCREEN_WIDTH * SCREEN_HEIGHT],
    /// V Registers
    v_reg: [u8; NUM_REGS],
    /// I Registers
    i_reg: u16,
    /// Stack: CPU can read and write to
    stack_pointer: u16,
    stack: [u16; STACK_SIZE],
    keys: [bool; NUM_KEYS],
    /// delay timer
    delay_timer: u8,
    /// sound timer
    sound_timer: u8,
}

impl Default for Emulator {
    fn default() -> Self {
        let mut new_emu = Self {
            pc: START_ADDRESS,
            ram: [0; RAM_SIZE],
            screen: [false; SCREEN_WIDTH * SCREEN_HEIGHT],
            v_reg: [0; NUM_REGS],
            i_reg: 0,
            stack_pointer: 0,
            stack: [0; STACK_SIZE],
            keys: [false; NUM_KEYS],
            delay_timer: 0,
            sound_timer: 0,
        };
        new_emu.ram[..FONTSET_SIZE].copy_from_slice(&FONTSET);
        new_emu
    }
}

impl Emulator {
    pub fn reset(&mut self) {
        self.pc = START_ADDRESS;
        self.ram = [0; RAM_SIZE];
        self.screen = [false; SCREEN_WIDTH * SCREEN_HEIGHT];
        self.v_reg = [0; NUM_REGS];
        self.i_reg = 0;
        self.stack_pointer = 0;
        self.stack = [0; STACK_SIZE];
        self.keys = [false; NUM_KEYS];
        self.delay_timer = 0;
        self.sound_timer = 0;
        self.ram[..FONTSET_SIZE].copy_from_slice(&FONTSET);
    }

    fn push(&mut self, val: u16) {
        self.stack[self.stack_pointer as usize] = val;
        self.stack_pointer += 1;
    }
    fn pop(&mut self) -> u16 {
        self.stack_pointer -= 1;
        self.stack[self.stack_pointer as usize]
    }

    pub fn tick(&mut self) {
        let opcode = self.fetch_opcode();
        self.execute_opcode(opcode);
    }

    fn fetch_opcode(&mut self) -> u16 {
        let pc = self.pc as usize;
        let byte1 = self.ram[pc] as u16;
        let byte2 = self.ram[pc + 1] as u16;
        self.pc += 2;
        byte1 << 8 | byte2
    }

    pub fn tick_timers(&mut self) {
        if self.delay_timer > 0 {
            self.delay_timer -= 1;
        }
        if self.sound_timer > 0 {
            if self.sound_timer == 1 {
                // TODO: BEEP
            }
            self.sound_timer -= 1;
        }
    }

    fn execute_opcode(&mut self, op: u16) {
        let digit1 = (op & 0xF000) >> 12;
        let digit2 = (op & 0x0F00) >> 8;
        let digit3 = (op & 0x00F0) >> 4;
        let digit4 = op & 0x000F;
        match (digit1, digit2, digit3, digit4) {
            // NOP
            (0, 0, 0, 0) => (),
            // 00E0 - CLS Clear the display
            (0, 0, 0xE, 0) => self.screen = [false; SCREEN_WIDTH * SCREEN_HEIGHT],
            // 00EE - RET Return from subroutine
            (0, 0, 0xE, 0xE) => self.pc = self.pop(),
            // 1NNN - JP Jump addr
            (0x1, _, _, _) => self.pc = op & 0x0FFF,
            // 2NNN - CALL Call subroutine
            (0x2, _, _, _) => {
                self.push(self.pc);
                self.pc = op & 0x0FFF;
            }
            // 3XNN - SE Skip next if Vx == NN
            (0x3, x, _, _) => {
                if self.v_reg[x as usize] == (op & 0x00FF) as u8 {
                    self.pc += 2;
                }
            }
            // 4XNN - SNE Skip next if Vx != NN
            (0x4, x, _, _) => {
                if self.v_reg[x as usize] != (op & 0x00FF) as u8 {
                    self.pc += 2;
                }
            }
            // 5XY0 - SE Skip next if Vx == Vy
            (0x5, x, y, 0) => {
                if self.v_reg[x as usize] == self.v_reg[y as usize] {
                    self.pc += 2;
                }
            }
            // 6XNN - LD Vx == NN
            (0x6, x, _, _) => self.v_reg[x as usize] = (op & 0x00FF) as u8,
            // 7XNN - ADD Vx += NN
            (0x7, x, _, _) => {
                let val = (op & 0x00FF) as u8;
                self.v_reg[x as usize] = self.v_reg[x as usize].wrapping_add(val);
            }
            // 8XY0 - LD Vx = Vy
            (0x8, x, y, 0) => self.v_reg[x as usize] = self.v_reg[y as usize],
            // 8XY1 - OR Vx |= Vy
            (0x8, x, y, 1) => self.v_reg[x as usize] |= self.v_reg[y as usize],
            // 8XY2 - AND Vx &= Vy
            (0x8, x, y, 2) => self.v_reg[x as usize] &= self.v_reg[y as usize],
            // 8XY3 - XOR Vx ^= Vy
            (0x8, x, y, 3) => self.v_reg[x as usize] ^= self.v_reg[y as usize],
            // 8XY4 - ADD Vx += Vy, VF = carry
            (0x8, x, y, 4) => {
                // VF is set to 1 when there's a carry, and to 0 when there isn't
                let (val, overflow) =
                    self.v_reg[x as usize].overflowing_add(self.v_reg[y as usize]);
                self.v_reg[x as usize] = val;
                self.v_reg[0xF] = overflow as u8;
            }
            // 8XY5 - SUB Vx -= Vy, VF = !borrow
            (0x8, x, y, 5) => {
                // VF is set to 0 when there's a borrow, and 1 when there isn't
                let (val, overflow) =
                    self.v_reg[x as usize].overflowing_sub(self.v_reg[y as usize]);
                self.v_reg[x as usize] = val;
                self.v_reg[0xF] = !overflow as u8;
            }
            // 8XY6 - SHR Vx >>= 1, VF = lsb
            (0x8, x, _, 6) => {
                self.v_reg[0xF] = self.v_reg[x as usize] & 0x1;
                self.v_reg[x as usize] >>= 1;
            }
            // 8XY7 - SUBN Vx = Vy - Vx, VF = !borrow,  like (0x8, x, y, 5) but reversed operands
            (0x8, x, y, 7) => {
                // VF is set to 0 when there's a borrow, and 1 when there isn't
                let (val, overflow) =
                    self.v_reg[y as usize].overflowing_sub(self.v_reg[x as usize]);
                self.v_reg[x as usize] = val;
                self.v_reg[0xF] = !overflow as u8;
            }
            // 8XYE - SHL Vx <<= 1, VF = msb
            (0x8, x, _, 0xE) => {
                self.v_reg[0xF] = (self.v_reg[x as usize] & 0x80) >> 7;
                self.v_reg[x as usize] <<= 1;
            }
            // 9XY0 - SNE Skip next if Vx != Vy
            (0x9, x, y, 0) => {
                if self.v_reg[x as usize] != self.v_reg[y as usize] {
                    self.pc += 2;
                }
            }
            // ANNN - LD I = addr, Load I with addr
            (0xA, _, _, _) => self.i_reg = op & 0x0FFF,
            // BNNN - JP V0 + addr, Jump to location V0 + addr
            (0xB, _, _, _) => self.pc = (op & 0x0FFF) + self.v_reg[0] as u16,
            // CXNN - RND Vx = rand & NN, Set Vx = random byte & NN
            (0xC, x, _, _) => {
                let rand = rand::random::<u8>();
                self.v_reg[x as usize] = rand & (op & 0x00FF) as u8;
            }
            // DXYN - DRW Draw sprite at Vx, Vy, height N
            (0xD, d2, d3, height) => {
                let x_coord = self.v_reg[d2 as usize] as usize;
                let y_coord = self.v_reg[d3 as usize] as usize;
                let height = height as usize;
                self.v_reg[0xF] = 0;
                for row in 0..height {
                    // Determine which memory address our row's data is stored
                    let addr = self.i_reg as usize + row;
                    let pixels = self.ram[addr];
                    for col in 0..8 {
                        let sprite_pixel = (pixels >> (7 - col)) & 0x1;
                        // Sprites should wrap around screen
                        let x = (x_coord + col) % SCREEN_WIDTH;
                        let y = (y_coord + row) % SCREEN_HEIGHT;
                        let screen_index = x + y * SCREEN_WIDTH;
                        let screen_pixel = &mut self.screen[screen_index];
                        if sprite_pixel == 1 && *screen_pixel {
                            self.v_reg[0xF] = 1;
                        }
                        *screen_pixel ^= sprite_pixel == 1;
                    }
                }
            }
            // EX9E - SKP Skip next if key[Vx] is pressed
            (0xE, x, 9, 0xE) => {
                let key = self.v_reg[x as usize] as usize;
                if key < NUM_KEYS && self.is_key_pressed(key) {
                    self.pc += 2;
                }
            }
            // EXA1 - SKNP Skip next if key[Vx] is not pressed
            (0xE, x, 0xA, 1) => {
                let key = self.v_reg[x as usize] as usize;
                if key < NUM_KEYS && !self.is_key_pressed(key) {
                    self.pc += 2;
                }
            }
            // FX07 - LD Vx = DT, Set Vx = delay timer value
            (0xF, x, 0, 7) => self.v_reg[x as usize] = self.delay_timer,
            // FX0A - LD Vx = K, Wait for key press, store value in Vx
            (0xF, x, 0, 0xA) => {
                let mut key_pressed = false;
                for (i, key) in self.keys.iter().enumerate() {
                    if *key {
                        self.v_reg[x as usize] = i as u8;
                        key_pressed = true;
                    }
                }
                if !key_pressed {
                    // If no key is pressed, return and don't increment the program counter
                    // we don't loop here because we don't want to block any new key presses
                    self.pc -= 2;
                }
            }
            // FX15 - LD DT = Vx, Set delay timer = Vx
            (0xF, x, 1, 5) => self.delay_timer = self.v_reg[x as usize],
            // FX18 - LD ST = Vx, Set sound timer = Vx
            (0xF, x, 1, 8) => self.sound_timer = self.v_reg[x as usize],
            // FX1E - ADD I, Vx, Set I = I + Vx
            (0xF, x, 1, 0xE) => self.i_reg = self.i_reg.wrapping_add(self.v_reg[x as usize] as u16),
            // FX29 - LD F, Vx, Set I = location of sprite for digit Vx
            // 5 is the number of bytes per character
            (0xF, x, 2, 9) => self.i_reg = self.v_reg[x as usize] as u16 * 5,
            // FX33 - LD B, Vx, Store BCD representation of Vx in memory locations I, I+1, and I+2
            (0xF, x, 3, 3) => {
                let val = self.v_reg[x as usize];
                let i = self.i_reg as usize;
                self.ram[i] = val / 100; // hundreds
                self.ram[i + 1] = (val / 10) % 10; // tens
                self.ram[i + 2] = val % 10; // ones
            }
            // FX55 - LD [I], Vx, Store registers V0 through Vx in memory starting at location I
            (0xF, x, 5, 5) => {
                let i = self.i_reg as usize;
                for offset in 0..=x {
                    let offset = offset as usize;
                    self.ram[i + offset] = self.v_reg[offset];
                }
            }
            // FX65 - LD Vx, [I], Read registers V0 through Vx from memory starting at location I
            (0xF, x, 6, 5) => {
                let i = self.i_reg as usize;
                for offset in 0..=x {
                    let offset = offset as usize;
                    self.v_reg[offset] = self.ram[i + offset];
                }
            }
            // 0x0000、0xE000 and 0xF000 are not used, some Chip-8 extensions use them
            (_, _, _, _) => unimplemented!("Unimplemented opcode: {:04X}", op),
        }
    }

    fn is_key_pressed(&self, key: usize) -> bool {
        self.keys[key]
    }

    pub fn get_display(&self) -> &[bool; SCREEN_WIDTH * SCREEN_HEIGHT] {
        &self.screen
    }

    pub fn set_key(&mut self, idx: usize, pressed: bool) {
        assert!(idx < NUM_KEYS, "Invalid key index: {}", idx);
        self.keys[idx] = pressed;
    }

    pub fn load_rom(&mut self, rom: &[u8]) {
        let start = START_ADDRESS as usize;
        let end = start + rom.len();
        self.ram[start..end].copy_from_slice(rom);
    }
}
