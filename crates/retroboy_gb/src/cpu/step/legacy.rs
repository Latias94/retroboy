use crate::cpu_micro::MicroInstrKind;

use super::super::{Bus, Cpu};

impl Cpu {
    pub(super) fn step_instruction_legacy<B: Bus>(&mut self, bus: &mut B) -> u32 {
        // Instruction‑level stepping ignores any in‑flight micro‑step state
        // and always executes a full instruction in one go.
        self.micro_cycles_remaining = 0;
        self.micro_from_interrupt = false;
        self.micro_instr = MicroInstrKind::None;
        self.micro_stage = 0;
        self.micro_imm16 = 0;
        self.micro_cond_taken = false;

        if self.locked {
            // CPU has executed an invalid opcode (see Pandocs CPU opcode
            // holes). On hardware the CPU is effectively dead until power‑off.
            // We approximate that by returning 0 cycles so higher‑level loops
            // can detect the condition and stop.
            return 0;
        }

        // Mark the beginning of a single instruction (or interrupt entry)
        // so that the bus can track per‑instruction state such as timer
        // IO events.
        bus.begin_instruction();

        // In STOP mode the CPU enters a deeper low‑power state than HALT. On
        // real hardware STOP is exited when a joypad input line (P10–P13)
        // goes low. We approximate this by polling P1 ($FF00) on each step
        // and clearing `stopped` once any of the lower four bits becomes 0.
        if self.stopped {
            let p1 = bus.read8(0xFF00);
            if (p1 & 0x0F) != 0x0F {
                // A button/d‑pad line is low: resume normal execution.
                self.stopped = false;
            }
            // In STOP mode the system counter used by DIV/TIMA does not
            // advance (see Pandocs "System counter"). We therefore avoid
            // calling `bus.tick` here so that timers/PPU remain frozen until
            // STOP is exited. We still report a small non‑zero cycle cost so
            // that callers that rely on forward progress do not spin forever.
            bus.end_instruction(0);
            return 4;
        }

        if let Some(cycles) = self.handle_interrupts(bus) {
            bus.end_instruction(cycles);
            return cycles;
        }

        if self.halted {
            // In HALT, the CPU effectively performs a NOP each cycle until
            // an interrupt occurs. For now we just return a fixed cost.
            bus.end_instruction(4);
            return 4;
        }

        let opcode = self.fetch8(bus);
        let cycles = self.exec_opcode(bus, opcode);

        bus.end_instruction(cycles);
        self.apply_ime_delay();
        cycles
    }
}
