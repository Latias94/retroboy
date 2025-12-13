mod legacy;
mod micro;

use super::{Bus, Cpu};

impl Cpu {
    /// Execute a single instruction and return the number of T-cycles taken.
    ///
    /// For now this entry point delegates to the legacy instruction-level
    /// implementation so that the higher-level machine continues to use
    /// the original `begin_instruction`/`end_instruction` integration.
    /// The micro-step API (`step_mcycle`) is used separately by dedicated
    /// tests and experiments.
    pub fn step<B: Bus>(&mut self, bus: &mut B) -> u32 {
        self.step_instruction_legacy(bus)
    }
}

