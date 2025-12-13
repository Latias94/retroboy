//! Micro-op helpers for cycle-level CPU stepping.
//!
//! This module contains small state machines used by `step_mcycle` to
//! execute selected instructions (e.g. CALL/RET) across multiple
//! M-cycles while preserving the same semantics as the monolithic
//! `exec_opcode` implementation used by `step`.

mod dispatch;
mod instructions;
mod kind;

pub use kind::MicroInstrKind;
