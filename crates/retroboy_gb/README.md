# RetroBoy Game Boy (DMG/CGB)

This crate hosts the Game Boy emulator for the RetroBoy workspace (DMG, with early CGB support).

Current status (high level):

- CPU: LR35902 core implemented; validated via unit tests, Blargg `cpu_instrs`, and a growing subset of Mooneye acceptance ROMs.
- Bus/cartridge: ROM-only + basic MBC1/MBC3/MBC5; CGB VBK/SVBK banking; KEY1 speed-switch latch (timing not fully modelled yet); HDMA GDMA/HBlank (simplified).
- PPU/video: simplified LY/STAT/VBlank timing; line-based DMG/CGB renderer (CGB palettes/attrs/priority); DMG-only dot-mode3 path for mealybug-tearoom-tests.
- APU: not implemented yet.
- Tooling/tests: ROM/screenshot harness under `cargo test -p retroboy_gb -- --ignored`; `gb_frame_dump` helper for dumping raw RGB24 frames.

The initial focus is on getting the CPU core correct (using well-known
test ROMs) and wiring a minimal machine that can run small programs, before
attempting full commercial games.

## Code Layout

- CPU core entry: `crates/retroboy_gb/src/cpu.rs`
- CPU submodules: `crates/retroboy_gb/src/cpu/` (e.g. `alu.rs`, `exec.rs`, `interrupts.rs`)
- Machine entry: `crates/retroboy_gb/src/machine.rs`
- High-level wrapper: `crates/retroboy_gb/src/machine/gameboy.rs`
- Bus entry: `crates/retroboy_gb/src/machine/bus.rs`
- Bus submodules: `crates/retroboy_gb/src/machine/bus/` (e.g. `mmio.rs`, `ppu.rs`, `timer_io.rs`)
- Machine submodules: `crates/retroboy_gb/src/machine/` (e.g. `timer.rs`, `cartridge.rs`, `serial.rs`, `video.rs`)

## Running

The workspace ships a small SDL frontend in the `retroboy` binary:

- Run a ROM: `cargo run -p retroboy -- gb path/to/rom.gb`
- PowerShell example with logging: `$env:RUST_LOG="info"; cargo run -p retroboy -- gb path/to/rom.gb`
- For "supports CGB" ROMs (header flag `0x80`), opt into CGB mode with: `$env:RETROBOY_GB_PREFER_CGB=1`

Key mapping (current default):

- D-pad: arrow keys
- A/B: Z/X
- Select/Start: A/S

Notes:

- Mooneye acceptance ROMs under `assets/mooneye/acceptance/` are mostly non-interactive; many will not show meaningful graphics.
- The current video output is a simplified renderer driven by scanline snapshots; cycle-accurate PPU behaviour is still a work in progress.
- The repo does not ship commercial game ROMs; use homebrew or your own dumps. Keep any local-only ROMs under `repo-ref/` (not pushed).

## Utilities

- Dump a frame to raw RGB24 (160x144x3 bytes):
  - `cargo run -p retroboy_gb --bin gb_frame_dump -- path/to/rom.gb out.rgb24 240`
  - `cargo run -p retroboy_gb --bin gb_frame_dump -- path/to/rom.gb out.rgb24 --until-ldbb`

## Testing

- Run the crate tests: `cargo test -p retroboy_gb`
- Run ROM tests that are `#[ignore]`: `cargo test -p retroboy_gb -- --ignored`
- Test ROMs are vendored under `assets/mooneye/acceptance/` and `assets/roms/gb_tests/` (see `assets/mooneye/README.md`).
- The test harness is designed to work without any local `repo-ref/` checkout.
