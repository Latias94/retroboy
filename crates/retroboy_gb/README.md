# RetroBoy Game Boy (DMG)

This crate will host the Game Boy (DMG) emulator for the RetroBoy workspace.

Current status:

- [x] Crate skeleton and basic module layout
- [ ] CPU core (LR35902) implementation
- [ ] Memory/bus and cartridge support
- [ ] PPU and frame timing
- [ ] APU and audio output
- [ ] Debugging tools and test ROM harness

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

Key mapping (current default):

- D-pad: arrow keys
- A/B: Z/X
- Select/Start: A/S

Notes:

- Mooneye acceptance ROMs under `assets/mooneye/acceptance/` are mostly non-interactive; many will not show meaningful graphics.
- The current video output is a simplified VRAM renderer; if `LCDC` or `BG` is disabled, the window will appear white.
- The repo does not ship commercial game ROMs; use homebrew or your own dumps. Keep any local-only ROMs under `repo-ref/` (not pushed).

## Testing

- Run the crate tests: `cargo test -p retroboy_gb`
- Run ROM tests that are `#[ignore]`: `cargo test -p retroboy_gb -- --ignored`
- Test ROMs are vendored under `assets/mooneye/acceptance/` and `assets/roms/gb_tests/` (see `assets/mooneye/README.md`).
- The test harness is designed to work without any local `repo-ref/` checkout.
