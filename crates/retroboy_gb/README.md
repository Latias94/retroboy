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

