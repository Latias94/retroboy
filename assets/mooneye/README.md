Mooneye Test ROMs
==================

This directory contains a curated subset of the [mooneye-gb test suite]
by Joonas Javanainen, used as executable specifications for the Game Boy
CPU, timer, interrupt controller, and PPU timing.

The upstream project and license:

- Project: https://github.com/Gekkio/mooneye-gb
- License: MIT (see headers in the original `.s` sources)

Only a small set of ROMs is checked in here. They are intended to be
committed to version control so that the emulator's correctness tests
do not depend on an external checkout under `repo-ref/`.

Layout
------

The directory structure mirrors the upstream `acceptance` layout:

- `assets/mooneye/acceptance/timer`
- `assets/mooneye/acceptance/interrupts`
- `assets/mooneye/acceptance/ppu`

The ROMs currently used by tests are:

### Timer (`assets/mooneye/acceptance/timer`)

- `tim00.gb`, `tim00_div_trigger.gb`
- `tim01.gb`, `tim01_div_trigger.gb`
- `tim10.gb`, `tim10_div_trigger.gb`
- `tim11.gb`, `tim11_div_trigger.gb`
- `div_write.gb`
- `rapid_toggle.gb`
- `tima_reload.gb`
- `tima_write_reloading.gb`
- `tma_write_reloading.gb`

These exercise DIV/TIMA/TMA/TAC behaviour, timer enable/disable edges,
reload timing, and interrupt generation.

### Interrupts (`assets/mooneye/acceptance/interrupts`)

- `interrupts/ie_push.gb`
- `ei_sequence.gb`
- `ei_timing.gb`
- `intr_timing.gb`
- `if_ie_registers.gb`

These cover EI/IME delay semantics, IE/IF register behaviour, nested
interrupt entry timing, and the special case where the interrupt vector
is selected after the high byte of PC has been pushed (as in `ie_push`).

### PPU (`assets/mooneye/acceptance/ppu`)

- `vblank_stat_intr-GS.gb`
- `intr_2_0_timing.gb`

These validate the interaction between VBlank and STAT interrupts and
the timing between STAT mode 2 (OAM scan) and mode 0 (HBlank) on a
mid-screen line.

Adding New ROMs
---------------

If additional mooneye ROMs are needed:

1. Copy the `.gb` file into the appropriate subdirectory under
   `assets/mooneye/acceptance/`.
2. Update the test helpers in `crates/retroboy_gb/src/cpu.rs` to load
   the ROM via the `load_mooneye_*_rom` functions.
3. Add `#[test]` wrappers under `mod tests` in `cpu.rs`, usually marked
   `#[ignore]` so they run only on demand.

