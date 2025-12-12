RetroBoy Space Invaders
=======================

This crate will contain the Space Invaders arcade emulator for the Retroboy
workspace.

Goals:

- Implement an Intel 8080 CPU core that passes the standard CPU test ROMs.
- Model the original Space Invaders memory map, IO ports, and video layout.
- Integrate with the shared `retroboy_common::app::App` interface so that
  it can be driven by the SDL2 frontend and, later, a WASM frontend.

