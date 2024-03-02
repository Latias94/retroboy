# RetroBoy Chip-8

A Chip-8 emulator

<p align="left">
  <img src="../../misc/screenshots/chip8_sdl2_tetris.png" width="49%" alt="chip8_sdl2_tetris" />
  <img src="../../misc/screenshots/chip8_web_pong.png" width="49%"  alt="chip8_web_pong"/>
</p>

## Build

Install [Just](https://github.com/casey/just?tab=readme-ov-file#installation).

### Build with SDL2

```shell
# run default rom: Tetris
just chip8
# custom rom
just chip8 "assets/roms/chip8/games/Pong (1 player).ch8"
```

### Build with WebAssembly

```shell
cargo install wasm-pack
# build wasm and run
just chip8-web
```

If you meet `failed to download binaryen-version_xxx-xxx` error, you can follow this [issue](https://github.com/rustwasm/wasm-pack/issues/864#issuecomment-1878525823) to fix it. 

## Todo

- [ ] Add sound

## References

- [aquova/chip8-book](https://github.com/aquova/chip8-book)
