# RetroBoy Chip-8

A Chip-8 emulator

## Build

Install [Just](https://github.com/casey/just?tab=readme-ov-file#installation) and run:

```shell
# run default rom: Tetris
just chip8
# custom rom
just chip8 "assets/roms/chip8/games/Pong (1 player).ch8"
# build wasm and run
just chip8-web
```

## Build WebAssembly

```shell
cargo install wasm-pack
just chip8-web
```

If you meet `failed to download binaryen-version_xxx-xxx` error, you can follow this [issue](https://github.com/rustwasm/wasm-pack/issues/864#issuecomment-1878525823) to fix it. 

## References

- [aquova/chip8-book](https://github.com/aquova/chip8-book)
