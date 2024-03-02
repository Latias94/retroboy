chip8 ROM_PATH="":
  echo 'Run RetroBoy Chip-8 Emulator(SDL2) with ROM: {{ROM_PATH}}'
  RUST_LOG="info" cargo run --release --bin retroboy -- "{{ROM_PATH}}"

chip8-web:
  @echo 'Run RetroBoy Chip-8 Emulator(Wasm)'
  cd crates/retroboy_chip8 && wasm-pack build --target web --features wasm && mv pkg/retroboy_chip8_bg.wasm ../../web/retroboy_chip8_bg.wasm && mv pkg/retroboy_chip8.js ../../web/retroboy_chip8.js
  @echo 'Open http://localhost:8000 in your browser to play!'
  cd web && python3 -m http.server
