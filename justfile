chip8 ROM_PATH="":
  echo 'Run RetroBoy Chip-8 Emulator(SDL2) with ROM: {{ROM_PATH}}'
  RUST_LOG="info" cargo run --release --bin retroboy -- "{{ROM_PATH}}"

chip8-web:
  @echo 'Run RetroBoy Chip-8 Emulator(Wasm)'
  cd crates/retroboy_wasm && wasm-pack build --target web && mv pkg/wasm_bg.wasm ../../web && mv pkg/wasm.js ../../web
  @echo 'Open http://localhost:8000 in your browser to play!'
  cd web && python3 -m http.server
