use std::path::PathBuf;

use retroboy_gb::{GameBoy, SCREEN_HEIGHT, SCREEN_WIDTH};

fn main() {
    env_logger::init();

    let mut args = std::env::args().skip(1);
    let rom_path: PathBuf = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            eprintln!("Usage: gb_frame_dump <rom_path> <out_rgb24_path> [frames|--until-ldbb]");
            std::process::exit(2);
        });
    let out_path: PathBuf = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            eprintln!("Usage: gb_frame_dump <rom_path> <out_rgb24_path> [frames|--until-ldbb]");
            std::process::exit(2);
        });
    let mode = args.next().unwrap_or_else(|| "120".to_string());
    let frames: Option<u32> = if mode == "--until-ldbb" {
        None
    } else {
        Some(mode.parse().unwrap_or_else(|_| {
            eprintln!("Invalid frames; expected integer or '--until-ldbb'.");
            std::process::exit(2);
        }))
    };

    let rom = std::fs::read(&rom_path).unwrap_or_else(|err| {
        eprintln!("Failed to read ROM '{}': {err}", rom_path.display());
        std::process::exit(1);
    });

    let mut gb = GameBoy::new();
    gb.load_rom(&rom);

    match frames {
        Some(frames) => {
            for _ in 0..frames {
                gb.step_frame();
            }
        }
        None => {
            let hit = gb.step_until_software_breakpoint(50_000_000);
            if !hit {
                eprintln!("Did not hit LD B,B software breakpoint within instruction budget.");
                std::process::exit(1);
            }
        }
    }

    let mut buffer = vec![0u8; SCREEN_WIDTH * SCREEN_HEIGHT * 3];
    gb.video_frame(&mut buffer);

    std::fs::write(&out_path, &buffer).unwrap_or_else(|err| {
        eprintln!("Failed to write '{}': {err}", out_path.display());
        std::process::exit(1);
    });

    if let Some(frames) = frames {
        println!(
            "Wrote {} bytes ({}x{} rgb24) after {} frames to '{}'",
            buffer.len(),
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            frames,
            out_path.display()
        );
    } else {
        println!(
            "Wrote {} bytes ({}x{} rgb24) at LD B,B breakpoint to '{}'",
            buffer.len(),
            SCREEN_WIDTH,
            SCREEN_HEIGHT,
            out_path.display()
        );
    }
}
