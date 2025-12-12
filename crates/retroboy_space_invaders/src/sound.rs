use std::collections::{HashMap, HashSet};
use std::fs;
use std::io::{BufReader, Cursor};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;

use log::{error, warn};
use rodio::{Decoder, OutputStream, Sink};

/// Logical sound identifiers for the discrete audio outputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum SoundType {
    Fire,
    InvaderDies,
    PlayerDies,
    Ufo,
    Invader1,
    Invader2,
    Invader3,
    Invader4,
    UfoHit,
}

/// Message sent from the main thread to the audio thread.
pub struct Message {
    pub sound_type: SoundType,
    pub on: bool,
}

struct SoundThread {
    receiver: Receiver<Message>,
    sound_files: HashMap<SoundType, Vec<u8>>,
}

impl SoundThread {
    fn new(receiver: Receiver<Message>) -> Option<Self> {
        let mut sound_files = HashMap::new();

        for info in ALL_SOUNDS.iter() {
            match fs::read(info.path) {
                Ok(bytes) => {
                    sound_files.insert(info.sound_type, bytes);
                }
                Err(e) => {
                    warn!(
                        "Failed to load sound {:?} from {}: {e}",
                        info.sound_type, info.path
                    );
                }
            }
        }

        if sound_files.is_empty() {
            warn!("No Space Invaders sound files could be loaded, disabling audio");
            return None;
        }

        Some(Self {
            receiver,
            sound_files,
        })
    }

    fn run(self) {
        // Keep the stream alive as long as the audio thread runs.
        let Ok((stream, stream_handle)) = OutputStream::try_default() else {
            error!("Failed to open default audio output stream, disabling audio");
            return;
        };
        let _stream = stream;

        let Ok(sink) = Sink::try_new(&stream_handle) else {
            error!("Failed to create audio sink, disabling audio");
            return;
        };

        loop {
            match self.receiver.recv() {
                Ok(msg) => {
                    if !msg.on {
                        // We only trigger sounds on rising edges.
                        continue;
                    }

                    if let Some(bytes) = self.sound_files.get(&msg.sound_type) {
                        let cursor = Cursor::new(bytes.clone());
                        let reader = BufReader::new(cursor);

                        match Decoder::new(reader) {
                            Ok(source) => {
                                sink.append(source);
                                // Block until the current sound finishes.
                                sink.sleep_until_end();
                            }
                            Err(e) => {
                                error!("Failed to decode sound {:?}: {e}", msg.sound_type);
                            }
                        }
                    } else {
                        warn!("No audio data for sound {:?}", msg.sound_type);
                    }
                }
                Err(e) => {
                    warn!("Audio channel closed: {e}");
                    break;
                }
            }
        }
    }
}

/// Mapping between output bits and logical sounds.
pub struct SoundInfo {
    pub sound_type: SoundType,
    pub path: &'static str,
    pub channel: u8,
    pub bit: u8,
}

impl SoundInfo {
    const fn new(sound_type: SoundType, path: &'static str, channel: u8, bit: u8) -> Self {
        Self {
            sound_type,
            path,
            channel,
            bit,
        }
    }
}

/// All sound definitions for Space Invaders.
///
/// Paths are relative to the repository root; we expect to be run from the
/// workspace root so that these assets can be found.
pub const ALL_SOUNDS: &[SoundInfo] = &[
    // Port 3: discrete sounds
    // bit 0 = UFO (repeats)
    SoundInfo::new(
        SoundType::Ufo,
        "assets/sounds/space_invaders/ufo_lowpitch.wav",
        3,
        0,
    ),
    // bit 1 = shot
    SoundInfo::new(
        SoundType::Fire,
        "assets/sounds/space_invaders/shoot.wav",
        3,
        1,
    ),
    // bit 2 = player die
    SoundInfo::new(
        SoundType::PlayerDies,
        "assets/sounds/space_invaders/explosion.wav",
        3,
        2,
    ),
    // bit 3 = invader die
    SoundInfo::new(
        SoundType::InvaderDies,
        "assets/sounds/space_invaders/invaderkilled.wav",
        3,
        3,
    ),
    // Port 5: fleet movement and UFO hit
    SoundInfo::new(
        SoundType::Invader1,
        "assets/sounds/space_invaders/fastinvader1.wav",
        5,
        0,
    ),
    SoundInfo::new(
        SoundType::Invader2,
        "assets/sounds/space_invaders/fastinvader2.wav",
        5,
        1,
    ),
    SoundInfo::new(
        SoundType::Invader3,
        "assets/sounds/space_invaders/fastinvader3.wav",
        5,
        2,
    ),
    SoundInfo::new(
        SoundType::Invader4,
        "assets/sounds/space_invaders/fastinvader4.wav",
        5,
        3,
    ),
    SoundInfo::new(
        SoundType::UfoHit,
        "assets/sounds/space_invaders/explosion.wav",
        5,
        4,
    ),
];

/// Lightweight controller living on the main thread that watches output
/// bits and sends messages to the audio thread when sound status changes.
pub struct SoundManager {
    sender: Sender<Message>,
    active: HashSet<SoundType>,
}

impl SoundManager {
    /// Try to start the audio thread and create a new manager.
    ///
    /// If audio initialization fails (e.g. no output device), this returns
    /// `None` and the emulator will run silently.
    pub fn new() -> Option<Self> {
        let (sender, receiver) = mpsc::channel::<Message>();

        let Some(sound_thread) = SoundThread::new(receiver) else {
            return None;
        };

        if let Err(e) = thread::Builder::new()
            .name("space_invaders_sound".into())
            .spawn(move || sound_thread.run())
        {
            error!("Failed to spawn Space Invaders audio thread: {e}");
            return None;
        }

        Some(Self {
            sender,
            active: HashSet::new(),
        })
    }

    /// Update the sound state based on the current values of OUT 3 and OUT 5.
    ///
    /// This performs the same "edge detection" logic as the reference
    /// implementation: we only send a message when a bit toggles, to avoid
    /// re-triggering sounds every frame.
    pub fn update(&mut self, out3: u8, out5: u8) {
        for info in ALL_SOUNDS.iter() {
            let value = match info.channel {
                3 => out3,
                5 => out5,
                _ => 0,
            };

            let sound_type = info.sound_type;
            let was_playing = self.active.contains(&sound_type);
            let on = (value & (1 << info.bit)) != 0;

            if on {
                self.active.insert(sound_type);
            } else {
                self.active.remove(&sound_type);
            }

            if on ^ was_playing {
                // Ignore send errors; if the audio thread has gone away we
                // simply stop playing new sounds.
                let _ = self.sender.send(Message { sound_type, on });
            }
        }
    }
}
