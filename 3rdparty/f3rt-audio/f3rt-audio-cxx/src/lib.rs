//! cxx.rs bridge from MAME's `taito_en_hle_device` to the f3rt HLE sound board.
//!
//! Unlike `f3rt_audio::abi` (one process-wide instance behind a C ABI), each `AudioBoard` is an
//! owned `Box` the device creates in `device_start` and drops in `device_stop`. Every call must come
//! from the thread that owns the board (MAME's emulation thread); the board runs its own synth
//! worker thread internally.

use f3rt_audio::audio::Audio;
use f3rt_audio::hle::events::{MAIN_CLOCK, SAMPLE_RATE};
use f3rt_audio::snapshot;

#[cxx::bridge(namespace = "f3rt")]
mod ffi {
    extern "Rust" {
        type AudioBoard;

        /// Output rate of `render`, in Hz.
        fn sample_rate() -> u32;
        /// Rate of the tick counter `advance` takes, in Hz.
        fn main_clock() -> u32;
        /// Size of the blob `save_state` writes and `load_state` reads.
        fn state_size() -> usize;

        /// `sound_rom`: the 512 KiB sound-CPU image (0xc00000-0xc7ffff) as big-endian bytes.
        /// `sample_words`: the ES5505 sample ROM as host-order 16-bit words.
        /// `shared_ram`: the 2 KiB dual-port RAM the main CPU writes.
        ///
        /// # Safety
        /// `sound_rom` and the 0x800 bytes at `shared_ram` must stay valid, and not move, until
        /// the returned board is dropped.
        unsafe fn new_audio_board(
            sound_rom: &[u8],
            sample_words: &[u16],
            shared_ram: *mut u8,
        ) -> Result<Box<AudioBoard>>;

        /// The main CPU wrote `shared_ram[offset]`.
        fn shared_write(self: &mut AudioBoard, offset: u32);
        /// Sound CPU reset line (true = held in reset).
        fn set_reset(self: &mut AudioBoard, asserted: bool);
        /// Sound CPU program bank for the 0xc20000-0xc3ffff window.
        fn set_sound_bank(self: &mut AudioBoard, page: u32);
        /// Moves board time forward by `main_cycles` ticks of `main_clock()`.
        fn advance(self: &mut AudioBoard, main_cycles: u32);
        /// Board time in ticks of `main_clock()`.
        fn clock_ticks(self: &AudioBoard) -> u64;
        /// Waits for the synth to reach board time, then drains up to `out.len() / 2`
        /// interleaved (L, R) frames in -1..1. Returns the number of frames written.
        fn render(self: &mut AudioBoard, out: &mut [f32]) -> usize;

        fn save_state(self: &AudioBoard, dst: &mut [u8]);
        fn load_state(self: &mut AudioBoard, src: &[u8]) -> Result<()>;
    }
}

pub struct AudioBoard(Audio);

fn sample_rate() -> u32 {
    SAMPLE_RATE
}

fn main_clock() -> u32 {
    MAIN_CLOCK
}

fn state_size() -> usize {
    snapshot::BYTES
}

unsafe fn new_audio_board(
    sound_rom: &[u8],
    sample_words: &[u16],
    shared_ram: *mut u8,
) -> Result<Box<AudioBoard>, String> {
    if shared_ram.is_null() {
        return Err("shared RAM pointer is null".into());
    }
    let words = sample_words.to_vec();
    if !Audio::supported(sound_rom, &words) {
        return Err("unsupported sound ROM (the HLE drives the Land Maker sound program, CRC32 5a7e9117)".into());
    }
    // SAFETY: the caller keeps `sound_rom` alive until the board is dropped (see the bridge docs).
    let rom: &'static [u8] = unsafe { std::slice::from_raw_parts(sound_rom.as_ptr(), sound_rom.len()) };
    Ok(Box::new(AudioBoard(Audio::new(rom, words, shared_ram))))
}

impl AudioBoard {
    fn shared_write(&mut self, offset: u32) {
        self.0.shared_write(offset);
    }

    fn set_reset(&mut self, asserted: bool) {
        self.0.set_reset(asserted);
    }

    fn set_sound_bank(&mut self, page: u32) {
        self.0.set_sound_bank(page);
    }

    fn advance(&mut self, main_cycles: u32) {
        self.0.advance(main_cycles);
    }

    fn clock_ticks(&self) -> u64 {
        self.0.clock_ticks()
    }

    fn render(&mut self, out: &mut [f32]) -> usize {
        let frames = out.len() / 2;
        self.0.render_f32(out, frames)
    }

    fn save_state(&self, dst: &mut [u8]) {
        self.0.save_state(dst);
    }

    fn load_state(&mut self, src: &[u8]) -> Result<(), String> {
        self.0.load_state(src).map_err(str::to_owned)
    }
}
