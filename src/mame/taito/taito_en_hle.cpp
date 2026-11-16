// license:BSD-3-Clause
// copyright-holders:ansxor
/***************************************************************************

    Taito Ensoniq sound board, high-level emulation

    Instead of running the 68000 sound program against the ES5505/ES5510,
    the f3rt HLE (from f3-recomp) reads the main CPU's mailbox commands out
    of the shared MB8421 RAM and synthesises the board's stereo output
    directly at the ES5505 output rate.

    Timing: the HLE counts board time in ticks of f3rt::main_clock(). Every
    host event (mailbox write, reset, bank change) and every stream update
    first advances that counter to the current machine time, so commands are
    timestamped exactly where the main CPU issued them.

    TODO:
    * Only the Land Maker sound program (CRC32 5a7e9117) is supported;
      device_start fails on anything else.
    * Save states only carry the board's mailbox/clock snapshot, so notes
      already playing when a state is saved are not restored.

****************************************************************************/

#include "emu.h"
#include "taito_en_hle.h"

#include "rust/cxx.h"
#include "f3rt-audio-cxx/src/lib.rs.h"

#include <algorithm>


DEFINE_DEVICE_TYPE(TAITO_EN_HLE, taito_en_hle_device, "taito_en_hle", "Taito Ensoniq Sound System (HLE)")

taito_en_hle_device::taito_en_hle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, TAITO_EN_HLE, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, taito_en_interface(mconfig, *this)
	, m_osrom(*this, "audiocpu")
	, m_otisrom(*this, "ensoniq")
	, m_stream(nullptr)
	, m_board(nullptr)
	, m_attos_per_tick(0)
	, m_state_size(0)
	, m_last{ 0.0f, 0.0f }
{
	m_dpram.fill(0);
}

taito_en_hle_device::~taito_en_hle_device()
{
	release_board();
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void taito_en_hle_device::device_start()
{
	if (m_osrom->bytes() < SOUND_IMAGE_BASE + SOUND_IMAGE_SIZE)
		throw emu_fatalerror("%s: audiocpu region too small for the sound program", tag());

	// The HLE wants the image as big-endian bytes. With no 68000 claiming the
	// region it stays 8-bit (raw ROM byte order); a 16-bit region holds host-order words.
	m_sound_image.resize(SOUND_IMAGE_SIZE);
	if (m_osrom->bytewidth() == 2)
	{
		for (offs_t i = 0; i < SOUND_IMAGE_SIZE / 2; i++)
		{
			const u16 word = m_osrom->as_u16(SOUND_IMAGE_BASE / 2 + i);
			m_sound_image[i * 2 + 0] = word >> 8;
			m_sound_image[i * 2 + 1] = word & 0xff;
		}
	}
	else
	{
		std::copy_n(m_osrom->base() + SOUND_IMAGE_BASE, SOUND_IMAGE_SIZE, m_sound_image.begin());
	}

	// the sample region is declared ROM_REGION16_BE, so it holds host-order words
	if (m_otisrom->bytewidth() != 2)
		throw emu_fatalerror("%s: ensoniq region must be a 16-bit region", tag());

	const rust::Slice<const u8> sound(m_sound_image.data(), m_sound_image.size());
	const rust::Slice<const u16> samples(&m_otisrom->as_u16(), m_otisrom->bytes() / 2);
	try
	{
		m_board = f3rt::new_audio_board(sound, samples, m_dpram.data()).into_raw();
	}
	catch (const rust::Error &err)
	{
		throw emu_fatalerror("%s: %s", tag(), err.what());
	}

	const u32 main_clock = f3rt::main_clock();
	if (ATTOSECONDS_PER_SECOND % main_clock)
		throw emu_fatalerror("%s: HLE clock %u Hz is not a whole number of attoseconds", tag(), main_clock);
	m_attos_per_tick = ATTOSECONDS_PER_SECOND / main_clock;

	m_stream = stream_alloc(0, 2, f3rt::sample_rate());

	m_state_size = f3rt::state_size();
	m_state = std::make_unique<u8[]>(m_state_size);

	save_item(NAME(m_dpram));
	save_pointer(NAME(m_state), m_state_size);
	save_item(NAME(m_last));
}

void taito_en_hle_device::device_stop()
{
	release_board();
}

void taito_en_hle_device::release_board()
{
	// joins the synth worker before the borrowed buffers go away
	if (m_board)
		(void)rust::Box<f3rt::AudioBoard>::from_raw(std::exchange(m_board, nullptr));
}


//-------------------------------------------------
//  save states
//-------------------------------------------------

void taito_en_hle_device::device_pre_save()
{
	m_board->save_state(rust::Slice<u8>(m_state.get(), m_state_size));
}

void taito_en_hle_device::device_post_load()
{
	try
	{
		m_board->load_state(rust::Slice<const u8>(m_state.get(), m_state_size));
	}
	catch (const rust::Error &err)
	{
		logerror("load_state rejected: %s\n", err.what());
	}
}


//-------------------------------------------------
//  board time
//-------------------------------------------------

void taito_en_hle_device::sync(const attotime &time)
{
	const u64 target = u64(time.seconds()) * f3rt::main_clock() + u64(time.attoseconds() / m_attos_per_tick);
	u64 current = m_board->clock_ticks();
	while (current < target)
	{
		const u32 step = u32(std::min<u64>(target - current, 0xffffffffU));
		m_board->advance(step);
		current += step;
	}
}


//-------------------------------------------------
//  taito_en_interface
//-------------------------------------------------

u8 taito_en_hle_device::dpram_r(offs_t offset)
{
	return m_dpram[offset & 0x7ff];
}

void taito_en_hle_device::dpram_w(offs_t offset, u8 data)
{
	offset &= 0x7ff;
	sync(machine().time());
	m_dpram[offset] = data;
	m_board->shared_write(offset);
}

void taito_en_hle_device::reset_w(int state)
{
	sync(machine().time());
	m_board->set_reset(state != 0);
}

void taito_en_hle_device::set_bank(int bank, int entry)
{
	// taito_en bank 1 is the 0xc20000-0xc3ffff window, the only one the HLE pages
	if (bank != 1)
	{
		logerror("set_bank(%d, %d): only bank 1 is banked in the HLE\n", bank, entry);
		return;
	}
	sync(machine().time());
	m_board->set_sound_bank(entry);
}


//-------------------------------------------------
//  sound_stream_update - render up to the end of
//  the stream window; hold the last frame if the
//  synth is a sample short of the window
//-------------------------------------------------

void taito_en_hle_device::sound_stream_update(sound_stream &stream)
{
	sync(stream.end_time());

	const int samples = stream.samples();
	m_pcm.resize(size_t(samples) * 2);
	const size_t frames = m_board->render(rust::Slice<float>(m_pcm.data(), m_pcm.size()));

	for (size_t i = 0; i < frames; i++)
	{
		m_last[0] = m_pcm[i * 2 + 0];
		m_last[1] = m_pcm[i * 2 + 1];
		stream.put(0, i, m_last[0]);
		stream.put(1, i, m_last[1]);
	}
	if (frames < size_t(samples))
	{
		stream.fill(0, m_last[0], frames);
		stream.fill(1, m_last[1], frames);
	}
}
