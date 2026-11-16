// license:BSD-3-Clause
// copyright-holders:ansxor
/***************************************************************************

    Taito Ensoniq sound board, high-level emulation

    Drop-in replacement for taito_en_device (same tag, ROM regions and
    taito_en_interface) backed by the f3rt HLE sound board from f3-recomp
    (3rdparty/f3rt-audio), bound through cxx.rs.

****************************************************************************/
#ifndef MAME_TAITO_TAITO_EN_HLE_H
#define MAME_TAITO_TAITO_EN_HLE_H

#pragma once

#include "taito_en.h"

#include <array>
#include <memory>
#include <vector>


namespace f3rt { struct AudioBoard; }

class taito_en_hle_device : public device_t, public device_sound_interface, public taito_en_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	taito_en_hle_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);
	virtual ~taito_en_hle_device();

	// taito_en_interface
	virtual u8 dpram_r(offs_t offset) override;
	virtual void dpram_w(offs_t offset, u8 data) override;
	virtual void reset_w(int state) override;
	virtual void set_bank(int bank, int entry) override;

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_pre_save() override;
	virtual void device_post_load() override;

	virtual void sound_stream_update(sound_stream &stream) override;

private:
	static constexpr offs_t SOUND_IMAGE_BASE = 0x100000; // audiocpu region offset of 0xc00000
	static constexpr offs_t SOUND_IMAGE_SIZE = 0x80000;  // 0xc00000-0xc7ffff

	void sync(const attotime &time);
	void release_board();

	required_memory_region m_osrom;
	required_memory_region m_otisrom;

	sound_stream *m_stream;
	f3rt::AudioBoard *m_board;     // owned; released through rust::Box in release_board()
	attoseconds_t m_attos_per_tick;

	std::vector<u8> m_sound_image; // big-endian bytes, borrowed by the board
	std::array<u8, 0x800> m_dpram; // shared RAM, borrowed by the board
	std::unique_ptr<u8[]> m_state;
	size_t m_state_size;
	std::vector<float> m_pcm;
	float m_last[2];
};

DECLARE_DEVICE_TYPE(TAITO_EN_HLE, taito_en_hle_device)

#endif // MAME_TAITO_TAITO_EN_HLE_H
