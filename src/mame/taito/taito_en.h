// license:BSD-3-Clause
// copyright-holders:Bryan McPhail, Aaron Giles, R. Belmont, Philip Bennett
/***************************************************************************

    Taito Ensoniq ES5505-based sound hardware

****************************************************************************/
#ifndef MAME_TAITO_TAITO_EN_H
#define MAME_TAITO_TAITO_EN_H

#pragma once

#include "cpu/es5510/es5510.h"
#include "cpu/m68000/m68000.h"
#include "machine/mc68681.h"
#include "machine/mb8421.h"
#include "sound/es5506.h"
#include "sound/esqpump.h"
#include "sound/mb87077.h"

// What the host board sees of the Taito Ensoniq sound board: the main-CPU side of the shared
// MB8421 mailbox, the sound CPU reset line and the sound program bank. Implemented by the
// low-level taito_en_device and by the HLE taito_en_hle_device.
class taito_en_interface : public device_interface
{
public:
	virtual u8 dpram_r(offs_t offset) = 0;
	virtual void dpram_w(offs_t offset, u8 data) = 0;
	virtual void reset_w(int state) = 0;
	virtual void set_bank(int bank, int entry) = 0;

protected:
	taito_en_interface(const machine_config &mconfig, device_t &device) : device_interface(device, "taito_en") { }
};

class taito_en_device : public device_t, public device_mixer_interface, public taito_en_interface
{
public:
	static constexpr feature_type imperfect_features() { return feature::SOUND; }

	taito_en_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	m68000_device &audiocpu() { return *m_audiocpu; }

	// taito_en_interface
	virtual u8 dpram_r(offs_t offset) override { return m_dpram->left_r(offset); }
	virtual void dpram_w(offs_t offset, u8 data) override { m_dpram->left_w(offset, data); }
	virtual void reset_w(int state) override { m_audiocpu->set_input_line(INPUT_LINE_RESET, state ? ASSERT_LINE : CLEAR_LINE); }
	virtual void set_bank(int bank, int entry) override { m_cpubank[bank]->set_entry(entry); }

protected:
	// device-level overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	void en_sound_map(address_map &map) ATTR_COLD;
	void en_otis_map(address_map &map) ATTR_COLD;
	void fc7_map(address_map &map) ATTR_COLD;

	// inherited devices/pointers
	required_device<m68000_device> m_audiocpu;
	required_device<es5505_device> m_ensoniq;
	required_device<es5510_device> m_esp;
	required_device<esq_5505_5510_pump_device> m_pump;
	required_device<mc68681_device> m_duart68681;
	required_device<mb87078_device> m_mb87078;
	required_device<mb8421_device> m_dpram;

	required_shared_ptr<uint16_t> m_osram;
	required_shared_ptr<uint16_t> m_otisbank;
	required_region_ptr<uint16_t> m_otisrom;

	required_memory_region m_osrom;
	required_memory_bank_array<3> m_cpubank;

	uint32_t m_bankmask = 0;
	uint32_t m_old_clock = ~0;

	std::unique_ptr<offs_t[]> m_calculated_otisbank;

	IRQ_CALLBACK_MEMBER(duart_iack);
	void duart_output(uint8_t data);

	void mb87078_gain_changed(offs_t offset, uint8_t data);
	void es5505_clock_changed(u32 data);

	void en_es5505_bank_w(offs_t offset, uint16_t data);
	void en_volume_w(offs_t offset, uint8_t data);
};

DECLARE_DEVICE_TYPE(TAITO_EN, taito_en_device)

#endif // MAME_TAITO_TAITO_EN_H
