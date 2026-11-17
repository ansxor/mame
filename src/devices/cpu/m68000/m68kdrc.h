// license:BSD-3-Clause
// copyright-holders:ansxor
/***************************************************************************

    Motorola MC68EC020 dynamic recompiler

    A drcuml front end layered on the Musashi interpreter. Musashi's
    register file stays authoritative: compiled code loads and stores the
    same members the interpreter uses, so any instruction the recompiler
    does not lower natively is handed to the interpreter one at a time
    without copying state back and forth.

****************************************************************************/
#ifndef MAME_CPU_M68000_M68KDRC_H
#define MAME_CPU_M68000_M68KDRC_H

#pragma once

#include "m68020.h"

#include "cpu/drcfe.h"
#include "cpu/drcuml.h"

#include <memory>
#include <vector>


class m68ec020drc_device : public m68ec020_device
{
public:
	m68ec020drc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);
	virtual ~m68ec020drc_device();

	// Plain RAM/ROM the recompiler may access directly instead of through the
	// address space. `base` holds host-order 32-bit words for [start, end];
	// `mirror` bits are ignored when matching. Read-only regions are also
	// the only memory code is compiled from; code anywhere else runs on the
	// interpreter, so self-modifying code needs no invalidation.
	void add_fastram(offs_t start, offs_t end, bool readonly, void *base, offs_t mirror = 0);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void execute_run() override;

private:
	class opcode_desc;
	class frontend;
	struct compiler_state;
	struct c_funcs;

	struct fastram_entry
	{
		offs_t start;
		offs_t end;
		offs_t mask;      // address bits compared against start/end
		bool readonly;
		u32 *base;
	};

	struct ea_addr
	{
		bool constant;   // address known at compile time
		offs_t value;
	};

	// codegen
	void code_flush_cache();
	void code_compile_block(offs_t pc);
	void static_generate_entry_point();
	void generate_interpreted_block(drcuml_block &block, offs_t pc);
	void generate_sequence_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc);
	void generate_fallback(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc);
	void generate_update_cycles(drcuml_block &block, int cycles, uml::parameter exitpc);
	void generate_redispatch_check(drcuml_block &block, compiler_state &compiler, offs_t nextpc);
	bool generate_native(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc);
	bool reject(int line) { m_reject_line = line; return false; }

	// native lowering helpers
	void load_reg(drcuml_block &block, uml::parameter dst, int reg);
	void store_reg(drcuml_block &block, int reg, uml::parameter src);
	void store_dreg(drcuml_block &block, int reg, uml::parameter src, int size);
	void store_flag(drcuml_block &block, u32 &flag, uml::parameter src);
	bool ea_supported(const opcode_desc &desc, int mode, int reg, int size, int &ext);
	void ea_index(drcuml_block &block, const opcode_desc &desc, int &ext, uml::parameter dst, bool pcrel, offs_t pcbase);
	void generate_runtime_fallback(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc);
	ea_addr ea_compute(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter dst);
	void ea_read(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter dst);
	void ea_write(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter src, bool movel_pd);
	u32 read_imm(const opcode_desc &desc, int size, int &ext) const;
	const fastram_entry *fastram_lookup(offs_t address, int size, bool write) const;
	void generate_slow_access(drcuml_block &block, const opcode_desc &desc, void (*func)(m68ec020drc_device &), uml::parameter addr, const uml::parameter *data);
	void generate_read(drcuml_block &block, const opcode_desc &desc, int size, ea_addr a, uml::parameter addr, uml::parameter dst);
	void generate_write(drcuml_block &block, const opcode_desc &desc, int size, ea_addr a, uml::parameter addr, uml::parameter src, bool movel_pd);
	void generate_push32(drcuml_block &block, const opcode_desc &desc, uml::parameter value);
	void generate_flags_logic(drcuml_block &block, int size, uml::parameter res);
	void generate_arith(drcuml_block &block, int op, int size, uml::parameter lhs, uml::parameter rhs, bool setx);
	void generate_alu(drcuml_block &block, int op, int size, uml::parameter src);
	void generate_condition(drcuml_block &block, int cond, uml::code_label skip);
	void generate_branch(drcuml_block &block, const opcode_desc &desc, offs_t target, int cycles);

	const fastram_entry *code_region(offs_t pc, offs_t length) const;

	// statistics (M68KDRC_STATS environment variable)
	void dump_stats();

	std::unique_ptr<drcuml_state> m_drcuml;
	std::unique_ptr<frontend> m_drcfe;
	bool m_drc_enabled;
	bool m_cache_dirty;

	uml::code_handle *m_entry;
	uml::code_handle *m_nocode;
	uml::code_handle *m_out_of_cycles;

	std::vector<fastram_entry> m_fastram;

	// compiled code <-> C helpers
	u32 m_drc_arg0;
	u32 m_drc_arg1;
	u32 m_drc_bail;

	uml::code_label m_labelnum;     // next free label in the block being compiled
	u32 m_drc_disable;
	int m_reject_line = 0;
	int m_ea_cycles = 0;            // extra cycles of full-format EAs in the current instruction          // source line of the last native rejection (stats)              // instruction families forced to the interpreter

	bool m_stats_enabled;
	std::unique_ptr<u64 []> m_stats_fallback;
	u64 m_stats_compiles;
};

DECLARE_DEVICE_TYPE(M68EC020DRC, m68ec020drc_device)

#endif // MAME_CPU_M68000_M68KDRC_H
