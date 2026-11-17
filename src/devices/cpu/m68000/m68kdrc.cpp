// license:BSD-3-Clause
// copyright-holders:ansxor
/***************************************************************************

    Motorola MC68EC020 dynamic recompiler

    Design notes (several carried over from the f3-recomp static
    recompiler, recomp/ and include/f3rt/cpu_abi.h in that project):

    * Musashi owns the CPU state. Compiled code reaches the register file,
      flags, PC and icount through UML LOAD/STORE on the device members, so
      handing an instruction to the interpreter (generate_fallback) is a
      plain call with no state transfer. Coverage can therefore grow one
      instruction family at a time while everything else stays exact.

    * Cycle accounting is Musashi's, instruction by instruction: every
      compiled instruction subtracts the same count the interpreter would and
      leaves when icount drops to zero, at the same instruction boundary.
      A run with -drc and one with -nodrc execute the same instruction
      stream against the same machine time, which is what makes the two
      directly comparable.

    * Memory: naturally aligned accesses to regions registered with
      add_fastram() are done inline on the host buffer (the recomp's RAM and
      ROM windows); everything else calls the interpreter's own m68ki_read_*
      and m68ki_write_* so devices see the identical bus traffic.

    * Code is only compiled from read-only fastram. Any other PC runs on the
      interpreter through a one-instruction block, so writes never have to
      invalidate translated code.

    * Every compiled instruction is a potential entry point (HASH per
      sequence head), and any instruction that leaves to the interpreter is
      followed by a PC check that redispatches when the interpreter changed
      the flow (exceptions, interrupts unmasked by an SR write, etc.).

****************************************************************************/

#include "emu.h"
#include "m68kdrc.h"

#include "m68kdasm.h"

#include "cpu/drcfe.ipp"
#include "cpu/drcumlsh.h"

#include "debugger.h"

#include "util/endianness.h"

#include <algorithm>
#include <bitset>
#include <cstdlib>


DEFINE_DEVICE_TYPE(M68EC020DRC, m68ec020drc_device, "m68ec020drc", "Motorola MC68EC020 (DRC)")


namespace {

constexpr u32 CACHE_SIZE = 32 * 1024 * 1024;
constexpr u32 COMPILE_BACKWARDS_BYTES = 128;
constexpr u32 COMPILE_FORWARDS_BYTES = 512;
constexpr u32 COMPILE_MAX_SEQUENCE = 64;
constexpr u32 COMPILE_MAX_INSTRUCTIONS = (COMPILE_BACKWARDS_BYTES + COMPILE_FORWARDS_BYTES) / 2;

constexpr int MAX_INSTRUCTION_WORDS = 11; // 68020: opcode + two full-format extensions

constexpr u32 EXECUTE_OUT_OF_CYCLES = 0;
constexpr u32 EXECUTE_MISSING_CODE  = 1;
constexpr u32 EXECUTE_UNMAPPED_CODE = 2;
constexpr u32 EXECUTE_RESET_CACHE   = 3;

inline void alloc_handle(drcuml_state &drcuml, uml::code_handle *&handleptr, const char *name)
{
	if (!handleptr)
		handleptr = drcuml.handle_alloc(name);
}

} // anonymous namespace


/***************************************************************************
    DESCRIPTORS AND FRONT END
***************************************************************************/

// flag "registers" tracked for liveness
class m68ec020drc_device::opcode_desc : public opcode_desc_base<opcode_desc, 8>
{
public:
	enum { REG_C = 0, REG_V, REG_Z, REG_N, REG_X, REG_COUNT };

	u16 op[MAX_INSTRUCTION_WORDS];

	void reset(offs_t curpc, bool in_delay_slot)
	{
		opcode_desc_base::reset(curpc, in_delay_slot);
		std::fill(std::begin(op), std::end(op), 0);
	}
};


class m68ec020drc_device::frontend : public drc_frontend_base<opcode_desc>
{
public:
	frontend(m68ec020drc_device &cpu)
		: drc_frontend_base(0, COMPILE_BACKWARDS_BYTES, COMPILE_FORWARDS_BYTES, COMPILE_MAX_SEQUENCE)
		, m_cpu(cpu)
		, m_dasm(m68k_disassembler::TYPE_68020)
	{
	}

	const opcode_desc *describe_code(offs_t startpc)
	{
		return do_describe_code(
				[this] (opcode_desc &desc, const opcode_desc *prev) { return describe(desc, prev); },
				startpc);
	}

	offs_t disassemble(std::ostream &stream, offs_t pc)
	{
		const buffer buf(m_cpu);
		return m_dasm.disassemble(stream, pc, buf, buf);
	}

private:
	class buffer : public util::disasm_interface::data_buffer
	{
	public:
		buffer(m68ec020drc_device &cpu) : m_cpu(cpu) { }
		virtual u8  r8 (offs_t pc) const override { return m_cpu.m_oprogram32.read_byte(pc); }
		virtual u16 r16(offs_t pc) const override { return m_cpu.m_oprogram32.read_word(pc); }
		virtual u32 r32(offs_t pc) const override { return (u32(r16(pc)) << 16) | r16(pc + 2); }
		virtual u64 r64(offs_t pc) const override { return (u64(r32(pc)) << 32) | r32(pc + 4); }
	private:
		m68ec020drc_device &m_cpu;
	};

	bool describe(opcode_desc &desc, const opcode_desc *prev);

	m68ec020drc_device &m_cpu;
	m68k_disassembler m_dasm;
};


bool m68ec020drc_device::frontend::describe(opcode_desc &desc, const opcode_desc *prev)
{
	const offs_t pc = desc.pc;

	// only compile from read-only fastram; leave everything else (and odd
	// PCs, which raise an address error) to the interpreter block
	if ((pc & 1) || !m_cpu.code_region(pc, MAX_INSTRUCTION_WORDS * 2))
	{
		desc.length = 2;
		desc.set_end_sequence();
		desc.set_redispatch();
		return true;
	}

	for (int i = 0; i < MAX_INSTRUCTION_WORDS; i++)
		desc.op[i] = m_cpu.m_oprogram32.read_word(pc + 2 * i);

	std::ostringstream dummy;
	const offs_t result = disassemble(dummy, pc);
	desc.length = std::max<offs_t>(result & util::disasm_interface::LENGTHMASK, 2);
	const u16 op = desc.op[0];

	// flow control: anything the interpreter may redirect is redispatched
	// at run time, so only mark what affects sequencing
	if ((op & 0xf000) == 0x6000)
	{
		// Bcc / BRA / BSR
		s32 disp = s8(op & 0xff);
		if ((op & 0xff) == 0x00)
			disp = s16(desc.op[1]);
		else if ((op & 0xff) == 0xff)
			disp = s32((u32(desc.op[1]) << 16) | desc.op[2]);
		desc.targetpc = pc + 2 + disp;
		const int cond = (op >> 8) & 0xf;
		if (cond == 0 || cond == 1)
		{
			desc.set_is_unconditional_branch();
			desc.set_end_sequence();
		}
		else
		{
			desc.set_is_conditional_branch();
		}
	}
	else if ((op & 0xf0f8) == 0x50c8)
	{
		// DBcc
		desc.targetpc = pc + 2 + s16(desc.op[1]);
		desc.set_is_conditional_branch();
	}
	else if ((op & 0xff80) == 0x4e80)
	{
		// JSR / JMP
		const int mode = (op >> 3) & 7, reg = op & 7;
		if (mode == 7 && reg == 0)
			desc.targetpc = s16(desc.op[1]);
		else if (mode == 7 && reg == 1)
			desc.targetpc = (u32(desc.op[1]) << 16) | desc.op[2];
		else if (mode == 7 && reg == 2)
			desc.targetpc = pc + 2 + s16(desc.op[1]);
		desc.set_is_unconditional_branch();
		desc.set_end_sequence();
	}
	else if ((op & 0xfff8) == 0x4e70 && op != 0x4e71 && op != 0x4e76)
	{
		// RESET, STOP, RTE, RTD, RTS, RTR (not NOP, TRAPV)
		desc.set_is_unconditional_branch();
		desc.set_end_sequence();
	}
	else if (op == 0x4afc || (op & 0xf000) == 0xa000 || (op & 0xfff0) == 0x4e40)
	{
		// ILLEGAL, line A, TRAP #n
		desc.set_is_unconditional_branch();
		desc.set_end_sequence();
	}

	return true;
}


/***************************************************************************
    COMPILER STATE AND C CALLBACKS
***************************************************************************/

struct m68ec020drc_device::compiler_state
{
	compiler_state(uml::code_label &labels) : labelnum(labels) { labelnum = 1; }

	uml::code_label &labelnum;
	uml::code_label redispatch = 0;  // label that HASHJMPs to I0, allocated on demand
	uml::code_label bail = 0;        // label that exits to execute_run, allocated on demand

	// Conditional EXIT is avoided on purpose: drcbe_x64::op_exit loads the
	// exit code with `xor eax,eax` between the test and the jcc, which
	// clobbers the flags when the code is 0.

	uml::code_label next_label() { return labelnum++; }
};


struct m68ec020drc_device::c_funcs
{
	// execute the instruction at m_pc on the interpreter, exactly as one
	// iteration of m68000_musashi_device::execute_run's loop
	static void interpret_one(m68ec020drc_device &d)
	{
		d.m_ppc = d.m_pc;
		d.m68ki_rerun_expire();
		d.m68ki_trace_t1();
		d.m_run_mode = RUN_MODE_NORMAL;

		if (d.m_stats_enabled)
			d.m_stats_fallback[d.m_oprogram32.read_word(d.m_pc)]++;

		bool ok = true;
		try
		{
			d.m_ir = d.m68ki_read_imm_16();
			(d.*m68k_handler_table[d.m_state_table[d.m_ir]])();
			d.m_icount -= d.m_cyc_instruction[d.m_ir];
		}
		catch (int error)
		{
			// address error; never unwind through generated code
			d.m_address_error = 1;
			ok = false;
		}
		if (ok)
			d.m68ki_exception_if_trace();

		d.m_drc_bail = (d.m_icount <= 0) || d.m_stopped || d.m_address_error || d.m_t1_flag || d.m_t0_flag;
	}

	static void read8(m68ec020drc_device &d)  { d.m_drc_arg1 = d.m68ki_read_8(d.m_drc_arg0); }
	static void read16(m68ec020drc_device &d) { d.m_drc_arg1 = d.m68ki_read_16(d.m_drc_arg0); }
	static void read32(m68ec020drc_device &d) { d.m_drc_arg1 = d.m68ki_read_32(d.m_drc_arg0); }
	static void write8(m68ec020drc_device &d)  { d.m68ki_write_8(d.m_drc_arg0, d.m_drc_arg1); }
	static void write16(m68ec020drc_device &d) { d.m68ki_write_16(d.m_drc_arg0, d.m_drc_arg1); }
	static void write32(m68ec020drc_device &d) { d.m68ki_write_32(d.m_drc_arg0, d.m_drc_arg1); }
	// MOVE.L to -(An): low word at ea+2 first, then the high word at ea
	static void write32pd(m68ec020drc_device &d)
	{
		d.m68ki_write_16(d.m_drc_arg0 + 2, d.m_drc_arg1 & 0xffff);
		d.m68ki_write_16(d.m_drc_arg0, d.m_drc_arg1 >> 16);
	}
};


/***************************************************************************
    DEVICE
***************************************************************************/

m68ec020drc_device::m68ec020drc_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: m68ec020_device(mconfig, tag, owner, clock, M68EC020DRC)
	, m_drc_enabled(false)
	, m_cache_dirty(true)
	, m_entry(nullptr)
	, m_nocode(nullptr)
	, m_out_of_cycles(nullptr)
	, m_drc_arg0(0)
	, m_drc_arg1(0)
	, m_drc_bail(0)
	, m_labelnum(1)
	, m_drc_disable(0)
	, m_stats_enabled(false)
	, m_stats_compiles(0)
{
}

m68ec020drc_device::~m68ec020drc_device()
{
}


void m68ec020drc_device::add_fastram(offs_t start, offs_t end, bool readonly, void *base, offs_t mirror)
{
	assert(!(start & 3) && ((end & 3) == 3));
	fastram_entry entry;
	entry.start = start;
	entry.end = end;
	entry.mask = 0x00ffffff & ~mirror;
	entry.readonly = readonly;
	entry.base = reinterpret_cast<u32 *>(base);
	m_fastram.push_back(entry);
	m_cache_dirty = true;
}


const m68ec020drc_device::fastram_entry *m68ec020drc_device::code_region(offs_t pc, offs_t length) const
{
	for (const fastram_entry &r : m_fastram)
	{
		const offs_t a = pc & r.mask;
		if (r.readonly && a >= r.start && (a + length - 1) <= r.end)
			return &r;
	}
	return nullptr;
}


void m68ec020drc_device::device_start()
{
	m68ec020_device::device_start();

	m_drc_enabled = allow_drc();
	if (!m_drc_enabled)
		return;

	const u32 umlflags = 0;
	m_drcuml = std::make_unique<drcuml_state>(*this, CACHE_SIZE);
	m_drcuml->initialize(umlflags, 1, 32, 0, COMPILE_MAX_INSTRUCTIONS);
	m_drcfe = std::make_unique<frontend>(*this);

	m_drcuml->symbol_add(&m_pc, sizeof(m_pc), "pc");
	m_drcuml->symbol_add(&m_icount, sizeof(m_icount), "icount");
	for (int i = 0; i < 16; i++)
	{
		char name[4];
		snprintf(name, sizeof(name), "%c%d", (i < 8) ? 'd' : 'a', i & 7);
		m_drcuml->symbol_add(&m_dar[i], sizeof(m_dar[i]), name);
	}
	m_drcuml->symbol_add(&m_x_flag, sizeof(m_x_flag), "xflag");
	m_drcuml->symbol_add(&m_n_flag, sizeof(m_n_flag), "nflag");
	m_drcuml->symbol_add(&m_not_z_flag, sizeof(m_not_z_flag), "notzflag");
	m_drcuml->symbol_add(&m_v_flag, sizeof(m_v_flag), "vflag");
	m_drcuml->symbol_add(&m_c_flag, sizeof(m_c_flag), "cflag");

	if (const char *disable = std::getenv("M68KDRC_DISABLE"); disable && *disable)
		m_drc_disable = u32(std::strtoul(disable, nullptr, 16));

	if (const char *stats = std::getenv("M68KDRC_STATS"); stats && *stats && *stats != '0')
	{
		m_stats_enabled = true;
		m_stats_fallback = std::make_unique<u64 []>(0x10000);
		std::fill_n(m_stats_fallback.get(), 0x10000, 0);
	}

	m_cache_dirty = true;
}


void m68ec020drc_device::device_reset()
{
	m68ec020_device::device_reset();
	m_cache_dirty = true;
}


void m68ec020drc_device::device_stop()
{
	if (m_stats_enabled)
		dump_stats();

	// fingerprint of the final CPU state and writable fastram, for comparing
	// -drc against -nodrc runs (M68KDRC_FINGERPRINT)
	if (const char *fp = std::getenv("M68KDRC_FINGERPRINT"); fp && *fp && *fp != '0')
	{
		u64 h = 1469598103934665603ULL;
		auto mix = [&h] (u32 v) { h = (h ^ v) * 1099511628211ULL; };
		for (u32 r : m_dar)
			mix(r);
		mix(m_pc);
		mix(m68ki_get_sr());
		mix(u32(total_cycles()));
		mix(u32(total_cycles() >> 32));
		for (const fastram_entry &r : m_fastram)
			if (!r.readonly)
				for (offs_t i = 0; i <= (r.end - r.start) / 4; i++)
					mix(r.base[i]);
		osd_printf_info("m68kdrc fingerprint: %016X pc=%08X cycles=%u\n", h, m_pc, total_cycles());
	}
	m_drcfe.reset();
	m_drcuml.reset();
	m68ec020_device::device_stop();
}


void m68ec020drc_device::dump_stats()
{
	std::vector<std::pair<u64, u16>> top;
	u64 total = 0;
	for (int op = 0; op < 0x10000; op++)
		if (m_stats_fallback[op])
		{
			top.emplace_back(m_stats_fallback[op], u16(op));
			total += m_stats_fallback[op];
		}
	std::sort(top.begin(), top.end(), [] (auto const &a, auto const &b) { return a.first > b.first; });

	m68k_disassembler dasm(m68k_disassembler::TYPE_68020);
	class opbuf : public util::disasm_interface::data_buffer
	{
	public:
		u16 op;
		virtual u8  r8 (offs_t pc) const override { return 0; }
		virtual u16 r16(offs_t pc) const override { return pc ? 0 : op; }
		virtual u32 r32(offs_t pc) const override { return (u32(r16(pc)) << 16) | r16(pc + 2); }
		virtual u64 r64(offs_t pc) const override { return 0; }
	} buf;

	osd_printf_info("m68kdrc: %u block compiles, %u fallback instructions\n", m_stats_compiles, total);
	for (size_t i = 0; i < std::min<size_t>(top.size(), 80); i++)
	{
		std::ostringstream str;
		buf.op = top[i].second;
		dasm.disassemble(str, 0, buf, buf);
		osd_printf_info("  %04X %12u %5.2f%%  %s\n", top[i].second, top[i].first, 100.0 * top[i].first / total, str.str());
	}
}


/***************************************************************************
    EXECUTION
***************************************************************************/

void m68ec020drc_device::execute_run()
{
	if (!m_drc_enabled || (machine().debug_flags & DEBUG_FLAG_ENABLED))
	{
		m68ec020_device::execute_run();
		return;
	}

	if (m_cache_dirty)
		code_flush_cache();

	// prologue of m68000_musashi_device::execute_run
	m_initial_cycles = m_icount;

	if (m_reset_cycles)
	{
		REG_SP() = m68ki_read_imm_32();
		m_pc = m68ki_read_imm_32();
		m68ki_jump(m_pc);

		const int rc = m_reset_cycles;
		m_reset_cycles = 0;
		m_icount -= rc;

		if (m_icount <= 0)
			return;
	}

	m68ki_check_interrupts();

	while (m_icount > 0)
	{
		if (m_stopped)
		{
			m_icount = 0;
			break;
		}

		// tracing and address errors stay on the interpreter for the rest
		// of the timeslice
		if (m_t1_flag || m_t0_flag || m_address_error)
		{
			m68ec020_device::execute_run();
			return;
		}

		m_drc_bail = 0;
		const int result = m_drcuml->execute(*m_entry);
		if (result == EXECUTE_MISSING_CODE)
			code_compile_block(m_pc);
		else if (result == EXECUTE_UNMAPPED_CODE)
			fatalerror("%s: attempted to execute unmapped code at PC=%08X\n", tag(), m_pc);
		else if (result == EXECUTE_RESET_CACHE)
			code_flush_cache();
	}

	m_ppc = m_pc;
}


/***************************************************************************
    CACHE MANAGEMENT AND STATIC CODE
***************************************************************************/

void m68ec020drc_device::code_flush_cache()
{
	m_drcuml->reset();
	try
	{
		static_generate_entry_point();
	}
	catch (drcuml_block::abort_compilation &)
	{
		fatalerror("%s: unable to generate static code\n", tag());
	}
	m_cache_dirty = false;
}


void m68ec020drc_device::static_generate_entry_point()
{
	drcuml_block &block(m_drcuml->begin_block(32));

	alloc_handle(*m_drcuml, m_entry, "entry");
	alloc_handle(*m_drcuml, m_nocode, "nocode");
	alloc_handle(*m_drcuml, m_out_of_cycles, "out_of_cycles");

	// entry: dispatch to m_pc
	UML_HANDLE(block, *m_entry);
	UML_LOAD(block, uml::I0, &m_pc, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_HASHJMP(block, 0, uml::I0, *m_nocode);

	// no code compiled for the PC in the exception parameter
	UML_HANDLE(block, *m_nocode);
	UML_GETEXP(block, uml::I0);
	UML_STORE(block, &m_pc, 0, uml::I0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_EXIT(block, EXECUTE_MISSING_CODE);

	// icount ran out; the exception parameter is the next PC
	UML_HANDLE(block, *m_out_of_cycles);
	UML_GETEXP(block, uml::I0);
	UML_STORE(block, &m_pc, 0, uml::I0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_EXIT(block, EXECUTE_OUT_OF_CYCLES);

	block.end();
}


/***************************************************************************
    BLOCK COMPILATION
***************************************************************************/

void m68ec020drc_device::code_compile_block(offs_t pc)
{
	auto profile = g_profiler.start(PROFILER_DRC_COMPILE);
	m_stats_compiles++;

	// anything outside compilable memory runs one instruction at a time
	if ((pc & 1) || !code_region(pc, MAX_INSTRUCTION_WORDS * 2))
	{
		for (int attempt = 0; attempt < 2; attempt++)
		{
			try
			{
				drcuml_block &block(m_drcuml->begin_block(32));
				generate_interpreted_block(block, pc);
				block.end();
				return;
			}
			catch (drcuml_block::abort_compilation &)
			{
				code_flush_cache();
			}
		}
		fatalerror("%s: unable to compile interpreter block at %08X\n", tag(), pc);
	}

	const opcode_desc *desclist = m_drcfe->describe_code(pc);

	bool override = false;
	for (int attempt = 0; attempt < 2; attempt++)
	{
		try
		{
			drcuml_block &block(m_drcuml->begin_block(COMPILE_MAX_INSTRUCTIONS * 96));
			compiler_state compiler(m_labelnum);

			const opcode_desc *seqlast = nullptr;
			for (const opcode_desc *seqhead = desclist; seqhead; seqhead = seqlast->next())
			{
				for (seqlast = seqhead; seqlast; seqlast = seqlast->next())
					if (seqlast->end_sequence())
						break;
				assert(seqlast);

				if (override || !m_drcuml->hash_exists(0, seqhead->pc))
				{
					UML_HASH(block, 0, seqhead->pc);
				}
				else if (seqhead == desclist)
				{
					// recompiling a PC we already have: allow overriding the rest
					override = true;
					UML_HASH(block, 0, seqhead->pc);
				}
				else
				{
					// already compiled elsewhere: just jump there
					UML_LABEL(block, seqhead->pc | 0x80000000);
					UML_HASHJMP(block, 0, seqhead->pc, *m_nocode);
					continue;
				}

				if (seqhead->is_branch_target())
					UML_LABEL(block, seqhead->pc | 0x80000000);

				for (const opcode_desc *curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
					generate_sequence_instruction(block, compiler, *curdesc);

				// fall out of the sequence
				const offs_t nextpc = seqlast->pc + seqlast->length;
				if (!seqlast->is_unconditional_branch() && (!seqlast->next() || seqlast->next()->pc != nextpc))
					UML_HASHJMP(block, 0, nextpc, *m_nocode);
			}

			if (compiler.redispatch)
			{
				UML_LABEL(block, compiler.redispatch);
				UML_HASHJMP(block, 0, uml::I0, *m_nocode);
			}
			if (compiler.bail)
			{
				UML_LABEL(block, compiler.bail);
				UML_EXIT(block, EXECUTE_OUT_OF_CYCLES);
			}

			block.end();
			return;
		}
		catch (drcuml_block::abort_compilation &)
		{
			code_flush_cache();
			override = false;
		}
	}
	fatalerror("%s: unable to compile block at %08X\n", tag(), pc);
}


void m68ec020drc_device::generate_interpreted_block(drcuml_block &block, offs_t pc)
{
	UML_HASH(block, 0, pc);
	UML_STORE(block, &m_pc, 0, pc, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_CALLC(block, &c_funcs::interpret_one, this);
	UML_LOAD(block, uml::I0, &m_drc_bail, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_TEST(block, uml::I0, uml::I0);
	UML_JMPc(block, uml::COND_NZ, 1);
	UML_LOAD(block, uml::I0, &m_pc, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_HASHJMP(block, 0, uml::I0, *m_nocode);
	UML_LABEL(block, 1);
	UML_EXIT(block, EXECUTE_OUT_OF_CYCLES);
}


void m68ec020drc_device::generate_sequence_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc)
{
	if (m_drcuml->logging())
	{
		std::ostringstream str;
		m_drcfe->disassemble(str, desc.pc);
		block.append_comment("%08X: %s", desc.pc, str.str());
	}

	if (!generate_native(block, compiler, desc))
	{
		if (m_stats_enabled)
		{
			std::ostringstream str;
			m_drcfe->disassemble(str, desc.pc);
			osd_printf_verbose("m68kdrc: interpreting %08X %s (line %d)\n", desc.pc, str.str(), m_reject_line);
		}
		generate_fallback(block, compiler, desc);
	}
}


void m68ec020drc_device::generate_fallback(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc)
{
	const offs_t nextpc = desc.pc + desc.length;

	UML_STORE(block, &m_pc, 0, desc.pc, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_CALLC(block, &c_funcs::interpret_one, this);
	UML_LOAD(block, uml::I0, &m_drc_bail, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_TEST(block, uml::I0, uml::I0);
	if (!compiler.bail)
		compiler.bail = compiler.next_label();
	UML_JMPc(block, uml::COND_NZ, compiler.bail);

	// the interpreter may have changed the flow
	UML_LOAD(block, uml::I0, &m_pc, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	if (desc.is_unconditional_branch())
	{
		UML_HASHJMP(block, 0, uml::I0, *m_nocode);
	}
	else
	{
		generate_redispatch_check(block, compiler, nextpc);
	}
}


void m68ec020drc_device::generate_redispatch_check(drcuml_block &block, compiler_state &compiler, offs_t nextpc)
{
	// expects the actual PC in I0
	if (!compiler.redispatch)
		compiler.redispatch = compiler.next_label();
	UML_CMP(block, uml::I0, nextpc);
	UML_JMPc(block, uml::COND_NE, compiler.redispatch);
}


void m68ec020drc_device::generate_update_cycles(drcuml_block &block, int cycles, uml::parameter exitpc)
{
	// same accounting as the interpreter loop: subtract, leave at <= 0
	UML_LOAD(block, uml::I2, &m_icount, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_SUB(block, uml::I2, uml::I2, cycles + m_ea_cycles);
	UML_STORE(block, &m_icount, 0, uml::I2, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_EXHc(block, uml::COND_LE, *m_out_of_cycles, exitpc);
}


/***************************************************************************
    NATIVE LOWERING

    Register conventions inside one instruction:
      I0  effective address (preserved across memory accesses)
      I1  destination operand
      I2  scratch: memory accessors, cycle counting
      I3  result
      I4  source operand
      I5, I6  flag scratch

    Flags are kept in Musashi's representation (N/V in bit 7, C/X in
    bit 8, Z inverted as a nonzero value), so the interpreter can pick up
    exactly where compiled code left off.
***************************************************************************/

namespace {

constexpr u32 size_mask(int size) { return (size == 1) ? 0xffU : (size == 2) ? 0xffffU : 0xffffffffU; }
constexpr uml::operand_size uml_size(int size) { return (size == 1) ? uml::SIZE_BYTE : (size == 2) ? uml::SIZE_WORD : uml::SIZE_DWORD; }

// big-endian byte/word address -> offset into host-order 32-bit words
constexpr u32 FASTRAM_XOR_BYTE = NATIVE_ENDIAN_VALUE_LE_BE(3, 0);
constexpr u32 FASTRAM_XOR_WORD = NATIVE_ENDIAN_VALUE_LE_BE(2, 0);
constexpr u32 fastram_xor(int size) { return (size == 1) ? FASTRAM_XOR_BYTE : (size == 2) ? FASTRAM_XOR_WORD : 0; }

// bit 7 of a Musashi N flag for a result of the given size
constexpr int nflag_shift(int size) { return (size == 1) ? 0 : (size == 2) ? 8 : 24; }

// top n bits of an operand (m68ki_shift_*_table)
constexpr u32 top_bits(int size, int n) { return (n >= size * 8) ? size_mask(size) : (size_mask(size) & ~(size_mask(size) >> n)); }

// families that can be forced back to the interpreter (M68KDRC_DISABLE, hex mask)
enum : u32
{
	FAM_MOVE    = 1U << 0,
	FAM_MOVEQ   = 1U << 1,
	FAM_LEA     = 1U << 2,
	FAM_QUICK   = 1U << 3,
	FAM_ALU     = 1U << 4,
	FAM_ALUI    = 1U << 5,
	FAM_UNARY   = 1U << 6,
	FAM_REGOP   = 1U << 7,
	FAM_BCC     = 1U << 8,
	FAM_DBCC    = 1U << 9,
	FAM_JUMP    = 1U << 10,
	FAM_RTS     = 1U << 11,
	FAM_SCC     = 1U << 12,
	FAM_SHIFT   = 1U << 13,
	FAM_BIT     = 1U << 14,
	FAM_MUL     = 1U << 15,
	FAM_MISC    = 1U << 16,
	FAM_SHIFT_REG = 1U << 17,   // sub-families of FAM_SHIFT, for bisecting
	FAM_SHIFT_ASL = 1U << 18,
	FAM_SHIFT_ROT = 1U << 19,
	FAM_SHIFT_ASR = 1U << 20,
	FAM_SHIFT_LS  = 1U << 21
};

enum alu_op { ALU_ADD, ALU_SUB, ALU_CMP, ALU_AND, ALU_OR, ALU_EOR };

} // anonymous namespace


void m68ec020drc_device::load_reg(drcuml_block &block, uml::parameter dst, int reg)
{
	UML_LOAD(block, dst, &m_dar[0], reg, uml::SIZE_DWORD, uml::SCALE_x4);
}

void m68ec020drc_device::store_reg(drcuml_block &block, int reg, uml::parameter src)
{
	UML_STORE(block, &m_dar[0], reg, src, uml::SIZE_DWORD, uml::SCALE_x4);
}

void m68ec020drc_device::store_dreg(drcuml_block &block, int reg, uml::parameter src, int size)
{
	// sized write to a data register leaves the upper bits alone
	if (size == 4)
		store_reg(block, reg, src);
	else if (size == 2)
		UML_STORE(block, &m_dar[0], 4 * reg + NATIVE_ENDIAN_VALUE_LE_BE(0, 2), src, uml::SIZE_WORD, uml::SCALE_x1);
	else
		UML_STORE(block, &m_dar[0], 4 * reg + NATIVE_ENDIAN_VALUE_LE_BE(0, 3), src, uml::SIZE_BYTE, uml::SCALE_x1);
}

void m68ec020drc_device::store_flag(drcuml_block &block, u32 &flag, uml::parameter src)
{
	UML_STORE(block, &flag, 0, src, uml::SIZE_DWORD, uml::SCALE_x4);
}


//-------------------------------------------------
//  effective addresses
//-------------------------------------------------

// check that an EA can be lowered, and step past its extension words
bool m68ec020drc_device::ea_supported(const opcode_desc &desc, int mode, int reg, int size, int &ext)
{
	auto brief = [&] ()
	{
		if (ext >= MAX_INSTRUCTION_WORDS)
			return false;
		const u16 w = desc.op[ext++];
		if (!BIT(w, 8))
			return true;

		// full extension format (m68ki_get_ea_ix): reject the reserved encodings
		const int bdsize = (w >> 4) & 3, iis = w & 7;
		if (BIT(w, 3) || bdsize == 0 || (BIT(w, 6) && iis >= 4) || (!BIT(w, 6) && iis == 4))
			return false;
		ext += (bdsize == 2) ? 1 : (bdsize == 3) ? 2 : 0;
		ext += ((iis & 3) == 2) ? 1 : ((iis & 3) == 3) ? 2 : 0;
		m_ea_cycles += m68ki_ea_idx_cycle_table[w & 0x3f];
		return ext <= MAX_INSTRUCTION_WORDS;
	};
	switch (mode)
	{
	case 0: case 1: case 2: case 3: case 4:
		return true;
	case 5:
		ext++;
		return true;
	case 6:
		return brief();
	case 7:
		switch (reg)
		{
		case 0: ext += 1; return true;
		case 1: ext += 2; return true;
		case 2: ext += 1; return true;
		case 3: return brief();
		case 4: ext += (size == 4) ? 2 : 1; return true;
		}
		return false;
	}
	return false;
}


// indexed EA (brief or full format) for the extension word at desc.op[ext];
// dst holds the base register value, or is loaded with `pcbase` for PC modes
void m68ec020drc_device::ea_index(drcuml_block &block, const opcode_desc &desc, int &ext, uml::parameter dst, bool pcrel, offs_t pcbase)
{
	const u16 w = desc.op[ext++];
	auto load_index = [&] (uml::parameter reg)
	{
		load_reg(block, reg, w >> 12);
		if (!BIT(w, 11))
			UML_SEXT(block, reg, reg, uml::SIZE_WORD);
		if ((w >> 9) & 3)
			UML_SHL(block, reg, reg, (w >> 9) & 3);
	};

	if (!BIT(w, 8))
	{
		if (pcrel)
			UML_MOV(block, dst, pcbase);
		load_index(uml::I2);
		UML_ADD(block, dst, dst, uml::I2);
		if (s8(w))
			UML_ADD(block, dst, dst, s32(s8(w)));
		return;
	}

	// full format
	const bool bs = BIT(w, 7), is = BIT(w, 6);
	const int bdsize = (w >> 4) & 3, iis = w & 7;
	if (bs)
		UML_MOV(block, dst, 0);
	else if (pcrel)
		UML_MOV(block, dst, pcbase);
	if (!is)
		load_index(uml::I5);

	u32 bd = 0, od = 0;
	if (bdsize == 2)
		bd = u32(s32(s16(desc.op[ext++])));
	else if (bdsize == 3)
	{
		bd = (u32(desc.op[ext]) << 16) | desc.op[ext + 1];
		ext += 2;
	}
	if (bd)
		UML_ADD(block, dst, dst, bd);

	if (!iis)
	{
		if (!is)
			UML_ADD(block, dst, dst, uml::I5);
		return;
	}

	if ((iis & 3) == 2)
		od = u32(s32(s16(desc.op[ext++])));
	else if ((iis & 3) == 3)
	{
		od = (u32(desc.op[ext]) << 16) | desc.op[ext + 1];
		ext += 2;
	}

	if (BIT(iis, 2))
	{
		// postindexed: [base + bd] + Xn + od
		generate_read(block, desc, 4, ea_addr{ false, 0 }, dst, uml::I6);
		UML_MOV(block, dst, uml::I6);
		if (!is)
			UML_ADD(block, dst, dst, uml::I5);
	}
	else
	{
		// preindexed: [base + bd + Xn] + od
		if (!is)
			UML_ADD(block, dst, dst, uml::I5);
		generate_read(block, desc, 4, ea_addr{ false, 0 }, dst, uml::I6);
		UML_MOV(block, dst, uml::I6);
	}
	if (od)
		UML_ADD(block, dst, dst, od);
}


// compute a memory EA into dst (modes 2-6, 7.0-7.3), applying (An)+/-(An)
m68ec020drc_device::ea_addr m68ec020drc_device::ea_compute(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter dst)
{
	const int step = (size == 1 && reg == 7) ? 2 : size;
	switch (mode)
	{
	case 2:
		load_reg(block, dst, 8 + reg);
		return ea_addr{ false, 0 };

	case 3:
		load_reg(block, dst, 8 + reg);
		UML_ADD(block, uml::I2, dst, step);
		store_reg(block, 8 + reg, uml::I2);
		return ea_addr{ false, 0 };

	case 4:
		load_reg(block, dst, 8 + reg);
		UML_SUB(block, dst, dst, step);
		store_reg(block, 8 + reg, dst);
		return ea_addr{ false, 0 };

	case 5:
		load_reg(block, dst, 8 + reg);
		UML_ADD(block, dst, dst, s32(s16(desc.op[ext++])));
		return ea_addr{ false, 0 };

	case 6:
		load_reg(block, dst, 8 + reg);
		ea_index(block, desc, ext, dst, false, 0);
		return ea_addr{ false, 0 };

	case 7:
		switch (reg)
		{
		case 0:
			return ea_addr{ true, u32(s32(s16(desc.op[ext++]))) };
		case 1:
		{
			const u32 a = (u32(desc.op[ext]) << 16) | desc.op[ext + 1];
			ext += 2;
			return ea_addr{ true, a };
		}
		case 2:
		{
			const u32 a = desc.pc + 2 * ext + s16(desc.op[ext]);
			ext++;
			return ea_addr{ true, a };
		}
		case 3:
			ea_index(block, desc, ext, dst, true, desc.pc + 2 * ext);
			return ea_addr{ false, 0 };
		}
		break;
	}
	throw emu_fatalerror("m68kdrc: bad EA %d/%d at %08X", mode, reg, desc.pc);
}


// read an operand of the given size into dst, zero-extended
void m68ec020drc_device::ea_read(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter dst)
{
	if (mode == 0 || mode == 1)
	{
		load_reg(block, dst, (mode << 3) | reg);
		if (size != 4)
			UML_AND(block, dst, dst, size_mask(size));
	}
	else if (mode == 7 && reg == 4)
	{
		u32 imm;
		if (size == 4)
		{
			imm = (u32(desc.op[ext]) << 16) | desc.op[ext + 1];
			ext += 2;
		}
		else
		{
			imm = desc.op[ext++] & size_mask(size);
		}
		UML_MOV(block, dst, imm);
	}
	else
	{
		const ea_addr a = ea_compute(block, desc, mode, reg, size, ext, uml::I0);
		generate_read(block, desc, size, a, uml::I0, dst);
	}
}


// immediate operand following the opcode (ORI/ANDI/.../CMPI, MULL etc.)
u32 m68ec020drc_device::read_imm(const opcode_desc &desc, int size, int &ext) const
{
	if (size == 4)
	{
		const u32 imm = (u32(desc.op[ext]) << 16) | desc.op[ext + 1];
		ext += 2;
		return imm;
	}
	return desc.op[ext++] & size_mask(size);
}


//-------------------------------------------------
//  memory
//-------------------------------------------------

const m68ec020drc_device::fastram_entry *m68ec020drc_device::fastram_lookup(offs_t address, int size, bool write) const
{
	if (address & (size - 1))
		return nullptr;
	for (const fastram_entry &r : m_fastram)
	{
		if (write && r.readonly)
			continue;
		const offs_t a = address & r.mask;
		if (a >= r.start && (a + size - 1) <= r.end)
			return &r;
	}
	return nullptr;
}


void m68ec020drc_device::generate_slow_access(drcuml_block &block, const opcode_desc &desc, void (*func)(m68ec020drc_device &), uml::parameter addr, const uml::parameter *data)
{
	// device handlers may look at the PC (logging), so publish it
	UML_STORE(block, &m_ppc, 0, desc.pc, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_STORE(block, &m_pc, 0, desc.pc + desc.length, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_STORE(block, &m_drc_arg0, 0, addr, uml::SIZE_DWORD, uml::SCALE_x4);
	if (data)
		UML_STORE(block, &m_drc_arg1, 0, *data, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_CALLC(block, func, this);
}


void m68ec020drc_device::generate_read(drcuml_block &block, const opcode_desc &desc, int size, ea_addr a, uml::parameter addr, uml::parameter dst)
{
	auto *const func = (size == 1) ? &c_funcs::read8 : (size == 2) ? &c_funcs::read16 : &c_funcs::read32;

	if (a.constant)
	{
		if (const fastram_entry *r = fastram_lookup(a.value, size, false))
		{
			const u32 offset = ((a.value & r->mask) - r->start) ^ fastram_xor(size);
			UML_LOAD(block, dst, r->base, offset, uml_size(size), uml::SCALE_x1);
		}
		else
		{
			generate_slow_access(block, desc, func, a.value, nullptr);
			UML_LOAD(block, dst, &m_drc_arg1, 0, uml::SIZE_DWORD, uml::SCALE_x4);
		}
		return;
	}

	const uml::code_label done = m_labelnum++;
	const uml::code_label slow = m_labelnum++;
	if (size != 1)
	{
		UML_TEST(block, addr, size - 1);
		UML_JMPc(block, uml::COND_NZ, slow);
	}
	for (const fastram_entry &r : m_fastram)
	{
		const uml::code_label next = m_labelnum++;
		UML_AND(block, uml::I2, addr, r.mask);
		if (r.start)
			UML_SUB(block, uml::I2, uml::I2, r.start);
		UML_CMP(block, uml::I2, r.end - r.start);
		UML_JMPc(block, uml::COND_A, next);
		if (fastram_xor(size))
			UML_XOR(block, uml::I2, uml::I2, fastram_xor(size));
		UML_LOAD(block, dst, r.base, uml::I2, uml_size(size), uml::SCALE_x1);
		UML_JMP(block, done);
		UML_LABEL(block, next);
	}
	UML_LABEL(block, slow);
	generate_slow_access(block, desc, func, addr, nullptr);
	UML_LOAD(block, dst, &m_drc_arg1, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_LABEL(block, done);
}


void m68ec020drc_device::generate_write(drcuml_block &block, const opcode_desc &desc, int size, ea_addr a, uml::parameter addr, uml::parameter src, bool movel_pd)
{
	auto *const func = (size == 1) ? &c_funcs::write8 : (size == 2) ? &c_funcs::write16 : movel_pd ? &c_funcs::write32pd : &c_funcs::write32;

	if (a.constant)
	{
		if (const fastram_entry *r = fastram_lookup(a.value, size, true))
		{
			const u32 offset = ((a.value & r->mask) - r->start) ^ fastram_xor(size);
			UML_STORE(block, r->base, offset, src, uml_size(size), uml::SCALE_x1);
		}
		else
		{
			generate_slow_access(block, desc, func, a.value, &src);
		}
		return;
	}

	const uml::code_label done = m_labelnum++;
	const uml::code_label slow = m_labelnum++;
	if (size != 1)
	{
		UML_TEST(block, addr, size - 1);
		UML_JMPc(block, uml::COND_NZ, slow);
	}
	for (const fastram_entry &r : m_fastram)
	{
		if (r.readonly)
			continue;
		const uml::code_label next = m_labelnum++;
		UML_AND(block, uml::I2, addr, r.mask);
		if (r.start)
			UML_SUB(block, uml::I2, uml::I2, r.start);
		UML_CMP(block, uml::I2, r.end - r.start);
		UML_JMPc(block, uml::COND_A, next);
		if (fastram_xor(size))
			UML_XOR(block, uml::I2, uml::I2, fastram_xor(size));
		UML_STORE(block, r.base, uml::I2, src, uml_size(size), uml::SCALE_x1);
		UML_JMP(block, done);
		UML_LABEL(block, next);
	}
	UML_LABEL(block, slow);
	generate_slow_access(block, desc, func, addr, &src);
	UML_LABEL(block, done);
}


// write an operand to a data register (sized) or to memory at a computed EA
void m68ec020drc_device::ea_write(drcuml_block &block, const opcode_desc &desc, int mode, int reg, int size, int &ext, uml::parameter src, bool movel_pd)
{
	if (mode == 0)
	{
		store_dreg(block, reg, src, size);
	}
	else
	{
		const ea_addr a = ea_compute(block, desc, mode, reg, size, ext, uml::I0);
		generate_write(block, desc, size, a, uml::I0, src, movel_pd);
	}
}


void m68ec020drc_device::generate_push32(drcuml_block &block, const opcode_desc &desc, uml::parameter value)
{
	// m68ki_push_32: predecrement SP, then write
	load_reg(block, uml::I0, 15);
	UML_SUB(block, uml::I0, uml::I0, 4);
	store_reg(block, 15, uml::I0);
	generate_write(block, desc, 4, ea_addr{ false, 0 }, uml::I0, value, false);
}


//-------------------------------------------------
//  flags
//-------------------------------------------------

// N, Z from a result already masked to size; V = C = 0
void m68ec020drc_device::generate_flags_logic(drcuml_block &block, int size, uml::parameter res)
{
	if (nflag_shift(size))
	{
		UML_SHR(block, uml::I5, res, nflag_shift(size));
		store_flag(block, m_n_flag, uml::I5);
	}
	else
	{
		store_flag(block, m_n_flag, res);
	}
	store_flag(block, m_not_z_flag, res);
	store_flag(block, m_v_flag, 0);
	store_flag(block, m_c_flag, 0);
}


// lhs - rhs or lhs + rhs at the operand size, with the 68k flags.
// lhs must be a register; rhs may be an immediate. Result in I3, masked.
void m68ec020drc_device::generate_arith(drcuml_block &block, int op, int size, uml::parameter lhs, uml::parameter rhs, bool setx)
{
	// align operands to the top of the word so the host flags apply
	const int shift = 32 - 8 * size;
	uml::parameter a = lhs, b = rhs;
	if (shift)
	{
		UML_SHL(block, uml::I6, lhs, shift);
		a = uml::I6;
		if (rhs.is_immediate())
		{
			b = u32(rhs.immediate() << shift);
		}
		else
		{
			UML_SHL(block, uml::I5, rhs, shift);
			b = uml::I5;
		}
	}

	if (op == ALU_ADD)
		UML_ADD(block, uml::I3, a, b);
	else
		UML_SUB(block, uml::I3, a, b);
	UML_SETc(block, uml::COND_C, uml::I2);
	UML_SETc(block, uml::COND_V, uml::I5);

	UML_SHL(block, uml::I2, uml::I2, 8);
	store_flag(block, m_c_flag, uml::I2);
	if (setx)
		store_flag(block, m_x_flag, uml::I2);
	UML_SHL(block, uml::I5, uml::I5, 7);
	store_flag(block, m_v_flag, uml::I5);
	UML_SHR(block, uml::I6, uml::I3, 24);
	store_flag(block, m_n_flag, uml::I6);
	store_flag(block, m_not_z_flag, uml::I3);
	if (shift)
		UML_SHR(block, uml::I3, uml::I3, shift);
}


// dst (I1, masked to size) op src -> I3, flags
void m68ec020drc_device::generate_alu(drcuml_block &block, int op, int size, uml::parameter src)
{
	switch (op)
	{
	case ALU_ADD:
		generate_arith(block, ALU_ADD, size, uml::I1, src, true);
		break;
	case ALU_SUB:
		generate_arith(block, ALU_SUB, size, uml::I1, src, true);
		break;
	case ALU_CMP:
		generate_arith(block, ALU_SUB, size, uml::I1, src, false);
		break;
	case ALU_AND:
		UML_AND(block, uml::I3, uml::I1, src);
		generate_flags_logic(block, size, uml::I3);
		break;
	case ALU_OR:
		UML_OR(block, uml::I3, uml::I1, src);
		generate_flags_logic(block, size, uml::I3);
		break;
	case ALU_EOR:
		UML_XOR(block, uml::I3, uml::I1, src);
		generate_flags_logic(block, size, uml::I3);
		break;
	}
}


// jump to skip when the 68k condition is false; fall through when true
void m68ec020drc_device::generate_condition(drcuml_block &block, int cond, uml::code_label skip)
{
	auto test_c = [&] () { UML_LOAD(block, uml::I2, &m_c_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4); UML_TEST(block, uml::I2, 0x100); };
	auto test_z = [&] () { UML_LOAD(block, uml::I2, &m_not_z_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4); UML_TEST(block, uml::I2, uml::I2); };
	auto test_v = [&] () { UML_LOAD(block, uml::I2, &m_v_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4); UML_TEST(block, uml::I2, 0x80); };
	auto test_n = [&] () { UML_LOAD(block, uml::I2, &m_n_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4); UML_TEST(block, uml::I2, 0x80); };
	auto test_nv = [&] ()
	{
		UML_LOAD(block, uml::I2, &m_n_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4);
		UML_LOAD(block, uml::I5, &m_v_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4);
		UML_XOR(block, uml::I2, uml::I2, uml::I5);
		UML_TEST(block, uml::I2, 0x80);
	};

	switch (cond)
	{
	case 0x0: break;                                                        // T
	case 0x1: UML_JMP(block, skip); break;                                  // F
	case 0x2: test_c(); UML_JMPc(block, uml::COND_NZ, skip); test_z(); UML_JMPc(block, uml::COND_Z, skip); break; // HI
	case 0x3:                                                               // LS
	{
		const uml::code_label take = m_labelnum++;
		test_c(); UML_JMPc(block, uml::COND_NZ, take);
		test_z(); UML_JMPc(block, uml::COND_NZ, skip);
		UML_LABEL(block, take);
		break;
	}
	case 0x4: test_c(); UML_JMPc(block, uml::COND_NZ, skip); break;        // CC
	case 0x5: test_c(); UML_JMPc(block, uml::COND_Z, skip); break;         // CS
	case 0x6: test_z(); UML_JMPc(block, uml::COND_Z, skip); break;         // NE
	case 0x7: test_z(); UML_JMPc(block, uml::COND_NZ, skip); break;        // EQ
	case 0x8: test_v(); UML_JMPc(block, uml::COND_NZ, skip); break;        // VC
	case 0x9: test_v(); UML_JMPc(block, uml::COND_Z, skip); break;         // VS
	case 0xa: test_n(); UML_JMPc(block, uml::COND_NZ, skip); break;        // PL
	case 0xb: test_n(); UML_JMPc(block, uml::COND_Z, skip); break;         // MI
	case 0xc: test_nv(); UML_JMPc(block, uml::COND_NZ, skip); break;       // GE
	case 0xd: test_nv(); UML_JMPc(block, uml::COND_Z, skip); break;        // LT
	case 0xe: test_nv(); UML_JMPc(block, uml::COND_NZ, skip); test_z(); UML_JMPc(block, uml::COND_Z, skip); break; // GT
	case 0xf:                                                               // LE
	{
		const uml::code_label take = m_labelnum++;
		test_nv(); UML_JMPc(block, uml::COND_NZ, take);
		test_z(); UML_JMPc(block, uml::COND_NZ, skip);
		UML_LABEL(block, take);
		break;
	}
	}
}


// account cycles, then transfer to a known target
void m68ec020drc_device::generate_branch(drcuml_block &block, const opcode_desc &desc, offs_t target, int cycles)
{
	generate_update_cycles(block, cycles, target);
	if (desc.intrablock_branch() && desc.targetpc == target)
		UML_JMP(block, target | 0x80000000);
	else
		UML_HASHJMP(block, 0, target, *m_nocode);
}


// hand the current instruction to the interpreter from inside native code;
// only valid before the native code has changed any state
void m68ec020drc_device::generate_runtime_fallback(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc)
{
	UML_STORE(block, &m_pc, 0, desc.pc, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_CALLC(block, &c_funcs::interpret_one, this);
	UML_LOAD(block, uml::I0, &m_drc_bail, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_TEST(block, uml::I0, uml::I0);
	if (!compiler.bail)
		compiler.bail = compiler.next_label();
	UML_JMPc(block, uml::COND_NZ, compiler.bail);
	UML_LOAD(block, uml::I0, &m_pc, 0, uml::SIZE_DWORD, uml::SCALE_x4);
	UML_HASHJMP(block, 0, uml::I0, *m_nocode);
}


//-------------------------------------------------
//  generate_native - lower one instruction, or
//  return false to leave it to the interpreter
//-------------------------------------------------

bool m68ec020drc_device::generate_native(drcuml_block &block, compiler_state &compiler, const opcode_desc &desc)
{
	const u16 op = desc.op[0];
	m_ea_cycles = 0;

	// never second-guess the interpreter about what is legal on this CPU
	if (m_state_table[op] == m68k_state_illegal)
		return reject(__LINE__);

	// anything that is not plain compiled memory stays on the interpreter
	if (!code_region(desc.pc, desc.length))
		return reject(__LINE__);

	const int cycles = m_cyc_instruction[op];
	const offs_t nextpc = desc.pc + desc.length;
	const int eamode = (op >> 3) & 7, eareg = op & 7;
	auto allowed = [&] (u32 family) { return !(m_drc_disable & family); };
	auto finish = [&] () { generate_update_cycles(block, cycles, nextpc); return true; };
	int ext = 1;

	switch (op >> 12)
	{
	//------------------------------------------- immediate ALU, bit ops
	case 0x0:
	{
		if ((op & 0xf9c0) == 0x00c0 && (op & 0x0600) != 0x0600)
		{
			// CMP2 <ea>,Rn (CHK2 shares the encoding and traps: interpreter)
			if (!allowed(FAM_ALU))
				return reject(__LINE__);
			const u16 word2 = desc.op[1];
			if (BIT(word2, 11) || (word2 & 0x07ff))
				return reject(__LINE__);
			if (!(eamode == 2 || eamode == 5 || eamode == 6 || (eamode == 7 && eareg <= 3)))
				return reject(__LINE__);
			const int size = 1 << ((op >> 9) & 3);
			const u32 sign = 1U << (size * 8 - 1);
			const bool areg = BIT(word2, 15);
			int check = 2;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);
			ext = 2;

			load_reg(block, uml::I4, (word2 >> 12) & 15);
			if (!areg && size != 4)
				UML_AND(block, uml::I4, uml::I4, size_mask(size));
			ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
			generate_read(block, desc, size, a, uml::I0, uml::I1);           // lower bound
			ea_addr b = a;
			if (a.constant)
				b.value += size;
			else
				UML_ADD(block, uml::I3, uml::I0, size);
			generate_read(block, desc, size, b, a.constant ? uml::parameter(uml::I0) : uml::parameter(uml::I3), uml::I5);   // upper bound

			// the lower bound's sign picks a signed or an unsigned compare
			const uml::code_label is_unsigned = m_labelnum++;
			const uml::code_label out = m_labelnum++;
			const uml::code_label cdone = m_labelnum++;
			const uml::code_label zero = m_labelnum++;
			const uml::code_label zdone = m_labelnum++;
			const uml::code_label in_signed = m_labelnum++;
			UML_TEST(block, uml::I1, sign);
			UML_JMPc(block, uml::COND_Z, is_unsigned);
			if (size != 4)
			{
				UML_SEXT(block, uml::I1, uml::I1, uml_size(size));
				UML_SEXT(block, uml::I5, uml::I5, uml_size(size));
				if (!areg)
					UML_SEXT(block, uml::I4, uml::I4, uml_size(size));
			}
			UML_CMP(block, uml::I4, uml::I1);
			UML_JMPc(block, uml::COND_L, out);
			UML_CMP(block, uml::I4, uml::I5);
			UML_JMPc(block, uml::COND_G, out);
			UML_JMP(block, in_signed);
			UML_LABEL(block, is_unsigned);
			UML_CMP(block, uml::I4, uml::I1);
			UML_JMPc(block, uml::COND_B, out);
			UML_CMP(block, uml::I4, uml::I5);
			UML_JMPc(block, uml::COND_A, out);
			UML_LABEL(block, in_signed);
			store_flag(block, m_c_flag, 0);
			UML_JMP(block, cdone);
			UML_LABEL(block, out);
			store_flag(block, m_c_flag, 0x100);
			UML_LABEL(block, cdone);
			UML_CMP(block, uml::I4, uml::I5);
			UML_JMPc(block, uml::COND_E, zero);
			UML_CMP(block, uml::I4, uml::I1);
			UML_JMPc(block, uml::COND_E, zero);
			store_flag(block, m_not_z_flag, 1);
			UML_JMP(block, zdone);
			UML_LABEL(block, zero);
			store_flag(block, m_not_z_flag, 0);
			UML_LABEL(block, zdone);
			return finish();
		}

		if (BIT(op, 8) || (op & 0x0f00) == 0x0800)
		{
			// BTST/BCHG/BCLR/BSET, dynamic (Dn) or static (#imm); mode 1 is MOVEP
			if (!allowed(FAM_BIT) || eamode == 1)
				return reject(__LINE__);
			const bool dynamic = BIT(op, 8);
			const int type = (op >> 6) & 3;   // 0 btst, 1 bchg, 2 bclr, 3 bset
			if (!dynamic && (op & 0xff00) != 0x0800)
				return reject(__LINE__);
			if (type != 0 && (eamode == 7 && eareg >= 2))
				return reject(__LINE__);
			const int size = (eamode == 0) ? 4 : 1;
			const u32 bitmask = (eamode == 0) ? 31 : 7;
			int check = dynamic ? 1 : 2;
			if (!ea_supported(desc, eamode, eareg, 1, check))
				return reject(__LINE__);

			// mask in I4
			if (dynamic)
			{
				load_reg(block, uml::I5, (op >> 9) & 7);
				UML_AND(block, uml::I5, uml::I5, bitmask);
				UML_MOV(block, uml::I4, 1);
				UML_SHL(block, uml::I4, uml::I4, uml::I5);
			}
			else
			{
				UML_MOV(block, uml::I4, 1U << (desc.op[ext++] & bitmask));
			}

			if (eamode == 0)
			{
				load_reg(block, uml::I1, eareg);
			}
			else if (type == 0)
			{
				ea_read(block, desc, eamode, eareg, 1, ext, uml::I1);
			}
			else
			{
				const ea_addr a = ea_compute(block, desc, eamode, eareg, 1, ext, uml::I0);
				generate_read(block, desc, 1, a, uml::I0, uml::I1);
				UML_AND(block, uml::I3, uml::I1, uml::I4);
				store_flag(block, m_not_z_flag, uml::I3);
				if (type == 1)
					UML_XOR(block, uml::I1, uml::I1, uml::I4);
				else if (type == 2)
				{
					UML_XOR(block, uml::I4, uml::I4, 0xffffffff);
					UML_AND(block, uml::I1, uml::I1, uml::I4);
				}
				else
					UML_OR(block, uml::I1, uml::I1, uml::I4);
				generate_write(block, desc, 1, a, uml::I0, uml::I1, false);
				return finish();
			}

			UML_AND(block, uml::I3, uml::I1, uml::I4);
			store_flag(block, m_not_z_flag, uml::I3);
			if (type != 0)
			{
				if (type == 1)
					UML_XOR(block, uml::I1, uml::I1, uml::I4);
				else if (type == 2)
				{
					UML_XOR(block, uml::I4, uml::I4, 0xffffffff);
					UML_AND(block, uml::I1, uml::I1, uml::I4);
				}
				else
					UML_OR(block, uml::I1, uml::I1, uml::I4);
				store_reg(block, eareg, uml::I1);
			}
			(void)size;
			return finish();
		}

		// ORI/ANDI/SUBI/ADDI/EORI/CMPI #imm,<ea>
		const int sz = (op >> 6) & 3;
		if (!allowed(FAM_ALUI) || sz == 3 || eamode == 1)
			return reject(__LINE__);
		alu_op aop;
		switch ((op >> 9) & 7)
		{
		case 0: aop = ALU_OR;  break;
		case 1: aop = ALU_AND; break;
		case 2: aop = ALU_SUB; break;
		case 3: aop = ALU_ADD; break;
		case 5: aop = ALU_EOR; break;
		case 6: aop = ALU_CMP; break;
		default: return reject(__LINE__);
		}
		if (eamode == 7 && eareg == 4)
			return reject(__LINE__);   // to CCR/SR
		const int size = 1 << sz;
		int check = 1 + ((size == 4) ? 2 : 1);
		if (!ea_supported(desc, eamode, eareg, size, check))
			return reject(__LINE__);

		const u32 imm = read_imm(desc, size, ext);
		if (eamode == 0)
		{
			load_reg(block, uml::I1, eareg);
			if (size != 4)
				UML_AND(block, uml::I1, uml::I1, size_mask(size));
			generate_alu(block, aop, size, imm);
			if (aop != ALU_CMP)
				store_dreg(block, eareg, uml::I3, size);
		}
		else if (aop == ALU_CMP)
		{
			ea_read(block, desc, eamode, eareg, size, ext, uml::I1);
			generate_alu(block, aop, size, imm);
		}
		else
		{
			if (eamode == 7 && eareg >= 2)
				return reject(__LINE__);
			const ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
			generate_read(block, desc, size, a, uml::I0, uml::I1);
			generate_alu(block, aop, size, imm);
			generate_write(block, desc, size, a, uml::I0, uml::I3, false);
		}
		return finish();
	}

	//------------------------------------------- MOVE / MOVEA
	case 0x1: case 0x2: case 0x3:
	{
		if (!allowed(FAM_MOVE))
			return reject(__LINE__);
		const int size = ((op >> 12) == 1) ? 1 : ((op >> 12) == 3) ? 2 : 4;
		const int dmode = (op >> 6) & 7, dreg = (op >> 9) & 7;
		if (size == 1 && (eamode == 1 || dmode == 1))
			return reject(__LINE__);
		if (dmode == 7 && dreg >= 2)
			return reject(__LINE__);
		int check = 1;
		if (!ea_supported(desc, eamode, eareg, size, check) || !ea_supported(desc, dmode, dreg, size, check))
			return reject(__LINE__);

		ea_read(block, desc, eamode, eareg, size, ext, uml::I1);
		if (dmode == 1)
		{
			// MOVEA: sign-extend words, no flags
			if (size == 2)
				UML_SEXT(block, uml::I1, uml::I1, uml::SIZE_WORD);
			store_reg(block, 8 + dreg, uml::I1);
			return finish();
		}
		ea_write(block, desc, dmode, dreg, size, ext, uml::I1, size == 4 && dmode == 4);
		generate_flags_logic(block, size, uml::I1);
		return finish();
	}

	//------------------------------------------- misc
	case 0x4:
	{
		if (op == 0x4e71)
		{
			if (!allowed(FAM_MISC))
				return reject(__LINE__);
			return finish();                                    // NOP
		}

		if (op == 0x4e74)
		{
			// RTD #disp
			if (!allowed(FAM_RTS))
				return reject(__LINE__);
			load_reg(block, uml::I0, 15);
			UML_ADD(block, uml::I3, uml::I0, 4);
			store_reg(block, 15, uml::I3);
			generate_read(block, desc, 4, ea_addr{ false, 0 }, uml::I0, uml::I1);
			load_reg(block, uml::I3, 15);
			UML_ADD(block, uml::I3, uml::I3, s32(s16(desc.op[1])));
			store_reg(block, 15, uml::I3);
			generate_update_cycles(block, cycles, uml::I1);
			UML_HASHJMP(block, 0, uml::I1, *m_nocode);
			return true;
		}

		if ((op & 0xffc0) == 0x4c40)
		{
			// DIVU.L / DIVS.L <ea>,Dq and <ea>,Dr:Dq (32-bit dividend only)
			if (!allowed(FAM_MUL))
				return reject(__LINE__);
			const u16 word2 = desc.op[1];
			if ((word2 & 0x83f8) || BIT(word2, 10))
				return reject(__LINE__);
			// divide by zero bails to the interpreter, so the divisor read
			// must be repeatable
			if (!(eamode == 0 || (eamode == 7 && eareg == 4)))
				return reject(__LINE__);
			ext = 2;
			const bool is_signed = BIT(word2, 11);
			const int dq = (word2 >> 12) & 7, dr = word2 & 7;
			const uml::code_label nonzero = m_labelnum++;
			const uml::code_label normal = m_labelnum++;
			const uml::code_label store = m_labelnum++;
			ea_read(block, desc, eamode, eareg, 4, ext, uml::I4);
			UML_TEST(block, uml::I4, uml::I4);
			UML_JMPc(block, uml::COND_NZ, nonzero);
			generate_runtime_fallback(block, compiler, desc);
			UML_LABEL(block, nonzero);
			load_reg(block, uml::I1, dq);
			if (is_signed)
			{
				// avoid the host trap on 0x80000000 / -1: quotient = -dividend
				UML_CMP(block, uml::I4, 0xffffffff);
				UML_JMPc(block, uml::COND_NE, normal);
				UML_SUB(block, uml::I3, 0, uml::I1);
				UML_MOV(block, uml::I5, 0);
				UML_JMP(block, store);
				UML_LABEL(block, normal);
				UML_DIVS(block, uml::I3, uml::I5, uml::I1, uml::I4);
			}
			else
			{
				UML_LABEL(block, normal);
				UML_DIVU(block, uml::I3, uml::I5, uml::I1, uml::I4);
			}
			UML_LABEL(block, store);
			store_reg(block, dr, uml::I5);
			store_reg(block, dq, uml::I3);
			generate_flags_logic(block, 4, uml::I3);
			return finish();
		}

		if (op == 0x4e75)
		{
			// RTS
			if (!allowed(FAM_RTS))
				return reject(__LINE__);
			load_reg(block, uml::I0, 15);
			UML_ADD(block, uml::I3, uml::I0, 4);
			store_reg(block, 15, uml::I3);
			generate_read(block, desc, 4, ea_addr{ false, 0 }, uml::I0, uml::I1);
			generate_update_cycles(block, cycles, uml::I1);
			UML_HASHJMP(block, 0, uml::I1, *m_nocode);
			return true;
		}

		if ((op & 0xff80) == 0x4e80)
		{
			// JSR / JMP <ea>
			if (!allowed(FAM_JUMP))
				return reject(__LINE__);
			const bool jsr = !BIT(op, 6);
			if (!(eamode == 2 || eamode == 5 || eamode == 6 || (eamode == 7 && eareg <= 3)))
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 4, check))
				return reject(__LINE__);
			const ea_addr a = ea_compute(block, desc, eamode, eareg, 4, ext, uml::I1);
			if (jsr)
			{
				if (!a.constant)
					UML_MOV(block, uml::I3, uml::I1);
				generate_push32(block, desc, nextpc);
			}
			if (a.constant)
			{
				generate_branch(block, desc, a.value, cycles);
			}
			else
			{
				const uml::parameter target = jsr ? uml::I3 : uml::I1;
				generate_update_cycles(block, cycles, target);
				UML_HASHJMP(block, 0, target, *m_nocode);
			}
			return true;
		}

		if ((op & 0xfff8) == 0x4880 || (op & 0xfff8) == 0x48c0 || (op & 0xfff8) == 0x49c0)
		{
			// EXT.W, EXT.L, EXTB.L
			if (!allowed(FAM_REGOP))
				return reject(__LINE__);
			load_reg(block, uml::I1, eareg);
			if ((op & 0xfff8) == 0x4880)
			{
				UML_SEXT(block, uml::I1, uml::I1, uml::SIZE_BYTE);
				UML_AND(block, uml::I1, uml::I1, 0xffff);
				store_dreg(block, eareg, uml::I1, 2);
				generate_flags_logic(block, 2, uml::I1);
			}
			else
			{
				UML_SEXT(block, uml::I1, uml::I1, ((op & 0xfff8) == 0x48c0) ? uml::SIZE_WORD : uml::SIZE_BYTE);
				store_reg(block, eareg, uml::I1);
				generate_flags_logic(block, 4, uml::I1);
			}
			return finish();
		}

		if ((op & 0xfff0) == 0x4e50 || (op & 0xfff8) == 0x4808)
		{
			// LINK An,#disp
			if (!allowed(FAM_MISC))
				return reject(__LINE__);
			const bool longdisp = (op & 0xfff8) == 0x4808;
			const u32 disp = longdisp ? ((u32(desc.op[1]) << 16) | desc.op[2]) : u32(s32(s16(desc.op[1])));
			if (BIT(op, 3) && !longdisp)
			{
				// UNLK An
				if (eareg == 7)
				{
					load_reg(block, uml::I0, 15);
					generate_read(block, desc, 4, ea_addr{ false, 0 }, uml::I0, uml::I1);
					store_reg(block, 15, uml::I1);
				}
				else
				{
					load_reg(block, uml::I0, 8 + eareg);
					UML_ADD(block, uml::I3, uml::I0, 4);
					store_reg(block, 15, uml::I3);
					generate_read(block, desc, 4, ea_addr{ false, 0 }, uml::I0, uml::I1);
					store_reg(block, 8 + eareg, uml::I1);
				}
				return finish();
			}
			load_reg(block, uml::I1, 8 + eareg);
			load_reg(block, uml::I0, 15);
			UML_SUB(block, uml::I0, uml::I0, 4);
			store_reg(block, 15, uml::I0);
			generate_write(block, desc, 4, ea_addr{ false, 0 }, uml::I0, (eareg == 7) ? uml::parameter(uml::I0) : uml::parameter(uml::I1), false);
			if (eareg != 7)
				store_reg(block, 8 + eareg, uml::I0);
			UML_ADD(block, uml::I0, uml::I0, disp);
			store_reg(block, 15, uml::I0);
			return finish();
		}

		if ((op & 0xfb80) == 0x4880 && eamode != 0)
		{
			// MOVEM
			if (!allowed(FAM_MISC))
				return reject(__LINE__);
			const bool load = BIT(op, 10);
			const int size = BIT(op, 6) ? 4 : 2;
			const u16 list = desc.op[1];
			if (load ? (eamode == 4 || (eamode == 7 && eareg > 3)) : (eamode == 3 || (eamode == 7 && eareg > 1)))
				return reject(__LINE__);
			int check = 2;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);
			ext = 2;
			const int count = int(std::bitset<16>(list).count());
			const int per = load ? ((size == 4) ? m_cyc_movem_l : m_cyc_movem_w)
					: ((size == 4) ? (m_cyc_movem_store_l ? m_cyc_movem_store_l : m_cyc_movem_l) : (m_cyc_movem_store_w ? m_cyc_movem_store_w : m_cyc_movem_w));

			if (eamode == 4)
			{
				// registers to -(An), A7 first
				load_reg(block, uml::I0, 8 + eareg);
				for (int i = 0; i < 16; i++)
					if (BIT(list, i))
					{
						UML_SUB(block, uml::I0, uml::I0, size);
						load_reg(block, uml::I1, 15 - i);
						generate_write(block, desc, size, ea_addr{ false, 0 }, uml::I0, uml::I1, size == 4);
					}
				store_reg(block, 8 + eareg, uml::I0);
			}
			else if (eamode == 3)
			{
				load_reg(block, uml::I0, 8 + eareg);
				for (int i = 0; i < 16; i++)
					if (BIT(list, i))
					{
						generate_read(block, desc, size, ea_addr{ false, 0 }, uml::I0, uml::I1);
						if (size == 2)
							UML_SEXT(block, uml::I1, uml::I1, uml::SIZE_WORD);
						store_reg(block, i, uml::I1);
						UML_ADD(block, uml::I0, uml::I0, size);
					}
				store_reg(block, 8 + eareg, uml::I0);
			}
			else
			{
				ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
				if (a.constant)
					UML_MOV(block, uml::I0, a.value);
				for (int i = 0; i < 16; i++)
					if (BIT(list, i))
					{
						if (load)
						{
							generate_read(block, desc, size, ea_addr{ false, 0 }, uml::I0, uml::I1);
							if (size == 2)
								UML_SEXT(block, uml::I1, uml::I1, uml::SIZE_WORD);
							store_reg(block, i, uml::I1);
						}
						else
						{
							load_reg(block, uml::I1, i);
							generate_write(block, desc, size, ea_addr{ false, 0 }, uml::I0, uml::I1, false);
						}
						UML_ADD(block, uml::I0, uml::I0, size);
					}
			}
			generate_update_cycles(block, cycles + count * per, nextpc);
			return true;
		}

		if ((op & 0xf1c0) == 0x41c0)
		{
			// LEA <ea>,An
			if (!allowed(FAM_LEA))
				return reject(__LINE__);
			if (!(eamode == 2 || eamode == 5 || eamode == 6 || (eamode == 7 && eareg <= 3)))
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 4, check))
				return reject(__LINE__);
			const ea_addr a = ea_compute(block, desc, eamode, eareg, 4, ext, uml::I1);
			store_reg(block, 8 + ((op >> 9) & 7), a.constant ? uml::parameter(a.value) : uml::parameter(uml::I1));
			return finish();
		}

		if ((op & 0xffc0) == 0x4840 && eamode != 0)
		{
			// PEA <ea>
			if (!allowed(FAM_LEA))
				return reject(__LINE__);
			if (!(eamode == 2 || eamode == 5 || eamode == 6 || (eamode == 7 && eareg <= 3)))
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 4, check))
				return reject(__LINE__);
			const ea_addr a = ea_compute(block, desc, eamode, eareg, 4, ext, uml::I1);
			if (a.constant)
				generate_push32(block, desc, a.value);
			else
			{
				UML_MOV(block, uml::I3, uml::I1);
				generate_push32(block, desc, uml::I3);
			}
			return finish();
		}

		if ((op & 0xfff8) == 0x4840)
		{
			// SWAP Dn
			if (!allowed(FAM_REGOP))
				return reject(__LINE__);
			load_reg(block, uml::I1, eareg);
			UML_ROL(block, uml::I1, uml::I1, 16);
			store_reg(block, eareg, uml::I1);
			generate_flags_logic(block, 4, uml::I1);
			return finish();
		}

		if ((op & 0xff00) == 0x4a00 || (op & 0xff00) == 0x4200 || (op & 0xff00) == 0x4400 || (op & 0xff00) == 0x4600)
		{
			// TST, CLR, NEG, NOT
			if (!allowed(FAM_UNARY))
				return reject(__LINE__);
			const int sz = (op >> 6) & 3;
			if (sz == 3)
				return reject(__LINE__);   // TAS, MOVE from/to CCR/SR
			const int size = 1 << sz;
			const int kind = (op >> 9) & 7;   // 5 tst, 1 clr, 2 neg, 3 not
			if (eamode == 1 && (kind != 5 || size == 1))
				return reject(__LINE__);
			if (kind != 5 && eamode == 7 && eareg >= 2)
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);

			if (kind == 5)
			{
				ea_read(block, desc, eamode, eareg, size, ext, uml::I1);
				generate_flags_logic(block, size, uml::I1);
				return finish();
			}
			if (kind == 1)
			{
				ea_write(block, desc, eamode, eareg, size, ext, 0, false);
				store_flag(block, m_n_flag, 0);
				store_flag(block, m_v_flag, 0);
				store_flag(block, m_c_flag, 0);
				store_flag(block, m_not_z_flag, 0);
				return finish();
			}

			ea_addr a{ false, 0 };
			if (eamode == 0)
			{
				load_reg(block, uml::I1, eareg);
				if (size != 4)
					UML_AND(block, uml::I1, uml::I1, size_mask(size));
			}
			else
			{
				a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
				generate_read(block, desc, size, a, uml::I0, uml::I1);
			}
			if (kind == 2)
			{
				// NEG: 0 - dst
				UML_MOV(block, uml::I4, 0);
				generate_arith(block, ALU_SUB, size, uml::I4, uml::I1, true);
			}
			else
			{
				UML_XOR(block, uml::I3, uml::I1, size_mask(size));
				generate_flags_logic(block, size, uml::I3);
			}
			if (eamode == 0)
				store_dreg(block, eareg, uml::I3, size);
			else
				generate_write(block, desc, size, a, uml::I0, uml::I3, false);
			return finish();
		}

		if ((op & 0xffc0) == 0x4c00)
		{
			// MULU.L / MULS.L <ea>,Dl  and  <ea>,Dh:Dl
			if (!allowed(FAM_MUL) || eamode == 1)
				return reject(__LINE__);
			const u16 word2 = desc.op[1];
			if (word2 & 0x83f8)
				return reject(__LINE__);   // not MULx.L (DIVx.L shares the opcode with bit 6 set)
			int check = 2;
			if (!ea_supported(desc, eamode, eareg, 4, check))
				return reject(__LINE__);
			ext = 2;
			ea_read(block, desc, eamode, eareg, 4, ext, uml::I4);
			const int dl = (word2 >> 12) & 7, dh = word2 & 7;
			const bool is_signed = BIT(word2, 11), quad = BIT(word2, 10);
			load_reg(block, uml::I1, dl);
			if (is_signed)
				UML_MULS(block, uml::I3, uml::I5, uml::I4, uml::I1);
			else
				UML_MULU(block, uml::I3, uml::I5, uml::I4, uml::I1);
			store_flag(block, m_c_flag, 0);
			if (quad)
			{
				store_reg(block, dh, uml::I5);
				store_reg(block, dl, uml::I3);
				UML_OR(block, uml::I6, uml::I3, uml::I5);
				store_flag(block, m_not_z_flag, uml::I6);
				UML_SHR(block, uml::I6, uml::I5, 24);
				store_flag(block, m_n_flag, uml::I6);
				store_flag(block, m_v_flag, 0);
			}
			else
			{
				store_reg(block, dl, uml::I3);
				store_flag(block, m_not_z_flag, uml::I3);
				UML_SHR(block, uml::I6, uml::I3, 24);
				store_flag(block, m_n_flag, uml::I6);
				// V: the product does not fit in 32 bits
				if (is_signed)
				{
					UML_SAR(block, uml::I6, uml::I3, 31);
					UML_CMP(block, uml::I6, uml::I5);
				}
				else
				{
					UML_CMP(block, uml::I5, 0);
				}
				UML_SETc(block, uml::COND_NE, uml::I6);
				UML_SHL(block, uml::I6, uml::I6, 7);
				store_flag(block, m_v_flag, uml::I6);
			}
			return finish();
		}
		return reject(__LINE__);
	}

	//------------------------------------------- ADDQ / SUBQ / Scc / DBcc
	case 0x5:
	{
		const int sz = (op >> 6) & 3;
		const int cond = (op >> 8) & 0xf;
		if (sz == 3 && eamode == 1)
		{
			// DBcc Dn,<disp>
			if (!allowed(FAM_DBCC))
				return reject(__LINE__);
			const offs_t target = desc.targetpc;
			const uml::code_label decrement = m_labelnum++;
			const uml::code_label expired = m_labelnum++;
			const uml::code_label done = m_labelnum++;
			if (cond == 0)
				return finish();
			if (cond != 1)
			{
				generate_condition(block, cond, decrement);
				generate_update_cycles(block, cycles, nextpc);
				UML_JMP(block, done);
			}
			UML_LABEL(block, decrement);
			load_reg(block, uml::I1, eareg);
			UML_SUB(block, uml::I3, uml::I1, 1);
			UML_AND(block, uml::I3, uml::I3, 0xffff);
			store_dreg(block, eareg, uml::I3, 2);
			UML_CMP(block, uml::I3, 0xffff);
			UML_JMPc(block, uml::COND_E, expired);
			generate_branch(block, desc, target, cycles + m_cyc_dbcc_f_noexp);
			UML_LABEL(block, expired);
			generate_update_cycles(block, cycles + m_cyc_dbcc_f_exp, nextpc);
			UML_LABEL(block, done);
			return true;
		}
		if (sz == 3)
		{
			// Scc <ea>
			if (!allowed(FAM_SCC) || (eamode == 7 && eareg >= 2))
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 1, check))
				return reject(__LINE__);
			const uml::code_label is_false = m_labelnum++;
			const uml::code_label done = m_labelnum++;
			if (eamode == 0)
			{
				generate_condition(block, cond, is_false);
				store_dreg(block, eareg, 0xff, 1);
				generate_update_cycles(block, cycles + m_cyc_scc_r_true, nextpc);
				UML_JMP(block, done);
				UML_LABEL(block, is_false);
				store_dreg(block, eareg, 0, 1);
				generate_update_cycles(block, cycles, nextpc);
				UML_LABEL(block, done);
				return true;
			}
			UML_MOV(block, uml::I1, 0);
			generate_condition(block, cond, is_false);
			UML_MOV(block, uml::I1, 0xff);
			UML_LABEL(block, is_false);
			ea_write(block, desc, eamode, eareg, 1, ext, uml::I1, false);
			return finish();
		}

		// ADDQ / SUBQ #<1-8>,<ea>
		if (!allowed(FAM_QUICK))
			return reject(__LINE__);
		const int size = 1 << sz;
		const u32 data = (((op >> 9) - 1) & 7) + 1;
		const bool sub = BIT(op, 8);
		if (eamode == 1)
		{
			// address register: whole register, no flags
			if (size == 1)
				return reject(__LINE__);
			load_reg(block, uml::I1, 8 + eareg);
			if (sub)
				UML_SUB(block, uml::I1, uml::I1, data);
			else
				UML_ADD(block, uml::I1, uml::I1, data);
			store_reg(block, 8 + eareg, uml::I1);
			return finish();
		}
		if (eamode == 7 && eareg >= 2)
			return reject(__LINE__);
		int check = 1;
		if (!ea_supported(desc, eamode, eareg, size, check))
			return reject(__LINE__);
		ea_addr a{ false, 0 };
		if (eamode == 0)
		{
			load_reg(block, uml::I1, eareg);
			if (size != 4)
				UML_AND(block, uml::I1, uml::I1, size_mask(size));
		}
		else
		{
			a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
			generate_read(block, desc, size, a, uml::I0, uml::I1);
		}
		generate_alu(block, sub ? ALU_SUB : ALU_ADD, size, data);
		if (eamode == 0)
			store_dreg(block, eareg, uml::I3, size);
		else
			generate_write(block, desc, size, a, uml::I0, uml::I3, false);
		return finish();
	}

	//------------------------------------------- Bcc / BRA / BSR
	case 0x6:
	{
		if (!allowed(FAM_BCC))
			return reject(__LINE__);
		const int cond = (op >> 8) & 0xf;
		const u8 disp8 = op & 0xff;
		const offs_t target = desc.targetpc;
		if (cond == 0)
		{
			generate_branch(block, desc, target, cycles);
			return true;
		}
		if (cond == 1)
		{
			generate_push32(block, desc, nextpc);
			generate_branch(block, desc, target, cycles);
			return true;
		}
		const uml::code_label not_taken = m_labelnum++;
		generate_condition(block, cond, not_taken);
		generate_branch(block, desc, target, cycles);
		UML_LABEL(block, not_taken);
		const int notake = (disp8 == 0) ? m_cyc_bcc_notake_w : (disp8 == 0xff) ? 0 : m_cyc_bcc_notake_b;
		generate_update_cycles(block, cycles + notake, nextpc);
		return true;
	}

	//------------------------------------------- MOVEQ
	case 0x7:
	{
		if (!allowed(FAM_MOVEQ) || BIT(op, 8))
			return reject(__LINE__);
		const u32 res = u32(s32(s8(op & 0xff)));
		store_reg(block, (op >> 9) & 7, res);
		store_flag(block, m_n_flag, res >> 24);
		store_flag(block, m_not_z_flag, res);
		store_flag(block, m_v_flag, 0);
		store_flag(block, m_c_flag, 0);
		return finish();
	}

	//------------------------------------------- OR / SUB / CMP / EOR / AND / ADD
	case 0x8: case 0x9: case 0xb: case 0xc: case 0xd:
	{
		const int group = op >> 12;
		const int dn = (op >> 9) & 7;
		const int opmode = (op >> 6) & 7;

		// EXG
		if ((op & 0xf1f8) == 0xc140 || (op & 0xf1f8) == 0xc148 || (op & 0xf1f8) == 0xc188)
		{
			if (!allowed(FAM_REGOP))
				return reject(__LINE__);
			const int rx = ((op & 0xf1f8) == 0xc148) ? 8 + dn : dn;
			const int ry = ((op & 0xf1f8) == 0xc140) ? eareg : 8 + eareg;
			load_reg(block, uml::I1, rx);
			load_reg(block, uml::I3, ry);
			store_reg(block, rx, uml::I3);
			store_reg(block, ry, uml::I1);
			return finish();
		}

		// ADDX / SUBX Dy,Dx
		if ((group == 0xd || group == 0x9) && (op & 0x0138) == 0x0100 && ((op >> 6) & 3) != 3)
		{
			if (!allowed(FAM_ALU))
				return reject(__LINE__);
			const int size = 1 << ((op >> 6) & 3);
			const int shift = 32 - 8 * size;
			const bool add = group == 0xd;
			load_reg(block, uml::I4, eareg);
			load_reg(block, uml::I1, dn);
			if (shift)
			{
				UML_SHL(block, uml::I4, uml::I4, shift);
				UML_SHL(block, uml::I1, uml::I1, shift);
				// let the carry-in ripple up to the operand's bit 0
				if (add)
					UML_OR(block, uml::I1, uml::I1, (1U << shift) - 1);
			}
			UML_LOAD(block, uml::I2, &m_x_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4);
			UML_CARRY(block, uml::I2, 8);
			if (add)
				UML_ADDC(block, uml::I3, uml::I1, uml::I4);
			else
				UML_SUBB(block, uml::I3, uml::I1, uml::I4);
			UML_SETc(block, uml::COND_C, uml::I2);
			UML_SETc(block, uml::COND_V, uml::I5);
			UML_SHL(block, uml::I2, uml::I2, 8);
			store_flag(block, m_c_flag, uml::I2);
			store_flag(block, m_x_flag, uml::I2);
			UML_SHL(block, uml::I5, uml::I5, 7);
			store_flag(block, m_v_flag, uml::I5);
			UML_SHR(block, uml::I6, uml::I3, 24);
			store_flag(block, m_n_flag, uml::I6);
			if (shift)
				UML_SHR(block, uml::I3, uml::I3, shift);
			// Z is only ever cleared
			UML_LOAD(block, uml::I6, &m_not_z_flag, 0, uml::SIZE_DWORD, uml::SCALE_x4);
			UML_OR(block, uml::I6, uml::I6, uml::I3);
			store_flag(block, m_not_z_flag, uml::I6);
			store_dreg(block, dn, uml::I3, size);
			return finish();
		}

		// DIVU.W / DIVS.W; divide by zero takes the exception on the interpreter
		if (group == 0x8 && (opmode == 3 || opmode == 7))
		{
			if (!allowed(FAM_MUL) || eamode == 1)
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 2, check))
				return reject(__LINE__);
			const bool is_signed = opmode == 7;
			const uml::code_label nonzero = m_labelnum++;
			const uml::code_label overflow = m_labelnum++;
			const uml::code_label done = m_labelnum++;

			// the operand read must not happen twice, so only a register or
			// immediate divisor can bail to the interpreter before any effect
			if (!(eamode == 0 || (eamode == 7 && eareg == 4)))
				return reject(__LINE__);
			ea_read(block, desc, eamode, eareg, 2, ext, uml::I4);
			UML_TEST(block, uml::I4, uml::I4);
			UML_JMPc(block, uml::COND_NZ, nonzero);
			generate_runtime_fallback(block, compiler, desc);
			UML_LABEL(block, nonzero);

			load_reg(block, uml::I1, dn);
			store_flag(block, m_c_flag, 0);
			if (is_signed)
			{
				const uml::code_label normal = m_labelnum++;
				UML_SEXT(block, uml::I4, uml::I4, uml::SIZE_WORD);
				UML_CMP(block, uml::I4, 0xffffffff);
				UML_JMPc(block, uml::COND_NE, normal);
				UML_CMP(block, uml::I1, 0x80000000);
				UML_JMPc(block, uml::COND_NE, normal);
				store_flag(block, m_not_z_flag, 0);
				store_flag(block, m_n_flag, 0);
				store_flag(block, m_v_flag, 0);
				store_reg(block, dn, 0);
				UML_JMP(block, done);
				UML_LABEL(block, normal);
				UML_DIVS(block, uml::I3, uml::I5, uml::I1, uml::I4);
				UML_SEXT(block, uml::I6, uml::I3, uml::SIZE_WORD);
				UML_CMP(block, uml::I6, uml::I3);
				UML_JMPc(block, uml::COND_NE, overflow);
			}
			else
			{
				UML_DIVU(block, uml::I3, uml::I5, uml::I1, uml::I4);
				UML_CMP(block, uml::I3, 0xffff);
				UML_JMPc(block, uml::COND_A, overflow);
			}
			store_flag(block, m_not_z_flag, uml::I3);
			UML_SHR(block, uml::I6, uml::I3, 8);
			store_flag(block, m_n_flag, uml::I6);
			store_flag(block, m_v_flag, 0);
			UML_AND(block, uml::I3, uml::I3, 0xffff);
			UML_SHL(block, uml::I5, uml::I5, 16);
			UML_OR(block, uml::I3, uml::I3, uml::I5);
			store_reg(block, dn, uml::I3);
			UML_JMP(block, done);
			UML_LABEL(block, overflow);
			store_flag(block, m_v_flag, 0x80);
			UML_LABEL(block, done);
			return finish();
		}

		// MULU.W / MULS.W
		if (group == 0xc && (opmode == 3 || opmode == 7))
		{
			if (!allowed(FAM_MUL) || eamode == 1)
				return reject(__LINE__);
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, 2, check))
				return reject(__LINE__);
			ea_read(block, desc, eamode, eareg, 2, ext, uml::I4);
			load_reg(block, uml::I1, dn);
			if (opmode == 7)
			{
				UML_SEXT(block, uml::I4, uml::I4, uml::SIZE_WORD);
				UML_SEXT(block, uml::I1, uml::I1, uml::SIZE_WORD);
			}
			else
			{
				UML_AND(block, uml::I1, uml::I1, 0xffff);
			}
			UML_MULULW(block, uml::I3, uml::I4, uml::I1);
			store_reg(block, dn, uml::I3);
			generate_flags_logic(block, 4, uml::I3);
			return finish();
		}

		if (!allowed(FAM_ALU))
			return reject(__LINE__);

		// ADDA / SUBA / CMPA
		if ((group == 0x9 || group == 0xb || group == 0xd) && (opmode == 3 || opmode == 7))
		{
			const int size = (opmode == 3) ? 2 : 4;
			int check = 1;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);
			ea_read(block, desc, eamode, eareg, size, ext, uml::I4);
			if (size == 2)
				UML_SEXT(block, uml::I4, uml::I4, uml::SIZE_WORD);
			load_reg(block, uml::I1, 8 + dn);
			if (group == 0xb)
			{
				generate_arith(block, ALU_SUB, 4, uml::I1, uml::I4, false);
			}
			else
			{
				if (group == 0xd)
					UML_ADD(block, uml::I1, uml::I1, uml::I4);
				else
					UML_SUB(block, uml::I1, uml::I1, uml::I4);
				store_reg(block, 8 + dn, uml::I1);
			}
			return finish();
		}

		const int size = 1 << (opmode & 3);
		const bool to_ea = BIT(opmode, 2);
		alu_op aop;
		switch (group)
		{
		case 0x8: aop = ALU_OR; break;
		case 0x9: aop = ALU_SUB; break;
		case 0xb: aop = to_ea ? ALU_EOR : ALU_CMP; break;
		case 0xc: aop = ALU_AND; break;
		default:  aop = ALU_ADD; break;
		}

		if (!to_ea || aop == ALU_EOR)
		{
			// <ea>,Dn (or EOR Dn,Dn which shares the encoding space)
			if (!to_ea)
			{
				if (eamode == 1 && (size == 1 || aop == ALU_AND || aop == ALU_OR))
					return reject(__LINE__);
				int check = 1;
				if (!ea_supported(desc, eamode, eareg, size, check))
					return reject(__LINE__);
				ea_read(block, desc, eamode, eareg, size, ext, uml::I4);
				load_reg(block, uml::I1, dn);
				if (size != 4)
					UML_AND(block, uml::I1, uml::I1, size_mask(size));
				generate_alu(block, aop, size, uml::I4);
				if (aop != ALU_CMP)
					store_dreg(block, dn, uml::I3, size);
				return finish();
			}
			// EOR Dn,<ea>; mode 1 is CMPM
			if (eamode == 1 || (eamode == 7 && eareg >= 2))
				return reject(__LINE__);
		}
		else
		{
			// Dn,<ea>: data register/address register forms are ADDX/SUBX/ABCD/SBCD/EXG/PACK/UNPK
			if (eamode == 0 || eamode == 1 || (eamode == 7 && eareg >= 2))
				return reject(__LINE__);
		}

		int check = 1;
		if (!ea_supported(desc, eamode, eareg, size, check))
			return reject(__LINE__);
		load_reg(block, uml::I4, dn);
		if (size != 4)
			UML_AND(block, uml::I4, uml::I4, size_mask(size));
		if (eamode == 0)
		{
			// EOR Dn,Dm
			load_reg(block, uml::I1, eareg);
			if (size != 4)
				UML_AND(block, uml::I1, uml::I1, size_mask(size));
			generate_alu(block, aop, size, uml::I4);
			store_dreg(block, eareg, uml::I3, size);
			return finish();
		}
		const ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
		generate_read(block, desc, size, a, uml::I0, uml::I1);
		generate_alu(block, aop, size, uml::I4);
		generate_write(block, desc, size, a, uml::I0, uml::I3, false);
		return finish();
	}

	//------------------------------------------- shifts and rotates, register
	case 0xe:
	{
		const int sz = (op >> 6) & 3;

		if ((op & 0xffc0) == 0xe8c0 || (op & 0xffc0) == 0xeac0 || (op & 0xffc0) == 0xecc0 || (op & 0xffc0) == 0xeec0)
		{
			// BFTST / BFCHG / BFCLR / BFSET with immediate offset and width
			if (!allowed(FAM_BIT))
				return reject(__LINE__);
			const u16 word2 = desc.op[1];
			if (word2 & 0xf820)
				return reject(__LINE__);
			const int kind = (op >> 9) & 7;   // 4 tst, 5 chg, 6 clr, 7 set
			if (eamode == 1 || eamode == 3 || eamode == 4 || (eamode == 7 && eareg > ((kind == 4) ? 3 : 1)))
				return reject(__LINE__);
			int offset = (word2 >> 6) & 31;
			const int width = ((word2 - 1) & 31) + 1;
			const u32 mask_base = 0xffffffffU << (32 - width);
			auto modify = [&] (uml::parameter reg, u32 mask)
			{
				if (kind == 5)
					UML_XOR(block, reg, reg, mask);
				else if (kind == 6)
					UML_AND(block, reg, reg, ~mask);
				else if (kind == 7)
					UML_OR(block, reg, reg, mask);
			};

			if (eamode == 0)
			{
				const u32 mask = offset ? ((mask_base >> offset) | (mask_base << (32 - offset))) : mask_base;
				load_reg(block, uml::I1, eareg);
				if (offset)
					UML_SHL(block, uml::I6, uml::I1, offset);
				UML_SHR(block, uml::I6, offset ? uml::parameter(uml::I6) : uml::parameter(uml::I1), 24);
				store_flag(block, m_n_flag, uml::I6);
				UML_AND(block, uml::I6, uml::I1, mask);
				store_flag(block, m_not_z_flag, uml::I6);
				store_flag(block, m_v_flag, 0);
				store_flag(block, m_c_flag, 0);
				if (kind != 4)
				{
					modify(uml::I1, mask);
					store_reg(block, eareg, uml::I1);
				}
				return finish();
			}

			const int byteoffs = offset / 8;
			offset %= 8;
			if (offset + width > 32)
				return reject(__LINE__);
			const int size = (offset + width <= 8) ? 1 : (offset + width <= 16) ? 2 : 4;
			const int align = 32 - 8 * size;
			const u32 mask_long = mask_base >> offset;
			int check = 2;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);
			ext = 2;
			ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
			if (a.constant)
				a.value += byteoffs;
			else if (byteoffs)
				UML_ADD(block, uml::I0, uml::I0, byteoffs);
			generate_read(block, desc, size, a, uml::I0, uml::I1);
			if (align)
				UML_SHL(block, uml::I1, uml::I1, align);
			if (offset)
				UML_SHL(block, uml::I6, uml::I1, offset);
			UML_SHR(block, uml::I6, offset ? uml::parameter(uml::I6) : uml::parameter(uml::I1), 24);
			store_flag(block, m_n_flag, uml::I6);
			UML_AND(block, uml::I6, uml::I1, mask_long);
			store_flag(block, m_not_z_flag, uml::I6);
			store_flag(block, m_v_flag, 0);
			store_flag(block, m_c_flag, 0);
			if (kind != 4)
			{
				modify(uml::I1, mask_long);
				if (align)
					UML_SHR(block, uml::I1, uml::I1, align);
				generate_write(block, desc, size, a, uml::I0, uml::I1, false);
			}
			return finish();
		}

		if ((op & 0xf0f8) == 0xe0b8 || (op & 0xf0f8) == 0xe038)
		{
			// ROL.L / ROR.L / ROR.B Dx,Dy; a zero count goes to the interpreter
			if (!allowed(FAM_SHIFT) || !allowed(FAM_SHIFT_REG))
				return reject(__LINE__);
			const bool left = BIT(op, 8);
			const bool is_long = (op & 0xf0f8) == 0xe0b8;
			if (!is_long && left)
				return reject(__LINE__);
			const uml::code_label nonzero = m_labelnum++;
			load_reg(block, uml::I4, (op >> 9) & 7);
			UML_AND(block, uml::I4, uml::I4, 0x3f);
			UML_JMPc(block, uml::COND_NZ, nonzero);
			generate_runtime_fallback(block, compiler, desc);
			UML_LABEL(block, nonzero);
			load_reg(block, uml::I1, eareg);
			if (is_long)
			{
				UML_AND(block, uml::I4, uml::I4, 31);
				if (left)
				{
					UML_ROL(block, uml::I3, uml::I1, uml::I4);
					UML_SUB(block, uml::I6, 32, uml::I4);       // carry: bit (32 - n) & 31
				}
				else
				{
					UML_ROR(block, uml::I3, uml::I1, uml::I4);
					UML_SUB(block, uml::I6, uml::I4, 1);        // carry: bit (n - 1) & 31
				}
				UML_AND(block, uml::I6, uml::I6, 31);
				UML_SHR(block, uml::I6, uml::I1, uml::I6);
				store_reg(block, eareg, uml::I3);
				generate_flags_logic(block, 4, uml::I3);
			}
			else
			{
				UML_AND(block, uml::I1, uml::I1, 0xff);
				UML_AND(block, uml::I4, uml::I4, 7);
				UML_SHR(block, uml::I5, uml::I1, uml::I4);
				UML_SUB(block, uml::I6, 8, uml::I4);
				UML_SHL(block, uml::I6, uml::I1, uml::I6);
				UML_OR(block, uml::I3, uml::I5, uml::I6);
				UML_AND(block, uml::I3, uml::I3, 0xff);
				UML_SUB(block, uml::I6, uml::I4, 1);
				UML_AND(block, uml::I6, uml::I6, 7);
				UML_SHR(block, uml::I6, uml::I1, uml::I6);
				store_dreg(block, eareg, uml::I3, 1);
				generate_flags_logic(block, 1, uml::I3);
			}
			UML_AND(block, uml::I6, uml::I6, 1);
			UML_SHL(block, uml::I6, uml::I6, 8);
			store_flag(block, m_c_flag, uml::I6);
			return finish();
		}

		if ((op & 0xffc0) == 0xe9c0 || (op & 0xffc0) == 0xebc0 || (op & 0xffc0) == 0xefc0)
		{
			// BFEXTU / BFEXTS / BFINS with immediate offset and width
			if (!allowed(FAM_BIT))
				return reject(__LINE__);
			const u16 word2 = desc.op[1];
			if (BIT(word2, 11) || BIT(word2, 5) || BIT(word2, 15))
				return reject(__LINE__);   // offset/width in registers
			const bool ins = (op & 0xffc0) == 0xefc0;
			const bool sext = (op & 0xffc0) == 0xebc0;
			if (eamode == 1 || eamode == 3 || eamode == 4 || (eamode == 7 && eareg > (ins ? 1 : 3)))
				return reject(__LINE__);
			const int dreg = (word2 >> 12) & 7;
			int offset = (word2 >> 6) & 31;
			const int width = ((word2 - 1) & 31) + 1;
			const u32 mask_base = 0xffffffffU << (32 - width) ;   // width 32: shift 0

			if (eamode == 0)
			{
				load_reg(block, uml::I1, eareg);
				if (ins)
				{
					load_reg(block, uml::I4, dreg);
					if (width < 32)
						UML_SHL(block, uml::I4, uml::I4, 32 - width);
					UML_SHR(block, uml::I6, uml::I4, 24);
					store_flag(block, m_n_flag, uml::I6);
					store_flag(block, m_not_z_flag, uml::I4);
					if (offset)
						UML_ROR(block, uml::I4, uml::I4, offset);
					UML_AND(block, uml::I1, uml::I1, ~((mask_base >> offset) | (offset ? (mask_base << (32 - offset)) : 0)));
					UML_OR(block, uml::I1, uml::I1, uml::I4);
					store_reg(block, eareg, uml::I1);
				}
				else
				{
					if (offset)
						UML_ROL(block, uml::I1, uml::I1, offset);
					UML_SHR(block, uml::I6, uml::I1, 24);
					store_flag(block, m_n_flag, uml::I6);
					if (width < 32)
					{
						if (sext)
							UML_SAR(block, uml::I1, uml::I1, 32 - width);
						else
							UML_SHR(block, uml::I1, uml::I1, 32 - width);
					}
					store_flag(block, m_not_z_flag, uml::I1);
					store_reg(block, dreg, uml::I1);
				}
				store_flag(block, m_v_flag, 0);
				store_flag(block, m_c_flag, 0);
				return finish();
			}

			// memory: byte-aligned window of 1, 2 or 4 bytes
			const int byteoffs = offset / 8;
			offset %= 8;
			if (offset + width > 32)
				return reject(__LINE__);
			const int size = (offset + width <= 8) ? 1 : (offset + width <= 16) ? 2 : 4;
			const int align = 32 - 8 * size;
			int check = 2;
			if (!ea_supported(desc, eamode, eareg, size, check))
				return reject(__LINE__);
			ext = 2;
			ea_addr a = ea_compute(block, desc, eamode, eareg, size, ext, uml::I0);
			if (a.constant)
				a.value += byteoffs;
			else if (byteoffs)
				UML_ADD(block, uml::I0, uml::I0, byteoffs);

			if (ins)
			{
				const u32 mask_long = mask_base >> offset;
				load_reg(block, uml::I4, dreg);
				if (width < 32)
					UML_SHL(block, uml::I4, uml::I4, 32 - width);
				UML_SHR(block, uml::I6, uml::I4, 24);
				store_flag(block, m_n_flag, uml::I6);
				store_flag(block, m_not_z_flag, uml::I4);
				if (offset)
					UML_SHR(block, uml::I4, uml::I4, offset);
				generate_read(block, desc, size, a, uml::I0, uml::I1);
				if (align)
					UML_SHL(block, uml::I1, uml::I1, align);
				UML_AND(block, uml::I1, uml::I1, ~mask_long);
				UML_OR(block, uml::I1, uml::I1, uml::I4);
				if (align)
					UML_SHR(block, uml::I1, uml::I1, align);
				store_flag(block, m_v_flag, 0);
				store_flag(block, m_c_flag, 0);
				generate_write(block, desc, size, a, uml::I0, uml::I1, false);
			}
			else
			{
				generate_read(block, desc, size, a, uml::I0, uml::I1);
				if (align + offset)
					UML_SHL(block, uml::I1, uml::I1, align + offset);
				UML_SHR(block, uml::I6, uml::I1, 24);
				store_flag(block, m_n_flag, uml::I6);
				if (width < 32)
				{
					if (sext)
						UML_SAR(block, uml::I1, uml::I1, 32 - width);
					else
						UML_SHR(block, uml::I1, uml::I1, 32 - width);
				}
				store_flag(block, m_not_z_flag, uml::I1);
				store_flag(block, m_v_flag, 0);
				store_flag(block, m_c_flag, 0);
				store_reg(block, dreg, uml::I1);
			}
			return finish();
		}
		if (!allowed(FAM_SHIFT) || sz == 3)
			return reject(__LINE__);   // memory forms stay interpreted
		const int type = (op >> 3) & 3;  // 0 AS, 1 LS, 2 ROX, 3 RO
		if (type == 2)
			return reject(__LINE__);

		if (BIT(op, 5) && !allowed(FAM_SHIFT_REG))
			return reject(__LINE__);
		if (!BIT(op, 5))
		{
			const bool l = BIT(op, 8);
			if ((type == 0 && l && !allowed(FAM_SHIFT_ASL)) || (type == 3 && !allowed(FAM_SHIFT_ROT)) ||
				(type == 0 && !l && !allowed(FAM_SHIFT_ASR)) || (type == 1 && !allowed(FAM_SHIFT_LS)))
				return reject(__LINE__);
		}
		if (BIT(op, 5))
		{
			// count in Dx: LSL/LSR/ASR inline for 0 < count < width; zero,
			// oversized counts, ASL and rotates go to the interpreter
			const bool left = BIT(op, 8);
			if (type == 3 || (type == 0 && left))
				return reject(__LINE__);
			const int size = 1 << sz;
			const int bits = size * 8;
			const u32 mask = size_mask(size);
			const uml::code_label inline_path = m_labelnum++;
			load_reg(block, uml::I4, (op >> 9) & 7);
			UML_AND(block, uml::I4, uml::I4, 0x3f);
			UML_SUB(block, uml::I2, uml::I4, 1);
			UML_CMP(block, uml::I2, bits - 1);
			UML_JMPc(block, uml::COND_B, inline_path);
			generate_runtime_fallback(block, compiler, desc);
			UML_LABEL(block, inline_path);

			load_reg(block, uml::I1, eareg);
			if (size != 4)
				UML_AND(block, uml::I1, uml::I1, mask);
			if (left)
			{
				UML_SHL(block, uml::I3, uml::I1, uml::I4);
				if (size != 4)
					UML_AND(block, uml::I3, uml::I3, mask);
				UML_MOV(block, uml::I6, bits);
				UML_SUB(block, uml::I6, uml::I6, uml::I4);
				UML_SHR(block, uml::I6, uml::I1, uml::I6);
			}
			else
			{
				if (type == 0)
				{
					if (size != 4)
						UML_SEXT(block, uml::I3, uml::I1, uml_size(size));
					else
						UML_MOV(block, uml::I3, uml::I1);
					UML_SAR(block, uml::I3, uml::I3, uml::I4);
					if (size != 4)
						UML_AND(block, uml::I3, uml::I3, mask);
				}
				else
				{
					UML_SHR(block, uml::I3, uml::I1, uml::I4);
				}
				UML_SHR(block, uml::I6, uml::I1, uml::I2);   // count - 1
			}
			// carry in I6: generate_flags_logic uses I5 as scratch
			UML_AND(block, uml::I6, uml::I6, 1);
			UML_SHL(block, uml::I6, uml::I6, 8);
			store_dreg(block, eareg, uml::I3, size);
			generate_flags_logic(block, size, uml::I3);
			store_flag(block, m_c_flag, uml::I6);
			store_flag(block, m_x_flag, uml::I6);
			return finish();
		}
		const int size = 1 << sz;
		const int bits = size * 8;
		const u32 mask = size_mask(size);
		const int shift = (((op >> 9) - 1) & 7) + 1;
		const bool left = BIT(op, 8);

		load_reg(block, uml::I1, eareg);
		if (size != 4)
			UML_AND(block, uml::I1, uml::I1, mask);

		// result in I3, carry (bit 8) in I4
		if (left)
		{
			if (type == 3)
			{
				if (size == 4)
					UML_ROL(block, uml::I3, uml::I1, shift);
				else
				{
					UML_SHL(block, uml::I5, uml::I1, shift);
					UML_SHR(block, uml::I6, uml::I1, bits - shift);
					UML_OR(block, uml::I3, uml::I5, uml::I6);
					UML_AND(block, uml::I3, uml::I3, mask);
				}
			}
			else
			{
				UML_SHL(block, uml::I3, uml::I1, shift);
				if (size != 4)
					UML_AND(block, uml::I3, uml::I3, mask);
			}
			UML_ROLAND(block, uml::I4, uml::I1, (8 + shift - bits) & 31, 0x100);
		}
		else
		{
			if (type == 3)
			{
				if (size == 4)
					UML_ROR(block, uml::I3, uml::I1, shift);
				else
				{
					UML_SHR(block, uml::I5, uml::I1, shift);
					UML_SHL(block, uml::I6, uml::I1, bits - shift);
					UML_OR(block, uml::I3, uml::I5, uml::I6);
					UML_AND(block, uml::I3, uml::I3, mask);
				}
			}
			else if (type == 0)
			{
				if (size != 4)
					UML_SEXT(block, uml::I3, uml::I1, uml_size(size));
				else
					UML_MOV(block, uml::I3, uml::I1);
				UML_SAR(block, uml::I3, uml::I3, shift);
				if (size != 4)
					UML_AND(block, uml::I3, uml::I3, mask);
			}
			else
			{
				UML_SHR(block, uml::I3, uml::I1, shift);
			}
			UML_ROLAND(block, uml::I4, uml::I1, 9 - shift, 0x100);
		}

		store_dreg(block, eareg, uml::I3, size);
		generate_flags_logic(block, size, uml::I3);   // N, Z; V = C = 0 for now
		store_flag(block, m_c_flag, uml::I4);
		if (type != 3)
			store_flag(block, m_x_flag, uml::I4);
		if (type == 0 && left)
		{
			// ASL: V when the bits shifted through the MSB were not all equal
			const u32 top = top_bits(size, shift + 1);
			const uml::code_label clear = m_labelnum++;
			const uml::code_label done = m_labelnum++;
			UML_AND(block, uml::I6, uml::I1, top);
			UML_JMPc(block, uml::COND_Z, clear);
			if (!(size == 1 && shift >= 8))
			{
				UML_CMP(block, uml::I6, top);
				UML_JMPc(block, uml::COND_E, clear);
			}
			store_flag(block, m_v_flag, 0x80);
			UML_JMP(block, done);
			UML_LABEL(block, clear);
			UML_LABEL(block, done);
		}
		return finish();
	}
	}

	return reject(__LINE__);
}
