// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Support for LoongArch64 processors with EJTAG-style debugging capability:
 *
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on MIPS code.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "loongarch64.h"
#include "target/target_type.h"
#include "helper/command.h"
#include "target/loongarch/loongarch_ejtag.h"

/*******************************************************************************
 *       Registers
 ******************************************************************************/

static const struct {
	unsigned int id;
	const char *name;
	enum reg_type type;
	const char *feature;
	const char *group;
} loongarch64_regs[] = {
	{  0,   "r0", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  1,   "r1", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  2,   "r2", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  3,   "r3", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  4,   "r4", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  5,   "r5", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  6,   "r6", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  7,   "r7", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  8,   "r8", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{  9,   "r9", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 10,  "r10", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 11,  "r11", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 12,  "r12", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 13,  "r13", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 14,  "r14", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 15,  "r15", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 16,  "r16", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 17,  "r17", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 18,  "r18", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 19,  "r19", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 20,  "r20", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 21,  "r21", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 22,  "r22", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 23,  "r23", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 24,  "r24", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 25,  "r25", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 26,  "r26", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 27,  "r27", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 28,  "r28", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 29,  "r29", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 30,  "r30", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 31,  "r31", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 32,   "pc", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CPU" },
	{ 33, "badv", REG_TYPE_UINT64, "org.gnu.gdb.loongarch.base", "CSR" },
};

// Merely transfers core register value from PrAcc buffer into core cache
static int loongarch64_get_core_reg(struct reg *reg)
{
	struct loongarch64_core_reg *loongarch64_reg = reg->arch_info;
	struct target *target = loongarch64_reg->target;
	struct loongarch64_common *loongarch64 = target->arch_info;
	int id = loongarch64_reg->id;
	uint64_t reg_value;

	if  (target->state != TARGET_HALTED) {
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((id < 0) || (id >= LOONG64_NUM_REGS)) {
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	reg_value = loongarch64->core_regs[id];
	buf_set_u64(loongarch64->core_cache->reg_list[id].value, 0, 64,
		reg_value);
	loongarch64->core_cache->reg_list[id].valid = true;
	loongarch64->core_cache->reg_list[id].dirty = false;
	
	return ERROR_OK;
}

// Just sets core cache register value, not transferred into PrAcc buffer
static int loongarch64_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct loongarch64_core_reg *loongarch64_reg = reg->arch_info;
	struct target *target = loongarch64_reg->target;
	uint64_t value = buf_get_u64(buf, 0, 64);

	if (target->state != TARGET_HALTED) {
		return ERROR_TARGET_NOT_HALTED;
	}

	buf_set_u64(reg->value, 0, 64, value);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type loongarch64_reg_type = {
	.get = loongarch64_get_core_reg,
	.set = loongarch64_set_core_reg
};

static inline int reg_type2size(enum reg_type type)
{
	switch (type) {
	case REG_TYPE_UINT32:
	case REG_TYPE_INT:
		return 32;
	case REG_TYPE_UINT64:
	case REG_TYPE_IEEE_DOUBLE:
		return 64;
	default:
		return 64;
	}
}

static int loongarch64_build_register_cache(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct reg_cache **cache_p, *cache = NULL;
	struct loongarch64_core_reg *arch_info = NULL;
	struct reg *reg_list = NULL;
	unsigned int i;

	cache = calloc(1, sizeof(*cache));
	if (!cache) {
		LOG_ERROR("unable to allocate cache");
		return ERROR_FAIL;
	}

	reg_list = calloc(LOONG64_NUM_REGS, sizeof(*reg_list));
	if (!reg_list) {
		LOG_ERROR("unable to allocate reg_list");
		goto alloc_fail;
	}

	arch_info = calloc(LOONG64_NUM_REGS, sizeof(*arch_info));
	if (!arch_info) {
		LOG_ERROR("unable to allocate arch_info");
		goto alloc_fail;
	}

	for (i = 0; i < LOONG64_NUM_REGS; i++) {
		struct loongarch64_core_reg *a = &arch_info[i];
		struct reg *r = &reg_list[i];

		r->arch_info = &arch_info[i];
		r->caller_save = true;	/* gdb defaults to true */
		r->exist = true;
		r->feature = &a->feature;
		r->feature->name = loongarch64_regs[i].feature;
		r->group = loongarch64_regs[i].group;
		r->name = loongarch64_regs[i].name;
		r->number = i;
		r->reg_data_type = &a->reg_data_type;
		r->reg_data_type->type = loongarch64_regs[i].type;
		r->size = reg_type2size(loongarch64_regs[i].type);
		r->type = &loongarch64_reg_type;
		r->value = &a->value[0];

		a->loongarch64_common = loongarch64;
		a->id = loongarch64_regs[i].id;
		a->target = target;
	}

	cache->name = "LoongArch64 Registers";
	cache->reg_list = reg_list;
	cache->num_regs = LOONG64_NUM_REGS;

	cache_p = register_get_last_cache_p(&target->reg_cache);
	(*cache_p) = cache;

	loongarch64->core_cache = cache;

	return ERROR_OK;

alloc_fail:
	free(cache);
	free(reg_list);
	free(arch_info);

	return ERROR_FAIL;
}

int loongarch64_invalidate_core_regs(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	unsigned int i;

	for (i = 0; i < loongarch64->core_cache->num_regs; i++) {
		loongarch64->core_cache->reg_list[i].valid = false;
		loongarch64->core_cache->reg_list[i].dirty = false;
	}

	return ERROR_OK;
}

/*******************************************************************************
 *       General target operations
 ******************************************************************************/

static int loongarch64_debug_entry(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	struct reg *pc = &loongarch64->core_cache->reg_list[LOONG64_PC];
	int retval;

	// Read registers and save context
	retval = loongarch64_pracc_read_regs(ejtag_info, loongarch64->core_regs);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read registers (%d)", retval);
		return retval;
	}
	for (unsigned int i = 0; i < LOONG64_NUM_REGS; i++) {
		retval = loongarch64_get_core_reg(
			&loongarch64->core_cache->reg_list[i]);
	}

	LOG_DEBUG("entered debug state at PC 0x%" PRIx64 ", target->state: %s",
		  buf_get_u64(pc->value, 0, 64), target_state_name(target));

	// TODO: do we need to disable stepping and find halt reason like mips_mips64_debug_entry?
	return ERROR_OK;
}

static int loongarch64_poll(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl;
	int retval;

	// Read Control register
	loongarch_ejtag_add_write_ir(ejtag_info, LAEJTAG_INST_CONTROL);
	loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	// The MIPS counterpart tries to detect Rocc bit here but LoongArch
	// processors don't seem to have implemented a reset mechanism from
	// EJTAG so we're omitting this.

	// Check if CPU is in debug mode
	if (ejtag_ctrl & LAEJTAG_CTRL_DM) {
		if (target->state == TARGET_RUNNING ||
			target->state == TARGET_UNKNOWN) {

			target->state = TARGET_HALTED;
			retval = loongarch64_debug_entry(target);
			if (retval != ERROR_OK) {
				return retval;
			}
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		} else if (target->state == TARGET_DEBUG_RUNNING) {
			target->state = TARGET_HALTED;
			retval = loongarch64_debug_entry(target);
			if (retval != ERROR_OK) {
				return retval;
			}
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	} else if (ejtag_ctrl & LAEJTAG_CTRL_PRACC) {
		/* This is a quirk found on LS2K0300: when exiting debug mode
		 * the processor still tries to make one more read from dmseg
		 * even though DM bit is already reset, and because of this
		 * dmseg access, the processor is still in halt state.
		 * We attempt letting it go and declare it's running if
		 * succeeded */
		retval = loongarch_ejtag_exit_debug(ejtag_info);
		if (retval == ERROR_OK) {
			target->state = TARGET_RUNNING;
		} else {
			LOG_ERROR("Quirk: target not in DM but stuck at dmseg");
			target->state = TARGET_UNKNOWN;
		}
	} else {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}


int loongarch64_arch_state(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct reg *pc = &loongarch64->core_cache->reg_list[LOONG64_PC];

	if (loongarch64->common_magic != LOONG64_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-LoongArch64 target");
		exit(-1);
	}

	LOG_USER("target halted due to %s, pc: 0x%" PRIx64 "",
		 debug_reason_name(target), buf_get_u64(pc->value, 0, 64));

	return ERROR_OK;
}

static int loongarch64_halt(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	int retval = ERROR_OK;

	LOG_DEBUG("target->state: %s", target_state_name(target));

	switch (target->state) {
	case TARGET_HALTED:
		LOG_WARNING("target was already halted");
		return ERROR_OK;
	case TARGET_UNKNOWN:
		LOG_WARNING("target was in unknown state when halt was requested");
		break;
	case TARGET_RESET:
		// FIXME: Since LoongArch devices don't have a method of holding
		// reset line with JTAG, this branch is just a stub
		LOG_ERROR("Can't halt a target while it's in reset state");
		return ERROR_TARGET_FAILURE;
	default:
		break;
	}

	retval = loongarch_ejtag_enter_debug(ejtag_info);
	if (retval != ERROR_OK) {
		return retval;
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return retval;
}

static int loongarch64_resume(struct target *target, bool current,
			      uint64_t address, bool handle_breakpoints,
			      bool debug_execution)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// TODO: address
	// TODO: handle_breakpoints
	// TODO: debug_execution

	// Exit debug mode
	retval = loongarch_ejtag_exit_debug(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	// TODO: Mark register cache as invalid

	return ERROR_OK;
}

static int loongarch64_read_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	int retval;
	void *t;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1))
	    || !count || !buffer)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7)) || ((size == 4) && (address & 0x3))
	    || ((size == 2) && (address & 0x1)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	if (size > 1) {
		t = calloc(count, size);
		if (!t) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
	} else
		t = buffer;

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		  address, size, count);

	retval = loongarch64_pracc_read_mem(ejtag_info, address, size, count, (void *)t);

	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch64_pracc_read_mem filed");
		goto read_done;
	}

	switch (size) {
	case 8:
		target_buffer_set_u64_array(target, buffer, count, t);
		break;
	case 4:
		target_buffer_set_u32_array(target, buffer, count, t);
		break;
	case 2:
		target_buffer_set_u16_array(target, buffer, count, t);
		break;
	}

read_done:
	if (size > 1)
		free(t);

	return retval;
}

static int loongarch64_write_memory(struct target *target, uint64_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;
	int retval;
	void *t = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 8) && (size != 4) && (size != 2) && (size != 1))
	    || !count || !buffer)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	if (((size == 8) && (address & 0x7)) || ((size == 4) && (address & 0x3))
	    || ((size == 2) && (address & 0x1)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* TODO: "Bulk" write */

	if (size > 1) {
		t = calloc(count, size);
		if (!t) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}

		switch (size) {
		case 8:
			target_buffer_get_u64_array(target, buffer, count, (uint64_t *)t);
			break;
		case 4:
			target_buffer_get_u32_array(target, buffer, count, (uint32_t *)t);
			break;
		case 2:
			target_buffer_get_u16_array(target, buffer, count, (uint16_t *)t);
			break;
		}
		buffer = t;
	}

	LOG_DEBUG("address: 0x%16.16" PRIx64 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
		  address, size, count);

	retval = loongarch64_pracc_write_mem(ejtag_info, address, size, count, (void *)buffer);

	if (retval != ERROR_OK) {
		LOG_ERROR("loongarch64_pracc_read_mem filed");
	}

	free(t);

	return retval;
}

static int loongarch64_target_create(struct target *target)
{
	struct loongarch64_common *loongarch64;

	loongarch64 = calloc(1, sizeof(*loongarch64));
	if (!loongarch64) {
		LOG_ERROR("unable to allocate loongarch64");
		return ERROR_FAIL;
	}

	loongarch64->common_magic = LOONG64_COMMON_MAGIC;
	target->arch_info = loongarch64;

	// Initialize arch_info
	loongarch64->ejtag_info.tap = target->tap;

	return ERROR_OK;
}

static int loongarch64_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int retval;

	retval = loongarch64_build_register_cache(target);
	if (retval != ERROR_OK) {
		return retval;
	}

	return ERROR_OK;
}

static int loongarch64_examine(struct target *target)
{
	int retval;
	struct loongarch64_common *loongarch64 = target->arch_info;

	retval = loongarch_ejtag_init(&loongarch64->ejtag_info);
	if (retval != ERROR_OK) {
		return retval;
	}

	return ERROR_OK;
}

static const struct command_registration loongarch64_commands[] = {
	{
		/* It has nowhere else to go, wired on LoongArch64 handler table for convenience */
		.chain = loongarch_ejtag_command_handlers
	},
	COMMAND_REGISTRATION_DONE
	// TODO: commands for LoongArch64 targets
};

struct target_type loongarch64_target = {
	.name = "loongarch64",
	
	.poll = loongarch64_poll,
	.arch_state = loongarch64_arch_state,

	.target_request_data = NULL,

	.halt = loongarch64_halt,
	.resume = loongarch64_resume,
	.step = NULL,

	.assert_reset = NULL,
	.deassert_reset = NULL,

	.get_gdb_reg_list = NULL,

	.read_memory = loongarch64_read_memory,
	.write_memory = loongarch64_write_memory,
	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.run_algorithm = NULL,

	.add_breakpoint = NULL,
	.remove_breakpoint = NULL,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = loongarch64_commands,
	.target_create = loongarch64_target_create,
	.init_target = loongarch64_init_target,
	.examine = loongarch64_examine,
};
