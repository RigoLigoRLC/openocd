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

static int loongarch64_poll(struct target *target)
{
	struct loongarch64_common *loongarch64 = target->arch_info;
	struct loongarch_ejtag *ejtag_info = &loongarch64->ejtag_info;

	// Scan input has PrAcc bit set, to prevent accidentally completing a
	// memory access
	uint32_t ejtag_ctrl = LAEJTAG_CTRL_PRACC | LAEJTAG_CTRL_PROBEN
			    | LAEJTAG_CTRL_PROBTRAP;

	// Read Control register
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);
	loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	// The MIPS counterpart tries to detect Rocc bit here but LoongArch
	// processors don't seem to have implemented a reset mechanism from
	// EJTAG so we're omitting this.

	// Check if CPU is in debug mode
	if (ejtag_ctrl & LAEJTAG_CTRL_DM) {
		target->state = TARGET_HALTED;
	} else if (ejtag_ctrl & LAEJTAG_CTRL_PRACC) {
		/* This is a quirk found on LS2K0300: when exiting debug mode
		 * the processor still tries to make one more read from dmseg
		 * even though DM bit is already reset, and because of this
		 * dmseg access, the processor is still in halt state. */

		/* FIXME: Should we try to let it start running again and
		 * report "RUNNING"? */
		target->state = TARGET_HALTED;
	} else {
		target->state = TARGET_RUNNING;
	}

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
		LOG_DEBUG("target was already halted");
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
	return ERROR_OK;
}

static int loongarch64_examine(struct target *target)
{
	// LoongArch EJTAG doesn't have a lot of interesting stuff
	int retval;
	struct loongarch64_common *loongarch64 = target->arch_info;

	retval = loongarch_ejtag_init(&loongarch64->ejtag_info);
	if (retval != ERROR_OK) {
		return retval;
	}

	return ERROR_OK;
}

struct target_type loongarch64_target = {
	.name = "loongarch64",
	
	.poll = loongarch64_poll,
	.arch_state = NULL,

	.target_request_data = NULL,

	.halt = loongarch64_halt,
	.resume = loongarch64_resume,
	.step = NULL,

	.assert_reset = NULL,
	.deassert_reset = NULL,

	.target_create = loongarch64_target_create,
	.init_target = loongarch64_init_target,
	.examine = loongarch64_examine,
};
