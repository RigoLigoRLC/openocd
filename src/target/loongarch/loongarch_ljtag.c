/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for LoongArch64 processors with LJTAG debugging capability:
 * reusable JTAG register read/write primitives.
 *
 *   Copyright (C) 2025-2026 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on OpenOCD MIPS implementation,
 * and LJTAG implementation from OpenLA132 open source CPU core
 * (https://github.com/loongson/OpenLACores/).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "loongarch64_dbgm.h"
#include "loongarch64.h"
#include "helper/command.h"
#include <stdint.h>

/*
 * Global configuration entries
 */
static bool cfg_use_fastdata = true; /* Should FASTDATA be used instead of DATA+CONTROL */

void loongarch_ljtag_add_write_ir(struct loongarch_ljtag *ljtag_info, uint32_t new_instr)
{
	assert(ljtag_info->tap);
	struct jtag_tap *tap = ljtag_info->tap;

	/*
	 * Quirk: At least as tested with LS2K0300: you MUST write IR before 
	 * reading/writing DR, otherwise you'll read back garbage data.
	 * The logic is kept and awaiting whether we can add a flag to apply
	 * this quirk fix only for the chips with this bug.
	 */
	if (true || buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr) {

		struct scan_field field;
		field.num_bits = tap->ir_length;

		uint8_t t[4] = { 0 };
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);

		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
	}
}

int loongarch_ljtag_drscan_64(struct loongarch_ljtag *ljtag_info, uint64_t *data)
{
	// TODO: bool readback
	struct jtag_tap *tap = ljtag_info->tap;
	if (!tap) {
		LOG_ERROR("ljtag_info->tap is NULL!");
		return ERROR_FAIL;
	}

	struct scan_field field;
	uint8_t out[8] = { 0 }, in[8];
	int retval;

	field.num_bits = 64;
	field.out_value = out;
	buf_set_u64(out, 0, field.num_bits, *data);
	field.in_value = in;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u64(field.in_value, 0, 64);

	keep_alive();

	return ERROR_OK;
}

/**
 * @brief Execute a DR scan with 64 bit data, provided you've already queued an IR scan that set it
 * to FASTDATA, or IR is already set to FASTDATA.
 * 
 * @param ljtag_info LJTAG context object
 * @param data pointer to a 64-bit data. Overwritten with DATA register readback if enabled.
 * @param pr the state of PR field you want to set. false for clear, true for assert.
 * @param readback whether you want DATA register be read back.
 * @return int error code
 */
int loongarch_ljtag_fastdata_scan_64(struct loongarch_ljtag *ljtag_info,
				     uint64_t *data,
				     bool pr,
				     bool readback)
{
	struct jtag_tap *tap = ljtag_info->tap;
	if (!tap) {
		LOG_ERROR("ljtag_info->tap is NULL!");
		return ERROR_FAIL;
	}

	struct scan_field field[2];
	uint8_t out[8] = { 0 }, in[8];
	uint8_t pr_u8 = pr;
	int retval;

	/* PR field first */
	field[0].num_bits = 1;
	field[0].out_value = &pr_u8;
	field[0].in_value = NULL;

	/* then comes the data register */
	field[1].num_bits = 64;
	field[1].out_value = out;
	buf_set_u64(out, 0, field[1].num_bits, *data);

	/* If TDO readback is not required, set in_value to NULL */
	/* This can save Loongson LJTAG some processing time */
	if (readback) {
		field[1].in_value = in;
	} else {
		field[1].in_value = NULL;
	}

	jtag_add_dr_scan(tap, 2, field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	if (readback) {
		*data = buf_get_u64(field[1].in_value, 0, 64);
	}

	keep_alive();

	return ERROR_OK;
}

static void loongarch_ljtag_drscan_32_queued(struct loongarch_ljtag *ljtag_info,
		uint32_t data_out, uint8_t *data_in)
{
	assert(ljtag_info->tap);
	struct jtag_tap *tap = ljtag_info->tap;

	struct scan_field field;
	field.num_bits = 32;

	uint8_t scan_out[4] = { 0 };
	field.out_value = scan_out;
	buf_set_u32(scan_out, 0, field.num_bits, data_out);

	field.in_value = data_in;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	keep_alive();
}

// FIXME: is this thing actually useful?
void loongarch_ljtag_drscan_32_out(struct loongarch_ljtag *ljtag_info, uint32_t data)
{
	loongarch_ljtag_drscan_32_queued(ljtag_info, data, NULL);
}

int loongarch_ljtag_drscan_32(struct loongarch_ljtag *ljtag_info, uint32_t *data)
{
	// TODO: bool readback
	uint8_t scan_in[4];
	loongarch_ljtag_drscan_32_queued(ljtag_info, *data, scan_in);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(scan_in, 0, 32);
	return ERROR_OK;
}

int loongarch_ljtag_get_idcode(struct loongarch_ljtag *ljtag_info)
{
	loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_IDCODE);

	ljtag_info->idcode = 0;
	return loongarch_ljtag_drscan_32(ljtag_info, &ljtag_info->idcode);
}

int loongarch_ljtag_init(struct loongarch_ljtag *ljtag_info)
{
	int retval = loongarch_ljtag_get_idcode(ljtag_info);
	if (retval != ERROR_OK)  {
		LOG_ERROR("IDCODE read failed");
		return retval;
	}

	/* Current LoongArch devices all have IDCODE=0x5a5a5a5a, check this */
	if (ljtag_info->idcode != 0x5A5A5A5A) {
		LOG_ERROR("Invalid IDCODE (0x%08" PRIx32 " != 0x5A5A5A5A)",
			  ljtag_info->idcode);
		return ERROR_FAIL;
	}

	/* Set ProbEn and ProbTrap as we're always going to use them */
	ljtag_info->ljtag_ctrl = LJTAG_CTRL_PR | LJTAG_CTRL_PEN |
			LJTAG_CTRL_PROBTRAP;

	return ERROR_OK;
}

int loongarch_ljtag_enter_debug(struct loongarch_ljtag *ljtag_info)
{
	uint32_t ljtag_ctrl;
	int retry_count;

	// Quirk: May require retrying a few times before DM bit can be set
	for (retry_count = 5; retry_count != 0; --retry_count) {
		loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_CONTROL);

		/* set debug break bit */
		ljtag_ctrl = LJTAG_CTRL_BREAK | LJTAG_CTRL_PR | LJTAG_CTRL_PEN
			| LJTAG_CTRL_PROBTRAP;
		loongarch_ljtag_drscan_32(ljtag_info, &ljtag_ctrl);
		
		/* See if we stopped processor */
		LOG_DEBUG("enter_debug: control=0x%08" PRIx32 "", ljtag_ctrl);
		if (ljtag_ctrl & LJTAG_CTRL_DM) {
			return ERROR_OK;
		}
	}

	LOG_ERROR("enter_debug: Failed to enter Debug Mode!");
	return ERROR_FAIL;
}

int loongarch_ljtag_exit_debug(struct loongarch_ljtag *ljtag_info)
{
	const uint32_t code[] = {
		LOONG64_ERTN,
		LOONG64_ERTN,
	};
	
	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code, 0,
		NULL, 0, NULL);
}

/**
 * @brief Get the config state of whether the use of FASTDATA is allowed
 */
bool loongarch_ljtag_get_use_fastdata(void)
{
	return cfg_use_fastdata;
}

/**
 * @defgroup Global LoongArch CPUs LJTAG debugging configuration commands
 * @{
 */

COMMAND_HANDLER(loongarch_ljtag_handle_use_fastdata)
{
	if (CMD_ARGC > 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC == 0) {
		command_print(CMD, "FASTDATA Usage: %s",
			(cfg_use_fastdata ? "enabled" : "disabled"));
		return ERROR_OK;
	}

	if (!strcmp(CMD_ARGV[0], "enable")) {
		cfg_use_fastdata = true;
	} else if (!strcmp(CMD_ARGV[0], "disable")) {
		cfg_use_fastdata = false;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static const struct command_registration loongarch_ljtag_exec_command_handlers[] = {
	{
		.name = "use-fastdata",
		.handler = loongarch_ljtag_handle_use_fastdata,
		.mode = COMMAND_ANY,
		.usage = "['enable'|'disable']",
		.help = "Specify whether to use FASTDATA for LJTAG accesses (enabled by default)",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration loongarch_ljtag_command_handlers[] = {
	{
		.name = "loongarch-ljtag",
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Configure global LoongArch CPU LJTAG debugging behavior",
		.chain = loongarch_ljtag_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * @} // Global LoongArch CPUs LJTAG debugging configuration commands
 */
