/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>                 *
 *   Initial LoongArch support is based on MIPS code.                      *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "loongarch64_pracc.h"
#include "loongarch64.h"

void loongarch_ejtag_set_instr(struct loongarch_ejtag *ejtag_info, uint32_t new_instr)
{
	assert(ejtag_info->tap);
	struct jtag_tap *tap = ejtag_info->tap;

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
		jtag_add_runtest(50, TAP_IDLE);
	}
}

int loongarch_ejtag_drscan_64(struct loongarch_ejtag *ejtag_info, uint64_t *data)
{
	struct jtag_tap *tap = ejtag_info->tap;
	if (!tap) {
		LOG_ERROR("ejtag_info->tap is NULL!");
		return ERROR_FAIL;
	}

	struct scan_field field;
	uint8_t t[8] = { 0 }, r[8];
	int retval;

	field.num_bits = 64;
	field.out_value = t;
	buf_set_u64(t, 0, field.num_bits, *data);
	field.in_value = r;

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

static void loongarch_ejtag_drscan_32_queued(struct loongarch_ejtag *ejtag_info,
		uint32_t data_out, uint8_t *data_in)
{
	assert(ejtag_info->tap);
	struct jtag_tap *tap = ejtag_info->tap;

	struct scan_field field;
	field.num_bits = 32;

	uint8_t scan_out[4] = { 0 };
	field.out_value = scan_out;
	buf_set_u32(scan_out, 0, field.num_bits, data_out);

	field.in_value = data_in;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	keep_alive();
}

void loongarch_ejtag_drscan_32_out(struct loongarch_ejtag *ejtag_info, uint32_t data)
{
	loongarch_ejtag_drscan_32_queued(ejtag_info, data, NULL);
}

int loongarch_ejtag_drscan_32(struct loongarch_ejtag *ejtag_info, uint32_t *data)
{
	uint8_t scan_in[4];
	loongarch_ejtag_drscan_32_queued(ejtag_info, *data, scan_in);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(scan_in, 0, 32);
	return ERROR_OK;
}

int loongarch_ejtag_get_idcode(struct loongarch_ejtag *ejtag_info)
{
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_IDCODE);

	ejtag_info->idcode = 0;
	return loongarch_ejtag_drscan_32(ejtag_info, &ejtag_info->idcode);
}

int loongarch_ejtag_init(struct loongarch_ejtag *ejtag_info)
{
	int retval = loongarch_ejtag_get_idcode(ejtag_info);
	if (retval != ERROR_OK)  {
		LOG_ERROR("IDCODE read failed");
		return retval;
	}

	/* Current LoongArch devices all have IDCODE=0x5a5a5a5a, check this */
	if (ejtag_info->idcode != 0x5A5A5A5A) {
		LOG_ERROR("Invalid IDCODE (not 0x5A5A5A5A)");
		return ERROR_FAIL;
	}

	/* Set ProbEn and ProbTrap as we're always going to use them */
	ejtag_info->ejtag_ctrl = LAEJTAG_CTRL_PRACC | LAEJTAG_CTRL_PROBEN |
			LAEJTAG_CTRL_PROBTRAP;

	return ERROR_OK;
}

int loongarch_ejtag_enter_debug(struct loongarch_ejtag *ejtag_info)
{
	uint32_t ejtag_ctrl;
	int retry_count;

	// Quirk: May require retrying a few times before DM bit can be set
	for (retry_count = 5; retry_count != 0; --retry_count) {
		loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);

		/* set debug break bit */
		ejtag_ctrl = LAEJTAG_CTRL_JTAGBRK | LAEJTAG_CTRL_PRACC
			| LAEJTAG_CTRL_PROBEN | LAEJTAG_CTRL_PROBTRAP;
		loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		
		/* See if we stopped processor */
		LOG_DEBUG("enter_debug: control=0x%08" PRIx32 "", ejtag_ctrl);
		if (ejtag_ctrl & LAEJTAG_CTRL_DM) {
			return ERROR_OK;
		}
	}

	LOG_ERROR("enter_debug: Failed to enter Debug Mode!");
	return ERROR_FAIL;
}

int loongarch_ejtag_exit_debug(struct loongarch_ejtag *ejtag_info)
{
	const uint32_t code[] = {
		LOONG64_ERTN,
		LOONG64_ERTN,
	};
	
	return loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code, 0,
		NULL, 0, NULL);
}
