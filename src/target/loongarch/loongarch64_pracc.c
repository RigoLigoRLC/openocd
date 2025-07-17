// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Support for LoongArch64 processors with EJTAG-style debugging capability
 *
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on MIPS code.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "loongarch64.h"
#include "loongarch64_pracc.h"

#include <jtag/adapter.h>

#define STACK_DEPTH	32

struct loongarch64_pracc_context {
	uint64_t *local_iparam;
	unsigned int num_iparam;
	uint64_t *local_oparam;
	unsigned int num_oparam;
	const uint32_t *code;
	unsigned int code_len;
	uint64_t stack[STACK_DEPTH];
	unsigned int stack_offset;
	struct loongarch_ejtag *ejtag_info;
};

static int wait_for_pracc_rw(struct loongarch_ejtag *ejtag_info, uint32_t *ctrl)
{
	uint32_t ejtag_ctrl;
	int nt = 5;
	int rc;

	while (1) {
		loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);
		ejtag_ctrl = ejtag_info->ejtag_ctrl;
		rc = loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (rc != ERROR_OK)
			return rc;

		if (ejtag_ctrl & LAEJTAG_CTRL_PRACC)
			break;
		LOG_DEBUG("DEBUGMODULE: No memory access in progress!\n");
		if (nt == 0)
			return ERROR_JTAG_DEVICE_ERROR;
		nt--;
	}

	*ctrl = ejtag_ctrl;
	return ERROR_OK;
}

static int loongarch64_pracc_exec_read(struct loongarch64_pracc_context *ctx, uint64_t address) {
	struct loongarch_ejtag *ejtag_info = ctx->ejtag_info;
	unsigned int offset;
	uint32_t ejtag_ctrl;
	uint64_t data;
	int rc;

	
}
