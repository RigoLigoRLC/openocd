/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>                 *
 *   Initial LoongArch support is based on MIPS code.                      *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH64_PRACC
#define OPENOCD_TARGET_LOONGARCH64_PRACC

#include "loongarch_ejtag.h"

#define LOONG64_PRACC_TEXT		0xDB00000000000000ull

/* Offset field of LoongArch st.d/ld.d only ranges from -0x800 to +0x7FF,
 * 0x400 is essentially the maximum size we can have for parameter areas */
#define LOONG64_PRACC_PARAM_IN		0xDB00000000001000ull
#define LOONG64_PRACC_PARAM_IN_SIZE	0x400
#define LOONG64_PRACC_PARAM_OUT		(LOONG64_PRACC_PARAM_IN + LOONG64_PRACC_PARAM_IN_SIZE)
#define LOONG64_PRACC_PARAM_OUT_SIZE	0x400
#define LOONG64_PRACC_STACK		0xDB00000000001800ull

#define NEG12(v) ((uint32_t)(((~(v)) + 1) & 0xFFF))
#define NEG26(v) ((uint32_t)(((~(v)) + 1) & 0x3FFFFFF))

#define LOONG64_PRACC_INSN_STEP 4
#define LOONG64_PRACC_DATA_STEP 8

int loongarch64_pracc_read_mem(struct loongarch_ejtag *ejtag_info, uint64_t addr,
			       unsigned int size, unsigned int count, void *buf);

int loongarch64_pracc_write_mem(struct loongarch_ejtag *ejtag_info, uint64_t addr,
				unsigned int size, unsigned int count, void *buf);

int loongarch64_pracc_exec(struct loongarch_ejtag *ejtag_info,
			   unsigned int code_len, const uint32_t *code,
			   unsigned int num_param_in, uint64_t *param_in,
			   unsigned int num_param_out, uint64_t *param_out);

int loongarch64_pracc_read_regs(struct loongarch_ejtag *ejtag_info,
				uint64_t *regs);

#endif /* OPENOCD_TARGET_LOONGARCH64_PRACC */
