/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Support for LoongArch64 processors with LJTAG debugging capability:
 * all the low-level support routines that require executing a miniprogram
 * through the memory accesses processed by debug probe.
 *
 *   Copyright (C) 2025-2026 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on OpenOCD MIPS implementation,
 * and LJTAG implementation from OpenLA132 open source CPU core
 * (https://github.com/loongson/OpenLACores/).
 */

#ifndef OPENOCD_TARGET_LOONGARCH64_DBGM
#define OPENOCD_TARGET_LOONGARCH64_DBGM

#include "loongarch_ljtag.h"

#define LOONG64_DBGM_BASE		0xDB00000000000000ull
#define LOONG64_DBGM_SIZE		0x0000000000100000ull

#define LOONG64_DBGM_TEXT		LOONG64_DBGM_BASE

/* Offset field of LoongArch st.d/ld.d only ranges from -0x800 to +0x7FF,
 * 0x400 is essentially the maximum size we can have for parameter areas */
#define LOONG64_DBGM_PARAM_IN		0xDB00000000001000ull
#define LOONG64_DBGM_PARAM_IN_SIZE	0x400
#define LOONG64_DBGM_PARAM_OUT		(LOONG64_DBGM_PARAM_IN + LOONG64_DBGM_PARAM_IN_SIZE)
#define LOONG64_DBGM_PARAM_OUT_SIZE	0x400
#define LOONG64_DBGM_STACK		0xDB00000000001800ull
#define LOONG64_DBGM_PIPE_IN           0xDB00000000001810ull
#define LOONG64_DBGM_PIPE_OUT          0xDB00000000001820ull

#define NEG12(v) ((uint32_t)(((~(v)) + 1) & 0xFFF))
#define NEG26(v) ((uint32_t)(((~(v)) + 1) & 0x3FFFFFF))

#define LOONG64_DBGM_INSN_STEP 4
#define LOONG64_DBGM_DATA_STEP 8

int loongarch64_dbgm_read_mem(struct loongarch_ljtag *ljtag_info, uint64_t addr,
			       unsigned int size, unsigned int count, void *buf);

int loongarch64_dbgm_write_mem(struct loongarch_ljtag *ljtag_info, uint64_t addr,
				unsigned int size, unsigned int count, void *buf);

int loongarch64_dbgm_exec(struct loongarch_ljtag *ljtag_info,
			   unsigned int code_len, const uint32_t *code,
			   unsigned int num_param_in, uint64_t *param_in,
			   unsigned int num_param_out, uint64_t *param_out);

int loongarch64_dbgm_read_regs(struct loongarch_ljtag *ljtag_info,
				uint64_t *regs);

int loongarch64_dbgm_read_csr(struct loongarch_ljtag *ljtag_info, uint32_t csr, uint64_t *value);

int loongarch64_dbgm_write_csr(struct loongarch_ljtag *ljtag_info, uint32_t csr, uint64_t value);

#endif /* OPENOCD_TARGET_LOONGARCH64_DBGM */
