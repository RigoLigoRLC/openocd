/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>                 *
 *   Initial LoongArch support is based on MIPS code.                      *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH64_H
#define OPENOCD_TARGET_LOONGARCH64_H

#include "target/target.h"
#include "target/register.h"
#include "loongarch64_pracc.h"

#define LOONG64_COMMON_MAGIC	0x62646264U

/* Offsets into LoongArch64 register dump */
#define LOONG64_NUM_CORE_REGS	33
#define LOONG64_NUM_REGS	(LOONG64_NUM_CORE_REGS)

struct loongarch64_common {
	unsigned int common_magic;

	void *arch_info;			/* back pointer to parent */
	struct reg_cache *core_cache;
	struct loongarch_ejtag ejtag_info;
	uint64_t core_regs[LOONG64_NUM_REGS];

	struct working_area *fast_data_area;
};

#define LOONG64_OP_OR		0x2a
#define LOONG64_OP_ORI		0xe
#define LOONG64_OP_ADD_D	0x21
#define LOONG64_OP_LU12I_W	0xa
#define LOONG64_OP_LU32I_D	0xb
#define LOONG64_OP_LU52I_D	0xc
#define LOONG64_OP_LD_B		0xa0
#define LOONG64_OP_LD_H		0xa1
#define LOONG64_OP_LD_W		0xa2
#define LOONG64_OP_LD_D		0xa3
#define LOONG64_OP_ST_B		0xa4
#define LOONG64_OP_ST_H		0xa5
#define LOONG64_OP_ST_W		0xa6
#define LOONG64_OP_ST_D		0xa7
#define LOONG64_OP_PCADDI	0xc
#define LOONG64_OP_DBAR		0x70e4
#define LOONG64_OP_IBAR		0x70e5
#define LOONG64_OP_CSROP	0x4
#define LOONG64_OP_B		0x14
#define LOONG64_OP_ERTN		0x1920e

#define LOONG64_INSN_2R(opcode, rd, rj)		(((opcode) << 10) | ((rj) << 5) | (rd))
#define LOONG64_INSN_3R(opcode, rd, rj, rk)	(((opcode) << 15) | ((rk) << 10) | ((rj) << 5) | (rd))
#define LOONG64_INSN_2RI12(opcode, rd, rj, imm)	(((opcode) << 22) | ((imm) << 10) | ((rj) << 5) | (rd))
#define LOONG64_INSN_2RI14(opcode, rd, rj, imm)	(((opcode) << 24) | ((imm) << 10) | ((rj) << 5) | (rd))
#define LOONG64_INSN_1RI20(opcode, rd, imm)	(((opcode) << 25) | ((imm) << 5) | (rd))
#define LOONG64_INSN_I15(opcode, imm)		(((opcode) << 15) | (imm))
#define LOONG64_INSN_I26(opcode, imm)		(((opcode) << 26) | (((imm) & 0xFFFF) << 10) | (((imm) >> 16) & 0x3FF))

#define LOONG64_OR(rd, rj, rk)			LOONG64_INSN_3R(LOONG64_OP_OR, rd, rj, rk)
#define LOONG64_ORI(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_ORI, rd, rj, imm)
#define LOONG64_ADD_D(rd, rj, rk)		LOONG64_INSN_3R(LOONG64_OP_ADD_D, rd, rj, rk)
#define LOONG64_LU12I_W(rd, imm)		LOONG64_INSN_1RI20(LOONG64_OP_LU12I_W, rd, imm)
#define LOONG64_LU32I_D(rd, imm)		LOONG64_INSN_1RI20(LOONG64_OP_LU32I_D, rd, imm)
#define LOONG64_LU52I_D(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_LU52I_D, rd, rj, imm)
#define LOONG64_LD_B(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_LD_B, rd, rj, imm)
#define LOONG64_LD_H(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_LD_H, rd, rj, imm)
#define LOONG64_LD_W(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_LD_W, rd, rj, imm)
#define LOONG64_LD_D(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_LD_D, rd, rj, imm)
#define LOONG64_ST_B(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_ST_B, rd, rj, imm)
#define LOONG64_ST_H(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_ST_H, rd, rj, imm)
#define LOONG64_ST_W(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_ST_W, rd, rj, imm)
#define LOONG64_ST_D(rd, rj, imm)		LOONG64_INSN_2RI12(LOONG64_OP_ST_D, rd, rj, imm)
#define LOONG64_PCADDI(rd, imm)			LOONG64_INSN_1RI20(LOONG64_OP_PCADDI, rd, imm)
#define LOONG64_DBAR(hint)			LOONG64_INSN_I15(LOONG64_OP_DBAR, hint)
#define LOONG64_IBAR(hint)			LOONG64_INSN_I15(LOONG64_OP_IBAR, hint)
#define LOONG64_CSRRD(rd, csr)			LOONG64_INSN_2RI14(LOONG64_OP_CSROP, rd, 0, csr)
#define LOONG64_CSRWR(rd, csr)			LOONG64_INSN_2RI14(LOONG64_OP_CSROP, rd, 1, csr)
#define LOONG64_CSRXCHG(rd, csr)		LOONG64_INSN_2RI14(LOONG64_OP_CSROP, rd, 2, csr)
#define LOONG64_B(imm)				LOONG64_INSN_I26(LOONG64_OP_B, imm)
#define LOONG64_ERTN				LOONG64_INSN_2R(LOONG64_OP_ERTN, 0, 0)

/* Pseudo instructions for convenience */
#define LOONG64_NOP				LOONG64_OR(0, 0, 0)
#define LOONG64_MOVE(rd, rj)			LOONG64_OR(rd, 0, rj)
#define LOONG64_LI_W(rd, word)			LOONG64_ORI(rd, 0, (word & 0xfffu)), \
						LOONG64_LU12I_W(rd, ((word & 0xfffff000u) >> 12))
#define LOONG64_LI_D(rd, dword)			LOONG64_ORI(rd, 0, (dword & 0xfffu)), \
						LOONG64_LU12I_W(rd, ((dword & 0xfffff000u) >> 12)), \
						LOONG64_LU32I_D(rd, ((dword & 0xfffff00000000u) >> 32)), \
						LOONG64_LU52I_D(rd, rd, ((dword & 0xfff0000000000000u) >> 52))

#endif /* OPENOCD_TARGET_LOONGARCH64_H */
