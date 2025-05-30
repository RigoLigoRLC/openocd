/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>                 *
 *   Initial LoongArch support is based on MIPS code.                      *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH64_PRACC
#define OPENOCD_TARGET_LOONGARCH64_PRACC

#include "loongarch_ejtag.h"

#define LOONG64_PRACC_TEXT		0xDB00000000000200ull

#define LOONG64_PRACC_STACK		0xDB00000000004000ull
#define LOONG64_PRACC_PARAM_IN		0xDB00000000001000ull
#define LOONG64_PRACC_PARAM_IN_SIZE	0x1000
#define LOONG64_PRACC_PARAM_OUT		(LOONG64_PRACC_PARAM_IN + LOONG64_PRACC_PARAM_IN_SIZE)
#define LOONG64_PRACC_PARAM_OUT_SIZE	0x1000


#define LOONG64_PRACC_ADDR_STEP 4
#define LOONG64_PRACC_DATA_STEP 8

#endif /* OPENOCD_TARGET_LOONGARCH64_PRACC */
