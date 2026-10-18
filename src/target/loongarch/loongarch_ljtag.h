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

#ifndef OPENOCD_TARGET_LOONGARCH_LJTAG_H
#define OPENOCD_TARGET_LOONGARCH_LJTAG_H

#include <jtag/jtag.h>

/*
 * TAP instructions
 * Definition is taken from OpenLA132 core's RTL implementation
 */
#define LJTAG_INST_BYPASS		0x0
#define LJTAG_INST_IDCODE		0x1
#define LJTAG_INST_IMP			0x2
#define LJTAG_INST_ADDRESS		0x3
#define LJTAG_INST_DATA			0x4
#define LJTAG_INST_CONTROL		0x5
#define LJTAG_INST_PC			0x6
#define LJTAG_INST_ALL			0x7
#define LJTAG_INST_FASTDATA		0x8

/* 
 * LJTAG control register bits
 * Definition is taken from OpenLA132 core's RTL implementation
 */
#define LJTAG_CTRL_DM			(1 << 3)  // Indicates processor is in debug mode
#define LJTAG_CTRL_BREAK		(1 << 12) // Set this bit to break into debug mode
#define LJTAG_CTRL_PROBTRAP		(1 << 14) // FIXME: This doesnt exist in OpenLA132 need verify on 2K0300&1C203
#define LJTAG_CTRL_PEN			(1 << 15) // Enables probes to handle processor's memory accesses
#define LJTAG_CTRL_PR			(1 << 18) // Indicates a pending memory access request from processor
#define LJTAG_CTRL_PW			(1 << 19) // 1=The pending request is a write  0=Read
#define LJTAG_CTRL_RC			(1 << 31) // Indicate a reset has occurred. Must write 0 to clear

struct loongarch_ljtag {
	struct jtag_tap *tap;

	uint32_t idcode;
	uint32_t ljtag_ctrl;
};

extern const struct command_registration loongarch_ljtag_command_handlers[];

void loongarch_ljtag_add_write_ir(struct loongarch_ljtag *ljtag_info, uint32_t new_instr);

int loongarch_ljtag_drscan_64(struct loongarch_ljtag *ljtag_info, uint64_t *data);
int loongarch_ljtag_fastdata_scan_64(struct loongarch_ljtag *ljtag_info,
				     uint64_t *data,
				     bool pr,
				     bool readback);
void loongarch_ljtag_drscan_32_out(struct loongarch_ljtag *ljtag_info, uint32_t data);
int loongarch_ljtag_drscan_32(struct loongarch_ljtag *ljtag_info, uint32_t *data);
int loongarch_ljtag_fastdata_scan_32(struct loongarch_ljtag *ljtag_info,
				     uint32_t *data,
				     bool pr,
				     bool readback); // TODO: LA32?

int loongarch_ljtag_get_idcode(struct loongarch_ljtag *ljtag_info);

int loongarch_ljtag_init(struct loongarch_ljtag *ljtag_info);
int loongarch_ljtag_enter_debug(struct loongarch_ljtag *ljtag_info);
int loongarch_ljtag_exit_debug(struct loongarch_ljtag *ljtag_info);

bool loongarch_ljtag_get_use_fastdata(void);

#endif /* OPENOCD_TARGET_LOONGARCH_LJTAG_H */
