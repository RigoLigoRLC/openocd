/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>                 *
 *   Initial LoongArch support is based on MIPS code.                      *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_LOONGARCH_EJTAG_H
#define OPENOCD_TARGET_LOONGARCH_EJTAG_H

#include <jtag/jtag.h>

/* TAP instructions */
#define LAEJTAG_INST_BYPASS		0x0
#define LAEJTAG_INST_IDCODE		0x1
#define LAEJTAG_INST_ADDRESS		0x3
#define LAEJTAG_INST_DATA		0x4
#define LAEJTAG_INST_CONTROL		0x5
#define LAEJTAG_INST_ALL		0x7
#define LAEJTAG_INST_FASTDATA		0x8

/* EJTAG control register bits
 * These might not be all bits implemented in LoongArch processors but are all
 * mandatory for debugging */
#define LAEJTAG_CTRL_DM			(1 << 3)
#define LAEJTAG_CTRL_JTAGBRK		(1 << 12)
#define LAEJTAG_CTRL_PROBTRAP		(1 << 14)
#define LAEJTAG_CTRL_PROBEN		(1 << 15)
#define LAEJTAG_CTRL_PRACC		(1 << 18)
#define LAEJTAG_CTRL_PRNW		(1 << 19)

struct loongarch_ejtag {
	struct jtag_tap *tap;

	uint32_t idcode;
	uint32_t ejtag_ctrl;
};

extern const struct command_registration loongarch_ejtag_command_handlers[];

void loongarch_ejtag_add_write_ir(struct loongarch_ejtag *ejtag_info, uint32_t new_instr);

int loongarch_ejtag_drscan_64(struct loongarch_ejtag *ejtag_info, uint64_t *data);
int loongarch_ejtag_fastdata_scan_64(struct loongarch_ejtag *ejtag_info,
				     uint64_t *data,
				     bool spracc,
				     bool readback);
void loongarch_ejtag_drscan_32_out(struct loongarch_ejtag *ejtag_info, uint32_t data);
int loongarch_ejtag_drscan_32(struct loongarch_ejtag *ejtag_info, uint32_t *data);
int loongarch_ejtag_fastdata_scan_32(struct loongarch_ejtag *ejtag_info,
				     uint32_t *data,
				     bool spracc,
				     bool readback); // TODO: LA32?

int loongarch_ejtag_get_idcode(struct loongarch_ejtag *ejtag_info);

int loongarch_ejtag_init(struct loongarch_ejtag *ejtag_info);
int loongarch_ejtag_enter_debug(struct loongarch_ejtag *ejtag_info);
int loongarch_ejtag_exit_debug(struct loongarch_ejtag *ejtag_info);

bool loongarch_ejtag_get_use_fastdata(void);

#endif /* OPENOCD_TARGET_LOONGARCH_EJTAG_H */
