// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Support for LoongArch64 processors with EJTAG-style debugging capability:
 * all the low-level support routines that require executing a miniprogram
 * through the memory accesses processed by debug probe.
 *
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on MIPS code.
 * 
 * In EJTAG-style debugging workflow, a Debug Exception is triggered on CPU by
 * setting a special JTAG register bit. After which the processor will save the
 * exception $pc to a CSR and jump to "debug memory segment" (dmseg) and execute
 * from there. But that piece of memory is neither backed by physical memory nor
 * translated through MMU: it's backed by the debug probe. Whenever processor
 * tries to do memory access inside dmseg, the core will stall because it has
 * wait until the debug probe finishes the memory access. If it's a read, the
 * probe will have to write to a JTAG register to supply the data that CPU will
 * see, and clear a bit (PrAcc, processor access bit) to show that the access is
 * done. If it's a write, the probe will have to read that JTAG register to see
 * what the CPU is trying to write, and save the value so that debugger software
 * can use it, and clear PrAcc bit to tell the CPU the access is done. Basically
 * this file contains all the logic for the emulated dmseg region: what to do if
 * CPU reads/writes some address inside dmseg, etc. Some basic operations like
 * memory access and register dumping is also included.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/types.h"
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
	// Set PrAcc bit when reading Control register, avoid accidentally
	// "completing" a pending access
	uint32_t ejtag_ctrl;
	int retry_count = 5;
	int retval;

	while (1) {
		loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);
		ejtag_ctrl = LAEJTAG_CTRL_PRACC | LAEJTAG_CTRL_PROBEN |
			     LAEJTAG_CTRL_PROBTRAP;
		retval = loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			return retval;
		}

		if (ejtag_ctrl & LAEJTAG_CTRL_PRACC) {
			break;
		}
		LOG_DEBUG("DEBUGMODULE: No memory access in progress! (Control=%08" PRIx32 ")", ejtag_ctrl);
		if (retry_count == 0) {
			return ERROR_JTAG_DEVICE_ERROR;
		}
		retry_count--;
	}

	*ctrl = ejtag_ctrl;
	return ERROR_OK;
}

static int loongarch64_pracc_exec_read(struct loongarch64_pracc_context *ctx, uint64_t address) {
	struct loongarch_ejtag *ejtag_info = ctx->ejtag_info;
	unsigned int offset;
	uint32_t ejtag_ctrl;
	uint64_t data;
	int retval;

	if ((address >= LOONG64_PRACC_PARAM_IN) &&
	    (address < LOONG64_PRACC_PARAM_IN + ctx->num_iparam * LOONG64_PRACC_DATA_STEP)) {
		
		offset = (address - LOONG64_PRACC_PARAM_IN) / LOONG64_PRACC_DATA_STEP;

		if (offset >= LOONG64_PRACC_PARAM_IN_SIZE) {
			LOG_ERROR("iparam size exceeds LOONG64_PRACC_PARAM_IN_SIZE");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		if (!ctx->local_iparam) {
			LOG_ERROR("unexpected reading of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_iparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_PRACC_PARAM_OUT) &&
		   (address < LOONG64_PRACC_PARAM_OUT + ctx->num_oparam * LOONG64_PRACC_DATA_STEP)) {
		
		offset = (address - LOONG64_PRACC_PARAM_OUT) / LOONG64_PRACC_DATA_STEP;
		if (!ctx->local_oparam) {
			LOG_ERROR("unexpected reading of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_oparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_PRACC_TEXT) &&
		   (address < LOONG64_PRACC_TEXT + ctx->code_len * LOONG64_PRACC_INSN_STEP)) {
		
		offset = ((address & ~7ull) - LOONG64_PRACC_TEXT) / LOONG64_PRACC_INSN_STEP;
		data = (uint64_t)ctx->code[offset];
		if (offset + 1 < ctx->code_len)
			data |= (uint64_t)ctx->code[offset + 1] << 32;
		LOG_DEBUG("Executing instructions %08" PRIx64 " at %08" PRIx64, data, address);
	} else if ((address & ~7ull) == LOONG64_PRACC_STACK) {

		/* Read from debug stack */
		if (ctx->stack_offset == 0) {
			LOG_ERROR("Error reading from stack: stack is empty");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->stack[--ctx->stack_offset];
		LOG_DEBUG("Reading stack %" PRIx64 " at %" PRIx64, data, address);
	} else {
		data = LOONG64_NOP | ((uint64_t)LOONG64_NOP) << 32;
		LOG_ERROR("Reading unexpected address %" PRIx64, address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Send the data out */
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_DATA);
	retval = loongarch_ejtag_drscan_64(ejtag_info, &data);
	if (retval != ERROR_OK) {
		return retval;
	}

	/* Finish the memory access */
	ejtag_ctrl = LAEJTAG_CTRL_PROBEN | LAEJTAG_CTRL_PROBTRAP;
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);
	retval = loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK) {
		return retval;
	}

	// jtag_add_clocks(5);

	return jtag_execute_queue();
}

static int loongarch64_pracc_exec_write(struct loongarch64_pracc_context *ctx, uint64_t address) {
	struct loongarch_ejtag *ejtag_info = ctx->ejtag_info;
	unsigned int offset;
	uint32_t ejtag_ctrl;
	uint64_t data;
	int retval;

	/* Get the data written by CPU */
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_DATA);
	retval = loongarch_ejtag_drscan_64(ejtag_info, &data);
	if (retval != ERROR_OK) {
		return retval;
	}

	/* Finish the memory access */
	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~LAEJTAG_CTRL_PRACC;
	loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_CONTROL);
	retval = loongarch_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK) {
		return retval;
	}

	// jtag_add_clocks(5);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		return retval;
	}

	if ((address >= LOONG64_PRACC_PARAM_IN) &&
	    (address < LOONG64_PRACC_PARAM_IN + ctx->num_iparam * LOONG64_PRACC_DATA_STEP)) {
		
		offset = (address - LOONG64_PRACC_PARAM_IN) / LOONG64_PRACC_DATA_STEP;

		if (offset >= LOONG64_PRACC_PARAM_IN_SIZE) {
			LOG_ERROR("iparam size exceeds LOONG64_PRACC_PARAM_IN_SIZE");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		if (!ctx->local_iparam) {
			LOG_ERROR("unexpected writing of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ctx->local_iparam[offset] = data;
		LOG_DEBUG("Writing %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_PRACC_PARAM_OUT) &&
		   (address < LOONG64_PRACC_PARAM_OUT + ctx->num_oparam * LOONG64_PRACC_DATA_STEP)) {
		
		offset = (address - LOONG64_PRACC_PARAM_OUT) / LOONG64_PRACC_DATA_STEP;
		if (!ctx->local_oparam) {
			LOG_ERROR("unexpected writing of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ctx->local_oparam[offset] = data;
		LOG_DEBUG("Writing %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address & ~7ull) == LOONG64_PRACC_STACK) {

		/* Write into debug stack */
		if (ctx->stack_offset == STACK_DEPTH) {
			LOG_ERROR("Error writing into stack: stack is full");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ctx->stack[ctx->stack_offset++] = data;
		LOG_DEBUG("Writing stack %" PRIx64 " at %" PRIx64, data, address);
	} else {
		LOG_ERROR("Writing to unexpected address %" PRIx64, address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int loongarch64_pracc_exec(struct loongarch_ejtag *ejtag_info,
			   unsigned int code_len, const uint32_t *code,
			   unsigned int num_param_in, uint64_t *param_in,
			   unsigned int num_param_out, uint64_t *param_out)
{
	uint32_t ejtag_ctrl;
	uint64_t address = 0;
	struct loongarch64_pracc_context ctx;
	int retval;
	int pass = 0;
	unsigned int i;

	for (i = 0; i < code_len; i++) {
		LOG_DEBUG("%08" PRIx32, code[i]);
	}

	ctx.local_iparam = param_in;
	ctx.local_oparam = param_out;
	ctx.num_iparam = num_param_in;
	ctx.num_oparam = num_param_out;
	ctx.code = code;
	ctx.code_len = code_len;
	ctx.ejtag_info = ejtag_info;
	ctx.stack_offset = 0;

	while (true) {
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			/*
			* The reason why this specific quirk exists is that, a silicon bug exists in
			* LS2K0300 causes the machine to read one more instruction from dmseg after it
			* has executed ERET instruction and returned from debug mode, kind of like a
			* MIPS delay slot but that extra instruction is never executed. Because of this
			* uncertainty, we detect if we're trying to exit debug mode when . If there's a
			* newer chip that has fixed this bug, OpenOCD may safely interrupt the
			* execution of ERET instruction flow.
			*/
			if (address >= LOONG64_PRACC_TEXT &&
			    address < LOONG64_PRACC_TEXT + code_len * LOONG64_PRACC_INSN_STEP &&
			    code[((address & ~7ull) - LOONG64_PRACC_TEXT) / LOONG64_PRACC_INSN_STEP]
				== LOONG64_ERTN) {
				
				LOG_DEBUG("Exited from debug mode");
				return ERROR_OK;
			}
			LOG_ERROR("ERROR wait_for_pracc_rw %d", retval);
			return retval;
		}

		loongarch_ejtag_set_instr(ejtag_info, LAEJTAG_INST_ADDRESS);
		loongarch_ejtag_drscan_64(ejtag_info, &address);
		LOG_DEBUG("-> %08" PRIx64, address);

		/* Since the memory logic is taken from MIPS code, we also inherit its access
		 * characteristics: because code is stored as 32-bit words and data (parameters) are
		 * stored as 64-bit words, unaligned accesses, and 32-bit-aligned accesses to data
		 * area is not allowed. */
		int psz = (ejtag_ctrl >> 29) & 3;
		int address20 = address & 7;
		switch (psz) {
		case 3:
			if (address20 != 7) {
				LOG_ERROR("Unaligned 64-bit access not supported (Psz=%d Address[2:0]=%d)", psz, address20);
				return ERROR_FAIL;
			}
			address &= ~7ull;
			break;
		case 2:
			if (address20 != 0 && address20 != 4) {
				LOG_ERROR("Unaligned 32-bit access not supported (Psz=%d Address[2:0]=%d)", psz, address20);
				return ERROR_FAIL;
			}
			break;
		default:
			LOG_ERROR("Accesses smaller than 32-bit not supported (Psz=%d Address[2:0]=%d)", psz, address20);
			return ERROR_FAIL;
		}

		if (ejtag_ctrl & LAEJTAG_CTRL_PRNW) {
			/* Write path */
			retval = loongarch64_pracc_exec_write(&ctx, address);
			if (retval != ERROR_OK) {
				LOG_ERROR("loongarch64_pracc_exec_write() failed (%d)", retval);
				return retval;
			}
		} else {
			/* Read path
			 * Just like what's written in MIPS, the second time when CPU reads at dmseg
			 * means that the program has ended execution */
			if ((address == LOONG64_PRACC_TEXT) && (pass++)) {
				LOG_DEBUG("Miniprogram finished execution");
				break;
			}
			retval = loongarch64_pracc_exec_read(&ctx, address);
			if (retval != ERROR_OK) {
				LOG_ERROR("loongarch64_pracc_exec_read() failed (%d)", retval);
				return retval;
			}
		}
	}

	/* Stack sanity check */
	if (ctx.stack_offset != 0) {
		LOG_ERROR("PrAcc stack not balanced after execution (offset = %d)", ctx.stack_offset);
	}

	return ERROR_OK;
}

/* Read 64-bit double-words from memory. Must not read more than size of param_out area */
static int loongarch64_pracc_read_u64(struct loongarch_ejtag *ejtag_info, uint64_t addr,
				      uint64_t count, uint64_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_PRACC_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_PRACC_STACK),
		/* Save $t0 (read address) to stack */
		LOONG64_ST_D(12, 21, 0),
		/* Save $t1 (count) to stack */
		LOONG64_ST_D(13, 21, 0),
		/* Save $t2 (write address) to stack */
		LOONG64_ST_D(14, 21, 0),
		/* Save $t3 (write data) to stack */
		LOONG64_ST_D(15, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_IN)),
		/* $t1 = param_in[1] (count) */
		LOONG64_LD_D(13, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_IN - 8)),
		/* $t2 = &param_out[0] (write address) */
		LOONG64_ADDI_D(14, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_OUT)),
		/* ld.d $t3, $t0, 0 (reads 64 bit from that address into $t3) */
		LOONG64_LD_D(15, 12, 0),
		/* *write_address (param_out[offset]) := $t3 */
		LOONG64_ST_D(15, 14, 0),
		/* addi.d $t0, $t0, 8 (increment read_address by 8) */
		LOONG64_ADDI_D(12, 12, 8),
		/* addi.d $t1, $t1, -1 (decrement count) */
		LOONG64_ADDI_D(13, 13, NEG12(1)),
		/* addi.d $t2, $t2, 8 (increment write_address by 8) */
		LOONG64_ADDI_D(14, 14, 8),
		/* bne zero, $t1, -20 (when not all memory is read, continue read loop) */
		LOONG64_BNE(0, 13, NEG16(5)),
		/* Restore $t3 from stack */
		LOONG64_LD_D(15, 21, 0),
		/* Restore $t2 from stack */
		LOONG64_LD_D(14, 21, 0),
		/* Restore $t1 from stack */
		LOONG64_LD_D(13, 21, 0),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(22)),
	};

	if (count > (LOONG64_PRACC_PARAM_OUT_SIZE / 8)) {
		LOG_ERROR("loongarch64_pracc_read_u64: requested more data than param_out area (%"
			  PRIu64 " bytes requested, maximum %" PRIu32 " bytes)", count * 8,
			  LOONG64_PRACC_PARAM_OUT_SIZE);
		return ERROR_BUF_TOO_SMALL;
	}

	uint64_t param_in[2] = { addr, count };

	return loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, count, buf);
}

static int loongarch64_pracc_read_u32(struct loongarch_ejtag *ejtag_info, uint64_t addr, uint32_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_PRACC_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_PRACC_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_IN)),
		/* ld.w $t0, $t0, 0 (reads 32 bit from that address into $t0) */
		LOONG64_LD_W(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(11)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint32_t)param_out[0];
	return retval;
}

static int loongarch64_pracc_read_u16(struct loongarch_ejtag *ejtag_info, uint64_t addr, uint16_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_PRACC_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_PRACC_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_IN)),
		/* ld.h $t0, $t0, 0 (reads 16 bit from that address into $t0) */
		LOONG64_LD_H(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(11)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint16_t)param_out[0];
	return retval;
}

static int loongarch64_pracc_read_u8(struct loongarch_ejtag *ejtag_info, uint64_t addr, uint8_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_PRACC_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_PRACC_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_IN)),
		/* ld.b $t0, $t0, 0 (reads 16 bit from that address into $t0) */
		LOONG64_LD_B(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_PRACC_STACK - LOONG64_PRACC_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(11)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint8_t)param_out[0];
	return retval;
}

static int loongarch64_pracc_read_mem64(struct loongarch_ejtag *ejtag_info, uint64_t addr,
					unsigned int count, uint64_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; ) {
		const int burst = MIN(count - i, LOONG64_PRACC_PARAM_OUT_SIZE / 8);
		retval = loongarch64_pracc_read_u64(ejtag_info, addr + 8 * i, burst, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
		i += burst;
	}
	return retval;
}

static int loongarch64_pracc_read_mem32(struct loongarch_ejtag *ejtag_info, uint64_t addr,
					unsigned int count, uint32_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_pracc_read_u32(ejtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

static int loongarch64_pracc_read_mem16(struct loongarch_ejtag *ejtag_info, uint64_t addr,
					unsigned int count, uint16_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_pracc_read_u16(ejtag_info, addr + 2 * i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

static int loongarch64_pracc_read_mem8(struct loongarch_ejtag *ejtag_info, uint64_t addr,
					unsigned int count, uint8_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_pracc_read_u8(ejtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

int loongarch64_pracc_read_mem(struct loongarch_ejtag *ejtag_info, uint64_t addr,
			       unsigned int size, unsigned int count, void *buf)
{
	switch (size) {
	case 1:
		return loongarch64_pracc_read_mem8(ejtag_info, addr, count, buf);
	case 2:
		return loongarch64_pracc_read_mem16(ejtag_info, addr, count, buf);
	case 4:
		return loongarch64_pracc_read_mem32(ejtag_info, addr, count, buf);
	case 8:
		return loongarch64_pracc_read_mem64(ejtag_info, addr, count, buf);
	}
	return ERROR_FAIL;
}

/* TODO: Memory writes */

int loongarch64_pracc_read_regs(struct loongarch_ejtag *ejtag_info,
				uint64_t *regs)
{
	const uint32_t code[] = {
		/* Write $r2 to CSR.DSAVE */
		LOONG64_CSRWR(2, 0x502),
		/* First load LOONG64_PRACC_PARAM_OUT into $r2 */
		LOONG64_LI_D(2, LOONG64_PRACC_PARAM_OUT),
		/* Save $r0 and $r1 first */
		LOONG64_ST_D(0, 2, 0x00),
		LOONG64_ST_D(1, 2, 0x08),
		/* Now that $r1 is saved, we use $r1 as the pointer register */
		LOONG64_MOVE(1, 2),
		/* Save $r3 ~ $r31 */
		LOONG64_ST_D(3, 1, 0x18),
		LOONG64_ST_D(4, 1, 0x20),
		LOONG64_ST_D(5, 1, 0x28),
		LOONG64_ST_D(6, 1, 0x30),
		LOONG64_ST_D(7, 1, 0x38),
		LOONG64_ST_D(8, 1, 0x40),
		LOONG64_ST_D(9, 1, 0x48),
		LOONG64_ST_D(10, 1, 0x50),
		LOONG64_ST_D(11, 1, 0x58),
		LOONG64_ST_D(12, 1, 0x60),
		LOONG64_ST_D(13, 1, 0x68),
		LOONG64_ST_D(14, 1, 0x70),
		LOONG64_ST_D(15, 1, 0x78),
		LOONG64_ST_D(16, 1, 0x80),
		LOONG64_ST_D(17, 1, 0x88),
		LOONG64_ST_D(18, 1, 0x90),
		LOONG64_ST_D(19, 1, 0x98),
		LOONG64_ST_D(20, 1, 0xa0),
		LOONG64_ST_D(21, 1, 0xa8),
		LOONG64_ST_D(22, 1, 0xb0),
		LOONG64_ST_D(23, 1, 0xb8),
		LOONG64_ST_D(24, 1, 0xc0),
		LOONG64_ST_D(25, 1, 0xc8),
		LOONG64_ST_D(26, 1, 0xd0),
		LOONG64_ST_D(27, 1, 0xd8),
		LOONG64_ST_D(28, 1, 0xe0),
		LOONG64_ST_D(29, 1, 0xe8),
		LOONG64_ST_D(30, 1, 0xf0),
		LOONG64_ST_D(31, 1, 0xf8),
		/* Save $pc (CSR.DERA, 0x501) */
		LOONG64_CSRRD(2, 0x501),
		LOONG64_ST_D(2, 1, 0x100),
		/* Save CSR.BADV (0x7) */
		LOONG64_CSRRD(2, 0x7),
		LOONG64_ST_D(2, 1, 0x108),
		/* Restore $r2 from CSR.DSAVE and save it */
		LOONG64_CSRRD(2, 0x502),
		LOONG64_ST_D(2, 1, 0x10),
		/* Restore $r1 from param_out[1] */
		LOONG64_LD_D(1, 1, 0x08),
		/* b start */
		LOONG64_B(NEG26(43)),
	};

	LOG_DEBUG("enter loong64_pracc_exec");
	return loongarch64_pracc_exec(ejtag_info, ARRAY_SIZE(code), code, 0,
		NULL, LOONG64_NUM_REGS, regs);
}
