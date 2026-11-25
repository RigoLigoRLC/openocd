// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Support for LoongArch64 processors with LJTAG debugging capability:
 * all the low-level support routines that require executing a miniprogram
 * through the memory accesses processed by debug probe.
 *
 *   Copyright (C) 2025 by RigoLigo <rigoligo03@gmail.com>
 * 
 * Initial LoongArch support is based on OpenOCD MIPS implementation,
 * and LJTAG implementation from OpenLA132 open source CPU core
 * (https://github.com/loongson/OpenLACores/).
 * 
 * In LJTAG debugging workflow, a Debug Exception is triggered on CPU by
 * setting a special JTAG register bit. After which the processor will save the
 * exception $pc to a CSR and jump to a 1MB "debug memory" (dbgm) and execute
 * from there. But that piece of memory is neither backed by physical memory nor
 * translated through MMU: it's backed by the debug probe. Whenever processor
 * tries to do memory access inside dbgm, the core will stall because it has
 * wait until the debug probe finishes the memory access. If it's a read, the
 * probe will have to write to a JTAG register to supply the data that CPU will
 * see, and clear a bit (PR, the Pending Request bit) to show that the access is
 * done. If it's a write, the probe will have to read that JTAG register to see
 * what the CPU is trying to write, and save the value so that debugger software
 * can use it, and clear PR bit to tell the CPU the access is done. Basically
 * this file contains all the logic for the emulated dbgm region: what to do if
 * CPU reads/writes some address inside dbgm, etc. Some basic operations like
 * memory access and register dumping is also included.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/types.h"
#include "loongarch64.h"
#include "loongarch64_dbgm.h"

#include <jtag/adapter.h>
#include "helper/log.h"
#include "target/loongarch/loongarch_ljtag.h"
#include <stdbool.h>

#define STACK_DEPTH	32

/**
 * @brief Context block used by OpenOCD when trying to execute debug miniprograms on target.
 * 
 */
struct loongarch64_dbgm_context {
	uint64_t *local_iparam;
	unsigned int num_iparam;
	uint64_t *local_oparam;
	unsigned int num_oparam;
	const uint32_t *code;
	unsigned int code_len;
	uint64_t stack[STACK_DEPTH];
	unsigned int stack_offset;
	struct loongarch_ljtag *ljtag_info;
};

static int wait_for_dbgm_rw(struct loongarch_ljtag *ljtag_info, uint32_t *ctrl)
{
	// Set PR bit when reading Control register, avoid accidentally "completing" a pending access
	uint32_t ljtag_ctrl;
	int retry_count = 5;
	int retval;

	while (1) {
		loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_CONTROL);
		ljtag_ctrl = LJTAG_CTRL_PR | LJTAG_CTRL_PEN |
			     LJTAG_CTRL_PROBTRAP;
		retval = loongarch_ljtag_drscan_32(ljtag_info, &ljtag_ctrl, true);
		if (retval != ERROR_OK) {
			return retval;
		}

		if (ljtag_ctrl & LJTAG_CTRL_PR) {
			break;
		}
		LOG_DEBUG("DEBUGMODULE: No memory access in progress! (Control=%08" PRIx32 ")", ljtag_ctrl);
		if (retry_count == 0) {
			return ERROR_JTAG_DEVICE_ERROR;
		}
		retry_count--;
	}

	*ctrl = ljtag_ctrl;
	return ERROR_OK;
}

/**
 * @brief A unified function to "handle CPU accesses", or, "read/write DATA and clear PR bit".
 * This function may choose to use FASTDATA or DATA+CONTROL transactions depending on an option
 * user can select with "loongarch-ljtag use-fastdata" command.
 * 
 * @param ljtag_info LoongArch LJTAG information block.
 * @param data The data CPU will see / CPU wants to write to probe side when this access finishes
 * @param is_read true: finish a memory read access from CPU (write *data to DATA and clear PR)
 * 		  false: finish a memory write access from CPU (read DATA to *data and clear PR)
 * @return int error code.
 */
static int loongarch64_dbgm_handle_access(struct loongarch_ljtag *ljtag_info,
					   uint64_t *data,
					   bool is_read) {
	const bool use_fastdata = loongarch_ljtag_get_use_fastdata();
	uint32_t ljtag_ctrl;
	int retval;

	if (is_read) {
		if (use_fastdata) {
			/* Send the data via FASTDATA */
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_FASTDATA);
			retval = loongarch_ljtag_fastdata_scan_64(ljtag_info, data, false, false);
		} else {
			/* Send the data out */
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_DATA);
			retval = loongarch_ljtag_drscan_64(ljtag_info, data, false);
			if (retval != ERROR_OK) {
				return retval;
			}

			/* Finish the memory access */
			ljtag_ctrl = LJTAG_CTRL_PEN | LJTAG_CTRL_PROBTRAP;
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_CONTROL);
			retval = loongarch_ljtag_drscan_32(ljtag_info, &ljtag_ctrl, false);
		}
	} else {
		if (use_fastdata) {
			/* Receive the data via FASTDATA */
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_FASTDATA);
			retval = loongarch_ljtag_fastdata_scan_64(ljtag_info, data, false, true);
		} else {
			/* Receive the data from DATA */
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_DATA);
			retval = loongarch_ljtag_drscan_64(ljtag_info, data, true);
			if (retval != ERROR_OK) {
				return retval;
			}

			/* Finish the memory access */
			ljtag_ctrl = LJTAG_CTRL_PEN | LJTAG_CTRL_PROBTRAP;
			loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_CONTROL);
			retval = loongarch_ljtag_drscan_32(ljtag_info, &ljtag_ctrl, false);
		}
	}

	return retval;
}

/**
 * @brief Handles a read access request from CPU.
 * 
 * @param ctx context block
 * @param address address that the CPU is trying to read from
 * @return int error code
 */
static int loongarch64_dbgm_exec_read(struct loongarch64_dbgm_context *ctx, uint64_t address) {
	struct loongarch_ljtag *ljtag_info = ctx->ljtag_info;
	unsigned int offset;
	uint64_t data;
	int retval;

	if ((address >= LOONG64_DBGM_PARAM_IN) &&
	    (address < LOONG64_DBGM_PARAM_IN + ctx->num_iparam * LOONG64_DBGM_DATA_STEP)) {
		
		offset = (address - LOONG64_DBGM_PARAM_IN) / LOONG64_DBGM_DATA_STEP;

		if (offset >= LOONG64_DBGM_PARAM_IN_SIZE) {
			LOG_ERROR("iparam size exceeds LOONG64_DBGM_PARAM_IN_SIZE");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		if (!ctx->local_iparam) {
			LOG_ERROR("unexpected reading of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_iparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_DBGM_PARAM_OUT) &&
		   (address < LOONG64_DBGM_PARAM_OUT + ctx->num_oparam * LOONG64_DBGM_DATA_STEP)) {
		
		offset = (address - LOONG64_DBGM_PARAM_OUT) / LOONG64_DBGM_DATA_STEP;
		if (!ctx->local_oparam) {
			LOG_ERROR("unexpected reading of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		data = ctx->local_oparam[offset];
		LOG_DEBUG("Reading %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_DBGM_TEXT) &&
		   (address < LOONG64_DBGM_TEXT + ctx->code_len * LOONG64_DBGM_INSN_STEP)) {
		
		offset = ((address & ~7ull) - LOONG64_DBGM_TEXT) / LOONG64_DBGM_INSN_STEP;
		data = (uint64_t)ctx->code[offset];
		if (offset + 1 < ctx->code_len)
			data |= (uint64_t)ctx->code[offset + 1] << 32;
		LOG_DEBUG("Executing instructions %08" PRIx64 " at %08" PRIx64, data, address);
	} else if ((address & ~7ull) == LOONG64_DBGM_STACK) {

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
	retval = loongarch64_dbgm_handle_access(ljtag_info, &data, true);

	if (retval != ERROR_OK) {
		return retval;
	}

	return jtag_execute_queue();
}


/**
 * @brief Handles a write access request from CPU.
 * 
 * @param ctx context block
 * @param address address that the CPU is trying to read from
 * @return int error code
 */
static int loongarch64_dbgm_exec_write(struct loongarch64_dbgm_context *ctx, uint64_t address) {
	struct loongarch_ljtag *ljtag_info = ctx->ljtag_info;
	unsigned int offset;
	uint32_t ljtag_ctrl;
	uint64_t data;
	int retval;

	/* Get the data written by CPU */
	loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_DATA);
	retval = loongarch_ljtag_drscan_64(ljtag_info, &data, true);
	if (retval != ERROR_OK) {
		return retval;
	}

	/* Finish the memory access */
	ljtag_ctrl = ljtag_info->ljtag_ctrl & ~LJTAG_CTRL_PR;
	loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_CONTROL);
	retval = loongarch_ljtag_drscan_32(ljtag_info, &ljtag_ctrl, false);
	if (retval != ERROR_OK) {
		return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		return retval;
	}

	if ((address >= LOONG64_DBGM_PARAM_IN) &&
	    (address < LOONG64_DBGM_PARAM_IN + ctx->num_iparam * LOONG64_DBGM_DATA_STEP)) {
		
		offset = (address - LOONG64_DBGM_PARAM_IN) / LOONG64_DBGM_DATA_STEP;

		if (offset >= LOONG64_DBGM_PARAM_IN_SIZE) {
			LOG_ERROR("iparam size exceeds LOONG64_DBGM_PARAM_IN_SIZE");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		if (!ctx->local_iparam) {
			LOG_ERROR("unexpected writing of input parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ctx->local_iparam[offset] = data;
		LOG_DEBUG("Writing %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address >= LOONG64_DBGM_PARAM_OUT) &&
		   (address < LOONG64_DBGM_PARAM_OUT + ctx->num_oparam * LOONG64_DBGM_DATA_STEP)) {
		
		offset = (address - LOONG64_DBGM_PARAM_OUT) / LOONG64_DBGM_DATA_STEP;
		if (!ctx->local_oparam) {
			LOG_ERROR("unexpected writing of output parameter");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ctx->local_oparam[offset] = data;
		LOG_DEBUG("Writing %" PRIx64 " at %" PRIx64, data, address);
	} else if ((address & ~7ull) == LOONG64_DBGM_STACK) {

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

int loongarch64_dbgm_exec(struct loongarch_ljtag *ljtag_info,
			   unsigned int code_len, const uint32_t *code,
			   unsigned int num_param_in, uint64_t *param_in,
			   unsigned int num_param_out, uint64_t *param_out)
{
	uint32_t ljtag_ctrl;
	uint64_t address = 0;
	struct loongarch64_dbgm_context ctx;
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
	ctx.ljtag_info = ljtag_info;
	ctx.stack_offset = 0;

	while (true) {
		retval = wait_for_dbgm_rw(ljtag_info, &ljtag_ctrl);
		if (retval != ERROR_OK) {
			/*
			* The reason why this specific quirk exists is that, a silicon bug exists in
			* LS2K0300 causes the machine to read one more instruction from dbgm after it
			* has executed ERET instruction and returned from debug mode, kind of like a
			* MIPS delay slot but that extra instruction is never executed. Because of this
			* uncertainty, we detect if we're trying to exit debug mode when . If there's a
			* newer chip that has fixed this bug, OpenOCD may safely interrupt the
			* execution of ERET instruction flow.
			*/
			if (address >= LOONG64_DBGM_TEXT &&
			    address < LOONG64_DBGM_TEXT + code_len * LOONG64_DBGM_INSN_STEP &&
			    code[((address & ~7ull) - LOONG64_DBGM_TEXT) / LOONG64_DBGM_INSN_STEP]
				== LOONG64_ERTN) {
				
				LOG_DEBUG("Exited from debug mode");
				return ERROR_OK;
			}
			LOG_ERROR("ERROR wait_for_dbgm_rw %d", retval);
			return retval;
		}

		loongarch_ljtag_add_write_ir(ljtag_info, LJTAG_INST_ADDRESS);
		loongarch_ljtag_drscan_64(ljtag_info, &address, true);
		LOG_DEBUG("-> %08" PRIx64, address);

		/* Since the memory logic is taken from MIPS code, we also inherit its access
		 * characteristics: because code is stored as 32-bit words and data (parameters) are
		 * stored as 64-bit words, unaligned accesses, and 32-bit-aligned accesses to data
		 * area is not allowed. */
		int psz = (ljtag_ctrl >> 29) & 3;
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

		if (ljtag_ctrl & LJTAG_CTRL_PW) {
			/* Write path */
			retval = loongarch64_dbgm_exec_write(&ctx, address);
			if (retval != ERROR_OK) {
				LOG_ERROR("loongarch64_dbgm_exec_write() failed (%d)", retval);
				return retval;
			}
		} else {
			/* Read path
			 * Just like what's written in MIPS, the second time when CPU reads at dbgm
			 * means that the program has ended execution */
			if ((address == LOONG64_DBGM_TEXT) && (pass++)) {
				LOG_DEBUG("Miniprogram finished execution");
				break;
			}
			retval = loongarch64_dbgm_exec_read(&ctx, address);
			if (retval != ERROR_OK) {
				LOG_ERROR("loongarch64_dbgm_exec_read() failed (%d)", retval);
				return retval;
			}
		}
	}

	/* Stack sanity check */
	if (ctx.stack_offset != 0) {
		LOG_ERROR("dbgm stack not balanced after execution (offset = %d)", ctx.stack_offset);
	}

	return ERROR_OK;
}

static int loongarch64_dbgm_read_u64(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint64_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* ld.d $t0, $t0, 0 (reads 64 bit from that address into $t0) */
		LOONG64_LD_D(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(12)),
	};

	uint64_t param_in[1] = { addr };

	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, buf);
}

static int loongarch64_dbgm_read_u32(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint32_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* ld.w $t0, $t0, 0 (reads 32 bit from that address into $t0) */
		LOONG64_LD_W(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(12)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint32_t)param_out[0];
	return retval;
}

static int loongarch64_dbgm_read_u16(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint16_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* ld.h $t0, $t0, 0 (reads 16 bit from that address into $t0) */
		LOONG64_LD_H(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(12)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint16_t)param_out[0];
	return retval;
}

static int loongarch64_dbgm_read_u8(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint8_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* $t0 = param_in[0] (read address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* ld.b $t0, $t0, 0 (reads 16 bit from that address into $t0) */
		LOONG64_LD_B(12, 12, 0),
		/* param_out[0] = $t0 */
		LOONG64_ST_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_OUT)),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(12)),
	};

	int retval;
	uint64_t param_in[1] = { addr };
	uint64_t param_out[1];

	retval = loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 1, param_out);
	buf[0] = (uint8_t)param_out[0];
	return retval;
}

static int loongarch64_dbgm_read_mem64(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint64_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_read_u64(ljtag_info, addr + 8 * i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

static int loongarch64_dbgm_read_mem32(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint32_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_read_u32(ljtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

static int loongarch64_dbgm_read_mem16(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint16_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_read_u16(ljtag_info, addr + 2 * i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

static int loongarch64_dbgm_read_mem8(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint8_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_read_u8(ljtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK) {
			return retval;
		}
	}
	return retval;
}

int loongarch64_dbgm_read_mem(struct loongarch_ljtag *ljtag_info, uint64_t addr,
			       unsigned int size, unsigned int count, void *buf)
{
	switch (size) {
	case 1:
		return loongarch64_dbgm_read_mem8(ljtag_info, addr, count, buf);
	case 2:
		return loongarch64_dbgm_read_mem16(ljtag_info, addr, count, buf);
	case 4:
		return loongarch64_dbgm_read_mem32(ljtag_info, addr, count, buf);
	case 8:
		return loongarch64_dbgm_read_mem64(ljtag_info, addr, count, buf);
	}
	return ERROR_FAIL;
}

static int loongarch64_dbgm_write_u64(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint64_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* Save $t1 to stack */
		LOONG64_ST_D(13, 21, 0),
		/* $t0 = param_in[0] (write address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* $t1 = param_in[1] (data to be written) */
		LOONG64_LD_D(13, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN - 8)),
		/* *$t0 = $t1 */
		LOONG64_ST_D(13, 12, 0),
		/* Restore $t1 from stack */
		LOONG64_LD_D(13, 21, 0),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(13)),
	};

	uint64_t param_in[2] = { addr, *buf };

	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int loongarch64_dbgm_write_u32(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint32_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* Save $t1 to stack */
		LOONG64_ST_D(13, 21, 0),
		/* $t0 = param_in[0] (write address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* $t1 = param_in[1] (data to be written) */
		LOONG64_LD_D(13, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN - 8)),
		/* *$t0 = $t1 */
		LOONG64_ST_W(13, 12, 0),
		/* Restore $t1 from stack */
		LOONG64_LD_D(13, 21, 0),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(13)),
	};

	uint64_t param_in[2] = { addr, *buf };

	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int loongarch64_dbgm_write_u16(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint16_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* Save $t1 to stack */
		LOONG64_ST_D(13, 21, 0),
		/* $t0 = param_in[0] (write address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* $t1 = param_in[1] (data to be written) */
		LOONG64_LD_D(13, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN - 8)),
		/* *$t0 = $t1 */
		LOONG64_ST_H(13, 12, 0),
		/* Restore $t1 from stack */
		LOONG64_LD_D(13, 21, 0),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(13)),
	};

	uint64_t param_in[2] = { addr, *buf };

	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int loongarch64_dbgm_write_u8(struct loongarch_ljtag *ljtag_info, uint64_t addr, uint8_t *buf) {
	const uint32_t code[] = {
		/* Write $r21 to CSR.DSAVE */
		LOONG64_CSRWR(21, 0x502),
		/* $r21 = LOONG64_DBGM_STACK (4 instructions inside) */
		LOONG64_LI_D(21, LOONG64_DBGM_STACK),
		/* Save $t0 to stack */
		LOONG64_ST_D(12, 21, 0),
		/* Save $t1 to stack */
		LOONG64_ST_D(13, 21, 0),
		/* $t0 = param_in[0] (write address) */
		LOONG64_LD_D(12, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN)),
		/* $t1 = param_in[1] (data to be written) */
		LOONG64_LD_D(13, 21, NEG12(LOONG64_DBGM_STACK - LOONG64_DBGM_PARAM_IN - 8)),
		/* *$t0 = $t1 */
		LOONG64_ST_B(13, 12, 0),
		/* Restore $t1 from stack */
		LOONG64_LD_D(13, 21, 0),
		/* Restore $t0 from stack */
		LOONG64_LD_D(12, 21, 0),
		/* Restore $r21 from CSR.DSAVE */
		LOONG64_CSRRD(21, 0x502),
		/* b start */
		LOONG64_B(NEG26(13)),
	};

	uint64_t param_in[2] = { addr, *buf };

	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code,
		ARRAY_SIZE(param_in), param_in, 0, NULL);
}

static int loongarch64_dbgm_write_mem64(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint64_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_write_u64(ljtag_info, addr + 8 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int loongarch64_dbgm_write_mem32(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint32_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_write_u32(ljtag_info, addr + 4 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int loongarch64_dbgm_write_mem16(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint16_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_write_u16(ljtag_info, addr + 2 * i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

static int loongarch64_dbgm_write_mem8(struct loongarch_ljtag *ljtag_info, uint64_t addr,
					unsigned int count, uint8_t *buf)
{
	int retval = ERROR_OK;

	for (unsigned int i = 0; i < count; i++) {
		retval = loongarch64_dbgm_write_u8(ljtag_info, addr + i, &buf[i]);
		if (retval != ERROR_OK)
			return retval;
	}
	return retval;
}

int loongarch64_dbgm_write_mem(struct loongarch_ljtag *ljtag_info, uint64_t addr,
				unsigned int size, unsigned int count, void *buf)
{
	switch (size) {
	case 1:
		return loongarch64_dbgm_write_mem8(ljtag_info, addr, count, buf);
	case 2:
		return loongarch64_dbgm_write_mem16(ljtag_info, addr, count, buf);
	case 4:
		return loongarch64_dbgm_write_mem32(ljtag_info, addr, count, buf);
	case 8:
		return loongarch64_dbgm_write_mem64(ljtag_info, addr, count, buf);
	}
	return ERROR_FAIL;
}

int loongarch64_dbgm_read_regs(struct loongarch_ljtag *ljtag_info,
				uint64_t *regs)
{
	const uint32_t code[] = {
		/* Write $r2 to CSR.DSAVE */
		LOONG64_CSRWR(2, 0x502),
		/* First load LOONG64_DBGM_PARAM_OUT into $r2 */
		LOONG64_LI_D(2, LOONG64_DBGM_PARAM_OUT),
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
		LOONG64_B(NEG26(44)),
	};

	LOG_DEBUG("enter loong64_dbgm_exec");
	return loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code, 0,
		NULL, LOONG64_NUM_REGS, regs);
}

int loongarch64_dbgm_read_csr(struct loongarch_ljtag *ljtag_info, uint32_t csr, uint64_t *value)
{
	const uint32_t code[] = {
		/* Write $r1 to CSR.DSAVE */
		LOONG64_CSRWR(1, 0x502),
		/* Use $r1 as output area pointer */
		LOONG64_LI_D(1, LOONG64_DBGM_PARAM_OUT),
		/* Save $r2 to param_out[1] */
		LOONG64_ST_D(2, 1, 0x8),
		/* Read value of desired CSR into $r2 */
		LOONG64_CSRRD(2, csr),
		/* Save the CSR value to param_out[0] */
		LOONG64_ST_D(2, 1, 0x0),
		/* Restore $r2 from param_out[1] */
		LOONG64_LD_D(2, 1, 0x8),
		/* Restore $r1 from CSR.DSAVE */
		LOONG64_CSRRD(1, 0x502),
		/* b start */
		LOONG64_B(NEG26(10)),
	};

	int retval;
	uint64_t param_out[2];

	retval = loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code, 0,
		NULL, 2, param_out);
	*value = param_out[0];
	return retval;
}

int loongarch64_dbgm_write_csr(struct loongarch_ljtag *ljtag_info, uint32_t csr, uint64_t value)
{
	const uint32_t code[] = {
		/* Write $r1 to CSR.DSAVE */
		LOONG64_CSRWR(1, 0x502),
		/* Use $r1 as input area pointer */
		LOONG64_LI_D(1, LOONG64_DBGM_PARAM_IN),
		/* Save $r2 to param_in[1] */
		LOONG64_ST_D(2, 1, 0x8),
		/* Read the value to be written to CSR from param_in[0] */
		LOONG64_LD_D(2, 1, 0x0),
		/* Read value of desired CSR into $r2 */
		LOONG64_CSRWR(2, csr),
		/* Restore $r2 from param_in[1] */
		LOONG64_LD_D(2, 1, 0x8),
		/* Restore $r1 from CSR.DSAVE */
		LOONG64_CSRRD(1, 0x502),
		/* b start */
		LOONG64_B(NEG26(10)),
	};

	int retval;
	uint64_t param_in[2] = { value, 0 };

	retval = loongarch64_dbgm_exec(ljtag_info, ARRAY_SIZE(code), code, 2,
		param_in, 0, NULL);
	return retval;
}
