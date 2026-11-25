// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2026 by RigoLigo <rigoligo03@gmail.com>
 */

/**
 * @file
 * This file implements the Loongson LJTAG adapter driver.
 * 
 * The knowledge mostly comes from reverse engineering.
 * 
 * Loongson LJTAG adapter comes in multiple implementations, both those from
 * Loongson and reverse-engineered reimplementations.
 * 
 * They all share a common VID-PID pair: 2961:6688 and has same endpoint layout
 * with EP2 as OUT and EP6 as IN, both in Bulk mode. It uses a command stream to
 * implement high level JTAG operations, such as reading/writing IR and DRs,
 * fast data transfer adapted to LJTAG-style debugging personality, etc.
 */


#include "helper/log.h"
#include "jtag/jtag.h"
#include <libusb.h>
#include <string.h>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/system.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>

#include "libusb_helper.h"

#define LOG_PREFIX "loongson_ljtag: "
#define LSLJTAG_IN_EP (0x86)
#define LSLJTAG_OUT_EP (0x02)

#ifndef MAX
#define MAX(a, b) ((a)>(b)?(a):(b))
#endif
#ifndef MIN
#define MIN(a, b) ((a)<(b)?(a):(b))
#endif
// Calculate how many words (32-bit word) can fit x bits
#define BIT2WORD(x) (((x)+31)/32)

/**
 * @brief
 * Handle to an opened Loongson LJTAG USB device and all the buffers containing
 * data directly coming in and out of it.
 */
struct lsljtag_device {
	/** */
	struct libusb_device_handle *hdev;
	/**
	 * @brief
	 * libusb transfer object.
	 * Loongson LJTAG doesn't have the concept of outstanding requests. You
	 * simply have to write "instruction stream" to its bulk endpoint, and
	 * the probe will start actioning the moment it can read anything out of
	 * its USB FIFO. IR/DR writes are immediately executed, and IR/DR reads
	 * will be buffered in another FIFO, and may or may not be sent back
	 * depending on the commands sent by host. So, there's always only one
	 * outstanding USB request, unlike CMSIS-DAP who may have 4 at maximum.
	 * OpenOCD may buffer multiple commands, like "Write IR, then read 64
	 * bits of DR and send it back", send it in one go and attempt read back
	 * right after the command write.
	 */
	struct libusb_transfer *transfer;
	
	unsigned int rx_ep; ///< Endpoint for reading from debug probe
	unsigned int tx_ep; ///< Endpoint for writing to debug probe

	/**
	 * @brief See \@transfer, the binary command stream buffered in OpenOCD.
	 */
	uint8_t *cmd_stream_buf;
};

static struct lsljtag_device priv; ///< Global Loonson LJTAG device object

static uint16_t lsetjag_vid; ///< USB vendor ID, may be user specified
static uint16_t lsetjag_pid; ///< USB product ID, may be user specified

/**
 * @brief Shared USB buffer for both transmission/reception.
 * 	  This works because for Loongson LJTAG, one command either sends back
 * 	  nothing, or spits out all queued data. The caller is responsible for
 * 	  maintaining the consistency between "how many bytes queued in probe vs
 * 	  how many bytes we must read to clear the queue".
 */
static uint8_t *usb_buf;
static size_t usb_buf_size;

static size_t bytes_tx; ///< Bytes of data awaiting for transmission
static size_t bytes_rx; ///< Bytes of data awaiting for reception
static size_t mps_tx; ///< Maximum packet size for OUT endpoint
static size_t mps_rx; ///< Maximum packet size for IN endpoint

/**
 * @defgroup Loongson LJTAG internal communication
 * @{
 */

// Opcode

#define OP_PROBE_MEM_RW 		0x01
#define OP_IO_MANIP			0x03
#define OP_IR_SCAN			0x04
#define OP_DR_SCAN			0x05
#define OP_LOOPTEST			0x08
#define OP_FASTWRITE			0x0C
#define OP_FASTWRITE_FASTDATA		0x0D
#define OP_FASTREAD			0x0E
#define OP_FASTREAD_FASTDATA		0x0F
#define OP_READ_VER			0x1F

// Probe memory RW - addresses

#define MEM_REG_JTAGCLK			0x81000070
#define MEM_PTR_IRLEN			0x40
#define MEM_PTR_IR_SKIP			0x44
#define MEM_PTR_IR_CONTROL		0x48
#define MEM_PTR_IR_DATA			0x4c
#define MEM_PTR_IR_ADDRESS		0x50
#define MEM_PTR_IR_FASTDATA		0x54

// IO port IDs

#define IO_LED				0x00
#define IO_OE				0x02
#define IO_TRST				0x03
#define IO_BRST				0x04
#define IO_DINT				0x05

/**
 * @brief
 * Loongson LJTAG command is mostly composed of these parts:
 * [Command word U16] [Optional settings] [Payload]
 * This structure describes command word structure. The highest 6 bits stores an
 * opcode, distinguishing the command type, and the lower 10 bits stores some
 * configurations. There may be more configuration data following it in some
 * commands. And finally, the payload to be scanned follows. The probe will
 * decide if TDO should be queued inside an internal buffer, and whether the
 * content of that buffer should be transferred back, based on the command word.
 */
union lsljtag_cmd_word {
	uint16_t word;
	struct {
		uint16_t cfg : 10;
		uint16_t op : 6;
	} split;
	union {
		struct {
			uint16_t is_write : 1;
			uint16_t reserved : 9;
			uint16_t op : 6;
		} probe_mem_rw; // Probe memory RW (Opcode = 0x01)
		struct {
			uint16_t level : 1;
			uint16_t pin_id : 7;
			uint16_t reserved : 2;
			uint16_t op : 6;
		} io_manip; // Manipulate probe IO pins (Opcode = 0x03)
		struct {
			uint16_t reserved : 8;
			uint16_t return_queued_data : 1;
			uint16_t queue_tdo : 1;
			uint16_t op : 6;
		} scan; // Do an IR/DR scan (Opcode = 0x04, DR scan = 0x05)
		struct {
			uint16_t reserved : 10;
			uint16_t op : 6;
		} looptest; // Send a word to probe and verify returned word (Opcode = 0x08)
		struct {
			uint16_t core_count : 7;
			uint16_t is_64bit : 1;
			uint16_t reserved : 2;
			uint16_t op : 6;
		} fastwrite; // Autonomous bulk write (Opcode = 0x0C, if enables FASTDATA use, 0x0D)
		struct {
			uint16_t core_count : 7;
			uint16_t is_64bit : 1;
			uint16_t reserved : 2;
			uint16_t op : 6;
		} fastread; // Autonomous bulk read (Opcode = 0x0E, if enables FASTDATA use, 0x0F)
		struct {
			uint16_t reserved : 10;
			uint16_t op : 6;
		} read_ver; // Read probe firmware version (Opcode = 0x1F)
	};
};

/**
 * @brief Ensure USB buffer is at least @\new_size bytes long, if not, double
 * 	  the size of USB buffer until fits
 */
static int lsljtag_impl_ensure_buf_size(size_t target_size)
{
	if (target_size > usb_buf_size) {
		// Resize
		size_t new_size = usb_buf_size;
		while (new_size < target_size) {
			new_size *= 2;
		}

		uint8_t *new_buf = realloc(usb_buf, new_size);
		if (new_buf == NULL) {
			LOG_ERROR(LOG_PREFIX "failed to resize USB buffer from %" PRIuMAX " to %" PRIuMAX,
				(uintmax_t)usb_buf_size, (uintmax_t)new_size);
			return ERROR_FAIL;
		}
		usb_buf = new_buf;
		usb_buf_size = new_size;
	}

	return ERROR_OK;
}

/**
 * @brief Helper to queue bytes into the USB buffer (for sending only).
 * 	  Resizes automatically. Intended to be called by lsljtag_cmd_*.
 */
static int lsljtag_impl_queue_tx(void *bytes, size_t length)
{
	int rc;

	rc = lsljtag_impl_ensure_buf_size(bytes_tx + length);
	if (rc != ERROR_OK) {
		return rc;
	}

	memcpy(usb_buf + bytes_tx, bytes, length);
	bytes_tx += length;

	return ERROR_OK;
}

/**
 * @brief Only called by lsljtag_cmd_* functions after they fill up the transfer
 * 	  buffer.
 * 	  Sends data awaiting transmission in the USB buffer and receives
 * 	  expected bytes of data from probe. \@bytes_rx is cleared after a
 * 	  successful reception.
 * @param do_recv Whether reception is done. May be useful for scan commands
 * 	  when they wanted to queue many scans and read accumulated TDO at once.
 */
static int lsljtag_impl_send_recv(bool do_recv)
{
	assert(bytes_tx != 0);

	bool send_zlp = (bytes_tx % mps_tx) == 0;
	int transferred;
	int offset = 0;

	// Transmission
	while (bytes_tx) {
		if (jtag_libusb_bulk_write(priv.hdev, LSLJTAG_OUT_EP, 
			((char *)usb_buf) + offset, MIN(mps_tx, bytes_tx), 1000,
			&transferred) != ERROR_OK) {
			LOG_ERROR(LOG_PREFIX "USB bulk write failed");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		bytes_tx -= transferred;
		offset += transferred;
	}

	if (send_zlp) {
		if (jtag_libusb_bulk_write(priv.hdev, LSLJTAG_OUT_EP,
			(char *)usb_buf, 0, 1000, &transferred) != ERROR_OK) {
			LOG_ERROR(LOG_PREFIX "USB bulk write failed");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	// Reception
	if (do_recv && bytes_rx)
	{
		offset = 0;
		lsljtag_impl_ensure_buf_size(bytes_rx);
		while (bytes_rx) {
			if (jtag_libusb_bulk_read(priv.hdev, LSLJTAG_IN_EP, 
				((char *)usb_buf) + offset, MIN(mps_rx, bytes_rx), 1000,
				&transferred) != ERROR_OK) {
				LOG_ERROR(LOG_PREFIX "USB bulk read failed");
				return ERROR_JTAG_DEVICE_ERROR;
			}

			bytes_rx -= transferred;
			offset += transferred;
		}
	}

	return ERROR_OK;
}

/**
 * @brief Loongson EJTAG internally operates at 15MHz by default. You can only divide the frequency 
 * down by factor of powers of 2. This function is used to search an appropriate division factor and
 * the resulting JTAG clock speed after divison for target functions.
 * 
 * @param target_khz Target JTAG clock speed to fit onto
 * @param result_khz Resulting JTAG clock speed that the probe can actually support. May be NULL.
 * @param result_div The division factor to write into the device. May be NULL.
 */
static void lsljtag_util_calc_jtag_freq(int target_khz, int *result_khz, int *result_div)
{
	int freq[] = { 150000, 75000, 3750, 1875, 938, 469, 235, 118, 58, 30, 15, 8, 4, 2, 1 };
	int ret_khz = freq[ARRAY_SIZE(freq) - 1], ret_div = (1 << (ARRAY_SIZE(freq) - 1));

	if (target_khz > freq[0]) {
		ret_khz = freq[0];
		ret_div = 1;
	} else {
		for (size_t i = 1; i < ARRAY_SIZE(freq); ++i) {
			if (freq[i] <= target_khz && freq[i - 1] > target_khz) {
				ret_khz = freq[i];
				ret_div = 1 << i;
			}
		}
	}

	LOG_DEBUG(LOG_PREFIX "JTAG Freq target_khz=%" PRId32 " result_khz=%" PRId32 " result_div=%" PRId32,
		  target_khz, ret_khz, ret_div);

	if (result_div != NULL) {
		*result_div = ret_div;
	}
	if (result_khz != NULL) {
		*result_khz = ret_khz;
	}
}

/**
 * @brief Read probe firmware version.
 * @param out On success, probe firmware version will be written to the location
 * 	      pointed by the pointer.
 * @return Error code.
 */
int lsljtag_cmd_read_ver(uint32_t *out)
{
	assert(bytes_rx == 0 && bytes_tx == 0);

	union lsljtag_cmd_word cmd = {
		.read_ver.op = OP_READ_VER
	};
	int rc;

	lsljtag_impl_queue_tx(&cmd, sizeof(cmd));
	bytes_rx = 4;
	rc = lsljtag_impl_send_recv(true);
	if (rc != ERROR_OK) {
		return rc;
	}

	memcpy(out, usb_buf, sizeof(*out));
	return ERROR_OK;
}

/**
 * @brief Manipulate certain IO port voltage level on probe.
 * @param pin_id ID of IO port. Use IO_XXX macros.
 * @param level Voltage level. true for high, false for low.
 */
int lsljtag_cmd_io_manip(int pin_id, bool level)
{
	assert(bytes_rx == 0 && bytes_tx == 0);

	union lsljtag_cmd_word cmd = {
		.io_manip = {
			.level = level,
			.pin_id = pin_id,
			.op = OP_IO_MANIP,
		}
	};

	lsljtag_impl_queue_tx(&cmd, sizeof(cmd));
	return lsljtag_impl_send_recv(false);
}

/**
 * @brief Issue an IR/DR scan to the probe. Probe will take care of TMS sequence
 * and you only need to provide IR/DR sequences.
 * Loongson LJTAG probes are capable of buffering or discarding the captured TDO
 * data. If you set \@buffer_tdo and \@return_buffer both to true, captured TDO
 * will be returned to PC immediately, padded to 32-bit word boundary. If you
 * only set \@buffer_tdo to true, TDO sequence will be captured but stored in
 * a FIFO on probe. If you set \@return_buffer to true, all content in that FIFO
 * will be sent back to PC after current scan is over.
 * 
 * @param is_ir true for an IR scan, false for a DR scan
 * @param buffer_tdo whether TDO data should be captured by probe
 * @param return_buffer whether TDO data buffer should be sent back
 * @param nbits bit length of the IR/DR scan
 * @param scan_in_data IR/DR scan data. Must be padded to 32-bit word boundary
 * @param scan_out_data IR/DR scan output. Must be padded to 32-bit word boundary
 * @return int Error code
 */
int lsljtag_cmd_ir_dr_scan(bool is_ir, bool buffer_tdo, bool return_buffer,
	uint16_t nbits, uint32_t *scan_in_data, uint32_t *scan_out_data)
{
	assert(!return_buffer || (return_buffer && scan_out_data != NULL));

	union lsljtag_cmd_word cmd = {
		.scan = {
			.return_queued_data = return_buffer,
			.queue_tdo = buffer_tdo,
			.op = (is_ir ? OP_IR_SCAN : OP_DR_SCAN)
		}
	};

	// How many bytes of TDO data can be queued on probe after this scan
	// The length is same as bytes of TDI data to be sent to device
	const int bytes_queued = BIT2WORD(nbits) * 4;
	if (buffer_tdo) {
		bytes_rx += bytes_queued;
	}

	lsljtag_impl_queue_tx(&cmd, sizeof(cmd));
	lsljtag_impl_queue_tx(&nbits, sizeof(nbits));
	lsljtag_impl_queue_tx(scan_in_data, bytes_queued);

	const size_t out_bytes = return_buffer ? bytes_rx : 0;
	int err = lsljtag_impl_send_recv(return_buffer);
	if (err != ERROR_OK) {
		return err;
	}

	if (return_buffer) {
		memcpy(scan_out_data, usb_buf, out_bytes);
	}

	return ERROR_OK;
}

/**
 * @brief The original implementation of Loongson's debug probe is an FPGA running a MIPS soft core.
 * Some configuration are variables in the processor, some are registers accessible via MMIO registers
 * of the processor core. This memory RW primitive is mostly used to do configuration changes.
 * Modern implementation is no longer this case but the protocol persisted.
 * 
 * @param is_write true for a write access, false for a read.
 * @param addr 32 bit address to write to.
 * @param data pointer to data that will be read (in a write) or rewritten (in a read).
 * @return int Error code.
 */
int lsljtag_cmd_probe_mem_rw(bool is_write, uint32_t addr, uint32_t *data)
{
	union lsljtag_cmd_word cmd = {
		.probe_mem_rw = {
			.is_write = is_write,
			.op = OP_PROBE_MEM_RW
		}
	};

	lsljtag_impl_queue_tx(&cmd, sizeof(cmd));
	lsljtag_impl_queue_tx(&addr, sizeof(addr));
	if (is_write) {
		lsljtag_impl_queue_tx(data, sizeof(*data));
	}

	int err = lsljtag_impl_send_recv(!is_write);
	if (err != ERROR_OK) {
		return err;
	}

	if (!is_write) {
		memcpy(data, usb_buf, sizeof(*data));
	}

	return ERROR_OK;
}

/**
 * @} // Loongson LJTAG internal communication
 */

/**
 * @defgroup Adapter driver functions
 * @{
 */

/**
 * @brief
 * Attempt to open a Loongson LJTAG probe connected via USB.
 */
static int lsljtag_init(void)
{
	int err;

	// struct libusb_device_descriptor dev_desc;
	LOG_DEBUG("lsljtag_init()");

	memset(&priv, 0, sizeof(struct lsljtag_device));

	// FIXME: remove hard coded vid pid
	const uint16_t vids[] = { 0x2961, lsetjag_vid, 0 };
	const uint16_t pids[] = { 0x6688, lsetjag_pid, 0 };

	err = jtag_libusb_open(vids, pids, NULL, &priv.hdev, NULL);
	if (err != ERROR_OK) {
		LOG_ERROR(LOG_PREFIX "could not find or open device!");
		goto out;
	}

	err = jtag_libusb_set_configuration(priv.hdev, 0);
	if (err != ERROR_OK) {
		LOG_ERROR(LOG_PREFIX "could not select USB configuration descriptor!");
		goto out;
	}

	/* 
	 * Heuristic to find an interface:
	 * Sole 2 endpoints, with endpoint 2 for OUT and endpoint 6 for IN
	 * This is what was used historically when the probe was implemented
	 * with Cypress FX2LP, Loongson's debugger software hard coded them,
	 * and this endpoint layout is then fixed forever. Reject if the vendor
	 * interface doesn't match
	 */
	err = jtag_libusb_choose_interface(priv.hdev, &priv.rx_ep, &priv.tx_ep,
		LIBUSB_CLASS_VENDOR_SPEC, 0, -1, LIBUSB_TRANSFER_TYPE_BULK);
	if (err != ERROR_OK) {
		LOG_ERROR(LOG_PREFIX "could not find a vendor-defined interface!");
		goto out;
	} else if (priv.rx_ep != LSLJTAG_IN_EP || priv.tx_ep != LSLJTAG_OUT_EP) {
		LOG_ERROR(LOG_PREFIX "vendor-defined interface doesn't have required endpoints! "
			"IN=0x%02X, OUT=0x%02X", priv.rx_ep, priv.tx_ep);
		err = ERROR_FAIL;
		goto out;
	}

	/**
	 * Loongson LJTAG didn't have a configuration process so not really much
	 * to do beyond this point, just some variable initialization
	 */
	bytes_rx = 0;
	bytes_tx = 0;

	/**
	 * The size is completely arbitrary. The protocol doesn't strictly limit
	 * the packet size for each transaction. The original official probe
	 * implemented with FPGA will simply process commands streaming, meaning
	 * that its internal buffer size only matters for how many bytes of TDO
	 * data can be queued before sent to host in a burst, but typically one
	 * wouldn't intentionally queue that many bits of TDO without sending
	 * back to host.
	 * Newer official compatible implementations include one implemented
	 * with LS2K0300 SoC, which has not been documented or reverse
	 * engineered.
	 */
	usb_buf_size = 512;
	usb_buf = calloc(512, 1);
	if (usb_buf == NULL) {
		LOG_ERROR(LOG_PREFIX "cannot allocate USB buffer");
		goto out;
	}

	struct libusb_device *udev = libusb_get_device(priv.hdev);
	mps_tx = libusb_get_max_packet_size(udev, LSLJTAG_OUT_EP);
	mps_rx = libusb_get_max_packet_size(udev, LSLJTAG_IN_EP);
	
	uint32_t ver;
	if (lsljtag_cmd_read_ver(&ver) != ERROR_OK) {
		LOG_ERROR("Cannot initiate probe communication correctly (Can't read version)");
		return ERROR_JTAG_DEVICE_ERROR;
	}
	LOG_INFO(LOG_PREFIX "Version = %08X", ver);

	return ERROR_OK;

out:
	if (priv.hdev != NULL) {
		jtag_libusb_close(priv.hdev);
		priv.hdev = NULL;
	}
	return ERROR_FAIL;
}

/**
 * @brief
 * Close an already opened Loongson LJTAG device. No-op if none is opened.
 */
static int lsljtag_quit(void)
{
	if (priv.hdev != NULL) {
		jtag_libusb_close(priv.hdev);
		priv.hdev = NULL;
	}
	if (usb_buf != NULL) {
		free(usb_buf);
		usb_buf = NULL;
	}
	return ERROR_OK;
}

/**
 * @brief Set SRST(BRST in LJTAG's pinout) and TRST pin levels.
 */
static int lsljtag_reset(int srst, int trst)
{
	lsljtag_cmd_io_manip(IO_BRST, !!srst);
	lsljtag_cmd_io_manip(IO_TRST, !!trst);
	return ERROR_OK;
}

/**
 * @brief Set probe clock speed. 
 * 
 * @param speed Clock divisor
 * @return int error code
 */
static int lsljtag_speed(int speed)
{
	uint32_t regw = 0x00010000 | speed;

	return lsljtag_cmd_probe_mem_rw(true, MEM_REG_JTAGCLK, &regw);
}

/**
 * @brief Convert the clock divisor to the actual JTAG clock speed
 * 
 * @param speed Desired target speed 
 * @param khz Output of actual speed
 * @return int error code
 */
static int lsljtag_speed_div(int speed, int *khz)
{
	*khz = 15000 / speed;
	return ERROR_OK;
}

/**
 * @brief Convert the desired kHz JTAG clock speed to the clock divisor that the probe supports.
 * The actual speed will be (15MHz/2^n) where n makes the resulting clock speed not greater than
 * the desired speed.
 * 
 * @param khz Input, desired kHz JTAG clock speed
 * @param jtag_speed Output, clock divisor
 * @return int 
 */
static int lsljtag_khz(int khz, int *jtag_speed)
{
	lsljtag_util_calc_jtag_freq(khz, NULL, jtag_speed);
	return ERROR_OK;
}

/**
 * @} // Adapter driver functions
 */

/**
 * @defgroup JTAG interface functions
 * @{
 */

/**
 * @brief Utility function to check if this scan command requires TDO readback.
 * Disabling TDO readback may reduce the load on the probe slightly.
 * 
 * @param scan Scan command
 * @return whether this scan command requires TDO readback.
 */
static bool lsljtag_check_scan_requires_readback(struct scan_command *scan)
{
	for (unsigned int i = 0; i < scan->num_fields; ++i) {
		if (scan->fields[i].in_value != NULL) {
			return true;
		}
	}
	return false;
}

int lsljtag_iface_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue; /* currently processed command */
	int retval = ERROR_OK;
	int scan_length = 0;
	bool readback;
	uint32_t *buffer = NULL;

	// Loongson LJTAG adapter do not support bit-banged JTAG. You can only
	// use it to generate IR/DR scans.
	while (cmd) {
		switch (cmd->type) {
		case JTAG_SCAN:
			LOG_DEBUG_IO(LOG_PREFIX "JTAG_SCAN");
			scan_length = jtag_build_buffer(cmd->cmd.scan, (uint8_t **)&buffer);
			if (scan_length % 32) {
				// Pad buffer to 32-bit boundary
				uint32_t *buf_new = realloc(buffer, BIT2WORD(scan_length) * 4);
				if (buf_new == NULL) {
					LOG_ERROR(LOG_PREFIX "failed to pad JTAG buffer to word boundary");
					free(buffer);
					break;
				}
				buffer = buf_new;
			}
			readback = lsljtag_check_scan_requires_readback(cmd->cmd.scan);
			retval = lsljtag_cmd_ir_dr_scan(cmd->cmd.scan->ir_scan, readback, readback,
				scan_length, buffer, buffer);
			if (retval != ERROR_OK) {
				LOG_ERROR(LOG_PREFIX "failed executing JTAG_SCAN (%" PRId32 ")", retval);
			}
			if (readback &&
				jtag_read_buffer((uint8_t *)buffer, cmd->cmd.scan) != ERROR_OK) {
				retval = ERROR_JTAG_QUEUE_FAILED;
			}
			free(buffer);
			break;
		case JTAG_TLR_RESET:
			LOG_ERROR(LOG_PREFIX "JTAG_TLR_RESET unsupported: No bit-banged JTAG support");
			break;
		case JTAG_RUNTEST:
			LOG_ERROR(LOG_PREFIX "JTAG_RUNTEST unsupported: No bit-banged JTAG support");
			break;
		case JTAG_RESET:
			LOG_DEBUG_IO(LOG_PREFIX "JTAG_RESET TRST=%" PRIu32 " SRST=%" PRIu32,
				cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			lsljtag_cmd_io_manip(IO_TRST, !cmd->cmd.reset->trst);
			lsljtag_cmd_io_manip(IO_BRST, !cmd->cmd.reset->srst);
			break;
		case JTAG_PATHMOVE:
			LOG_ERROR(LOG_PREFIX "JTAG_PATHMOVE unsupported: No bit-banged JTAG support");
			break;
		case JTAG_SLEEP:
			LOG_DEBUG_IO(LOG_PREFIX "sleep %" PRIu32 "us", cmd->cmd.sleep->us);
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_STABLECLOCKS:
			LOG_ERROR(LOG_PREFIX "JTAG_STABLECLOCKS unsupported: No bit-banged JTAG support");
		case JTAG_TMS:
			LOG_ERROR(LOG_PREFIX "JTAG_TMS unsupported: No bit-banged JTAG support");
			break;
		}

		cmd = cmd->next;
	}

	return retval;
}

/**
 * @} // JTAG interface functions
 */

/**
 * @defgroup Loongson LJTAG related commands
 * @{
 */

COMMAND_HANDLER(lsljtag_cmdhandler_version)
{
	uint32_t ver;
	int rc = lsljtag_cmd_read_ver(&ver);
	if (rc != ERROR_OK) {
		ver = 0;
		LOG_ERROR(LOG_PREFIX "cannot read probe firmware version");
	}
	command_print(CMD, "%" PRIx32, ver);
	return ERROR_OK;
}

static const struct command_registration loongson_ljtag_subcommand_handlers[] = {
	{
		.name = "version",
		.handler = lsljtag_cmdhandler_version,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "Prints the probe firmware version",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration loongson_ljtag_command_handlers[] = {
	{
		.name = "loongson-ljtag",
		.mode = COMMAND_ANY,
		.help = "Loongson LJTAG debug probe actions",
		.usage = "<cmd>",
		.chain = loongson_ljtag_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * @} // Loongson LJTAG related commands
 */

static struct jtag_interface loongson_ljtag_interface = {
	.supported = 0,
	.execute_queue = lsljtag_iface_execute_queue,
};

struct adapter_driver loongson_ljtag_adapter_driver = {
	.name = "loongson-ljtag",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = loongson_ljtag_command_handlers,

	.init = lsljtag_init,
	.quit = lsljtag_quit,
	.reset = lsljtag_reset,
	.speed = lsljtag_speed,
	.khz = lsljtag_khz,
	.speed_div = lsljtag_speed_div,
	.config_trace = NULL,
	.poll_trace = NULL,

	.jtag_ops = &loongson_ljtag_interface,
};
