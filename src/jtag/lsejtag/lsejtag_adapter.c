// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2026 by RigoLigo <rigoligo03@gmail.com>
 */

/**
 * @file
 * This file implements the Loongson EJTAG adapter driver.
 * 
 * Loongson EJTAG adapter comes in multiple implementations, both those from
 * Loongson and reverse-engineered reimplementations. 
 * They all share a common VID-PID pair: 2961:6688 and has same endpoint layout
 * with EP2 as OUT and EP6 as IN, both in Bulk mode. It uses a command stream to
 * implement high level JTAG operations, such as reading/writing IR and DRs,
 * fast data transfer adapted to EJTAG-style debugging personality, etc.
 * All this make Loongson EJTAG-compatible adapters a separate transport.
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>

/**
 * @brief
 * Handle to an opened Loongson EJTAG USB device and all the buffers containing
 * data directly coming in and out of it.
 */
struct lsejtag_usb_handle {
	/** */
	struct libusb_device_handle *fd;
	/**
	 * @brief
	 * libusb transfer object.
	 * Loongson EJTAG doesn't use the concept of outstanding requests. It
	 * simply ingest the command stream, and do actions accordingly. Writes
	 * will be written, reads will be buffered and host must read them out.
	 * The only way to backpressure the host is by replying NAK tokens when
	 * adapter's reception buffer is full and itself is busy executing the
	 * command stream already received. Therefore on OpenOCD side all we do
	 * is converting the commands to binary command stream and send all of
	 * it to the adapter and wait for any incoming readback data, hence the
	 * single transfer object.
	 */
	struct libusb_transfer *transfer;
	/** */
	uint8_t rx_ep;
	/** */
	uint8_t tx_ep;
	/**
	 * @brief See \@transfer, the binary command stream buffered in OpenOCD.
	 */
	uint8_t *cmd_stream_buf;
};

static int lsejtag_open(void **fd)
{
	

}

static int lsejtag_init(void)
{
	LOG_DEBUG("lsejtag_init()");

	if (!transport_is_lsejtag()) {
		LOG_ERROR("Loongson EJTAG adapter supports only \"lsejtag\" transport. "
			  "The currently selected transport is unsupported.");
		return ERROR_FAIL;
	}


}

struct adapter_driver loongson_ejtag_adapter_driver = {
	.name = "loongson-ejtag",
	.transport_ids = TRANSPORT_LSEJTAG,
	.transport_preferred_id = TRANSPORT_LSEJTAG,
	.commands = NULL,

	
};
