// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2026 by RigoLigo <rigoligo03@gmail.com>
 */

/**
 * @file
 * This file implements support for Loongson EJTAG debug probes released by
 * Loongson Technology. Exposes operations supported by this kind of probes.
 */

struct lsejtag_interface {
	int (*ir_scan)(void);
};
