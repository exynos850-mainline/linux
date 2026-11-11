/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared memory interface of the Maxwell Bluetooth service
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 */

#ifndef __SCSC_SHM_H__
#define __SCSC_SHM_H__

#include <linux/types.h>
#include <scsc/api/bsmhcp.h>

int scsc_bt_shm_init(void);
void scsc_bt_shm_exit(void);

/*
 * Read/write a stream of H4 packets (packet type byte followed by the HCI
 * packet). Reads block until data is available, unless @nonblock is set.
 */
ssize_t scsc_bt_shm_h4_read(u8 *buf, size_t len, bool nonblock);
ssize_t scsc_bt_shm_h4_write(const u8 *buf, size_t count);

#endif
