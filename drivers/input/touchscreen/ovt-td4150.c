// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Omnivision TD4150 touchscreen
 */

#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

/* TCM protocol definitions */

#define TCM_MSG_MARKER			0xa5
#define TCM_MSG_PADDING			0x5a
#define TCM_MSG_HEADER_SIZE		4

#define TCM_MIN_READ_LEN		9
#define TCM_MAX_READ_LEN		2048
#define TCM_WR_CHUNK_SIZE		512

#define TCM_RESPONSE_TIMEOUT_MS		1000
#define TCM_FW_DOWNLOAD_TIMEOUT_MS	10000	/* HDL push + internal programming is slow */
#define TCM_RESET_ACTIVE_MS		10
#define TCM_RESET_DELAY_MS		200
#define TCM_MODE_SWITCH_DELAY_MS	200
#define TCM_POWER_ON_DELAY_US		10000	/* rail settling time before reset */

/* Firmware image is loaded from /lib/firmware/<this name>, */
#define TS_FIRMWARE_PATH		"td4375_a13_boe.bin" /* TODO: let something in like the device tree set this,

/* vendor TCM firmware image container format (image_header + N area
 * descriptors, each pointing at a named, CRC32-checked data blob)
 */
#define TCM_FW_IMAGE_MAGIC		0x4818472b
#define TCM_FW_AREA_MAGIC		0x7c05e516
#define TCM_FW_RESERVED_BYTES		14	/* CMD_ROMBOOT_DOWNLOAD header */
#define TCM_FW_ROMBOOT_APP_ID		"ROMBOOT_APP_CODE"

#define TCM_MAX_OBJECTS			10

enum tcm_command {
	TCM_CMD_CONTINUE_WRITE			= 0x01,
	TCM_CMD_IDENTIFY			= 0x02,
	TCM_CMD_RESET				= 0x04,
	TCM_CMD_RUN_APPLICATION_FIRMWARE	= 0x14,
	TCM_CMD_GET_APPLICATION_INFO		= 0x20,
	TCM_CMD_GET_TOUCH_REPORT_CONFIG	= 0x25,
	TCM_CMD_ENTER_DEEP_SLEEP		= 0x2c,
	TCM_CMD_EXIT_DEEP_SLEEP		= 0x2d,
	TCM_CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE = 0x42,
	TCM_CMD_ROMBOOT_DOWNLOAD		= 0x45,
};

enum tcm_status {
	TCM_STATUS_IDLE				= 0x00,
	TCM_STATUS_OK					= 0x01,
	TCM_STATUS_BUSY					= 0x02,
	TCM_STATUS_CONTINUED_READ			= 0x03,
	TCM_STATUS_NOT_EXECUTED_IN_DEEP_SLEEP		= 0x0b,
	TCM_STATUS_RECEIVE_BUFFER_OVERFLOW		= 0x0c,
	TCM_STATUS_PREVIOUS_COMMAND_PENDING		= 0x0d,
	TCM_STATUS_NOT_IMPLEMENTED			= 0x0e,
	TCM_STATUS_ERROR				= 0x0f,
	TCM_STATUS_INVALID				= 0xff,
};

enum tcm_report {
	TCM_REPORT_IDENTIFY	= 0x10,
	TCM_REPORT_TOUCH	= 0x11,
};

enum tcm_mode {
	TCM_MODE_APPLICATION_FIRMWARE		= 0x01,
	TCM_MODE_HOSTDOWNLOAD_FIRMWARE		= 0x02,
	TCM_MODE_ROMBOOTLOADER			= 0x04,
	TCM_MODE_BOOTLOADER			= 0x0b,
	TCM_MODE_TDDI_BOOTLOADER		= 0x0c,
};

#define TCM_IS_FW_MODE(mode) \
	((mode) == TCM_MODE_APPLICATION_FIRMWARE || \
	 (mode) == TCM_MODE_HOSTDOWNLOAD_FIRMWARE)

enum tcm_command_status {
	TCM_CMD_STATUS_IDLE	= 0,
	TCM_CMD_STATUS_BUSY	= 1,
	TCM_CMD_STATUS_ERROR	= -1,
};

/* opcodes used inside a touch report *configuration* blob, as returned by
 * CMD_GET_TOUCH_REPORT_CONFIG. This is not a fixed report layout: the
 * firmware hands us a small bytecode describing how to walk the actual
 * touch report payload bit-by-bit.
 */
enum tcm_touch_report_code {
	TCM_TOUCH_END				= 0x00,
	TCM_TOUCH_FOREACH_ACTIVE_OBJECT	= 0x01,
	TCM_TOUCH_FOREACH_OBJECT		= 0x02,
	TCM_TOUCH_FOREACH_END			= 0x03,
	TCM_TOUCH_PAD_TO_NEXT_BYTE		= 0x04,
	TCM_TOUCH_TIMESTAMP			= 0x05,
	TCM_TOUCH_OBJECT_N_INDEX		= 0x06,
	TCM_TOUCH_OBJECT_N_CLASSIFICATION	= 0x07,
	TCM_TOUCH_OBJECT_N_X_POSITION		= 0x08,
	TCM_TOUCH_OBJECT_N_Y_POSITION		= 0x09,
	TCM_TOUCH_OBJECT_N_Z			= 0x0a,
	TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS	= 0x18,
};

enum tcm_object_status {
	TCM_OBJECT_LIFT		= 0,
	TCM_OBJECT_FINGER	= 1,
	TCM_OBJECT_GLOVED_FINGER = 2,
};

struct tcm_message_header {
	u8 marker;
	u8 code;
	u8 length[2];
} __packed;

/* firmware image container (see TCM_FW_* defines above) */
struct tcm_fw_image_header {
	u8 magic_value[4];
	u8 num_of_areas[4];
} __packed;

struct tcm_fw_area_descriptor {
	u8 magic_value[4];
	u8 id_string[16];
	u8 flags[4];
	u8 flash_addr_words[4];
	u8 length[4];
	u8 checksum[4];
} __packed;

struct tcm_identify_info {
	u8 version;
	u8 mode;
	u8 part_number[16];
	u8 build_id[4];
	u8 max_write_size[2];
} __packed;

struct tcm_app_info {
	u8 version[2];
	u8 status[2];
	u8 static_config_size[2];
	u8 dynamic_config_size[2];
	u8 app_config_start_write_block[2];
	u8 app_config_size[2];
	u8 max_touch_report_config_size[2];
	u8 max_touch_report_payload_size[2];
	u8 customer_config_id[16];
	u8 max_x[2];
	u8 max_y[2];
	u8 max_objects[2];
	u8 num_of_buttons[2];
	u8 num_of_image_rows[2];
	u8 num_of_image_cols[2];
	u8 has_hybrid_data[2];
	u8 num_of_force_elecs[2];
} __packed;

struct tcm_object_data {
	u8 status;
	unsigned int x;
	unsigned int y;
	unsigned int z;
};

struct ovt_tcm_data {
	struct spi_device *spi;
	struct input_dev *input_dev;
	struct touchscreen_properties props;
	struct gpio_desc *reset_gpio;

	/* digital core supply, host interface supply - real regulators or
	 * the core's dummy regulator, see ovt_tcm_power_on()
	 */
	struct regulator *vdd;
	struct regulator *vddio;

	/* serializes a whole command + response transaction */
	struct mutex command_lock;
	/* serializes raw bus access between the command path and the IRQ
	 * thread reading unsolicited reports
	 */
	struct mutex rw_lock;
	struct completion response_done;

	u8 *in_buf;
	unsigned int in_buf_size;
	unsigned int read_length;

	unsigned int wr_chunk_size;
	unsigned int payload_length;
	u8 status_report_code;

	u8 command;
	int command_status;
	u8 response_code;

	u8 *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	const u8 *report_buf;
	unsigned int report_buf_len;

	struct tcm_identify_info id_info;
	struct tcm_app_info app_info;

	u8 *report_config;
	unsigned int report_config_size;

	unsigned int max_x;
	unsigned int max_y;
	unsigned int max_objects;

	struct tcm_object_data objects[TCM_MAX_OBJECTS];
};

static inline unsigned int le2_to_uint(const u8 *src)
{
	return (unsigned int)src[0] | ((unsigned int)src[1] << 8);
}

static inline unsigned int le4_to_uint(const u8 *src)
{
	return (unsigned int)src[0] | ((unsigned int)src[1] << 8) |
	       ((unsigned int)src[2] << 16) | ((unsigned int)src[3] << 24);
}

/* ------------------------------------------------------------------
 * Raw SPI byte transport
 *
 * The TCM SPI protocol is a plain full-duplex byte stream: to read N
 * bytes the host clocks out N filler 0xff bytes while capturing MISO;
 * to write N bytes the host just clocks them out.
 * ------------------------------------------------------------------ */

static int ovt_tcm_spi_read(struct ovt_tcm_data *ts, void *buf, size_t len)
{
	struct spi_transfer xfer = { 0 };
	struct spi_message msg;
	u8 *txbuf;
	int ret;

	txbuf = kmalloc(len, GFP_KERNEL);
	if (!txbuf)
		return -ENOMEM;
	memset(txbuf, 0xff, len);

	xfer.tx_buf = txbuf;
	xfer.rx_buf = buf;
	xfer.len = len;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(ts->spi, &msg);

	kfree(txbuf);

	return ret;
}

static int ovt_tcm_spi_write(struct ovt_tcm_data *ts, const void *buf, size_t len)
{
	return spi_write(ts->spi, buf, len);
}

/* ------------------------------------------------------------------
 * TCM message layer
 * ------------------------------------------------------------------ */

static int ovt_tcm_ensure_in_buf(struct ovt_tcm_data *ts, unsigned int size)
{
	u8 *newbuf;

	if (size <= ts->in_buf_size)
		return 0;

	newbuf = krealloc(ts->in_buf, size, GFP_KERNEL);
	if (!newbuf)
		return -ENOMEM;

	ts->in_buf = newbuf;
	ts->in_buf_size = size;

	return 0;
}

static void ovt_tcm_report_touch(struct ovt_tcm_data *ts);

static void ovt_tcm_dispatch_report(struct ovt_tcm_data *ts)
{
	ts->report_buf = &ts->in_buf[TCM_MSG_HEADER_SIZE];
	ts->report_buf_len = ts->payload_length;

	if (ts->status_report_code == TCM_REPORT_TOUCH && ts->input_dev)
		ovt_tcm_report_touch(ts);
}

static void ovt_tcm_dispatch_response(struct ovt_tcm_data *ts)
{
	if (ts->command_status != TCM_CMD_STATUS_BUSY)
		return;

	ts->response_code = ts->status_report_code;

	if (ts->payload_length) {
		if (ts->payload_length > ts->resp_buf_size) {
			u8 *newbuf = krealloc(ts->resp_buf, ts->payload_length,
					       GFP_KERNEL);
			if (!newbuf) {
				ts->command_status = TCM_CMD_STATUS_ERROR;
				complete(&ts->response_done);
				return;
			}
			ts->resp_buf = newbuf;
			ts->resp_buf_size = ts->payload_length;
		}

		memcpy(ts->resp_buf, &ts->in_buf[TCM_MSG_HEADER_SIZE],
		       ts->payload_length);
	}
	ts->resp_length = ts->payload_length;

	ts->command_status = TCM_CMD_STATUS_IDLE;
	complete(&ts->response_done);
}

static void ovt_tcm_dispatch_message(struct ovt_tcm_data *ts)
{
	if (ts->status_report_code == TCM_REPORT_IDENTIFY) {
		unsigned int len = min_t(unsigned int, sizeof(ts->id_info),
					  ts->payload_length);

		memcpy(&ts->id_info, &ts->in_buf[TCM_MSG_HEADER_SIZE], len);

		dev_dbg(&ts->spi->dev, "identify report, mode=0x%02x\n",
			ts->id_info.mode);

		if (ts->command_status == TCM_CMD_STATUS_BUSY) {
			if (ts->command == TCM_CMD_RESET ||
			    ts->command == TCM_CMD_IDENTIFY ||
			    ts->command == TCM_CMD_RUN_APPLICATION_FIRMWARE ||
			    ts->command == TCM_CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE) {
				/*
				 * These commands all cause the controller to
				 * reboot into a new mode as their "response" -
				 * it answers with an unsolicited-shaped
				 * REPORT_IDENTIFY instead of plain STATUS_OK.
				 * Same payload, so deliver it like a normal
				 * response for callers reading ts->resp_buf.
				 */
				if (ts->payload_length > ts->resp_buf_size) {
					u8 *newbuf = krealloc(ts->resp_buf,
							ts->payload_length,
							GFP_KERNEL);
					if (!newbuf) {
						ts->command_status = TCM_CMD_STATUS_ERROR;
						complete(&ts->response_done);
						return;
					}
					ts->resp_buf = newbuf;
					ts->resp_buf_size = ts->payload_length;
				}
				if (ts->payload_length)
					memcpy(ts->resp_buf,
					       &ts->in_buf[TCM_MSG_HEADER_SIZE],
					       ts->payload_length);
				ts->resp_length = ts->payload_length;
				ts->response_code = TCM_STATUS_OK;
				ts->command_status = TCM_CMD_STATUS_IDLE;
			} else {
				dev_info(&ts->spi->dev,
					 "unsolicited device reset while command 0x%02x pending\n",
					 ts->command);
				ts->command_status = TCM_CMD_STATUS_ERROR;
			}
			complete(&ts->response_done);
			return;
		}
	}

	if (ts->status_report_code >= TCM_REPORT_IDENTIFY)
		ovt_tcm_dispatch_report(ts);
	else
		ovt_tcm_dispatch_response(ts);
}

/* Fetch the remainder of a message that didn't fit in the initial
 * ts->read_length-sized read. total_length includes the 4-byte header
 * and the trailing padding byte.
 */
static int ovt_tcm_continued_read(struct ovt_tcm_data *ts, unsigned int total_length)
{
	unsigned int remaining = total_length - ts->read_length;
	u8 *tmp;
	int ret;

	ret = ovt_tcm_ensure_in_buf(ts, total_length + 1);
	if (ret)
		return ret;

	/* continuation packets are prefixed with their own 2-byte header
	 * (marker + STATUS_CONTINUED_READ)
	 */
	tmp = kmalloc(remaining + 2, GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	ret = ovt_tcm_spi_read(ts, tmp, remaining + 2);
	if (ret)
		goto out;

	if (tmp[0] != TCM_MSG_MARKER || tmp[1] != TCM_STATUS_CONTINUED_READ) {
		dev_err(&ts->spi->dev,
			"bad continued-read header (0x%02x, 0x%02x)\n",
			tmp[0], tmp[1]);
		ret = -EIO;
		goto out;
	}

	memcpy(&ts->in_buf[ts->read_length], &tmp[2], remaining);

out:
	kfree(tmp);
	return ret;
}

static int ovt_tcm_read_message(struct ovt_tcm_data *ts)
{
	struct tcm_message_header *header;
	unsigned int total_length;
	int ret;

	mutex_lock(&ts->rw_lock);

	ret = ovt_tcm_spi_read(ts, ts->in_buf, ts->read_length);
	if (ret) {
		dev_err(&ts->spi->dev, "failed to read from device: %d\n", ret);
		goto out;
	}

	header = (struct tcm_message_header *)ts->in_buf;
	if (header->marker != TCM_MSG_MARKER) {
		ret = -ENXIO;
		goto out;
	}

	ts->status_report_code = header->code;
	ts->payload_length = le2_to_uint(header->length);

	if (ts->status_report_code <= TCM_STATUS_ERROR ||
	    ts->status_report_code == TCM_STATUS_INVALID) {
		switch (ts->status_report_code) {
		case TCM_STATUS_OK:
			break;
		case TCM_STATUS_IDLE:
		case TCM_STATUS_BUSY:
		case TCM_STATUS_CONTINUED_READ:
			/* nothing pending / out-of-sync continuation; not an
			 * error, just nothing to dispatch this time around
			 */
			ret = 0;
			goto out;
		default:
			dev_err(&ts->spi->dev, "bad status code 0x%02x\n",
				ts->status_report_code);
			ts->payload_length = 0;
			break;
		}
	}

	total_length = TCM_MSG_HEADER_SIZE + ts->payload_length + 1;
	if (total_length > TCM_MAX_READ_LEN) {
		dev_err(&ts->spi->dev, "implausible payload length %u\n",
			ts->payload_length);
		ret = -EIO;
		goto out;
	}

	if (total_length <= ts->read_length) {
		/* already have everything */
	} else if (total_length == ts->read_length + 1) {
		ret = ovt_tcm_ensure_in_buf(ts, total_length + 1);
		if (ret)
			goto out;
		ts->in_buf[total_length - 1] = TCM_MSG_PADDING;
	} else {
		ret = ovt_tcm_continued_read(ts, total_length);
		if (ret)
			goto out;
	}

	if (ts->in_buf[total_length - 1] != TCM_MSG_PADDING) {
		dev_err(&ts->spi->dev, "bad message padding byte (0x%02x)\n",
			ts->in_buf[total_length - 1]);
		ret = -EIO;
		goto out;
	}

	/* predictive read sizing for next time */
	ts->read_length = clamp(total_length, (unsigned int)TCM_MIN_READ_LEN,
				 (unsigned int)TCM_MAX_READ_LEN);

	ovt_tcm_dispatch_message(ts);
	ret = 0;

out:
	if (ret < 0 && ts->command_status == TCM_CMD_STATUS_BUSY) {
		ts->command_status = TCM_CMD_STATUS_ERROR;
		complete(&ts->response_done);
	}

	mutex_unlock(&ts->rw_lock);

	return ret;
}

/**
 * ovt_tcm_write_message() - send a command and wait for its response
 *
 * On success, resp and resp_len (if non-NULL) point at ts->resp_buf, which
 * stays valid only until the next call to this function.
 */
static int ovt_tcm_write_message(struct ovt_tcm_data *ts, u8 command,
				  const u8 *payload, unsigned int length,
				  u8 **resp, unsigned int *resp_len)
{
	unsigned int remaining = length + 2;
	unsigned int chunk_space;
	unsigned int chunks;
	unsigned int idx, xfer_len;
	unsigned int timeout_ms = TCM_RESPONSE_TIMEOUT_MS;
	u8 *out;
	int ret;

	/* CMD_ROMBOOT_DOWNLOAD ignores the normal write-chunk-size cap and
	 * goes out as one single (potentially large) transfer, matching the
	 * vendor driver's HDL_WR_CHUNK_SIZE=0 behavior - and needs much
	 * longer than a normal command to get a response.
	 */
	if (command == TCM_CMD_ROMBOOT_DOWNLOAD) {
		chunk_space = remaining;
		timeout_ms = TCM_FW_DOWNLOAD_TIMEOUT_MS;
	} else {
		chunk_space = ts->wr_chunk_size ? ts->wr_chunk_size - 1 : remaining;
	}
	chunks = max_t(unsigned int, 1, DIV_ROUND_UP(remaining, chunk_space));

	mutex_lock(&ts->command_lock);

	ts->command = command;
	ts->command_status = TCM_CMD_STATUS_BUSY;
	reinit_completion(&ts->response_done);

	/* chunk_space can be firmware-image sized for CMD_ROMBOOT_DOWNLOAD -
	 * use kvmalloc rather than kmalloc since that can be a large,
	 * possibly non-order-friendly allocation.
	 */
	out = kvmalloc(chunk_space + 1, GFP_KERNEL);
	if (!out) {
		ret = -ENOMEM;
		goto unlock;
	}

	mutex_lock(&ts->rw_lock);

	for (idx = 0; idx < chunks; idx++) {
		xfer_len = min(remaining, chunk_space);

		if (idx == 0) {
			out[0] = command;
			out[1] = (u8)length;
			out[2] = (u8)(length >> 8);
			if (xfer_len > 2 && length)
				memcpy(&out[3], payload, xfer_len - 2);
		} else {
			out[0] = TCM_CMD_CONTINUE_WRITE;
			memcpy(&out[1], payload + (idx * chunk_space - 2),
			       xfer_len);
		}

		ret = ovt_tcm_spi_write(ts, out, xfer_len + 1);
		if (ret) {
			mutex_unlock(&ts->rw_lock);
			goto free_out;
		}

		remaining -= xfer_len;

		if (chunks > 1)
			usleep_range(500, 1000);
	}

	mutex_unlock(&ts->rw_lock);

	ret = wait_for_completion_timeout(&ts->response_done,
			msecs_to_jiffies(timeout_ms));
	if (!ret) {
		dev_err(&ts->spi->dev,
			"timed out waiting for response to command 0x%02x\n",
			command);
		ret = -ETIMEDOUT;
		goto free_out;
	}

	if (ts->command_status != TCM_CMD_STATUS_IDLE) {
		ret = -EIO;
		goto free_out;
	}

	if (ts->response_code != TCM_STATUS_OK) {
		dev_err(&ts->spi->dev,
			"command 0x%02x failed, status 0x%02x\n",
			command, ts->response_code);
		ret = -EIO;
		goto free_out;
	}

	if (resp)
		*resp = ts->resp_buf;
	if (resp_len)
		*resp_len = ts->resp_length;

	ret = 0;

free_out:
	kvfree(out);
unlock:
	ts->command = 0;
	ts->command_status = TCM_CMD_STATUS_IDLE;
	mutex_unlock(&ts->command_lock);

	return ret;
}

/* ------------------------------------------------------------------
 * Touch report parsing
 *
 * The firmware describes its own report layout as a small bytecode
 * (fetched once via CMD_GET_TOUCH_REPORT_CONFIG) that we walk bit by bit
 * against each incoming REPORT_TOUCH payload. This lets the same parser
 * handle whatever fields+order the particular firmware build emits.
 * ------------------------------------------------------------------ */

static int tcm_get_report_bits(const u8 *report, unsigned int report_len,
				unsigned int offset, unsigned int bits,
				unsigned int *out)
{
	unsigned int byte_off, bit_off, avail, take, got = 0, val = 0;

	if (!bits || bits > 32)
		return -EINVAL;

	if (offset + bits > report_len * 8) {
		*out = 0;
		return 0;
	}

	byte_off = offset / 8;
	bit_off = offset % 8;

	while (got < bits) {
		u8 byte = report[byte_off] >> bit_off;

		avail = 8 - bit_off;
		take = min(avail, bits - got);
		byte &= (u8)((1u << take) - 1);
		val |= (unsigned int)byte << got;

		got += take;
		bit_off = 0;
		byte_off++;
	}

	*out = val;

	return 0;
}

static void ovt_tcm_parse_touch_report(struct ovt_tcm_data *ts)
{
	const u8 *config = ts->report_config;
	unsigned int config_size = ts->report_config_size;
	const u8 *report = ts->report_buf;
	unsigned int report_len = ts->report_buf_len;
	unsigned int idx = 0, offset = 0, obj = 0, next = 0;
	unsigned int active_seen = 0, num_active = 0;
	unsigned int end_of_foreach = 0;
	bool active_only = false, have_num_active = false;

	if (!config || !config_size)
		return;

	memset(ts->objects, 0, sizeof(ts->objects));

	while (idx < config_size) {
		u8 code = config[idx++];
		unsigned int bits, data;

		switch (code) {
		case TCM_TOUCH_END:
			goto done;

		case TCM_TOUCH_FOREACH_ACTIVE_OBJECT:
			obj = 0;
			next = idx;
			active_only = true;
			break;

		case TCM_TOUCH_FOREACH_OBJECT:
			obj = 0;
			next = idx;
			active_only = false;
			break;

		case TCM_TOUCH_FOREACH_END:
			end_of_foreach = idx;
			if (active_only) {
				if (have_num_active) {
					active_seen++;
					if (active_seen < num_active)
						idx = next;
				} else if (offset < report_len * 8) {
					idx = next;
				}
			} else {
				obj++;
				if (obj < ts->max_objects)
					idx = next;
			}
			break;

		case TCM_TOUCH_PAD_TO_NEXT_BYTE:
			offset = round_up(offset, 8);
			break;

		case TCM_TOUCH_OBJECT_N_INDEX:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &obj);
			offset += bits;
			break;

		case TCM_TOUCH_OBJECT_N_CLASSIFICATION:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &data);
			if (obj < ts->max_objects)
				ts->objects[obj].status = data;
			offset += bits;
			break;

		case TCM_TOUCH_OBJECT_N_X_POSITION:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &data);
			if (obj < ts->max_objects)
				ts->objects[obj].x = data;
			offset += bits;
			break;

		case TCM_TOUCH_OBJECT_N_Y_POSITION:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &data);
			if (obj < ts->max_objects)
				ts->objects[obj].y = data;
			offset += bits;
			break;

		case TCM_TOUCH_OBJECT_N_Z:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &data);
			if (obj < ts->max_objects)
				ts->objects[obj].z = data;
			offset += bits;
			break;

		case TCM_TOUCH_NUM_OF_ACTIVE_OBJECTS:
			bits = config[idx++];
			tcm_get_report_bits(report, report_len, offset, bits, &data);
			num_active = data;
			have_num_active = true;
			offset += bits;
			if (!num_active) {
				if (!end_of_foreach)
					goto done;
				idx = end_of_foreach;
			}
			break;

		default:
			/* field we don't care about: still have to skip the
			 * right number of bits to stay in sync
			 */
			bits = config[idx++];
			offset += bits;
			break;
		}
	}

done:
	;
}

static void ovt_tcm_report_touch(struct ovt_tcm_data *ts)
{
	unsigned int i;

	ovt_tcm_parse_touch_report(ts);

	for (i = 0; i < ts->max_objects && i < TCM_MAX_OBJECTS; i++) {
		struct tcm_object_data *obj = &ts->objects[i];

		input_mt_slot(ts->input_dev, i);

		if (obj->status == TCM_OBJECT_FINGER ||
		    obj->status == TCM_OBJECT_GLOVED_FINGER) {
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			touchscreen_report_pos(ts->input_dev, &ts->props,
						obj->x, obj->y, true);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
					  obj->z ?: 1);
		} else {
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
		}
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/* ------------------------------------------------------------------
 * Controller bring-up
 * ------------------------------------------------------------------ */

static void ovt_tcm_hw_reset(struct ovt_tcm_data *ts)
{
	if (!ts->reset_gpio)
		return;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(TCM_RESET_ACTIVE_MS * 1000, TCM_RESET_ACTIVE_MS * 1000 + 2000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(TCM_RESET_DELAY_MS);
}

/*
 * Bring up the controller's power rails. Both vdd (digital core) and vddio
 * (host interface level shifter / pull-ups) are fetched with a plain
 * devm_regulator_get(), the same pattern panfrost uses for its "mali"
 * supply: if the board's DT doesn't define vdd-supply/vddio-supply at all,
 * the regulator core transparently substitutes its always-on dummy
 * regulator (requires CONFIG_REGULATOR_DUMMY) instead of erroring out, so
 * boards where these LDOs are latched on by the bootloader (e.g. uniLoader
 * on Exynos) and never described as controllable regulators just work with
 * no special-casing here. If vdd-supply *is* present in DT but its
 * provider can't probe (e.g. stuck behind an ACPM mailbox), this still
 * correctly returns -EPROBE_DEFER instead of masking the problem.
 */
static int ovt_tcm_power_on(struct ovt_tcm_data *ts)
{
	int ret;

	/* keep the controller held in reset while the rails come up */
	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 1);

	ret = regulator_enable(ts->vdd);
	if (ret) {
		dev_err(&ts->spi->dev, "failed to enable vdd: %d\n", ret);
		return ret;
	}

	ret = regulator_enable(ts->vddio);
	if (ret) {
		dev_err(&ts->spi->dev, "failed to enable vddio: %d\n", ret);
		regulator_disable(ts->vdd);
		return ret;
	}

	/* allow rails to settle before pulling the controller out of reset.
	 * Harmless when both supplies are dummy/bootloader-managed too.
	 */
	usleep_range(TCM_POWER_ON_DELAY_US, TCM_POWER_ON_DELAY_US + 2000);

	return 0;
}

static void ovt_tcm_power_off(struct ovt_tcm_data *ts)
{
	/* hold the controller in reset while cutting power, so the SPI/IRQ
	 * lines don't glitch while the rails collapse
	 */
	if (ts->reset_gpio)
		gpiod_set_value_cansleep(ts->reset_gpio, 1);

	regulator_disable(ts->vddio);
	regulator_disable(ts->vdd);
}

static int ovt_tcm_identify(struct ovt_tcm_data *ts)
{
	u8 *resp;
	unsigned int resp_len;
	unsigned int max_write_size;
	int ret;

	ret = ovt_tcm_write_message(ts, TCM_CMD_IDENTIFY, NULL, 0, &resp, &resp_len);
	if (ret)
		return ret;

	memcpy(&ts->id_info, resp, min_t(unsigned int, sizeof(ts->id_info), resp_len));

	max_write_size = le2_to_uint(ts->id_info.max_write_size);
	ts->wr_chunk_size = min_t(unsigned int, max_write_size, TCM_WR_CHUNK_SIZE);
	if (!ts->wr_chunk_size)
		ts->wr_chunk_size = max_write_size;

	return 0;
}

static int ovt_tcm_get_app_info(struct ovt_tcm_data *ts)
{
	u8 *resp;
	unsigned int resp_len;
	int ret;

	ret = ovt_tcm_write_message(ts, TCM_CMD_GET_APPLICATION_INFO, NULL, 0,
				     &resp, &resp_len);
	if (ret)
		return ret;

	memcpy(&ts->app_info, resp, min_t(unsigned int, sizeof(ts->app_info), resp_len));

	ts->max_x = le2_to_uint(ts->app_info.max_x);
	ts->max_y = le2_to_uint(ts->app_info.max_y);
	ts->max_objects = le2_to_uint(ts->app_info.max_objects);
	if (ts->max_objects > TCM_MAX_OBJECTS)
		ts->max_objects = TCM_MAX_OBJECTS;

	if (!ts->max_x || !ts->max_y || !ts->max_objects) {
		dev_err(&ts->spi->dev, "implausible application info (%ux%u, %u objects)\n",
			ts->max_x, ts->max_y, ts->max_objects);
		return -EINVAL;
	}

	return 0;
}

static int ovt_tcm_get_report_config(struct ovt_tcm_data *ts)
{
	u8 *resp;
	unsigned int resp_len;
	int ret;

	ret = ovt_tcm_write_message(ts, TCM_CMD_GET_TOUCH_REPORT_CONFIG, NULL, 0,
				     &resp, &resp_len);
	if (ret)
		return ret;

	if (!resp_len)
		return -EINVAL;

	kfree(ts->report_config);
	ts->report_config = kmemdup(resp, resp_len, GFP_KERNEL);
	if (!ts->report_config)
		return -ENOMEM;

	ts->report_config_size = resp_len;

	return 0;
}

/* ------------------------------------------------------------------
 * ROM-bootloader firmware download (zeroflash)
 *
 * This controller has no persistent flash for application firmware - see
 * the file header comment. On every cold boot it comes up in
 * TCM_MODE_ROMBOOTLOADER and must be pushed a firmware image before
 * application mode exists at all.
 * ------------------------------------------------------------------ */

/*
 * Walk the vendor firmware image container (image_header + N area
 * descriptors) looking for the ROMBOOT_APP_CODE area, verifying its CRC32,
 * and returning a pointer/length into the caller's buffer (no copy).
 */
static int ovt_tcm_find_romboot_fw(const u8 *image, size_t image_size,
				    const u8 **out_data, unsigned int *out_len)
{
	const struct tcm_fw_image_header *header;
	unsigned int num_areas, i, offset;

	if (image_size < sizeof(*header))
		return -EINVAL;

	header = (const struct tcm_fw_image_header *)image;
	if (le4_to_uint(header->magic_value) != TCM_FW_IMAGE_MAGIC) {
		pr_err("ovt-td4150: bad firmware image magic\n");
		return -EINVAL;
	}

	num_areas = le4_to_uint(header->num_of_areas);
	offset = sizeof(*header);

	for (i = 0; i < num_areas; i++) {
		const struct tcm_fw_area_descriptor *desc;
		const u8 *content;
		unsigned int addr, length;
		u32 crc, want_crc;

		if ((u64)offset + 4 > image_size)
			return -EINVAL;
		addr = le4_to_uint(image + offset);
		offset += 4;

		if ((u64)addr + sizeof(*desc) > image_size)
			continue;
		desc = (const struct tcm_fw_area_descriptor *)(image + addr);

		if (le4_to_uint(desc->magic_value) != TCM_FW_AREA_MAGIC)
			continue;

		if (strncmp((const char *)desc->id_string, TCM_FW_ROMBOOT_APP_ID,
			    strlen(TCM_FW_ROMBOOT_APP_ID)) != 0)
			continue;

		length = le4_to_uint(desc->length);
		content = (const u8 *)desc + sizeof(*desc);

		if ((u64)addr + sizeof(*desc) + length > image_size)
			return -EINVAL;

		want_crc = le4_to_uint(desc->checksum);
		crc = crc32(~0, content, length) ^ ~0;
		if (crc != want_crc) {
			pr_err("ovt-td4150: firmware area checksum mismatch\n");
			return -EINVAL;
		}

		*out_data = content;
		*out_len = length;
		return 0;
	}

	return -ENOENT;
}

static int ovt_tcm_romboot_download(struct ovt_tcm_data *ts)
{
	const struct firmware *fw;
	const u8 *fw_data;
	unsigned int fw_len;
	unsigned int out_len;
	u8 *out;
	u8 *resp;
	unsigned int resp_len;
	int ret;

	ret = request_firmware(&fw, TS_FIRMWARE_PATH, &ts->spi->dev);
	if (ret) {
		dev_err(&ts->spi->dev, "failed to load /lib/firmware/%s: %d\n",
			TS_FIRMWARE_PATH, ret);
		return ret;
	}

	ret = ovt_tcm_find_romboot_fw(fw->data, fw->size, &fw_data, &fw_len);
	if (ret) {
		dev_err(&ts->spi->dev,
			"failed to find ROMBOOT_APP_CODE in firmware image: %d\n", ret);
		goto out_release;
	}

	out_len = TCM_FW_RESERVED_BYTES + fw_len;
	out = kvzalloc(out_len, GFP_KERNEL);
	if (!out) {
		ret = -ENOMEM;
		goto out_release;
	}

	/* CMD_ROMBOOT_DOWNLOAD payload: 14 reserved bytes (top byte of the
	 * 24-bit image size, rest zero) followed by the raw firmware bytes
	 */
	out[0] = (u8)(fw_len >> 16);
	memcpy(out + TCM_FW_RESERVED_BYTES, fw_data, fw_len);

	dev_info(&ts->spi->dev, "downloading %u byte application firmware\n", fw_len);

	ret = ovt_tcm_write_message(ts, TCM_CMD_ROMBOOT_DOWNLOAD, out, out_len,
				     &resp, &resp_len);
	kvfree(out);
	if (ret) {
		dev_err(&ts->spi->dev, "firmware download failed: %d\n", ret);
		goto out_release;
	}

	ret = ovt_tcm_write_message(ts, TCM_CMD_ROMBOOT_RUN_BOOTLOADER_FIRMWARE,
				     NULL, 0, &resp, &resp_len);
	if (ret) {
		dev_err(&ts->spi->dev,
			"failed to run downloaded bootloader firmware: %d\n", ret);
		goto out_release;
	}

	msleep(TCM_MODE_SWITCH_DELAY_MS);
	ret = 0;

out_release:
	release_firmware(fw);
	return ret;
}

static int ovt_tcm_init_device(struct ovt_tcm_data *ts)
{
	u8 *resp;
	unsigned int resp_len;
	int ret;

	ovt_tcm_hw_reset(ts);

	ret = ovt_tcm_identify(ts);
	if (ret)
		return ret;

	if (ts->id_info.mode == TCM_MODE_ROMBOOTLOADER) {
		dev_info(&ts->spi->dev,
			 "controller in ROM bootloader, pushing application firmware\n");

		ret = ovt_tcm_romboot_download(ts);
		if (ret)
			return ret;

		ret = ovt_tcm_identify(ts);
		if (ret)
			return ret;
	}

	if (!TCM_IS_FW_MODE(ts->id_info.mode)) {
		/*
		 * Controller is in some other bootloader mode (not ROM
		 * bootloader) - the firmware may already be resident and
		 * just needs a nudge to run.
		 */
		dev_warn(&ts->spi->dev,
			 "controller in mode 0x%02x, requesting application firmware\n",
			 ts->id_info.mode);

		ret = ovt_tcm_write_message(ts, TCM_CMD_RUN_APPLICATION_FIRMWARE,
					     NULL, 0, &resp, &resp_len);
		if (ret)
			return ret;

		msleep(TCM_MODE_SWITCH_DELAY_MS);

		ret = ovt_tcm_identify(ts);
		if (ret)
			return ret;

		if (!TCM_IS_FW_MODE(ts->id_info.mode)) {
			dev_err(&ts->spi->dev,
				"controller stuck in mode 0x%02x after firmware download\n",
				ts->id_info.mode);
			return -ENODEV;
		}
	}

	ret = ovt_tcm_get_app_info(ts);
	if (ret)
		return ret;

	ret = ovt_tcm_get_report_config(ts);
	if (ret)
		return ret;

	dev_info(&ts->spi->dev,
		 "TD4150 ready: %ux%u, %u objects, fw build %u\n",
		 ts->max_x, ts->max_y, ts->max_objects,
		 le4_to_uint(ts->id_info.build_id));

	return 0;
}

static int ovt_tcm_input_init(struct ovt_tcm_data *ts)
{
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(&ts->spi->dev);
	if (!input)
		return -ENOMEM;

	input->name = "Omnivision TD4150 Touchscreen";
	input->id.bustype = BUS_SPI;
	input->dev.parent = &ts->spi->dev;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->props);

	ret = input_mt_init_slots(input, ts->max_objects,
				   INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	ts->input_dev = input;

	return 0;
}

static irqreturn_t ovt_tcm_irq_thread(int irq, void *data)
{
	struct ovt_tcm_data *ts = data;

	ovt_tcm_read_message(ts);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------
 * Probe / remove / power management
 * ------------------------------------------------------------------ */

static int ovt_tcm_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ovt_tcm_data *ts;
	int ret;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX) {
		dev_err(dev, "full-duplex SPI controller required\n");
		return -EINVAL;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi_set_drvdata(spi, ts);

	spi->bits_per_word = 8;
	if (!spi->mode)
		spi->mode = SPI_MODE_0;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	mutex_init(&ts->command_lock);
	mutex_init(&ts->rw_lock);
	init_completion(&ts->response_done);

	ts->read_length = TCM_MIN_READ_LEN;
	ts->in_buf_size = TCM_MIN_READ_LEN;
	ts->in_buf = kzalloc(ts->in_buf_size, GFP_KERNEL);
	if (!ts->in_buf)
		return -ENOMEM;

	ts->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio)) {
		ret = dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");
		goto err_free_buf;
	}

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd)) {
		ret = dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "failed to get vdd-supply\n");
		goto err_free_buf;
	}

	ts->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ts->vddio)) {
		ret = dev_err_probe(dev, PTR_ERR(ts->vddio),
				     "failed to get vddio-supply\n");
		goto err_free_buf;
	}

	if (spi->irq <= 0) {
		dev_err(dev, "no usable interrupt\n");
		ret = -EINVAL;
		goto err_free_buf;
	}

	ret = ovt_tcm_power_on(ts);
	if (ret) {
		dev_err_probe(dev, ret, "failed to power on controller\n");
		goto err_free_buf;
	}

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ovt_tcm_irq_thread,
					 IRQF_ONESHOT, "ovt-tcm", ts);
	if (ret) {
		dev_err_probe(dev, ret, "failed to request irq\n");
		goto err_power_off;
	}

	ret = ovt_tcm_init_device(ts);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize controller\n");
		goto err_power_off;
	}

	ret = ovt_tcm_input_init(ts);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register input device\n");
		goto err_power_off;
	}

	device_init_wakeup(dev, true);

	return 0;

err_power_off:
	ovt_tcm_power_off(ts);
err_free_buf:
	kfree(ts->report_config);
	kfree(ts->resp_buf);
	kfree(ts->in_buf);
	return ret;
}

static void ovt_tcm_remove(struct spi_device *spi)
{
	struct ovt_tcm_data *ts = spi_get_drvdata(spi);

	device_init_wakeup(&spi->dev, false);

	ovt_tcm_power_off(ts);

	kfree(ts->report_config);
	kfree(ts->resp_buf);
	kfree(ts->in_buf);
}

static int ovt_tcm_suspend(struct device *dev)
{
	struct ovt_tcm_data *ts = spi_get_drvdata(to_spi_device(dev));
	u8 *resp;
	unsigned int resp_len;
	int ret;

	if (device_may_wakeup(dev)) {
		enable_irq_wake(ts->spi->irq);
		return 0;
	}

	/* best-effort: quiesce the scan engine before cutting power */
	ret = ovt_tcm_write_message(ts, TCM_CMD_ENTER_DEEP_SLEEP, NULL, 0,
				     &resp, &resp_len);
	if (ret)
		dev_warn(dev, "failed to enter deep sleep: %d\n", ret);

	disable_irq(ts->spi->irq);
	ovt_tcm_power_off(ts);

	return 0;
}

static int ovt_tcm_resume(struct device *dev)
{
	struct ovt_tcm_data *ts = spi_get_drvdata(to_spi_device(dev));
	int ret;

	if (device_may_wakeup(dev)) {
		disable_irq_wake(ts->spi->irq);
		return 0;
	}

	ret = ovt_tcm_power_on(ts);
	if (ret) {
		dev_err(dev, "failed to power on controller: %d\n", ret);
		return ret;
	}

	enable_irq(ts->spi->irq);

	/* rails were fully cut, so the controller needs a real re-init
	 * rather than just an EXIT_DEEP_SLEEP command
	 */
	ret = ovt_tcm_init_device(ts);
	if (ret) {
		dev_err(dev, "failed to reinitialize controller after resume: %d\n", ret);
		return ret;
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ovt_tcm_pm_ops, ovt_tcm_suspend, ovt_tcm_resume);

static const struct spi_device_id ovt_tcm_spi_id[] = {
	{ "td4150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ovt_tcm_spi_id);

static const struct of_device_id ovt_tcm_of_match[] = {
	{ .compatible = "ovt,td4150" },
	{ }
};
MODULE_DEVICE_TABLE(of, ovt_tcm_of_match);

static struct spi_driver ovt_tcm_spi_driver = {
	.driver = {
		.name = "ovt-td4150",
		.of_match_table = ovt_tcm_of_match,
		.pm = pm_sleep_ptr(&ovt_tcm_pm_ops),
	},
	.probe = ovt_tcm_probe,
	.remove = ovt_tcm_remove,
	.id_table = ovt_tcm_spi_id,
};
module_spi_driver(ovt_tcm_spi_driver);

MODULE_AUTHOR("schoosh212");
MODULE_DESCRIPTION("Omnivision TD4150 touchscreen driver");
MODULE_FIRMWARE(TS_FIRMWARE_PATH);
MODULE_LICENSE("GPL");
