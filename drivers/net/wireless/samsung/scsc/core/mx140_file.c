// SPDX-License-Identifier: GPL-2.0
/*
 * Firmware and configuration file loading for the Maxwell WLBT subsystem
 *
 * Copyright (c) 2014 - 2021 Samsung Electronics Co., Ltd. All rights reserved
 *
 * All files are loaded through the firmware loader, from the "scsc/"
 * directory, e.g. scsc/mx140.bin and scsc/mx140_wlan.hcf.
 */

#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/string.h>
#include <scsc/scsc_logring.h>
#include <scsc/scsc_mx.h>

#include "scsc_mx_impl.h"

#define MX140_FW_DIR		"scsc"
#define MX140_FW_PATH_MAX_LENGTH	128

/* Firmware variant, may be overridden e.g. for a test firmware */
static char *firmware_variant = "mx140";
module_param(firmware_variant, charp, 0444);
MODULE_PARM_DESC(firmware_variant, "Firmware variant, default mx140");

/* Firmware suffixes for the known RF chip IDs */
struct fw_suffix {
	char suffix[6];
	u32 hw_ver;
};

static const struct fw_suffix fw_suffixes[] = {
	{ .suffix = "",      .hw_ver = 0xff, }, /* plain mx140.bin, always used if found */
	{ .suffix = "_s612", .hw_ver = 0xb1, },
	{ .suffix = "_s611", .hw_ver = 0xb0, },
};

/* Once set, this firmware suffix is always used */
static int fw_suffix_found = -1;

static const char *mx140_fw_suffix(void)
{
	return fw_suffix_found < 0 ? "" : fw_suffixes[fw_suffix_found].suffix;
}

static int __mx140_request_file(struct scsc_mx *mx, const char *path,
				const struct firmware **firmp, bool optional)
{
	struct device *dev = scsc_mx_get_device(mx);
	int ret;

	if (!dev)
		return -EIO;

	scsc_mx_request_firmware_mutex_lock(mx);
	scsc_mx_request_firmware_wake_lock(mx);
	if (optional)
		ret = firmware_request_nowarn(firmp, path, dev);
	else
		ret = request_firmware(firmp, path, dev);
	scsc_mx_request_firmware_wake_unlock(mx);
	scsc_mx_request_firmware_mutex_unlock(mx);

	return ret;
}

int mx140_request_file(struct scsc_mx *mx, char *path, const struct firmware **firmp)
{
	return __mx140_request_file(mx, path, firmp, false);
}
EXPORT_SYMBOL(mx140_request_file);

int mx140_release_file(struct scsc_mx *mx, const struct firmware *firmp)
{
	if (!firmp)
		return -EINVAL;

	release_firmware(firmp);
	return 0;
}
EXPORT_SYMBOL(mx140_release_file);

/* Reads a configuration file, e.g. scsc/mx140_wlan.hcf */
int mx140_file_request_conf(struct scsc_mx *mx, const struct firmware **conf,
			    const char *config_rel_path, const char *filename)
{
	char path[MX140_FW_PATH_MAX_LENGTH];
	int r;

	snprintf(path, sizeof(path), MX140_FW_DIR "/%s%s_%s",
		 firmware_variant, mx140_fw_suffix(), filename);

	/* Not every configuration file exists, the callers report missing ones */
	r = __mx140_request_file(mx, path, conf, true);
	if (!r)
		SCSC_TAG_INFO(MX_FILE, "loaded %s\n", path);

	return r;
}
EXPORT_SYMBOL(mx140_file_request_conf);

/* Reads a debug configuration file, e.g. scsc/mx140/debug/log_strings.bin */
int mx140_file_request_debug_conf(struct scsc_mx *mx, const struct firmware **conf,
				  const char *config_rel_path)
{
	char path[MX140_FW_PATH_MAX_LENGTH];

	snprintf(path, sizeof(path), MX140_FW_DIR "/%s%s/debug/%s",
		 firmware_variant, mx140_fw_suffix(), config_rel_path);

	return __mx140_request_file(mx, path, conf, true);
}
EXPORT_SYMBOL(mx140_file_request_debug_conf);

/* Reads a device configuration file, e.g. scsc/conf/wlan/mac.txt */
int mx140_file_request_device_conf(struct scsc_mx *mx, const struct firmware **conf,
				   const char *config_rel_path)
{
	char path[MX140_FW_PATH_MAX_LENGTH];

	snprintf(path, sizeof(path), MX140_FW_DIR "/conf/%s", config_rel_path);

	return __mx140_request_file(mx, path, conf, true);
}
EXPORT_SYMBOL(mx140_file_request_device_conf);

void mx140_file_release_conf(struct scsc_mx *mx, const struct firmware *conf)
{
	mx140_release_file(mx, conf);
}
EXPORT_SYMBOL(mx140_file_release_conf);

static int __mx140_file_download_fw(struct scsc_mx *mx, void *dest, size_t dest_size,
				    u32 *fw_image_size, const char *fw_suffix)
{
	char path[MX140_FW_PATH_MAX_LENGTH];
	const struct firmware *firm;
	int r;

	snprintf(path, sizeof(path), MX140_FW_DIR "/%s%s.bin", firmware_variant, fw_suffix);

	SCSC_TAG_INFO(MX_FILE, "Load WLBT fw %s in shared address %p\n", path, dest);
	r = __mx140_request_file(mx, path, &firm, fw_suffix[0] != '\0');
	if (r)
		return r;

	if (firm->size > dest_size) {
		SCSC_TAG_ERR(MX_FILE, "firmware image too big for buffer (%zu > %zu)\n",
			     firm->size, dest_size);
		r = -EINVAL;
	} else {
		memcpy(dest, firm->data, firm->size);
		*fw_image_size = firm->size;
	}
	release_firmware(firm);

	return r;
}

/* Download firmware binary into a buffer supplied by the caller */
int mx140_file_download_fw(struct scsc_mx *mx, void *dest, size_t dest_size, u32 *fw_image_size)
{
	int r = -ENOENT;
	int i;

	/* If we know which f/w suffix to use, select it immediately */
	if (fw_suffix_found != -1)
		return __mx140_file_download_fw(mx, dest, dest_size, fw_image_size,
						fw_suffixes[fw_suffix_found].suffix);

	/* Otherwise try the list */
	for (i = 0; i < ARRAY_SIZE(fw_suffixes); i++) {
		r = __mx140_file_download_fw(mx, dest, dest_size, fw_image_size,
					     fw_suffixes[i].suffix);
		if (r != -ENOENT)
			break;
	}

	if (!r)
		fw_suffix_found = i;
	else
		SCSC_TAG_ERR(MX_FILE, "Error loading firmware: %d\n", r);

	return r;
}

/* Select the firmware for the RF h/w version (LSB is the RF chip ID) */
int mx140_file_select_fw(struct scsc_mx *mx, u32 hw_ver)
{
	int i;

	SCSC_TAG_INFO(MX_FILE, "select f/w for 0x%04x\n", hw_ver);

	hw_ver &= 0x00ff;
	for (i = 0; i < ARRAY_SIZE(fw_suffixes); i++) {
		if (fw_suffixes[i].hw_ver == hw_ver) {
			fw_suffix_found = i;
			return 0;
		}
	}

	SCSC_TAG_ERR(MX_FILE, "No known f/w for 0x%04x, default to catchall\n", hw_ver);
	fw_suffix_found = -1;

	return -EINVAL;
}

/* Query whether this HW is supported by the current FW file set */
bool mx140_file_supported_hw(struct scsc_mx *mx, u32 hw_ver)
{
	if (fw_suffix_found < 0)
		return false;

	/* Assume 0xff is always compatible, and f/w will panic if it isn't */
	if (fw_suffixes[fw_suffix_found].hw_ver == 0xff)
		return true;

	return fw_suffixes[fw_suffix_found].hw_ver == (hw_ver & 0x00ff);
}
