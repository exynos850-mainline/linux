// SPDX-License-Identifier: GPL-2.0
/*
 * Firmware and configuration file loading for the Maxwell WLBT subsystem
 *
 * Copyright (c) 2014 - 2021 Samsung Electronics Co., Ltd. All rights reserved
 *
 * All files are loaded through the firmware loader, from the "scsc/"
 * directory, e.g. scsc/mx140.bin and scsc/mx140_wlan.hcf.
 *
 * The following optional properties on the WLBT device tree node change
 * where and how the files are looked up:
 *
 *  firmware-path    (string) Directory, relative to the kernel firmware
 *                   search path (normally /lib/firmware), that is used
 *                   instead of "scsc", e.g. "samsung/a13/scsc".
 *  firmware-initfs  (bool) Files are in the initramfs. Load them with no
 *                   userspace fallback and fail immediately if missing.
 *  firmware-rootfs  (bool) Files are on the root filesystem, which may not
 *                   be mounted yet. Like firmware-initfs, but a missing
 *                   mandatory file is retried for a few seconds.
 *
 * Without firmware-initfs or firmware-rootfs, request_firmware() is used.
 * Setting both flags is invalid and both are ignored.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/stdarg.h>
#include <linux/string.h>
#include <scsc/scsc_logring.h>
#include <scsc/scsc_mx.h>

#include "scsc_mx_impl.h"

#define MX140_FW_DIR		"scsc"
#define MX140_FW_PATH_MAX_LENGTH	128
#define MX140_FW_ROOTFS_RETRIES		20
#define MX140_FW_ROOTFS_RETRY_MS	250

enum mx140_fw_source {
	MX140_FW_SRC_DEFAULT,
	MX140_FW_SRC_INITFS,
	MX140_FW_SRC_ROOTFS,
};

/* Set once any file has been loaded, i.e. the rootfs is known to be there */
static bool fw_rootfs_ready;

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

static enum mx140_fw_source mx140_fw_source(struct device *dev)
{
	bool initfs = of_property_read_bool(dev->of_node, "firmware-initfs");
	bool rootfs = of_property_read_bool(dev->of_node, "firmware-rootfs");

	if (initfs && rootfs) {
		dev_warn_once(dev, "firmware-initfs and firmware-rootfs are exclusive, ignoring both\n");
		return MX140_FW_SRC_DEFAULT;
	}
	if (initfs)
		return MX140_FW_SRC_INITFS;
	if (rootfs)
		return MX140_FW_SRC_ROOTFS;
	return MX140_FW_SRC_DEFAULT;
}

/* Firmware directory, relative to the firmware search path */
static const char *mx140_fw_dir(struct scsc_mx *mx)
{
	struct device *dev = scsc_mx_get_device(mx);
	const char *dir;

	if (!dev || of_property_read_string(dev->of_node, "firmware-path", &dir))
		return MX140_FW_DIR;

	if (!*dir || dir[0] == '/' || strstr(dir, "..")) {
		dev_warn_once(dev, "invalid firmware-path \"%s\", using \"%s\"\n",
			      dir, MX140_FW_DIR);
		return MX140_FW_DIR;
	}

	return dir;
}

/* Builds "<firmware dir>/<fmt...>" */
static __printf(4, 5)
int mx140_fw_path(struct scsc_mx *mx, char *buf, size_t len, const char *fmt, ...)
{
	va_list args;
	int n;

	n = snprintf(buf, len, "%s/", mx140_fw_dir(mx));
	if (n >= len)
		return -ENAMETOOLONG;

	va_start(args, fmt);
	n += vsnprintf(buf + n, len - n, fmt, args);
	va_end(args);

	return n >= len ? -ENAMETOOLONG : 0;
}

static int __mx140_request_file(struct scsc_mx *mx, const char *path,
				const struct firmware **firmp, bool optional)
{
	struct device *dev = scsc_mx_get_device(mx);
	enum mx140_fw_source src;
	int tries = 0;
	int ret;

	if (!dev)
		return -EIO;

	src = mx140_fw_source(dev);

	for (;;) {
		scsc_mx_request_firmware_mutex_lock(mx);
		scsc_mx_request_firmware_wake_lock(mx);
		if (src != MX140_FW_SRC_DEFAULT)
			ret = request_firmware_direct(firmp, path, dev);
		else if (optional)
			ret = firmware_request_nowarn(firmp, path, dev);
		else
			ret = request_firmware(firmp, path, dev);
		scsc_mx_request_firmware_wake_unlock(mx);
		scsc_mx_request_firmware_mutex_unlock(mx);

		/*
		 * The rootfs may not be mounted yet: retry mandatory files,
		 * until a file has been found once. Optional files are
		 * routinely missing and must not be delayed.
		 */
		if (ret != -ENOENT || src != MX140_FW_SRC_ROOTFS || optional ||
		    READ_ONCE(fw_rootfs_ready) || ++tries > MX140_FW_ROOTFS_RETRIES)
			break;
		msleep(MX140_FW_ROOTFS_RETRY_MS);
	}

	if (!ret)
		WRITE_ONCE(fw_rootfs_ready, true);
	else if (!optional && src != MX140_FW_SRC_DEFAULT)
		dev_err(dev, "firmware %s not found (%d)\n", path, ret);

	return ret;
}

/* Reads a file, path is relative to the firmware directory */
int mx140_request_file(struct scsc_mx *mx, char *path, const struct firmware **firmp)
{
	char full[MX140_FW_PATH_MAX_LENGTH];
	int r;

	r = mx140_fw_path(mx, full, sizeof(full), "%s", path);
	if (r)
		return r;

	return __mx140_request_file(mx, full, firmp, false);
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

	r = mx140_fw_path(mx, path, sizeof(path), "%s%s_%s",
			  firmware_variant, mx140_fw_suffix(), filename);
	if (r)
		return r;

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
	int r;

	r = mx140_fw_path(mx, path, sizeof(path), "%s%s/debug/%s",
			  firmware_variant, mx140_fw_suffix(), config_rel_path);
	if (r)
		return r;

	return __mx140_request_file(mx, path, conf, true);
}
EXPORT_SYMBOL(mx140_file_request_debug_conf);

/* Reads a device configuration file, e.g. scsc/conf/wlan/mac.txt */
int mx140_file_request_device_conf(struct scsc_mx *mx, const struct firmware **conf,
				   const char *config_rel_path)
{
	char path[MX140_FW_PATH_MAX_LENGTH];
	int r;

	r = mx140_fw_path(mx, path, sizeof(path), "conf/%s", config_rel_path);
	if (r)
		return r;

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

	r = mx140_fw_path(mx, path, sizeof(path), "%s%s.bin", firmware_variant, fw_suffix);
	if (r)
		return r;

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
