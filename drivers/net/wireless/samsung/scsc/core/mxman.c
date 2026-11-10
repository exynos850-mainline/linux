// SPDX-License-Identifier: GPL-2.0
/****************************************************************************
 *
 * Copyright (c) 2014 - 2020 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kmod.h>
#include <linux/notifier.h>
#include "scsc_mx_impl.h"
#include "miframman.h"
#include "mifmboxman.h"
#include "mxman.h"
#include "srvman.h"
#include "mxmgmt_transport.h"
#include "gdb_transport.h"
#include "mxconf.h"
#include "fwimage.h"
#include "fwhdr.h"
#include "mxlog.h"
#include "fw_panic_record.h"
#include "panicmon.h"
#include "mxlog_transport.h"
#include "mxsyserr.h"
#include "mxfwconfig.h"
#include <scsc/scsc_release.h>
#include <scsc/scsc_mx.h>
#include <linux/fs.h>

#include <scsc/scsc_logring.h>


#include <asm/page.h>


#define STRING_BUFFER_MAX_LENGTH 512
#define NUMBER_OF_STRING_ARGS	5
#define MX_DRAM_SIZE (4 * 1024 * 1024)
#define MX_DRAM_SIZE_SECTION_1 (8 * 1024 * 1024)

#define MX_DRAM_SIZE_SECTION_2 (4 * 1024 * 1024)
/* sizeof(struct scsc_bt_audio_abox) (16640 bytes), rounded up to pages */
#define MX_DRAM_SIZE_BT_AUDIO (20 * 1024)

#define MX_DRAM_OFFSET_SECTION_2 MX_DRAM_SIZE_SECTION_1

#define MX_FW_RUNTIME_LENGTH (1024 * 1024)
#define WAIT_FOR_FW_TO_START_DELAY_MS 1000
#define MBOX2_MAGIC_NUMBER 0xbcdeedcb
#define MBOX_INDEX_0 0
#define MBOX_INDEX_1 1
#define MBOX_INDEX_2 2
#define MBOX_INDEX_3 3

#define SCSC_PANIC_ORIGIN_FW   (0x0 << 15)
#define SCSC_PANIC_ORIGIN_HOST (0x1 << 15)

#define SCSC_PANIC_TECH_WLAN   (0x0 << 13)
#define SCSC_PANIC_TECH_CORE   (0x1 << 13)
#define SCSC_PANIC_TECH_BT     (0x2 << 13)
#define SCSC_PANIC_TECH_UNSP   (0x3 << 13)

#define SCSC_PANIC_CODE_MASK 0xFFFF
#define SCSC_PANIC_ORIGIN_MASK  0x8000
#define SCSC_PANIC_TECH_MASK    0x6000
#define SCSC_PANIC_SUBCODE_MASK_LEGACY 0x0FFF
#define SCSC_PANIC_SUBCODE_MASK 0x7FFF

#define SCSC_R4_V2_MINOR_52 52
#define SCSC_R4_V2_MINOR_53 53
#define SCSC_R4_V2_MINOR_54 54

#define MM_HALT_RSP_TIMEOUT_MS 100

/* If limits below are exceeded, a service level reset will be raised to level 7 */
#define SYSERR_LEVEL7_HISTORY_SIZE      (4)
/* Minimum time between system error service resets (ms) */
#define SYSERR_LEVEL7_MIN_INTERVAL      (300000)
/* No more then SYSERR_RESET_HISTORY_SIZE system error service resets in this period (ms)*/
#define SYSERR_LEVEL7_MONITOR_PERIOD    (3600000)

static BLOCKING_NOTIFIER_HEAD(firmware_chain);

/**
 * This will be returned as fw version ONLY if Maxwell
 * was never found or was unloaded.
 */
static char saved_fw_build_id[FW_BUILD_ID_SZ] = "Maxwell WLBT unavailable";

static bool allow_unidentified_firmware;
module_param(allow_unidentified_firmware, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(allow_unidentified_firmware, "Allow unidentified firmware");

static bool skip_header;
module_param(skip_header, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(skip_header, "Skip header, assuming unidentified firmware");

static bool crc_check_allow_none = true;
module_param(crc_check_allow_none, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(crc_check_allow_none, "Allow skipping firmware CRC checks if CRC is not present");

static int crc_check_period_ms = 30000;
module_param(crc_check_period_ms, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(crc_check_period_ms, "Time period for checking the firmware CRCs");

static ulong mm_completion_timeout_ms = 2000;
module_param(mm_completion_timeout_ms, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mm_completion_timeout_ms, "Timeout wait_for_mm_msg_start_ind (ms) - default 1000. 0 = infinite");

static bool skip_mbox0_check;
module_param(skip_mbox0_check, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(skip_mbox0_check, "Allow skipping firmware mbox0 signature check");

static uint firmware_startup_flags;
module_param(firmware_startup_flags, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(firmware_startup_flags, "0 = Proceed as normal (default); Bit 0 = 1 - spin at start of CRT0; Other bits reserved = 0");

static uint trigger_moredump_level = MX_SYSERR_LEVEL_8;
module_param(trigger_moredump_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(trigger_moredump_level, "System error level that triggers moredump - may be 7 or 8 only");


static bool disable_auto_coredump;
module_param(disable_auto_coredump, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_auto_coredump, "Disable driver automatic coredump");

static bool disable_error_handling;
module_param(disable_error_handling, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_error_handling, "Disable error handling");

#define DISABLE_RECOVERY_HANDLING_SCANDUMP 3 /* Halt kernel and scandump on FW failure */

static int disable_recovery_handling;

module_param(disable_recovery_handling, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_recovery_handling, "Disable recovery handling");
static bool disable_recovery_until_reboot;

static uint scandump_trigger_fw_panic = 0;
module_param(scandump_trigger_fw_panic, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(scandump_trigger_fw_panic, "Specify fw panic ID");

static uint panic_record_delay = 1;
module_param(panic_record_delay, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(panic_record_delay, "Delay in ms before accessing the panic record");


static uint syserr_level7_min_interval = SYSERR_LEVEL7_MIN_INTERVAL;
module_param(syserr_level7_min_interval, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(syserr_level7_min_interval, "Minimum time between system error level 7 resets (ms)");

static uint syserr_level7_monitor_period = SYSERR_LEVEL7_MONITOR_PERIOD;
module_param(syserr_level7_monitor_period, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(syserr_level7_monitor_period, "No more then 4 system error level 7 resets in this period (ms)");

/* Time stamps of last level7 resets in jiffies */
static unsigned long syserr_level7_history[SYSERR_LEVEL7_HISTORY_SIZE] = {0};
static int syserr_level7_history_index;




/* Track when WLBT reset fails to allow debug */
static u64 reset_failed_time;

/* Status of FM driver request, which persists beyond the lifecyle
 * of the scsx_mx driver.
 */

static int firmware_runtime_flags;
static int syserr_command;
/**
 * This mxman reference is initialized/nullified via mxman_init/deinit
 * called by scsc_mx_create/destroy on module probe/remove.
 */
static struct mxman *active_mxman;
static bool send_fw_config_to_active_mxman(uint32_t fw_runtime_flags);
static bool send_syserr_cmd_to_active_mxman(u32 syserr_cmd);
static void mxman_fail_level8(struct mxman *mxman, u16 scsc_panic_code, const char *reason);


static bool reset_failed;
static bool mxman_check_reset_failed(struct scsc_mif_abs *mif)
{
	return reset_failed; // || mif->mif_reset_failure(mif);
}

static void mxman_set_reset_failed(void)
{
	reset_failed = true;
}

static int fw_runtime_flags_setter(const char *val, const struct kernel_param *kp)
{
	int ret = -EINVAL;
	uint32_t fw_runtime_flags = 0;

	if (!val)
		return ret;
	ret = kstrtouint(val, 10, &fw_runtime_flags);
	if (!ret) {
		if (send_fw_config_to_active_mxman(fw_runtime_flags))
			firmware_runtime_flags = fw_runtime_flags;
		else
			ret = -EINVAL;
	}
	return ret;
}

/**
 * We don't bother to keep an updated copy of the runtime flags effectively
 * currently set into FW...we should add a new message answer handling both in
 * Kenrel and FW side to be sure and this is just to easy debug at the end.
 */
static struct kernel_param_ops fw_runtime_kops = {
	.set = fw_runtime_flags_setter,
	.get = NULL
};

module_param_cb(firmware_runtime_flags, &fw_runtime_kops, NULL, 0200);
MODULE_PARM_DESC(firmware_runtime_flags,
		 "0 = Proceed as normal (default); nnn = Provides FW runtime flags bitmask: unknown bits will be ignored.");

static int syserr_setter(const char *val, const struct kernel_param *kp)
{
	int ret = -EINVAL;
	u32 syserr_cmd = 0;

	if (!val)
		return ret;
	ret = kstrtouint(val, 10, &syserr_cmd);
	if (!ret) {
		u8 sub_system = (u8)(syserr_cmd / 10);
		u8 level = (u8)(syserr_cmd % 10);

		if (((sub_system > 2) && (sub_system < 8)) || (sub_system > 8) || (level > MX_SYSERR_LEVEL_8))
			ret = -EINVAL;
		else if (level == MX_SYSERR_LEVEL_8) {
			if (active_mxman)
				mxman_fail_level8(active_mxman, SCSC_PANIC_CODE_HOST << 15, __func__);
		} else if (send_syserr_cmd_to_active_mxman(syserr_cmd))
			syserr_command = syserr_cmd;
		else
			ret = -EINVAL;
	}
	return ret;
}

static struct kernel_param_ops syserr_kops = {
	.set = syserr_setter,
	.get = NULL
};

module_param_cb(syserr_command, &syserr_kops, NULL, 0200);
MODULE_PARM_DESC(syserr_command,
		 "Decimal XY - Trigger Type X(0,1,2,8), Level Y(1-8). Some combinations not supported");

/**
 * Maxwell Agent Management Messages.
 *
 * TODO: common defn with firmware, generated.
 *
 * The numbers here *must* match the firmware!
 */
enum {
	MM_START_IND = 0,
	MM_HALT_REQ = 1,
	MM_FORCE_PANIC = 2,
	MM_HOST_SUSPEND = 3,
	MM_HOST_RESUME = 4,
	MM_FW_CONFIG = 5,
	MM_HALT_RSP = 6,
	MM_FM_RADIO_CONFIG = 7,
	MM_LERNA_CONFIG = 8,
	MM_SYSERR_IND = 9,
	MM_SYSERR_CMD = 10
} ma_msg;

/**
 * Format of the Maxwell agent messages
 * on the Maxwell management transport stream.
 */
struct ma_msg_packet {

	uint8_t ma_msg; /* Message from ma_msg enum */
	uint32_t arg;	/* Optional arg set by f/w in some to-host messages */
} __packed;

/**
 * Special case Maxwell management, carrying FM radio configuration structure
 */
struct ma_msg_packet_fm_radio_config {

	uint8_t ma_msg;				/* Message from ma_msg enum */
	struct wlbt_fm_params fm_params;	/* FM Radio parameters */
} __packed;

static bool send_fw_config_to_active_mxman(uint32_t fw_runtime_flags)
{
	bool ret = false;
	struct srvman *srvman = NULL;

	SCSC_TAG_INFO(MXMAN, "\n");
	if (!active_mxman) {
		SCSC_TAG_ERR(MXMAN, "Active MXMAN NOT FOUND...cannot send running FW config.\n");
		return ret;
	}

	mutex_lock(&active_mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(active_mxman->mx);
	if (srvman && srvman_in_error(srvman)) {
		mutex_unlock(&active_mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return ret;
	}

	if (active_mxman->mxman_state == MXMAN_STATE_STARTED) {
		struct ma_msg_packet message = { .ma_msg = MM_FW_CONFIG,
			.arg = fw_runtime_flags };

		SCSC_TAG_INFO(MXMAN, "MM_FW_CONFIG -  firmware_runtime_flags:%d\n", message.arg);
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(active_mxman->mx),
				MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message,
				sizeof(message));
		ret = true;
	} else {
		SCSC_TAG_INFO(MXMAN, "MXMAN is NOT STARTED...cannot send MM_FW_CONFIG msg.\n");
	}
	mutex_unlock(&active_mxman->mxman_mutex);

	return ret;
}

static bool send_syserr_cmd_to_active_mxman(u32 syserr_cmd)
{
	bool ret = false;
	struct srvman *srvman = NULL;

	SCSC_TAG_INFO(MXMAN, "\n");
	if (!active_mxman) {
		SCSC_TAG_ERR(MXMAN, "Active MXMAN NOT FOUND...cannot send running FW config.\n");
		return ret;
	}

	mutex_lock(&active_mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(active_mxman->mx);
	if (srvman && srvman_in_error(srvman)) {
		mutex_unlock(&active_mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return ret;
	}

	if (active_mxman->mxman_state == MXMAN_STATE_STARTED) {
		struct ma_msg_packet message = { .ma_msg = MM_SYSERR_CMD,
			.arg = syserr_cmd};

		SCSC_TAG_INFO(MXMAN, "MM_SYSERR_CMD - Args %02d\n", message.arg);
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(active_mxman->mx),
				MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message,
				sizeof(message));
		ret = true;
	} else {
		SCSC_TAG_INFO(MXMAN, "MXMAN is NOT STARTED...cannot send MM_SYSERR_CMD msg.\n");
	}
	mutex_unlock(&active_mxman->mxman_mutex);

	return ret;
}


static void mxman_stop(struct mxman *mxman);
static void print_mailboxes(struct mxman *mxman);
static int wait_for_mm_msg(struct mxman *mxman, struct completion *mm_msg_completion, ulong timeout_ms)
{
	int r;

	(void)mxman; /* unused */

	if (timeout_ms == 0) {
		/* Zero implies infinite wait */
		r = wait_for_completion_interruptible(mm_msg_completion);
		/* r = -ERESTARTSYS if interrupted, 0 if completed */
		return r;
	}
	r = wait_for_completion_timeout(mm_msg_completion, msecs_to_jiffies(timeout_ms));
	if (r == 0) {
		SCSC_TAG_ERR(MXMAN, "timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_mm_msg_start_ind(struct mxman *mxman)
{
	return wait_for_mm_msg(mxman, &mxman->mm_msg_start_ind_completion, mm_completion_timeout_ms);
}

static int wait_for_mm_msg_halt_rsp(struct mxman *mxman)
{
	int r;
	(void)mxman; /* unused */

	if (MM_HALT_RSP_TIMEOUT_MS == 0) {
		/* Zero implies infinite wait */
		r = wait_for_completion_interruptible(&mxman->mm_msg_halt_rsp_completion);
		/* r = -ERESTARTSYS if interrupted, 0 if completed */
		return r;
	}

	r = wait_for_completion_timeout(&mxman->mm_msg_halt_rsp_completion, msecs_to_jiffies(MM_HALT_RSP_TIMEOUT_MS));
	if (r)
		SCSC_TAG_INFO(MXMAN, "Received MM_HALT_RSP from firmware");

	return r;
}

static int send_mm_msg_stop_blocking(struct mxman *mxman)
{
	int r;
	struct ma_msg_packet message = { .ma_msg = MM_HALT_REQ };
	mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));

	r = wait_for_mm_msg_halt_rsp(mxman);
	if (r) {
		/*
		 * MM_MSG_HALT_RSP is not implemented in all versions of firmware, so don't treat it's non-arrival
		 * as an error
		 */
		SCSC_TAG_INFO(MXMAN, "wait_for_MM_HALT_RSP completed");
	}

	return 0;
}

static char *chip_version(u32 rf_hw_ver)
{
	switch (rf_hw_ver & 0x00ff) {
	default:
		break;
	case 0x00b0:
		if ((rf_hw_ver & 0xff00) > 0x1000)
			return "S610/S611";
		else
			return "S610";
	case 0x00b1:
		return "S612";
	case 0x00b2:
		return "S620";
	case 0x0000:
		return "Error: check if RF chip is present";
	}
	return "Unknown";
}

/*
 * This function is used in this file and in mxproc.c to generate consistent
 * RF CHIP VERSION string for logging on console and for storing the same
 * in proc/drivers/mxman_info/rf_chip_version file.
 */
static int mxman_print_rf_hw_version(struct mxman *mxman, char *buf, const size_t bufsz)
{
	int r;

	r = snprintf(buf, bufsz, "RF_CHIP_VERSION: 0x%04x: %s (0x%02x), EVT%x.%x\n",
				  mxman->rf_hw_ver,
				  chip_version(mxman->rf_hw_ver), (mxman->rf_hw_ver & 0x00ff),
				  ((mxman->rf_hw_ver >> 12) & 0xfU), ((mxman->rf_hw_ver >> 8) & 0xfU));

	return r;
}

static void mxman_print_versions(struct mxman *mxman)
{
	char buf[80];

	memset(buf, '\0', sizeof(buf));

	(void)mxman_print_rf_hw_version(mxman, buf, sizeof(buf));

	SCSC_TAG_INFO(MXMAN, "%s", buf);
	SCSC_TAG_INFO(MXMAN, "WLBT FW: %s\n", mxman->fw_build_id);
	SCSC_TAG_INFO(MXMAN, "WLBT Driver: %d.%d.%d.%d.%d\n",
		SCSC_RELEASE_PRODUCT, SCSC_RELEASE_ITERATION, SCSC_RELEASE_CANDIDATE, SCSC_RELEASE_POINT, SCSC_RELEASE_CUSTOMER);
}

/** Receive handler for messages from the FW along the maxwell management transport */
static void mxman_message_handler(const void *message, void *data)
{
	struct mxman        *mxman = (struct mxman *)data;

	/* Forward the message to the applicable service to deal with */
	const struct ma_msg_packet *msg = message;

	switch (msg->ma_msg) {
	case MM_START_IND:
		/* The arg can be used to determine the WLBT/S610 hardware revision */
		SCSC_TAG_INFO(MXMAN, "Received MM_START_IND message from the firmware, arg=0x%04x\n", msg->arg);
		mxman->rf_hw_ver = msg->arg;
		mxman_print_versions(mxman);
		atomic_inc(&mxman->boot_count);
		complete(&mxman->mm_msg_start_ind_completion);
		break;
	case MM_HALT_RSP:
		complete(&mxman->mm_msg_halt_rsp_completion);
		SCSC_TAG_INFO(MXMAN, "Received MM_HALT_RSP message from the firmware\n");
		break;
	case MM_LERNA_CONFIG:
		/* Message response to a firmware configuration query. */
		SCSC_TAG_INFO(MXMAN, "Ignoring MM_LERNA_CONFIG message from firmware\n");
		break;
	case MM_SYSERR_IND:
		/* System Error report from firmware */
		SCSC_TAG_INFO(MXMAN, "Received MM_SYSERR_IND message from firmware\n");
		mx_syserr_handler(mxman, message);
		break;
	default:
		/* HERE: Unknown message, raise fault */
		SCSC_TAG_WARNING(MXMAN, "Received unknown message from the firmware: msg->ma_msg=%d\n", msg->ma_msg);
		break;
	}
}


/*
 * This function calulates and checks two or three (depending on crc32_over_binary flag)
 * crc32 values in the firmware header. The function will check crc32 over the firmware binary
 * (i.e. everything in the file following the header) only if the crc32_over_binary is set to 'true'.
 * This includes initialised data regions so it can be used to check when loading but will not be
 * meaningful once execution starts.
 */
static int do_fw_crc32_checks(char *fw, u32 fw_image_size, struct fwhdr *fwhdr, bool crc32_over_binary)
{
	int r;

	if ((fwhdr->fw_crc32 == 0 || fwhdr->header_crc32 == 0 || fwhdr->const_crc32 == 0) && crc_check_allow_none == 0) {
		SCSC_TAG_ERR(MXMAN, "error: CRC is missing fw_crc32=%d header_crc32=%d crc_check_allow_none=%d\n",
			     fwhdr->fw_crc32, fwhdr->header_crc32, crc_check_allow_none);
		return -EINVAL;
	}

	if (fwhdr->header_crc32 == 0 && crc_check_allow_none == 1) {
		SCSC_TAG_INFO(MXMAN, "Skipping CRC check header_crc32=%d crc_check_allow_none=%d\n",
			      fwhdr->header_crc32, crc_check_allow_none);
	} else {
		/*
		 * CRC-32-IEEE of all preceding header fields (including other CRCs).
		 * Always the last word in the header.
		 */
		r = fwimage_check_fw_header_crc(fw, fwhdr->hdr_length, fwhdr->header_crc32);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_header_crc() failed\n");
			return r;
		}
	}

	if (fwhdr->const_crc32 == 0 && crc_check_allow_none == 1) {
		SCSC_TAG_INFO(MXMAN, "Skipping CRC check const_crc32=%d crc_check_allow_none=%d\n",
			      fwhdr->const_crc32, crc_check_allow_none);
	} else {
		/*
		 * CRC-32-IEEE over the constant sections grouped together at start of firmware binary.
		 * This CRC should remain valid during execution. It can be used by run-time checker on
		 * host to detect firmware corruption (not all memory masters are subject to MPUs).
		 */
		r = fwimage_check_fw_const_section_crc(fw, fwhdr->const_crc32, fwhdr->const_fw_length, fwhdr->hdr_length);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_const_section_crc() failed\n");
			return r;
		}
	}

	if (crc32_over_binary) {
		if (fwhdr->fw_crc32 == 0 && crc_check_allow_none == 1)
			SCSC_TAG_INFO(MXMAN, "Skipping CRC check fw_crc32=%d crc_check_allow_none=%d\n",
				      fwhdr->fw_crc32, crc_check_allow_none);
		else {
			/*
			 * CRC-32-IEEE over the firmware binary (i.e. everything
			 * in the file following this header).
			 * This includes initialised data regions so it can be used to
			 * check when loading but will not be meaningful once execution starts.
			 */
			r = fwimage_check_fw_crc(fw, fw_image_size, fwhdr->hdr_length, fwhdr->fw_crc32);
			if (r) {
				SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_crc() failed\n");
				return r;
			}
		}
	}

	return 0;
}

static void fw_crc_wq_start(struct mxman *mxman)
{
	if (mxman->check_crc && crc_check_period_ms)
		queue_delayed_work(mxman->fw_crc_wq, &mxman->fw_crc_work, msecs_to_jiffies(crc_check_period_ms));
}

static void fw_crc_work_func(struct work_struct *work)
{
	int          r;
	struct mxman *mxman = container_of((struct delayed_work *)work, struct mxman, fw_crc_work);

	r = do_fw_crc32_checks(mxman->fw, mxman->fw_image_size, &mxman->fwhdr, false);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "do_fw_crc32_checks() failed r=%d\n", r);
		mxman_fail(mxman, SCSC_PANIC_CODE_HOST << 15, __func__);
		return;
	}
	fw_crc_wq_start(mxman);
}

static void fw_crc_wq_init(struct mxman *mxman)
{
	mxman->fw_crc_wq = create_singlethread_workqueue("fw_crc_wq");
	INIT_DELAYED_WORK(&mxman->fw_crc_work, fw_crc_work_func);
}

static void fw_crc_wq_stop(struct mxman *mxman)
{
	mxman->check_crc = false;
	cancel_delayed_work(&mxman->fw_crc_work);
	flush_workqueue(mxman->fw_crc_wq);
}

static void fw_crc_wq_deinit(struct mxman *mxman)
{
	fw_crc_wq_stop(mxman);
	destroy_workqueue(mxman->fw_crc_wq);
}

static int transports_init(struct mxman *mxman)
{
	struct mxconf  *mxconf;
	int            r;
	struct scsc_mx *mx = mxman->mx;

	/* Initialise mx management stack */
	r = mxmgmt_transport_init(scsc_mx_get_mxmgmt_transport(mx), mx);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mxmgmt_transport_init() failed %d\n", r);
		return r;
	}

	/* Initialise gdb transport for cortex-R4 */
	r = gdb_transport_init(scsc_mx_get_gdb_transport_r4(mx), mx, GDB_TRANSPORT_R4);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "gdb_transport_init() failed %d\n", r);
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}

	/* Initialise gdb transport for cortex-M4 */
	r = gdb_transport_init(scsc_mx_get_gdb_transport_m4(mx), mx, GDB_TRANSPORT_M4);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "gdb_transport_init() failed %d\n", r);
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}

	/* Initialise mxlog transport */
	r = mxlog_transport_init(scsc_mx_get_mxlog_transport(mx), mx);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mxlog_transport_init() failed %d\n", r);
		gdb_transport_release(scsc_mx_get_gdb_transport_m4(mx));
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}

	/*
	 * Allocate & Initialise Infrastructre Config Structure
	 * including the mx management stack config information.
	 */
	mxconf = miframman_alloc(scsc_mx_get_ramman(mx), sizeof(struct mxconf), 4, MIFRAMMAN_OWNER_COMMON);
	if (!mxconf) {
		SCSC_TAG_ERR(MXMAN, "miframman_alloc() failed\n");
		gdb_transport_release(scsc_mx_get_gdb_transport_m4(mx));
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		mxlog_transport_release(scsc_mx_get_mxlog_transport(mx));
		return -ENOMEM;
	}
	mxman->mxconf = mxconf;
	mxconf->magic = MXCONF_MAGIC;
	mxconf->version.major = MXCONF_VERSION_MAJOR;

	mxconf->version.minor = MXCONF_VERSION_MINOR;
	/* Pass pre-existing FM status to FW */
	mxconf->flags = 0;
	SCSC_TAG_INFO(MXMAN, "mxconf flags 0x%08x\n", mxconf->flags);

	/* serialise mxmgmt transport */
	mxmgmt_transport_config_serialise(scsc_mx_get_mxmgmt_transport(mx), &mxconf->mx_trans_conf);
	/* serialise Cortex-R4 gdb transport */
	gdb_transport_config_serialise(scsc_mx_get_gdb_transport_r4(mx), &mxconf->mx_trans_conf_gdb_r4);
	/* serialise Cortex-M4 gdb transport */
	gdb_transport_config_serialise(scsc_mx_get_gdb_transport_m4(mx), &mxconf->mx_trans_conf_gdb_m4);

	/* Default to Fleximac M4_1 monitor channel not in use.
	 * Allows CONFIG_SCSC_MX450_GDB_SUPPORT to be turned off in Kconfig even though mxconf
	 * struct v5 defines M4_1 channel
	 */
	mxconf->mx_trans_conf_gdb_m4_1.from_ap_stream_conf.buf_conf.buffer_loc = 0;
	/* serialise mxlog transport */
	mxlog_transport_config_serialise(scsc_mx_get_mxlog_transport(mx), &mxconf->mxlogconf);
	SCSC_TAG_DEBUG(MXMAN, "read_bit_idx=%d write_bit_idx=%d buffer=%p num_packets=%d packet_size=%d read_index=%d write_index=%d\n",
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.read_bit_idx,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.write_bit_idx,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.buffer,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.num_packets,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.packet_size,
		       *scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.read_index,
		       *scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.write_index
		      );

	/* Need to initialise fwconfig or else random data can make firmware data abort. */
	mxconf->fwconfig.offset = 0;
	mxconf->fwconfig.size = 0;
	/* Load Common Config HCF */
	mxfwconfig_load(mxman->mx, &mxconf->fwconfig);
	return 0;
}

static void transports_release(struct mxman *mxman)
{
	mxlog_transport_release(scsc_mx_get_mxlog_transport(mxman->mx));
	mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mxman->mx));
	gdb_transport_release(scsc_mx_get_gdb_transport_r4(mxman->mx));
	gdb_transport_release(scsc_mx_get_gdb_transport_m4(mxman->mx));
	miframman_free(scsc_mx_get_ramman(mxman->mx), mxman->mxconf);
}

static void mbox_init(struct mxman *mxman, u32 firmware_entry_point)
{
	u32                 *mbox0;
	u32                 *mbox1;
	u32                 *mbox2;
	u32                 *mbox3;
	scsc_mifram_ref     mifram_ref;
	struct scsc_mx      *mx = mxman->mx;
	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);

	/* Place firmware entry address in MIF MBOX 0 so R4 ROM knows where to jump to! */
	mbox0 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_0);
	mbox1 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_1);

	/* Write (and flush) entry point to MailBox 0, config address to MBOX 1 */
	*mbox0 = firmware_entry_point;
	mif->get_mifram_ref(mif, mxman->mxconf, &mifram_ref);
	*mbox1 = mifram_ref; /* must be R4-relative address here */

	/*
	 * write the magic number "0xbcdeedcb" to MIF Mailbox #2 &
	 * copy the firmware_startup_flags to MIF Mailbox #3 before starting (reset = 0) the R4
	 */
	mbox2 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_2);
	*mbox2 = MBOX2_MAGIC_NUMBER;
	mbox3 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_3);
	*mbox3 = firmware_startup_flags;

	/* CPU memory barrier */
	smp_wmb();
}

static int fwhdr_init(char *fw, struct fwhdr *fwhdr, bool *fwhdr_parsed_ok, bool *check_crc)
{
	/*
	 * Validate the fw image including checking the firmware header, majic #, version, checksum  so on
	 * then do CRC on the entire image
	 *
	 * Derive some values from header -
	 *
	 * PORT: assumes little endian
	 */
	if (skip_header)
		*fwhdr_parsed_ok = false; /* Allows the forced start address to be used */
	else
		*fwhdr_parsed_ok = fwhdr_parse(fw, fwhdr);
	*check_crc = false;
	if (*fwhdr_parsed_ok) {
		SCSC_TAG_INFO(MXMAN, "FW HEADER version: hdr_major: %d hdr_minor: %d\n", fwhdr->hdr_major, fwhdr->hdr_minor);
		switch (fwhdr->hdr_major) {
		case 0:
			switch (fwhdr->hdr_minor) {
			case 2:
				*check_crc = true;
				break;
			default:
				SCSC_TAG_ERR(MXMAN, "Unsupported FW HEADER version: hdr_major: %d hdr_minor: %d\n",
					fwhdr->hdr_major, fwhdr->hdr_minor);
				return -EINVAL;
			}
			break;
		case 1:
			*check_crc = true;
			break;
		default:
			SCSC_TAG_ERR(MXMAN, "Unsupported FW HEADER version: hdr_major: %d hdr_minor: %d\n",
				fwhdr->hdr_major, fwhdr->hdr_minor);
			return -EINVAL;
		}
		switch (fwhdr->fwapi_major) {
		case 0:
			switch (fwhdr->fwapi_minor) {
			case 2:
				SCSC_TAG_INFO(MXMAN, "FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
					fwhdr->fwapi_major, fwhdr->fwapi_minor);
				break;
			default:
				SCSC_TAG_ERR(MXMAN, "Unsupported FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
					fwhdr->fwapi_major, fwhdr->fwapi_minor);
				return -EINVAL;
			}
			break;
		default:
			SCSC_TAG_ERR(MXMAN, "Unsupported FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
				fwhdr->fwapi_major, fwhdr->fwapi_minor);
			return -EINVAL;
		}
	} else {
		/* This is unidetified pre-header firmware - assume it is built to run at 0xb8000000 == 0 for bootrom */
		if (allow_unidentified_firmware) {
			SCSC_TAG_INFO(MXMAN, "Unidentified firmware override\n");
			fwhdr->firmware_entry_point = 0;
			fwhdr->fw_runtime_length = MX_FW_RUNTIME_LENGTH;
		} else {
			SCSC_TAG_ERR(MXMAN, "Unidentified firmware is not allowed\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int fw_init(struct mxman *mxman, void *start_dram, size_t size_dram, bool *fwhdr_parsed_ok)
{
	int                 r;
	char                *build_id;
	char                *ttid;
	u32                 fw_image_size;
	struct fwhdr        *fwhdr = &mxman->fwhdr;
	char                *fw = start_dram;

	r = mx140_file_download_fw(mxman->mx, start_dram, size_dram, &fw_image_size);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mx140_file_download_fw() failed (%d)\n", r);
		return r;
	}

	r = fwhdr_init(fw, fwhdr, fwhdr_parsed_ok, &mxman->check_crc);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "fwhdr_init() failed\n");
		return r;
	}
	mxman->fw = fw;
	mxman->fw_image_size = fw_image_size;
	if (mxman->check_crc) {
		/* do CRC on the entire image */
		r = do_fw_crc32_checks(fw, fw_image_size, &mxman->fwhdr, true);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "do_fw_crc32_checks() failed\n");
			return r;
		}
		fw_crc_wq_start(mxman);
	}

	if (*fwhdr_parsed_ok) {
		build_id = fwhdr_get_build_id(fw, fwhdr);
		if (build_id) {
			(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "%s", build_id);
			SCSC_TAG_INFO(MXMAN, "Firmware BUILD_ID: %s\n", mxman->fw_build_id);
			memcpy(saved_fw_build_id, mxman->fw_build_id,
			       sizeof(saved_fw_build_id));
		} else
			SCSC_TAG_ERR(MXMAN, "Failed to get Firmware BUILD_ID\n");

		ttid = fwhdr_get_ttid(fw, fwhdr);
		if (ttid) {
			(void)snprintf(mxman->fw_ttid, sizeof(mxman->fw_ttid), "%s", ttid);
			SCSC_TAG_INFO(MXMAN, "Firmware ttid: %s\n", mxman->fw_ttid);
		}
	}

	SCSC_TAG_DEBUG(MXMAN, "firmware_entry_point=0x%x fw_runtime_length=%d\n", fwhdr->firmware_entry_point, fwhdr->fw_runtime_length);

	return 0;

}


static int mxman_start(struct mxman *mxman)
{
	void                *start_dram;
	void                *start_dram_section2;
	size_t              size_dram = MX_DRAM_SIZE;
	struct scsc_mif_abs *mif;
	struct fwhdr        *fwhdr = &mxman->fwhdr;
	bool                fwhdr_parsed_ok;
	void                *start_mifram_heap;
	u32                 length_mifram_heap;
	u32                 length_mifram_heap2;
	int                 r;

	mif = scsc_mx_get_mif_abs(mxman->mx);
	if (mxman_check_reset_failed(mif)) {
		struct timespec64 tval = ns_to_timespec64(reset_failed_time);

		SCSC_TAG_ERR(MXMAN, "previous reset failed at [%6lu.%06ld], ignoring\n", (unsigned long)tval.tv_sec, tval.tv_nsec / NSEC_PER_USEC);
		return -EIO;
	}

	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");

	/* If the option is set to skip header, we must allow unidentified f/w */
	if (skip_header) {
		SCSC_TAG_INFO(MXMAN, "Ignoring firmware header block\n");
		allow_unidentified_firmware = true;
	}

	mif = scsc_mx_get_mif_abs(mxman->mx);
	start_dram = mif->map(mif, &size_dram);

	if (!start_dram) {
		SCSC_TAG_ERR(MXMAN, "Error allocating dram\n");
		return -ENOMEM;
	}

	SCSC_TAG_INFO(MXMAN, "Allocated %zu bytes\n", size_dram);


	mxman->start_dram = start_dram;

	r = fw_init(mxman, start_dram, size_dram, &fwhdr_parsed_ok);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "fw_init() failed\n");
		mif->unmap(mif, mxman->start_dram);
		return r;
	}

	/*
	 * The end of section 2 is reserved for the BT audio (ABOX) shared
	 * area. Bluetooth audio is not supported, but keep the reservation so
	 * that the memory layout matches what the firmware expects.
	 */
	length_mifram_heap2 = MX_DRAM_SIZE_SECTION_2 - MX_DRAM_SIZE_BT_AUDIO;

	start_dram_section2 = (char *)start_dram + MX_DRAM_SIZE_SECTION_1;
	miframman_init(scsc_mx_get_ramman2(mxman->mx),
		start_dram_section2,
		length_mifram_heap2,
		start_dram_section2);
	/* set up memory protection (read only) from start_dram to start_dram+fw_length
	 * rounding up the size if required
	 */
	start_mifram_heap = (char *)start_dram + fwhdr->fw_runtime_length;
	length_mifram_heap = MX_DRAM_SIZE_SECTION_1 - fwhdr->fw_runtime_length;

	miframman_init(scsc_mx_get_ramman(mxman->mx), start_mifram_heap, length_mifram_heap, start_dram);
	mifmboxman_init(scsc_mx_get_mboxman(mxman->mx));
	mifintrbit_init(scsc_mx_get_intrbit(mxman->mx), mif);
	mxfwconfig_init(mxman->mx);

	/* Initialise transports */
	r = transports_init(mxman);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "transports_init() failed\n");
		fw_crc_wq_stop(mxman);
		mifintrbit_deinit(scsc_mx_get_intrbit(mxman->mx));
		miframman_deinit(scsc_mx_get_ramman(mxman->mx));
		miframman_deinit(scsc_mx_get_ramman2(mxman->mx));
		mifmboxman_deinit(scsc_mx_get_mboxman(mxman->mx));
		/* Release the MIF memory resources */
		mif->unmap(mif, mxman->start_dram);
		return r;
	}
	mif->recovery_disabled_reg(mif, mxman_recovery_disabled);
	mbox_init(mxman, fwhdr->firmware_entry_point);
	init_completion(&mxman->mm_msg_start_ind_completion);
	init_completion(&mxman->mm_msg_halt_rsp_completion);
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mxman->mx),
						MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						&mxman_message_handler, mxman);

	mxlog_init(scsc_mx_get_mxlog(mxman->mx), mxman->mx, mxman->fw_build_id);
	panicmon_init(scsc_mx_get_panicmon(mxman->mx), mxman->mx);

	/* Change state to STARTING to allow coredump as we come out of reset */
	mxman->mxman_state = MXMAN_STATE_STARTING;

	/* release Maxwell from reset */

	r = mif->reset(mif, 0);
	if (r) {
		mxman_set_reset_failed();
		SCSC_TAG_INFO(MXMAN, "HW reset deassertion failed\n");

		/* Save log at point of failure */
		mx140_log_dump();
	}
	if (fwhdr_parsed_ok) {
		r = wait_for_mm_msg_start_ind(mxman);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "wait_for_MM_START_IND() failed: r=%d\n", r);
			print_mailboxes(mxman);
			if (skip_mbox0_check) {
				SCSC_TAG_ERR(MXMAN, "timeout ignored in skip_mbox0_check mode\n");
				return 0;
			}
			mxman_stop(mxman);
			return r;
		}
	} else {
		msleep(WAIT_FOR_FW_TO_START_DELAY_MS);
	}

	return 0;
}

static void print_panic_code_legacy(u16 code)
{
	u16 tech = code & SCSC_PANIC_TECH_MASK;
	u16 origin = code & SCSC_PANIC_ORIGIN_MASK;

	SCSC_TAG_INFO(MXMAN, "Decoding panic code=0x%x:\n", code);
	switch (origin) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic origin\n");
		break;
	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_ORIGIN_FW\n");
		break;
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_ORIGIN_HOST\n");
		break;
	}

	switch (tech) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic technology\n");
		break;
	case SCSC_PANIC_TECH_WLAN:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_WLAN\n");
		break;
	case SCSC_PANIC_TECH_CORE:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_CORE\n");
		break;
	case SCSC_PANIC_TECH_BT:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_BT\n");
		break;
	case SCSC_PANIC_TECH_UNSP:
		SCSC_TAG_INFO(MXMAN, "PANIC_TECH_UNSP\n");
		break;
	}
	SCSC_TAG_INFO(MXMAN, "panic subcode=0x%x\n", code & SCSC_PANIC_SUBCODE_MASK_LEGACY);
}

static void print_panic_code(u16 code)
{
	u16 origin = code & SCSC_PANIC_ORIGIN_MASK;	/* Panic origin (host/fw) */
	u16 subcode = code & SCSC_PANIC_SUBCODE_MASK;	/* The panic code */

	SCSC_TAG_INFO(MXMAN, "Decoding panic code=0x%x:\n", code);
	SCSC_TAG_INFO(MXMAN, "panic subcode=0x%x\n", code & SCSC_PANIC_SUBCODE_MASK);

	switch (origin) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic origin\n");
		break;
	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "WLBT FW PANIC: 0x%02x\n", subcode);
		break;
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "WLBT HOST detected FW failure, service:\n");
		switch (subcode >> SCSC_SYSERR_HOST_SERVICE_SHIFT) {
		case SCSC_SERVICE_ID_WLAN:
			SCSC_TAG_INFO(MXMAN, " WLAN\n");
			break;
		case SCSC_SERVICE_ID_BT:
			SCSC_TAG_INFO(MXMAN, " BT\n");
			break;
		case SCSC_SERVICE_ID_ANT:
			SCSC_TAG_INFO(MXMAN, " ANT\n");
			break;
		case SCSC_SERVICE_ID_CLK20MHZ:
			SCSC_TAG_INFO(MXMAN, " CLK20MHZ\n");
			break;
		default:
			SCSC_TAG_INFO(MXMAN, " Service 0x%x\n", subcode);
			break;
		}
		break;
	}
}

/**
 * Print the last panic record collected to aid in post mortem.
 *
 * Helps when all we have is kernel log showing WLBT failed some time ago
 *
 * Only prints the R4 record
 */
void mxman_show_last_panic(struct mxman *mxman)
{
	u32 r4_panic_record_length = 0;	/* in u32s */
	u32 r4_panic_stack_record_length = 0;	/* in u32s */

	/* Any valid panic? */
	if (mxman->scsc_panic_code == 0)
		return;

	SCSC_TAG_INFO(MXMAN, "\n\n--- DETAILS OF LAST WLBT FAILURE ---\n\n");

	switch (mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK) {
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "Last panic was host induced:\n");
		break;

	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "Last panic was FW:\n");
		fw_parse_r4_panic_record(mxman->last_panic_rec_r, &r4_panic_record_length, NULL, true);
		fw_parse_r4_panic_stack_record(mxman->last_panic_stack_rec_r, &r4_panic_stack_record_length, true);
		break;

	default:
		SCSC_TAG_INFO(MXMAN, "Last panic unknown origin %d\n", mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK);
		break;
	}

	print_panic_code(mxman->scsc_panic_code);

	SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n", mxman->failure_reason[0] ? mxman->failure_reason : "<null>");
	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", disable_recovery_handling ? "off" : "on");

	if (mxman_recovery_disabled()) {
		/* Labour the point that a reboot is needed when autorecovery is disabled */
		SCSC_TAG_INFO(MXMAN, "\n\n*** HANDSET REBOOT NEEDED TO RESTART WLAN AND BT ***\n\n");
	}

	SCSC_TAG_INFO(MXMAN, "\n\n--- END DETAILS OF LAST WLBT FAILURE ---\n\n");
}

static void process_panic_record(struct mxman *mxman, bool dump)
{
	u32 *r4_panic_record = NULL;
	u32 *r4_panic_stack_record = NULL;
	u32 *m4_panic_record = NULL;
	u32 r4_panic_record_length = 0;	/* in u32s */
	u32 r4_panic_stack_record_offset = 0; /* in bytes */
	u32 r4_panic_stack_record_length = 0;	/* in u32s */
	u32 m4_panic_record_length = 0; /* in u32s */
	u32 full_panic_code = 0;
	bool r4_panic_record_ok = false;
	bool r4_panic_stack_record_ok = false;
	bool m4_panic_record_ok = false;
	bool r4_sympathetic_panic_flag = false;
	bool m4_sympathetic_panic_flag = false;

	/* some configurable delay before accessing the panic record */
	msleep(panic_record_delay);

	/*
	* Check if the panic was trigered by MX and set the subcode if so.
	*/
	if ((mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK) == SCSC_PANIC_ORIGIN_FW) {
		if (mxman->fwhdr.r4_panic_record_offset) {
			r4_panic_record = (u32 *)(mxman->fw + mxman->fwhdr.r4_panic_record_offset);
			r4_panic_record_ok = fw_parse_r4_panic_record(r4_panic_record, &r4_panic_record_length,
								      &r4_panic_stack_record_offset, dump);
		} else {
			SCSC_TAG_INFO(MXMAN, "R4 panic record doesn't exist in the firmware header\n");
		}
		if (mxman->fwhdr.m4_panic_record_offset) {
			m4_panic_record = (u32 *)(mxman->fw + mxman->fwhdr.m4_panic_record_offset);
			m4_panic_record_ok = fw_parse_m4_panic_record(m4_panic_record, &m4_panic_record_length, dump);
		} else {
			SCSC_TAG_INFO(MXMAN, "M4 panic record doesn't exist in the firmware header\n");
		}

		/* Extract and print the panic code */
		switch (r4_panic_record_length) {
		default:
			SCSC_TAG_WARNING(MXMAN, "Bad panic record length/subversion\n");
			break;
		case SCSC_R4_V2_MINOR_52:
			if (r4_panic_record_ok) {
				full_panic_code = r4_panic_record[2];
				mxman->scsc_panic_code |= SCSC_PANIC_CODE_MASK & full_panic_code;
			} else if (m4_panic_record_ok)
				mxman->scsc_panic_code |= SCSC_PANIC_CODE_MASK & m4_panic_record[2];
			/* Set unspecified technology for now */
			mxman->scsc_panic_code |= SCSC_PANIC_TECH_UNSP;
			print_panic_code_legacy(mxman->scsc_panic_code);
			break;
		case SCSC_R4_V2_MINOR_54:
		case SCSC_R4_V2_MINOR_53:
			if (r4_panic_record_ok) {
				/* Save the last R4 panic record for future display */
				BUG_ON(sizeof(mxman->last_panic_rec_r) < r4_panic_record_length * sizeof(u32));
				memcpy((u8 *)mxman->last_panic_rec_r, (u8 *)r4_panic_record, r4_panic_record_length * sizeof(u32));
				mxman->last_panic_rec_sz = r4_panic_record_length;

				r4_sympathetic_panic_flag = fw_parse_get_r4_sympathetic_panic_flag(r4_panic_record);
				if (dump)
					SCSC_TAG_INFO(MXMAN, "r4_panic_record_ok=%d r4_sympathetic_panic_flag=%d\n",
							r4_panic_record_ok,
							r4_sympathetic_panic_flag);
				/* Check panic stack if present */
				if (r4_panic_record_length >= SCSC_R4_V2_MINOR_54) {
					r4_panic_stack_record = (u32 *)(mxman->fw + r4_panic_stack_record_offset);
					r4_panic_stack_record_ok = fw_parse_r4_panic_stack_record(r4_panic_stack_record, &r4_panic_stack_record_length, dump);
				} else {
					r4_panic_stack_record_ok = false;
					r4_panic_stack_record_length = 0;
				}
				if (r4_sympathetic_panic_flag == false) {
					/* process R4 record */
					full_panic_code = r4_panic_record[3];
					mxman->scsc_panic_code |= SCSC_PANIC_CODE_MASK & full_panic_code;
					if (dump)
						print_panic_code(mxman->scsc_panic_code);
					break;
				}
			}
			if (m4_panic_record_ok) {
				m4_sympathetic_panic_flag = fw_parse_get_m4_sympathetic_panic_flag(m4_panic_record);
				if (dump)
					SCSC_TAG_INFO(MXMAN, "m4_panic_record_ok=%d m4_sympathetic_panic_flag=%d\n",
							m4_panic_record_ok,
							m4_sympathetic_panic_flag);
				if (m4_sympathetic_panic_flag == false) {
					/* process M4 record */
					mxman->scsc_panic_code |= SCSC_PANIC_CODE_MASK & m4_panic_record[3];
				} else if (r4_panic_record_ok) {
					/* process R4 record */
					mxman->scsc_panic_code |= SCSC_PANIC_CODE_MASK & r4_panic_record[3];
				}
				if (dump)
					print_panic_code(mxman->scsc_panic_code);
			}
			break;
		}
	}
	if (r4_panic_record_ok) {
		/* Populate syserr info with panic equivalent, but don't modify level  */
		mxman->last_syserr.subsys = (u8) ((full_panic_code >> SYSERR_SUB_SYSTEM_POSN) & SYSERR_SUB_SYSTEM_MASK);
		mxman->last_syserr.type = (u8) ((full_panic_code >> SYSERR_TYPE_POSN) & SYSERR_TYPE_MASK);
		mxman->last_syserr.subcode = (u16) ((full_panic_code >> SYSERR_SUB_CODE_POSN) & SYSERR_SUB_CODE_MASK);
	}
}

/* Check whether syserr should be promoted based on frequency or service driver override */
static void mxman_check_promote_syserr(struct mxman *mxman)
{
	int i;
	int entry = -1;
	unsigned long now = jiffies;

	/* We use 0 as a NULL timestamp so avoid this */
	now = (now) ? now : 1;

	/* Promote all L7 to L8 to maintain existing moredump scheme,
	 * unless code is found in the filter list
	 */
	if (mxman->last_syserr.level == MX_SYSERR_LEVEL_7) {
		u8 new_level = MX_SYSERR_LEVEL_7;
		for (i = 0; i < ARRAY_SIZE(mxfwconfig_syserr_no_promote); i++) {
			/* End of list reached without match, promote to L8 by default */
			if (mxfwconfig_syserr_no_promote[i] == 0) {
				new_level = MX_SYSERR_LEVEL_8;
				entry = i;
				break;
			}

			/* If 0xFFFFFFFF in list: only if host induced, promote to L8 */
			if (mxfwconfig_syserr_no_promote[i] == 0xFFFFFFFF) {
				if ((mxman->last_syserr.subsys == SYSERR_SUB_SYSTEM_HOST || mxman->last_syserr.subcode == 0xF0)) {
					/* Host induced so promote */
					new_level = MX_SYSERR_LEVEL_8;
				}
				entry = i;
				break;
			}

			/* If code is in list, don't promote. Note that subsequent loop
			 * detection checks may promote later, though.
			 */
			if (mxfwconfig_syserr_no_promote[i] == mxman->last_syserr.subcode) {
				entry = i;
				break;
			}
		}

		SCSC_TAG_INFO(MXMAN, "entry %d = 0x%x: syserr in %d, subcode 0x%0x: L%d -> L%d\n",
			      entry,
			      (entry != -1) ? mxfwconfig_syserr_no_promote[entry] : 0,
			      mxman->last_syserr.subsys,
			      mxman->last_syserr.subcode,
			      mxman->last_syserr.level,
			      new_level);

		mxman->last_syserr.level = new_level;
	}

	/* last_syserr_level7_recovery_time is always zero-ed before we restart the chip */
	if (mxman->last_syserr_level7_recovery_time) {
		/* Have we had a too recent system error level 7 reset
		 * Chance of false positive here is low enough to be acceptable
		 */
		if ((syserr_level7_min_interval) && (time_in_range(now, mxman->last_syserr_level7_recovery_time,
				mxman->last_syserr_level7_recovery_time + msecs_to_jiffies(syserr_level7_min_interval)))) {

			SCSC_TAG_INFO(MXMAN, "Level 7 failure raised to level 8 (less than %dms after last)\n",
				syserr_level7_min_interval);
			mxman->last_syserr.level = MX_SYSERR_LEVEL_8;
		} else if (syserr_level7_monitor_period) {
			/* Have we had too many system error level 7 resets in one period? */
			/* This will be the case if all our stored history was in this period */
			bool out_of_danger_period_found = false;

			for (i = 0; (i < SYSERR_LEVEL7_HISTORY_SIZE) && (!out_of_danger_period_found); i++)
				out_of_danger_period_found = ((!syserr_level7_history[i]) ||
						      (!time_in_range(now, syserr_level7_history[i],
							syserr_level7_history[i] + msecs_to_jiffies(syserr_level7_monitor_period))));

			if (!out_of_danger_period_found) {
				SCSC_TAG_INFO(MXMAN, "Level 7 failure raised to level 8 (too many within %dms)\n",
					syserr_level7_monitor_period);
				mxman->last_syserr.level = MX_SYSERR_LEVEL_8;
			}
		}
	} else
		/* First syserr level 7 reset since chip was (re)started - zap history */
		for (i = 0; i < SYSERR_LEVEL7_HISTORY_SIZE; i++)
			syserr_level7_history[i] = 0;

	if ((mxman->last_syserr.level != MX_SYSERR_LEVEL_8) && (trigger_moredump_level > MX_SYSERR_LEVEL_7)) {
		/* Allow services to raise to level 8 */
		mxman->last_syserr.level = srvman_notify_services(scsc_mx_get_srvman(mxman->mx), &mxman->last_syserr);
	}

	if (mxman->last_syserr.level != MX_SYSERR_LEVEL_8) {
		/* Log this in our history */
		syserr_level7_history[syserr_level7_history_index++ % SYSERR_LEVEL7_HISTORY_SIZE] = now;
		mxman->last_syserr_level7_recovery_time = now;
	}
}

#define MAX_UHELP_TMO_MS	20000
/*
 * workqueue thread
 */
static void mxman_failure_work(struct work_struct *work)
{
	struct mxman  *mxman = container_of(work, struct mxman, failure_work);
	struct srvman *srvman;
	struct scsc_mx *mx = mxman->mx;
	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);

	wake_lock(&mxman->failure_recovery_wake_lock);
	/* Take mutex shared with syserr recovery */
	mutex_lock(&mxman->mxman_recovery_mutex);

	/* Check panic code for error promotion early on.
	 * Attempt to parse the panic record, to get the panic ID. This will
	 * only succeed for FW induced panics. Later we'll try again and dump.
	 */
	process_panic_record(mxman, false); /* check but don't dump */
	mxman_check_promote_syserr(mxman);

	SCSC_TAG_INFO(MXMAN, "This syserr level %d. Triggering moredump at level %d\n",
		mxman->last_syserr.level, trigger_moredump_level);

	if (mxman->last_syserr.level >= trigger_moredump_level) {
		/* Mark as level 8 as services neeed to know this has happened */
		if (mxman->last_syserr.level < MX_SYSERR_LEVEL_8) {
			mxman->last_syserr.level = MX_SYSERR_LEVEL_8;
			SCSC_TAG_INFO(MXMAN, "Syserr level raised to 8\n");
		}
	}

	blocking_notifier_call_chain(&firmware_chain, SCSC_FW_EVENT_FAILURE, NULL);

	SCSC_TAG_INFO(MXMAN, "Complete mm_msg_start_ind_completion\n");
	complete(&mxman->mm_msg_start_ind_completion);
	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);

	if (mxman->mxman_state != MXMAN_STATE_STARTED && mxman->mxman_state != MXMAN_STATE_STARTING) {
		SCSC_TAG_WARNING(MXMAN, "Not in started state: mxman->mxman_state=%d\n", mxman->mxman_state);
		wake_unlock(&mxman->failure_recovery_wake_lock);
		mutex_unlock(&mxman->mxman_mutex);
		mutex_unlock(&mxman->mxman_recovery_mutex);
		return;
	}

	/**
	 * Set error on mxlog and unregister mxlog msg-handlers.
	 * mxlog ISR and kthread will ignore further messages
	 * but mxlog_thread is NOT stopped here.
	 */
	mxlog_transport_set_error(scsc_mx_get_mxlog_transport(mx));
	mxlog_release(scsc_mx_get_mxlog(mx));
	/* unregister channel handler */
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						  NULL, NULL);
	mxmgmt_transport_set_error(scsc_mx_get_mxmgmt_transport(mx));
	srvman_set_error_complete(srvman, NOT_ALLOWED_START_STOP);
	fw_crc_wq_stop(mxman);

	mxman->mxman_state = mxman->mxman_next_state;

	/* Mark any single service recovery as no longer in progress */
	mxman->syserr_recovery_in_progress = false;
	mxman->last_syserr_recovery_time = 0;

	if (mxman->mxman_state != MXMAN_STATE_FAILED
	    && mxman->mxman_state != MXMAN_STATE_FROZEN) {
		WARN_ON(mxman->mxman_state != MXMAN_STATE_FAILED
			&& mxman->mxman_state != MXMAN_STATE_FROZEN);
		SCSC_TAG_ERR(MXMAN, "Bad state=%d\n", mxman->mxman_state);
		wake_unlock(&mxman->failure_recovery_wake_lock);
		mutex_unlock(&mxman->mxman_mutex);
		mutex_unlock(&mxman->mxman_recovery_mutex);
		return;
	}
	/* Signal panic to r4 and m4 processors */
	SCSC_TAG_INFO(MXMAN, "Setting MIFINTRBIT_RESERVED_PANIC_R4\n");
	mif->irq_bit_set(mif, MIFINTRBIT_RESERVED_PANIC_R4, SCSC_MIF_ABS_TARGET_R4); /* SCSC_MIFINTR_TARGET_R4 */
	SCSC_TAG_INFO(MXMAN, "Setting MIFINTRBIT_RESERVED_PANIC_M4\n");
	mif->irq_bit_set(mif, MIFINTRBIT_RESERVED_PANIC_M4, SCSC_MIF_ABS_TARGET_M4); /* SCSC_MIFINTR_TARGET_M4 */
	srvman_freeze_services(srvman, &mxman->last_syserr);
	if (mxman->mxman_state == MXMAN_STATE_FAILED) {
		mxman->last_panic_time = local_clock();

		/* Process and dump panic record, which should be valid now even for host induced panic */
		process_panic_record(mxman, true);

		SCSC_TAG_INFO(MXMAN, "Trying to schedule coredump\n");
		SCSC_TAG_INFO(MXMAN, "scsc_release %d.%d.%d.%d.%d\n",
			SCSC_RELEASE_PRODUCT,
			SCSC_RELEASE_ITERATION,
			SCSC_RELEASE_CANDIDATE,
			SCSC_RELEASE_POINT,
			SCSC_RELEASE_CUSTOMER);
		SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", mxman_recovery_disabled() ? "off" : "on");

		if (mxman->last_syserr.level != MX_SYSERR_LEVEL_8) {
			/* schedule system error and wait for it to finish */
		} else {
			/* Reset level 7 loop protection */
			mxman->last_syserr_level7_recovery_time = 0;

			/* No coredump helper: log the panic record instead */
			print_panic_code(mxman->scsc_panic_code);
			SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n",
				      mxman->failure_reason[0] ? mxman->failure_reason : "<null>");
		}

		/* Clean up the MIF following error handling */
		if (mif->mif_cleanup && mxman_recovery_disabled())
			mif->mif_cleanup(mif);
	}

	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n",
		mxman_recovery_disabled() ? "off" : "on");

	if (!mxman_recovery_disabled())
		srvman_set_error(srvman, NOT_ALLOWED_START);
	mutex_unlock(&mxman->mxman_mutex);
	if (!mxman_recovery_disabled()) {
		SCSC_TAG_INFO(MXMAN, "Calling srvman_unfreeze_services\n");
		srvman_unfreeze_services(srvman, &mxman->last_syserr);
		if (scsc_mx_module_reset() < 0)
			SCSC_TAG_INFO(MXMAN, "failed to call scsc_mx_module_reset\n");
		srvman_set_error(srvman, ALLOWED_START_STOP);
		atomic_inc(&mxman->recovery_count);
	}

	/**
	 * If recovery is disabled and an scsc_mx_service_open has been hold up,
	 * release it, rather than wait for the recovery_completion to timeout.
	 */
	if (mxman_recovery_disabled())
		complete(&mxman->recovery_completion);

	/* Safe to allow syserr recovery thread to run */
	mutex_unlock(&mxman->mxman_recovery_mutex);

	wake_unlock(&mxman->failure_recovery_wake_lock);
}

static void failure_wq_init(struct mxman *mxman)
{
	mxman->failure_wq = create_singlethread_workqueue("failure_wq");
	INIT_WORK(&mxman->failure_work, mxman_failure_work);
}

static void failure_wq_stop(struct mxman *mxman)
{
	cancel_work_sync(&mxman->failure_work);
	flush_workqueue(mxman->failure_wq);
}

static void failure_wq_deinit(struct mxman *mxman)
{
	failure_wq_stop(mxman);
	destroy_workqueue(mxman->failure_wq);
}

static void failure_wq_start(struct mxman *mxman)
{
	if (disable_error_handling)
		SCSC_TAG_INFO(MXMAN, "error handling disabled\n");
	else
		queue_work(mxman->failure_wq, &mxman->failure_work);
}

/*
 * workqueue thread
 */
static void mxman_syserr_recovery_work(struct work_struct *work)
{
	struct mxman  *mxman = container_of(work, struct mxman, syserr_recovery_work);
	struct srvman *srvman;

	wake_lock(&mxman->syserr_recovery_wake_lock);
	if (!mutex_trylock(&mxman->mxman_recovery_mutex)) {
		SCSC_TAG_WARNING(MXMAN, "Syserr during full reset - ignored\n");
		wake_unlock(&mxman->syserr_recovery_wake_lock);
		return;
	}

	mutex_lock(&mxman->mxman_mutex);

	if (mxman->mxman_state != MXMAN_STATE_STARTED && mxman->mxman_state != MXMAN_STATE_STARTING) {
		SCSC_TAG_WARNING(MXMAN, "Syserr reset ignored: mxman->mxman_state=%d\n", mxman->mxman_state);
		wake_unlock(&mxman->syserr_recovery_wake_lock);
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}

	srvman = scsc_mx_get_srvman(mxman->mx);

	srvman_freeze_sub_system(srvman, &mxman->last_syserr);


	srvman_unfreeze_sub_system(srvman, &mxman->last_syserr);

	wake_unlock(&mxman->syserr_recovery_wake_lock);
	mutex_unlock(&mxman->mxman_recovery_mutex);
	mutex_unlock(&mxman->mxman_mutex);
}

static void syserr_recovery_wq_init(struct mxman *mxman)
{
	mxman->syserr_recovery_wq = create_singlethread_workqueue("syserr_recovery_wq");
	INIT_WORK(&mxman->syserr_recovery_work, mxman_syserr_recovery_work);
}

static void syserr_recovery_wq_stop(struct mxman *mxman)
{
	cancel_work_sync(&mxman->syserr_recovery_work);
	flush_workqueue(mxman->syserr_recovery_wq);
}

static void syserr_recovery_wq_deinit(struct mxman *mxman)
{
	syserr_recovery_wq_stop(mxman);
	destroy_workqueue(mxman->syserr_recovery_wq);
}

static void syserr_recovery_wq_start(struct mxman *mxman)
{
	queue_work(mxman->syserr_recovery_wq, &mxman->syserr_recovery_work);
}

static void print_mailboxes(struct mxman *mxman)
{
	struct scsc_mif_abs *mif;
	struct mifmboxman   *mboxman;
	int                 i;

	mif = scsc_mx_get_mif_abs(mxman->mx);
	mboxman = scsc_mx_get_mboxman(mxman->mx);

	SCSC_TAG_INFO(MXMAN, "Printing mailbox values:\n");
	for (i = 0; i < MIFMBOX_NUM; i++)
		SCSC_TAG_INFO(MXMAN, "MBOX_%d: 0x%x\n", i, *mifmboxman_get_mbox_ptr(mboxman, mif, i));
}


/*
 * Check for matching f/w and h/w
 *
 * Returns	0:  f/w and h/w match
 *		1:  f/w and h/w mismatch, try the next config
 *		-ve fatal error
 */
static int mxman_hw_ver_check(struct mxman *mxman)
{
	if (mx140_file_supported_hw(mxman->mx, mxman->rf_hw_ver))
		return 0;
	else
		return 1;
}

/*
 * Select the f/w version to load next
 */
static int mxman_select_next_fw(struct mxman *mxman)
{
	return mx140_file_select_fw(mxman->mx, mxman->rf_hw_ver);
}

/* Boot MX140 with given f/w */
static int __mxman_open(struct mxman *mxman)
{
	int r;
	struct srvman *srvman;

	mutex_lock(&mxman->mxman_mutex);
	if (mxman->scsc_panic_code) {
		SCSC_TAG_INFO(MXMAN, "Previously recorded crash panic code: scsc_panic_code=0x%x\n", mxman->scsc_panic_code);
		SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n", mxman->failure_reason[0] ? mxman->failure_reason : "<null>");
		print_panic_code(mxman->scsc_panic_code);
	}
	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", mxman_recovery_disabled() ? "off" : "on");
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman_in_error(srvman)) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return -EINVAL;
	}

	/* Reset the state after a previous crash during f/w boot */
	if (mxman->mxman_state == MXMAN_STATE_STARTING)
		mxman->mxman_state = MXMAN_STATE_STOPPED;

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		/* if in the STARTED state there MUST already be some users */
		if (WARN_ON(!mxman->users)) {
			SCSC_TAG_ERR(MXMAN, "ERROR mxman->mxman_state=%d users=%d\n", mxman->mxman_state, mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return -EINVAL;
		}
		mxman->users++;
		SCSC_TAG_INFO(MXMAN, "Already opened: users=%d\n", mxman->users);
		mxman_print_versions(mxman);
		mutex_unlock(&mxman->mxman_mutex);
		return 0;
	} else if (mxman->mxman_state == MXMAN_STATE_STOPPED) {
		r = mxman_start(mxman);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "maxwell_manager_start() failed r=%d users=%d\n", r, mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return r;
		}
		mxman->users++;
		mxman->mxman_state = MXMAN_STATE_STARTED;
		mutex_unlock(&mxman->mxman_mutex);
		return 0;
	}
	WARN_ON(mxman->mxman_state != MXMAN_STATE_STARTED && mxman->mxman_state != MXMAN_STATE_STOPPED);
	SCSC_TAG_ERR(MXMAN, "Bad state: mxman->mxman_state=%d\n", mxman->mxman_state);
	mutex_unlock(&mxman->mxman_mutex);
	return -EIO;
}

int mxman_open(struct mxman *mxman)
{
	int r;
	int try = 0;

	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);

	for (try = 0; try < 2; try++) {
		/* Boot WLBT. This will determine the h/w version */
		r = __mxman_open(mxman);
		if (r)
			return r;

		/* On retries, restore USBPLL owner as WLBT */
		if (try > 0 && mif->mif_restart)
			mif->mif_restart(mif);

		/* Check the h/w and f/w versions are compatible */
		r = mxman_hw_ver_check(mxman);
		if (r > 0) {
			/* Not compatible, so try next f/w */
			SCSC_TAG_INFO(MXMAN, "Incompatible h/w 0x%04x vs f/w, close and try next\n", mxman->rf_hw_ver);

			/* Temporarily return USBPLL owner to AP to keep USB alive */
			if (mif->mif_cleanup)
				mif->mif_cleanup(mif);

			/* Stop WLBT */
			mxman_close(mxman);

			/* Select the new f/w for this hw ver */
			mxman_select_next_fw(mxman);
		} else
			break; /* Running or given up */
	}


	return r;
}

static void mxman_stop(struct mxman *mxman)
{
	int r;
	struct scsc_mif_abs *mif;

	SCSC_TAG_INFO(MXMAN, "\n");

	mif = scsc_mx_get_mif_abs(mxman->mx);
	/* If reset is failed, prevent new resets */
	if (mxman_check_reset_failed(mif)) {
		struct timespec64 tval = ns_to_timespec64(reset_failed_time);

		SCSC_TAG_ERR(MXMAN, "previous reset failed at [%6lu.%06ld], ignoring\n", (unsigned long)tval.tv_sec, tval.tv_nsec / NSEC_PER_USEC);
		return;
	}

	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");


	/* Shutdown the hardware */
	mif = scsc_mx_get_mif_abs(mxman->mx);
	r = mif->reset(mif, 1);
	if (r) {
		reset_failed_time = local_clock();
		SCSC_TAG_INFO(MXMAN, "HW reset failed\n");
		mxman_set_reset_failed();

		/* Save log at point of failure */
		mx140_log_dump();
	}

	panicmon_deinit(scsc_mx_get_panicmon(mxman->mx));
	transports_release(mxman);
	mxfwconfig_unload(mxman->mx);

	mxlog_release(scsc_mx_get_mxlog(mxman->mx));
	/* unregister channel handler */
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						  NULL, NULL);
	fw_crc_wq_stop(mxman);

	/* Unitialise components (they may perform some checks - e.g. all memory freed) */
	mxfwconfig_deinit(mxman->mx);
	mifintrbit_deinit(scsc_mx_get_intrbit(mxman->mx));
	miframman_deinit(scsc_mx_get_ramman(mxman->mx));
	miframman_deinit(scsc_mx_get_ramman2(mxman->mx));
	mifmboxman_deinit(scsc_mx_get_mboxman(mxman->mx));
	mif->recovery_disabled_unreg(mif);
	/* Release the MIF memory resources */
	mif->unmap(mif, mxman->start_dram);
}

void mxman_close(struct mxman *mxman)
{
	int r;
	struct srvman *srvman;

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && !srvman_allow_close(srvman)) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error(%d) - ignore\n", srvman->error);
		return;
	}

	SCSC_TAG_INFO(MXMAN, "\n");

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		if (WARN_ON(!mxman->users)) {
			SCSC_TAG_ERR(MXMAN, "ERROR users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}
		mxman->users--;
		if (mxman->users) {
			SCSC_TAG_INFO(MXMAN, "Current number of users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}
		/*
		 * Ask the subsystem to stop (MM_STOP_REQ), and wait
		 * for response (MM_STOP_RSP).
		 */
		r = send_mm_msg_stop_blocking(mxman);
		if (r)
			SCSC_TAG_ERR(MXMAN, "send_mm_msg_stop_blocking failed: r=%d\n", r);

		mxman_stop(mxman);
		mxman->mxman_state = MXMAN_STATE_STOPPED;
		mutex_unlock(&mxman->mxman_mutex);
	} else if (mxman->mxman_state == MXMAN_STATE_FAILED) {
		if (WARN_ON(!mxman->users))
			SCSC_TAG_ERR(MXMAN, "ERROR users=%d\n", mxman->users);

		mxman->users--;
		if (mxman->users) {
			SCSC_TAG_INFO(MXMAN, "Current number of users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}

		mxman_stop(mxman);
		mxman->mxman_state = MXMAN_STATE_STOPPED;
		mutex_unlock(&mxman->mxman_mutex);
		complete(&mxman->recovery_completion);
	} else {
		WARN_ON(mxman->mxman_state != MXMAN_STATE_STARTED);
		SCSC_TAG_ERR(MXMAN, "Bad state: mxman->mxman_state=%d\n", mxman->mxman_state);
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}
}

void mxman_syserr(struct mxman *mxman, struct mx_syserr_decode *syserr)
{
	mxman->syserr_recovery_in_progress = true;

	mxman->last_syserr.subsys = syserr->subsys;
	mxman->last_syserr.level = syserr->level;
	mxman->last_syserr.type = syserr->type;
	mxman->last_syserr.subcode = syserr->subcode;

	syserr_recovery_wq_start(mxman);
}

void mxman_fail(struct mxman *mxman, u16 failure_source, const char *reason)
{
	SCSC_TAG_WARNING(MXMAN, "WLBT FW failure 0x%x\n", failure_source);

	/* For FW failure, scsc_panic_code is not set up fully until process_panic_record() checks it */
	if (disable_recovery_handling == DISABLE_RECOVERY_HANDLING_SCANDUMP) {
		/* Support not present, fallback to vanilla moredump and stop WLBT */
		disable_recovery_handling = 1;
		SCSC_TAG_WARNING(MXMAN, "WLBT FW failure - scandump requested but not supported in kernel\n");
	}

	/* The STARTING state allows a crash during firmware boot to be handled */
	if (mxman->mxman_state == MXMAN_STATE_STARTED || mxman->mxman_state == MXMAN_STATE_STARTING) {
		mxman->mxman_next_state = MXMAN_STATE_FAILED;
		mxman->scsc_panic_code = failure_source;
		strscpy(mxman->failure_reason, reason, sizeof(mxman->failure_reason));
		/* If recovery is disabled, don't let it be
		 * re-enabled from now on. Device must reboot
		 */
		if (mxman_recovery_disabled())
			disable_recovery_until_reboot  = true;

		/* Populate syserr info with panic equivalent or best we can */
		mxman->last_syserr.subsys = failure_source >> SYSERR_SUB_SYSTEM_POSN;
		mxman->last_syserr.level = MX_SYSERR_LEVEL_7;
		mxman->last_syserr.type = failure_source;
		mxman->last_syserr.subcode = failure_source;
		atomic_inc(&mxman->cancel_resume);
		failure_wq_start(mxman);
	} else {
		SCSC_TAG_WARNING(MXMAN, "Not in MXMAN_STATE_STARTED state, ignore (state %d)\n", mxman->mxman_state);
	}
}

void mxman_fail_level8(struct mxman *mxman, u16 failure_source, const char *reason)
{
	SCSC_TAG_WARNING(MXMAN, "WLBT FW level 8 failure 0x%0x\n", failure_source);

	/* The STARTING state allows a crash during firmware boot to be handled */
	if (mxman->mxman_state == MXMAN_STATE_STARTED || mxman->mxman_state == MXMAN_STATE_STARTING) {
		mxman->mxman_next_state = MXMAN_STATE_FAILED;
		mxman->scsc_panic_code = failure_source;
		strscpy(mxman->failure_reason, reason, sizeof(mxman->failure_reason));
		/* If recovery is disabled, don't let it be
		 * re-enabled from now on. Device must reboot
		 */
		if (mxman_recovery_disabled())
			disable_recovery_until_reboot  = true;

		/* Populate syserr info with panic equivalent or best we can */
		mxman->last_syserr.subsys = failure_source >> SYSERR_SUB_SYSTEM_POSN;
		mxman->last_syserr.level = MX_SYSERR_LEVEL_8;
		mxman->last_syserr.type = failure_source;
		mxman->last_syserr.subcode = failure_source;

		failure_wq_start(mxman);
	} else {
		SCSC_TAG_WARNING(MXMAN, "Not in MXMAN_STATE_STARTED state, ignore (state %d)\n", mxman->mxman_state);
	}
}

void mxman_freeze(struct mxman *mxman)
{
	SCSC_TAG_WARNING(MXMAN, "WLBT FW frozen\n");

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		mxman->mxman_next_state = MXMAN_STATE_FROZEN;
		failure_wq_start(mxman);
	} else {
		SCSC_TAG_WARNING(MXMAN, "Not in MXMAN_STATE_STARTED state, ignore (state %d)\n", mxman->mxman_state);
	}
}

void mxman_init(struct mxman *mxman, struct scsc_mx *mx)
{
	mxman->mx = mx;
	mxman->suspended = 0;
	fw_crc_wq_init(mxman);
	failure_wq_init(mxman);
	syserr_recovery_wq_init(mxman);
	mutex_init(&mxman->mxman_mutex);
	mutex_init(&mxman->mxman_recovery_mutex);
	init_completion(&mxman->recovery_completion);
	wake_lock_init(NULL, &mxman->failure_recovery_wake_lock.ws, "mxman_recovery");
        wake_lock_init(NULL, &mxman->syserr_recovery_wake_lock.ws, "mxman_syserr_recovery");
	mxman->last_syserr_level7_recovery_time = 0;

	atomic_set(&mxman->cancel_resume, 0);

	mxman->syserr_recovery_in_progress = false;
	mxman->last_syserr_recovery_time = 0;

	/* set the initial state */
	mxman->mxman_state = MXMAN_STATE_STOPPED;
	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");
	memcpy(saved_fw_build_id, mxman->fw_build_id,
	       sizeof(saved_fw_build_id));
	(void)snprintf(mxman->fw_ttid, sizeof(mxman->fw_ttid), "unknown");
	active_mxman = mxman;



}

void mxman_deinit(struct mxman *mxman)
{
	active_mxman = NULL;
	fw_crc_wq_deinit(mxman);
	failure_wq_deinit(mxman);
	syserr_recovery_wq_deinit(mxman);
	wake_lock_destroy(&mxman->failure_recovery_wake_lock);
	wake_lock_destroy(&mxman->syserr_recovery_wake_lock);
	mutex_destroy(&mxman->mxman_recovery_mutex);
	mutex_destroy(&mxman->mxman_mutex);
}

int mxman_force_panic(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_FORCE_PANIC };

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman_in_error(srvman)) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return -EINVAL;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mutex_unlock(&mxman->mxman_mutex);
		return 0;
	}
	mutex_unlock(&mxman->mxman_mutex);
	return -EINVAL;
}

int mxman_suspend(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_HOST_SUSPEND };
	int ret;

	SCSC_TAG_INFO(MXMAN, "\n");

	atomic_set(&mxman->cancel_resume, 0);
	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);

	if (srvman && srvman_in_error(srvman)) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return 0;
	}

	/* Call Service suspend callbacks */
	ret = srvman_suspend_services(srvman);
	if (ret) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Service Suspend canceled - ignore %d\n", ret);
		return ret;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		SCSC_TAG_INFO(MXMAN, "MM_HOST_SUSPEND\n");
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mxman->suspended = 1;
		atomic_inc(&mxman->suspend_count);
	}
	mutex_unlock(&mxman->mxman_mutex);
	return 0;
}


void mxman_resume(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_HOST_RESUME };
	int ret;

	SCSC_TAG_INFO(MXMAN, "\n");
	if (atomic_read(&mxman->cancel_resume)) {
		SCSC_TAG_INFO(MXMAN, "Recovery in progress ... ignoring");
		return;
	}

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman_in_error(srvman)) {
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		SCSC_TAG_INFO(MXMAN, "MM_HOST_RESUME\n");
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mxman->suspended = 0;
	}

	/* Call Service Resume callbacks */
	ret = srvman_resume_services(srvman);
	if (ret)
		SCSC_TAG_INFO(MXMAN, "Service Resume error %d\n", ret);

	mutex_unlock(&mxman->mxman_mutex);
}

int mx140_log_dump(void)
{
	return 0;
}
EXPORT_SYMBOL(mx140_log_dump);

bool mxman_recovery_disabled(void)
{
	/* If FW has panicked when recovery was disabled, don't allow it to
	 * be enabled. The horse has bolted.
	 */
	if (disable_recovery_until_reboot)
		return true;

	return disable_recovery_handling ? true : false;
}
EXPORT_SYMBOL(mxman_recovery_disabled);

/**
 * This returns the last known loaded FW build_id
 * even when the fw is NOT running at the time of the request.
 *
 * It could be used anytime by Android Enhanced Logging
 * to query for fw version.
 */
void mxman_get_fw_version(char *version, size_t ver_sz)
{
	/* unavailable only if chip not probed ! */
	snprintf(version, ver_sz, "%s", saved_fw_build_id);
}
EXPORT_SYMBOL(mxman_get_fw_version);

void mxman_get_driver_version(char *version, size_t ver_sz)
{
	/* IMPORTANT - Do not change the formatting as User space tooling is parsing the string
	* to read SAP fapi versions. */
	snprintf(version, ver_sz, "drv_ver: %u.%u.%u.%u.%u",
		 SCSC_RELEASE_PRODUCT, SCSC_RELEASE_ITERATION, SCSC_RELEASE_CANDIDATE, SCSC_RELEASE_POINT, SCSC_RELEASE_CUSTOMER);
}
EXPORT_SYMBOL(mxman_get_driver_version);

int mxman_register_firmware_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&firmware_chain, nb);
}
EXPORT_SYMBOL(mxman_register_firmware_notifier);

int mxman_unregister_firmware_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&firmware_chain, nb);
}
EXPORT_SYMBOL(mxman_unregister_firmware_notifier);


