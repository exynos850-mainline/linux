// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth HCI driver for the Samsung SCSC (Maxwell) WLBT subsystem
 *
 * Copyright (c) 2014 - 2016 Samsung Electronics Co., Ltd. All rights reserved
 *
 * The Bluetooth controller runs in the WLBT firmware as the BT service.
 * Commands, events and ACL data are exchanged with it through transfer
 * rings in the shared memory (BSMHCP, see scsc_shm.c), which the
 * downstream driver exposed as an H4 character device. Here the same H4
 * stream is bridged to an HCI device instead.
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/hex.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>
#include <scsc/scsc_logring.h>
#include <scsc/scsc_mx.h>

#include "scsc_bt_priv.h"

#define SLSI_BT_SERVICE_CLOSE_RETRY		60
#define SLSI_BT_SERVICE_STOP_RECOVERY_TIMEOUT	20000
#define SLSI_BT_SERVICE_STOP_RECOVERY_DISABLED_TIMEOUT 2000

/* Large enough for any event or ACL packet of the firmware, plus header */
#define SCSC_BT_RX_BUF_SIZE			2048

struct scsc_common_service common_service;
struct scsc_bt_service bt_service;

static u16 bt_module_irq_mask;
static DEFINE_MUTEX(bt_start_mutex);
static int recovery_timeout = SLSI_BT_SERVICE_STOP_RECOVERY_TIMEOUT;

static struct hci_dev *scsc_hdev;
static struct task_struct *rx_thread;
static u8 *rx_buf;
static size_t rx_fill;

/*
 * Bluetooth address handed to the firmware when the service starts. If not
 * set, the firmware uses its default address. Userspace may write it before
 * Bluetooth is first enabled, e.g. from the EFS bt_addr file.
 */
static char bd_addr[32];
module_param_string(bd_addr, bd_addr, sizeof(bd_addr), 0644);
MODULE_PARM_DESC(bd_addr, "Bluetooth address (xx:xx:xx:xx:xx:xx)");

static bool disable_service;
module_param(disable_service, bool, 0644);
MODULE_PARM_DESC(disable_service, "Disables service startup");

static u32 firmware_control;
module_param(firmware_control, uint, 0644);
MODULE_PARM_DESC(firmware_control, "Control how the firmware behaves");

/*
 * Service event callbacks called from mx-core when things go wrong
 */
static u8 bt_failure_notification(struct scsc_service_client *client, struct mx_syserr_decode *err)
{
	SCSC_TAG_INFO(BT_COMMON, "Error level %d\n", err->level);

	return err->level;
}

static bool bt_stop_on_failure(struct scsc_service_client *client, struct mx_syserr_decode *err)
{
	SCSC_TAG_ERR(BT_COMMON, "Error level %d\n", err->level);

	reinit_completion(&bt_service.recovery_probe_complete);
	bt_service.recovery_level = err->level;

	/* The reader reports a hardware error event to the stack */
	atomic_inc(&bt_service.error_count);

	return false;
}

static void bt_failure_reset(struct scsc_service_client *client, u8 level, u16 scsc_syserr_code)
{
	SCSC_TAG_ERR(BT_COMMON, "level %u, code 0x%04x\n", level, scsc_syserr_code);

	wake_up(&bt_service.read_wait);
}

static int bt_ap_resumed(struct scsc_service_client *client)
{
	if (bt_service.interrupt_count != bt_service.last_suspend_interrupt_count)
		SCSC_TAG_INFO(BT_COMMON, "Possible Bluetooth firmware wake up detected\n");

	return 0;
}

static int bt_ap_suspended(struct scsc_service_client *client)
{
	bt_service.last_suspend_interrupt_count = bt_service.interrupt_count;

	return 0;
}

static struct scsc_service_client mx_bt_client = {
	.failure_notification =	bt_failure_notification,
	.stop_on_failure_v2 =	bt_stop_on_failure,
	.failure_reset_v2 =	bt_failure_reset,
	.suspend =		bt_ap_suspended,
	.resume =		bt_ap_resumed,
};

/* Firmware information interrupt, only used by firmware debugging */
static void scsc_bt_info_irq_handler(int irqbit, void *data)
{
	scsc_service_mifintrbit_bit_clear(bt_service.service, irqbit);
	wake_up(&bt_service.info_wait);
}

static void slsi_sm_bt_service_cleanup_interrupts(void)
{
	if (bt_module_irq_mask & BIT(0))
		scsc_service_mifintrbit_unregister_tohost(bt_service.service,
			bt_service.bsmhcp_protocol->header.info_bg_to_ap_int_src);
	if (bt_module_irq_mask & BIT(1))
		scsc_service_mifintrbit_free_fromhost(bt_service.service,
			bt_service.bsmhcp_protocol->header.info_ap_to_bg_int_src,
			SCSC_MIFINTR_TARGET_R4);
	bt_module_irq_mask = 0;
}

static int slsi_sm_bt_service_init_interrupts(void)
{
	int irq;

	irq = scsc_service_mifintrbit_register_tohost(bt_service.service,
						      scsc_bt_info_irq_handler, NULL);
	if (irq < 0)
		return irq;
	bt_service.bsmhcp_protocol->header.info_bg_to_ap_int_src = irq;
	bt_module_irq_mask |= BIT(0);

	irq = scsc_service_mifintrbit_alloc_fromhost(bt_service.service,
						     SCSC_MIFINTR_TARGET_R4);
	if (irq < 0)
		return irq;
	bt_service.bsmhcp_protocol->header.info_ap_to_bg_int_src = irq;
	bt_module_irq_mask |= BIT(1);

	return 0;
}

static int slsi_sm_bt_service_cleanup_stop_service(void)
{
	int ret;

	/* Stop the service first, then the shared memory can be released */
	ret = scsc_mx_service_stop(bt_service.service);
	if (ret < 0 && ret != -EPERM) {
		SCSC_TAG_ERR(BT_COMMON, "scsc_mx_service_stop failed err: %d\n", ret);

		/* Trigger recovery unless it is already in progress */
		if (atomic_read(&bt_service.error_count) == 0 && ret != -EILSEQ) {
			scsc_mx_service_service_failed(bt_service.service, "BT service stop failed");
			return ret;
		}
	}

	return 0;
}

static bool scsc_recovery_in_progress(void)
{
	return bt_service.recovery_level != 0;
}

static int slsi_sm_bt_service_cleanup(void)
{
	int ret;
	int retry;

	if (bt_service.service) {
		/* If stopping fails, recovery does the rest later */
		if (bt_service.service_started) {
			ret = slsi_sm_bt_service_cleanup_stop_service();
			bt_service.service_started = false;
			if (ret < 0)
				return -EIO;
		}

		/* Make sure the reader leaves the shared memory */
		wake_up_interruptible(&bt_service.read_wait);

		if (bt_service.bsmhcp_protocol)
			slsi_sm_bt_service_cleanup_interrupts();

		scsc_bt_shm_exit();
		scsc_avdtp_detect_exit();

		if (bt_service.config_ref) {
			scsc_mx_service_mifram_free(bt_service.service, bt_service.config_ref);
			bt_service.config_ref = 0;
		}
		if (bt_service.bsmhcp_ref) {
			scsc_mx_service_mifram_free(bt_service.service, bt_service.bsmhcp_ref);
			bt_service.bsmhcp_ref = 0;
		}
		if (bt_service.bhcs_ref) {
			scsc_mx_service_mifram_free(bt_service.service, bt_service.bhcs_ref);
			bt_service.bhcs_ref = 0;
		}

		ret = scsc_mx_service_close(bt_service.service);
		/* Error handling in progress, retry for up to 30 seconds */
		for (retry = 0; ret < 0 && ret != -EPERM && retry < SLSI_BT_SERVICE_CLOSE_RETRY; retry++) {
			msleep(500);
			ret = scsc_mx_service_close(bt_service.service);
		}
		if (ret < 0 && ret != -EPERM)
			SCSC_TAG_ERR(BT_COMMON, "scsc_mx_service_close failed: %d\n", ret);

		bt_service.service = NULL;
	}

	atomic_set(&bt_service.error_count, 0);

	if (wake_lock_active(&bt_service.write_wake_lock)) {
		bt_service.write_wake_unlock_count++;
		wake_unlock(&bt_service.write_wake_lock);
	}

	return 0;
}

static const struct firmware *load_config(void)
{
	const struct firmware *firm;

	/* scsc/mx140_bt.hcf, optional */
	if (mx140_file_request_conf(common_service.maxwell_core, &firm, "bluetooth", SCSC_BT_CONF))
		return NULL;

	if (!firm->size) {
		mx140_file_release_conf(common_service.maxwell_core, firm);
		return NULL;
	}

	return firm;
}

static int setup_bhcs(struct scsc_service *service, struct BHCS *bhcs,
		      u32 protocol_ref, u32 protocol_length,
		      scsc_mifram_ref *config_ref)
{
	const struct firmware *firm;
	u8 addr[ETH_ALEN];
	void *conf_ptr;

	bhcs->version = BHCS_VERSION;
	bhcs->bsmhcp_protocol_offset = protocol_ref;
	bhcs->bsmhcp_protocol_length = protocol_length;
	bhcs->configuration_offset = 0;
	bhcs->configuration_length = 0;
	bhcs->bluetooth_address_lap = 0;
	bhcs->bluetooth_address_uap = 0;
	bhcs->bluetooth_address_nap = 0;
	*config_ref = 0;

	firm = load_config();
	if (firm) {
		if (scsc_mx_service_mifram_alloc(service, firm->size, config_ref,
						 BSMHCP_ALIGNMENT)) {
			mx140_file_release_conf(common_service.maxwell_core, firm);
			return -ENOMEM;
		}

		conf_ptr = scsc_mx_service_mif_addr_to_ptr(service, *config_ref);
		if (!conf_ptr) {
			mx140_file_release_conf(common_service.maxwell_core, firm);
			return -EINVAL;
		}

		memcpy(conf_ptr, firm->data, firm->size);
		bhcs->configuration_offset = *config_ref;
		bhcs->configuration_length = firm->size;
		mx140_file_release_conf(common_service.maxwell_core, firm);
	}

	if (bd_addr[0] && mac_pton(strim(bd_addr), addr)) {
		bhcs->bluetooth_address_nap = (addr[0] << 8) | addr[1];
		bhcs->bluetooth_address_uap = addr[2];
		bhcs->bluetooth_address_lap = (addr[3] << 16) | (addr[4] << 8) | addr[5];
		SCSC_TAG_INFO(BT_COMMON, "Bluetooth address %pMR\n", addr);
	}

	return 0;
}

/* Start the BT service */
static int slsi_sm_bt_service_start(void)
{
	struct BHCS *bhcs;
	int err = 0;

	mutex_lock(&bt_start_mutex);

	if (disable_service) {
		err = -EBUSY;
		goto out_unlock;
	}

	if (scsc_recovery_in_progress()) {
		SCSC_TAG_WARNING(BT_COMMON, "recovery in progress\n");
		err = -EBUSY;
		goto out_unlock;
	}

	if (!common_service.maxwell_core) {
		err = -ENODEV;
		goto out_unlock;
	}

	if (atomic_inc_return(&bt_service.service_users) > 1) {
		if (!bt_service.service_started) {
			atomic_dec(&bt_service.service_users);
			err = -EFAULT;
		}
		goto out_unlock;
	}

	wake_lock(&bt_service.service_wake_lock);

	/* Opening the service boots the firmware, if needed */
	bt_service.service = scsc_mx_service_open(common_service.maxwell_core,
						  SCSC_SERVICE_ID_BT, &mx_bt_client, &err);
	if (!bt_service.service) {
		SCSC_TAG_WARNING(BT_COMMON, "service open failed %d\n", err);
		atomic_dec(&bt_service.service_users);
		err = err ? err : -EIO;
		goto out_wake_unlock;
	}

	/* Recovery is never signalled when it is disabled */
	if (mxman_recovery_disabled())
		recovery_timeout = SLSI_BT_SERVICE_STOP_RECOVERY_DISABLED_TIMEOUT;
	else
		recovery_timeout = SLSI_BT_SERVICE_STOP_RECOVERY_TIMEOUT;

	err = scsc_mx_service_mifram_alloc(bt_service.service, sizeof(struct BHCS),
					   &bt_service.bhcs_ref, BSMHCP_ALIGNMENT);
	if (err)
		goto exit;

	err = scsc_mx_service_mifram_alloc(bt_service.service, sizeof(struct BSMHCP_PROTOCOL),
					   &bt_service.bsmhcp_ref, BSMHCP_ALIGNMENT);
	if (err)
		goto exit;

	bhcs = scsc_mx_service_mif_addr_to_ptr(bt_service.service, bt_service.bhcs_ref);
	if (!bhcs) {
		err = -ENOMEM;
		goto exit;
	}

	err = setup_bhcs(bt_service.service, bhcs, bt_service.bsmhcp_ref,
			 sizeof(struct BSMHCP_PROTOCOL), &bt_service.config_ref);
	if (err)
		goto exit;

	err = scsc_bt_shm_init();
	if (err) {
		SCSC_TAG_ERR(BT_COMMON, "scsc_bt_shm_init err %d\n", err);
		goto exit;
	}

	err = slsi_sm_bt_service_init_interrupts();
	if (err < 0)
		goto exit;

	bt_service.bsmhcp_protocol->header.firmware_control = firmware_control;
	/* No A-Box audio path */
	bt_service.bsmhcp_protocol->header.abox_offset = 0;
	bt_service.bsmhcp_protocol->header.abox_length = 0;

	/* Start the service last, after setting up the shared memory */
	err = scsc_mx_service_start(bt_service.service, bt_service.bhcs_ref);
	if (err < 0) {
		SCSC_TAG_ERR(BT_COMMON, "scsc_mx_service_start err %d\n", err);
		goto exit;
	}

	bt_service.service_started = true;
	SCSC_TAG_INFO(BT_COMMON, "Bluetooth service running\n");

exit:
	if (err < 0 && slsi_sm_bt_service_cleanup() == 0)
		atomic_dec(&bt_service.service_users);
out_wake_unlock:
	wake_unlock(&bt_service.service_wake_lock);
out_unlock:
	mutex_unlock(&bt_start_mutex);
	return err;
}

/* Stop the BT service, called with bt_start_mutex held */
static int slsi_sm_bt_service_stop(void)
{
	if (atomic_read(&bt_service.service_users) > 1) {
		atomic_dec(&bt_service.service_users);
	} else if (atomic_read(&bt_service.service_users) == 1) {
		if (slsi_sm_bt_service_cleanup())
			return -EIO;
		atomic_dec(&bt_service.service_users);
	}

	return 0;
}

static void scsc_bt_reset_stream(void)
{
	bt_service.read_offset = 0;
	bt_service.read_operation = 0;
	bt_service.read_index = 0;
	bt_service.h4_write_offset = 0;
}

static int scsc_bt_h4_open(void)
{
	int ret;

	if (bt_service.h4_users)
		return -EBUSY;

	ret = slsi_sm_bt_service_start();
	if (!ret)
		bt_service.h4_users = true;

	return ret;
}

static void scsc_bt_h4_release(void)
{
	int ret;

	mutex_lock(&bt_start_mutex);
	wake_lock(&bt_service.service_wake_lock);

	/* service_started is false if recovery timed out waiting for the release */
	if (bt_service.recovery_level < MX_SYSERR_LEVEL_7 && bt_service.service_started) {
		if (slsi_sm_bt_service_stop() == -EIO)
			goto recovery;

		scsc_bt_reset_stream();

		/* Release and recovery may cross, take the recovery path then */
		if (bt_service.recovery_level >= MX_SYSERR_LEVEL_7)
			goto recovery;

		bt_service.recovery_level = 0;
		bt_service.h4_users = false;
		wake_unlock(&bt_service.service_wake_lock);
		mutex_unlock(&bt_start_mutex);
		return;
	}

recovery:
	/* Already re-initialised if the service was closed before the release */
	if (bt_service.service_started)
		complete_all(&bt_service.recovery_release_complete);
	wake_unlock(&bt_service.service_wake_lock);
	mutex_unlock(&bt_start_mutex);

	ret = wait_for_completion_timeout(&bt_service.recovery_probe_complete,
					  msecs_to_jiffies(recovery_timeout));
	if (!ret)
		SCSC_TAG_INFO(BT_COMMON, "recovery_probe_complete timeout\n");

	bt_service.h4_users = false;
}

/*
 * Split the H4 stream from the firmware into HCI packets. Only events and
 * ACL data come from the controller.
 */
static void scsc_bt_rx_parse(struct hci_dev *hdev)
{
	size_t hdr_len, len;
	struct sk_buff *skb;

	while (rx_fill) {
		switch (rx_buf[0]) {
		case HCI_EVENT_PKT:
			hdr_len = HCI_EVENT_HDR_SIZE;
			if (rx_fill < 1 + hdr_len)
				return;
			len = 1 + hdr_len + rx_buf[2];
			break;
		case HCI_ACLDATA_PKT:
			hdr_len = HCI_ACL_HDR_SIZE;
			if (rx_fill < 1 + hdr_len)
				return;
			len = 1 + hdr_len + get_unaligned_le16(&rx_buf[3]);
			break;
		default:
			bt_dev_err(hdev, "unknown H4 packet type 0x%02x", rx_buf[0]);
			hdev->stat.err_rx++;
			rx_fill = 0;
			return;
		}

		if (len > SCSC_BT_RX_BUF_SIZE) {
			bt_dev_err(hdev, "H4 packet too long (%zu)", len);
			hdev->stat.err_rx++;
			rx_fill = 0;
			return;
		}

		if (rx_fill < len)
			return;

		skb = bt_skb_alloc(len - 1, GFP_KERNEL);
		if (skb) {
			hci_skb_pkt_type(skb) = rx_buf[0];
			skb_put_data(skb, &rx_buf[1], len - 1);
			hdev->stat.byte_rx += len - 1;
			hci_recv_frame(hdev, skb);
		} else {
			hdev->stat.err_rx++;
		}

		rx_fill -= len;
		memmove(rx_buf, &rx_buf[len], rx_fill);
	}
}

static int scsc_bt_rx_thread(void *data)
{
	struct hci_dev *hdev = data;
	ssize_t ret;

	while (!kthread_should_stop() && !bt_service.rx_stop) {
		ret = scsc_bt_shm_h4_read(&rx_buf[rx_fill], SCSC_BT_RX_BUF_SIZE - rx_fill, false);
		if (ret > 0) {
			rx_fill += ret;
			scsc_bt_rx_parse(hdev);
			continue;
		}

		if (ret == -EINTR || bt_service.rx_stop)
			break;

		/*
		 * The firmware failed and the hardware error event was
		 * delivered: wait until the stack closes the device.
		 */
		wait_event_interruptible(bt_service.read_wait,
					 bt_service.rx_stop || kthread_should_stop());
	}

	/* kthread_stop() must find the thread alive */
	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (!kthread_should_stop())
			schedule();
		__set_current_state(TASK_RUNNING);
	}

	return 0;
}

static int scsc_bt_hci_open(struct hci_dev *hdev)
{
	int ret;

	ret = scsc_bt_h4_open();
	if (ret) {
		bt_dev_err(hdev, "failed to start the BT service: %d", ret);
		return ret;
	}

	rx_fill = 0;
	bt_service.rx_stop = false;
	rx_thread = kthread_run(scsc_bt_rx_thread, hdev, "scsc_bt_rx");
	if (IS_ERR(rx_thread)) {
		ret = PTR_ERR(rx_thread);
		rx_thread = NULL;
		scsc_bt_h4_release();
		return ret;
	}

	return 0;
}

static int scsc_bt_hci_close(struct hci_dev *hdev)
{
	if (rx_thread) {
		bt_service.rx_stop = true;
		wake_up_interruptible(&bt_service.read_wait);
		kthread_stop(rx_thread);
		rx_thread = NULL;
	}

	scsc_bt_h4_release();

	return 0;
}

static int scsc_bt_hci_send(struct hci_dev *hdev, struct sk_buff *skb)
{
	u8 type = hci_skb_pkt_type(skb);
	ssize_t ret;

	switch (type) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	default:
		/* SCO goes through the audio path of the firmware */
		kfree_skb(skb);
		return -EOPNOTSUPP;
	}

	memcpy(skb_push(skb, 1), &type, 1);
	ret = scsc_bt_shm_h4_write(skb->data, skb->len);
	if (ret < 0) {
		hdev->stat.err_tx++;
		kfree_skb(skb);
		return ret;
	}

	hdev->stat.byte_tx += skb->len;
	kfree_skb(skb);

	return 0;
}

static int scsc_bt_hci_register(struct device *dev)
{
	struct hci_dev *hdev;
	int ret;

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_VIRTUAL;
	SET_HCIDEV_DEV(hdev, dev);
	hdev->open = scsc_bt_hci_open;
	hdev->close = scsc_bt_hci_close;
	hdev->send = scsc_bt_hci_send;

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		hci_free_dev(hdev);
		return ret;
	}

	scsc_hdev = hdev;

	return 0;
}

static void scsc_bt_hci_unregister(void)
{
	if (!scsc_hdev)
		return;

	hci_unregister_dev(scsc_hdev);
	hci_free_dev(scsc_hdev);
	scsc_hdev = NULL;
}

/* A new MX instance is available */
static void slsi_bt_service_probe(struct scsc_mx_module_client *module_client,
				  struct scsc_mx *mx, enum scsc_module_client_reason reason)
{
	int ret;

	mutex_lock(&bt_start_mutex);
	if (reason == SCSC_MODULE_CLIENT_REASON_RECOVERY && bt_service.recovery_level == 0)
		goto done;

	bt_service.dev = scsc_mx_get_device(mx);
	common_service.maxwell_core = mx;
	get_device(bt_service.dev);

	if (reason == SCSC_MODULE_CLIENT_REASON_RECOVERY) {
		complete_all(&bt_service.recovery_probe_complete);
		bt_service.recovery_level = 0;
	}

	/* The HCI device stays registered across firmware recoveries */
	if (!scsc_hdev) {
		ret = scsc_bt_hci_register(bt_service.dev);
		if (ret)
			SCSC_TAG_ERR(BT_COMMON, "failed to register the HCI device: %d\n", ret);
	}

done:
	mutex_unlock(&bt_start_mutex);
}

/* The MX instance is now unavailable */
static void slsi_bt_service_remove(struct scsc_mx_module_client *module_client,
				   struct scsc_mx *mx, enum scsc_module_client_reason reason)
{
	mutex_lock(&bt_start_mutex);
	if (reason == SCSC_MODULE_CLIENT_REASON_RECOVERY && bt_service.recovery_level == 0)
		goto done;

	if (reason == SCSC_MODULE_CLIENT_REASON_RECOVERY) {
		bool service_active = bt_service.service_started;

		mutex_unlock(&bt_start_mutex);

		/* Wait for the stack to close the HCI device */
		if (service_active &&
		    !wait_for_completion_timeout(&bt_service.recovery_release_complete,
						 msecs_to_jiffies(SLSI_BT_SERVICE_STOP_RECOVERY_TIMEOUT)))
			SCSC_TAG_INFO(BT_COMMON, "recovery_release_complete timeout\n");

		mutex_lock(&bt_start_mutex);

		if (service_active)
			reinit_completion(&bt_service.recovery_release_complete);

		if (slsi_sm_bt_service_stop() == -EIO)
			SCSC_TAG_INFO(BT_COMMON, "Service stop or close failed during recovery.\n");

		scsc_bt_reset_stream();
	} else {
		mutex_unlock(&bt_start_mutex);
		scsc_bt_hci_unregister();
		mutex_lock(&bt_start_mutex);
	}

	put_device(bt_service.dev);
	common_service.maxwell_core = NULL;

done:
	mutex_unlock(&bt_start_mutex);
}

static struct scsc_mx_module_client bt_driver = {
	.name = "BT driver",
	.probe = slsi_bt_service_probe,
	.remove = slsi_bt_service_remove,
};

static int __init scsc_bt_module_init(void)
{
	rx_buf = kmalloc(SCSC_BT_RX_BUF_SIZE, GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	init_waitqueue_head(&bt_service.read_wait);
	init_waitqueue_head(&bt_service.info_wait);
	init_completion(&bt_service.recovery_probe_complete);
	init_completion(&bt_service.recovery_release_complete);
	spin_lock_init(&bt_service.avdtp_detect.lock);
	spin_lock_init(&bt_service.avdtp_detect.fw_write_lock);

	wake_lock_init(NULL, &bt_service.read_wake_lock.ws, "bt_read_wake_lock");
	wake_lock_init(NULL, &bt_service.write_wake_lock.ws, "bt_write_wake_lock");
	wake_lock_init(NULL, &bt_service.service_wake_lock.ws, "bt_service_wake_lock");

	return scsc_mx_module_register_client_module(&bt_driver);
}

static void __exit scsc_bt_module_exit(void)
{
	scsc_mx_module_unregister_client_module(&bt_driver);
	scsc_bt_hci_unregister();

	complete_all(&bt_service.recovery_probe_complete);
	complete_all(&bt_service.recovery_release_complete);
	wake_lock_destroy(&bt_service.write_wake_lock);
	wake_lock_destroy(&bt_service.read_wake_lock);
	wake_lock_destroy(&bt_service.service_wake_lock);
	kfree(rx_buf);
}

module_init(scsc_bt_module_init);
module_exit(scsc_bt_module_exit);

MODULE_DESCRIPTION("Samsung SCSC Maxwell Bluetooth driver");
MODULE_AUTHOR("Samsung Electronics Co., Ltd.");
MODULE_LICENSE("GPL");
