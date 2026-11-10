// SPDX-License-Identifier: GPL-2.0
/****************************************************************************
 *
 * Copyright (c) 2014 - 2021 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/
#include <linux/types.h>
#include "debug.h"
#include "dev.h"
#include "sap.h"
#include "sap_mlme.h"
#include "hip.h"
#include "mgt.h"
#include "mlme.h"

#define SUPPORTED_OLD_VERSION   0

static int sap_mlme_version_supported(u16 version);
static int sap_mlme_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb);

static int sap_mlme_notifier(struct slsi_dev *sdev, unsigned long event);

static struct sap_api sap_mlme = {
	.sap_class = SAP_MLME,
	.sap_version_supported = sap_mlme_version_supported,
	.sap_handler = sap_mlme_rx_handler,
	.sap_versions = { FAPI_CONTROL_SAP_VERSION, SUPPORTED_OLD_VERSION },
	.sap_notifier = sap_mlme_notifier,
};

static int sap_mlme_notifier(struct slsi_dev *sdev, unsigned long event)
{
	int i;
	int level;
	struct netdev_vif *ndev_vif;
	bool is_recovery = false;

	SLSI_INFO_NODEV("Notifier event received: %lu\n", event);
	if (event >= SCSC_MAX_NOTIFIER)
		return -EIO;

	switch (event) {
	case SCSC_WIFI_STOP:
		/* Stop sending signals down*/
		sdev->mlme_blocked = true;
		sdev->detect_vif_active = false;
		/* cleanup all the VIFs and scan data */
		SLSI_MUTEX_LOCK(sdev->netdev_add_remove_mutex);
		level = atomic_read(&sdev->cm_if.reset_level);
		SLSI_INFO_NODEV("MLME BLOCKED system error level:%d\n", level);
		if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC)
			is_recovery = true;
		complete_all(&sdev->sig_wait.completion);
		/*WLAN system down actions*/
		for (i = 1; i <= SLSI_MAX_INTERFACES; i++)
			if (sdev->netdev[i]) {
				bool vif_type_ap = false;
				ndev_vif = netdev_priv(sdev->netdev[i]);
				complete_all(&ndev_vif->sig_wait.completion);
				slsi_scan_cleanup(sdev, sdev->netdev[i]);
				cancel_work_sync(&ndev_vif->set_multicast_filter_work);
				cancel_work_sync(&ndev_vif->update_pkt_filter_work);
/* For level8 use the older panic flow */
				if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC && ndev_vif->vif_type == FAPI_VIFTYPE_AP)
					vif_type_ap = true;
				sdev->require_vif_delete[ndev_vif->ifnum] = false;
				SLSI_MUTEX_LOCK(ndev_vif->vif_mutex);
				slsi_vif_cleanup(sdev, sdev->netdev[i], 0, is_recovery);
				if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC && vif_type_ap)
					ndev_vif->vif_type = FAPI_VIFTYPE_AP;
				SLSI_MUTEX_UNLOCK(ndev_vif->vif_mutex);
			}
		if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC)
			sdev->device_state = SLSI_DEVICE_STATE_STOPPING;
		if (sdev->netdev_up_count == 0)
			sdev->mlme_blocked = false;
		SLSI_MUTEX_UNLOCK(sdev->netdev_add_remove_mutex);
		SLSI_INFO_NODEV("Force cleaned all VIFs\n");
		break;

	case SCSC_WIFI_FAILURE_RESET:
		level = atomic_read(&sdev->cm_if.reset_level);
		if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC || sdev->require_service_close) {
			SLSI_INFO(sdev, "Error Level:%d, start recovery_work_on_stop queue!!\n", level);
			queue_work(sdev->device_wq, &sdev->recovery_work_on_stop);
		}
		break;

	case SCSC_WIFI_SUSPEND:
		SLSI_MUTEX_LOCK(sdev->device_config_mutex);
		if (!(sdev->device_config.user_suspend_mode) || (sdev->device_config.host_state & SLSI_HOSTSTATE_LCD_ACTIVE)) {
			SLSI_WARN(sdev, "SUSPEND but no SETSUSPENDMODE\n");
			SLSI_INFO(sdev, "UserSuspendMode:%d HostState:%0.2x\n",
				 sdev->device_config.user_suspend_mode, sdev->device_config.host_state);
		}
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		break;

	case SCSC_WIFI_RESUME:
		SLSI_MUTEX_LOCK(sdev->device_config_mutex);
		if (!(sdev->device_config.user_suspend_mode) || (sdev->device_config.host_state & SLSI_HOSTSTATE_LCD_ACTIVE)) {
			SLSI_WARN(sdev, "RESUME but no SETSUSPENDMODE\n");
			SLSI_INFO(sdev, "UserSuspendMode:%d HostState:%0.2x\n",
				 sdev->device_config.user_suspend_mode, sdev->device_config.host_state);
		}
		SLSI_MUTEX_UNLOCK(sdev->device_config_mutex);
		break;
	case SCSC_WIFI_SUBSYSTEM_RESET:
		/*wlan system down actions*/
		queue_work(sdev->device_wq, &sdev->recovery_work);
		break;
	case SCSC_WIFI_CHIP_READY:
		level = atomic_read(&sdev->cm_if.reset_level);
		if (level < SLSI_WIFI_CM_IF_SYSTEM_ERROR_PANIC && sdev->netdev_up_count != 0)
			queue_work(sdev->device_wq, &sdev->recovery_work_on_start);
		break;
	default:
		SLSI_INFO_NODEV("Unknown event code %lu\n", event);
		break;
	}

	return 0;
}

static int sap_mlme_version_supported(u16 version)
{
	unsigned int major = SAP_MAJOR(version);
	unsigned int minor = SAP_MINOR(version);
	u8           i = 0;

	SLSI_INFO_NODEV("Reported version: %d.%d\n", major, minor);

	for (i = 0; i < SAP_MAX_VER; i++)
		if (SAP_MAJOR(sap_mlme.sap_versions[i]) == major)
			return 0;

	SLSI_ERR_NODEV("Version %d.%d Not supported\n", major, minor);

	return -EINVAL;
}

static int slsi_rx_netdev_mlme(struct slsi_dev *sdev, struct net_device *dev, struct sk_buff *skb)
{
	u16 id = fapi_get_u16(skb, id);

	/* The skb is consumed by the functions called.
	 */
	switch (id) {
	case MLME_SCAN_IND:
		slsi_rx_scan_ind(sdev, dev, skb);
		break;
	case MLME_SCAN_DONE_IND:
		slsi_rx_scan_done_ind(sdev, dev, skb);
		break;
	case MLME_CONNECT_IND:
		slsi_rx_connect_ind(sdev, dev, skb);
		break;
	case MLME_CONNECTED_IND:
		slsi_rx_connected_ind(sdev, dev, skb);
		break;
	case MLME_RECEIVED_FRAME_IND:
		slsi_rx_received_frame_ind(sdev, dev, skb);
		break;
	case MLME_DISCONNECT_IND:
		slsi_rx_disconnect_ind(sdev, dev, skb);
		break;
	case MLME_DISCONNECTED_IND:
		slsi_rx_disconnected_ind(sdev, dev, skb);
		break;
	case MLME_PROCEDURE_STARTED_IND:
		slsi_rx_procedure_started_ind(sdev, dev, skb);
		break;
	case MLME_FRAME_TRANSMISSION_IND:
		slsi_rx_frame_transmission_ind(sdev, dev, skb);
		break;
	case MA_BLOCKACK_IND:
		slsi_rx_blockack_ind(sdev, dev, skb);
		break;
	case MLME_ROAMED_IND:
		slsi_rx_roamed_ind(sdev, dev, skb);
		break;
	case MLME_ROAM_IND:
		slsi_rx_roam_ind(sdev, dev, skb);
		break;
	case MLME_MIC_FAILURE_IND:
		slsi_rx_mic_failure_ind(sdev, dev, skb);
		break;
	case MLME_REASSOCIATE_IND:
		slsi_rx_reassoc_ind(sdev, dev, skb);
		break;
	case MLME_TDLS_PEER_IND:
		slsi_tdls_peer_ind(sdev, dev, skb);
		break;
	case MLME_LISTEN_END_IND:
		slsi_rx_listen_end_ind(dev, skb);
		break;
	case MLME_CHANNEL_SWITCHED_IND:
		slsi_rx_channel_switched_ind(sdev, dev, skb);
		break;
	case MLME_AC_PRIORITY_UPDATE_IND:
		SLSI_DBG1(sdev, SLSI_MLME,
			  "Unexpected MLME_AC_PRIORITY_UPDATE_IND\n");
		kfree_skb(skb);
		break;
	case MLME_SYNCHRONISED_IND:
		slsi_rx_synchronised_ind(sdev, dev, skb);
		break;
	case MLME_BLACKLISTED_IND:
		slsi_rx_blacklisted_ind(sdev, dev, skb);
		break;
	case MLME_ROAMING_CHANNEL_LIST_IND:
		/* Only consumed by the vendor Wi-Fi HAL */
		kfree_skb(skb);
		break;
	case MLME_START_DETECT_IND:
		slsi_rx_start_detect_ind(sdev, dev, skb);
		break;
	case SAP_DRV_MA_TO_MLME_DELBA_REQ:
		slsi_rx_ma_to_mlme_delba_req(sdev, dev, skb);
		break;
	default:
		kfree_skb(skb);
		SLSI_NET_ERR(dev, "Unhandled Ind/Cfm: 0x%.4x\n", id);
		break;
	}
	return 0;
}

void slsi_rx_netdev_mlme_work(struct work_struct *work)
{
	struct slsi_skb_work *w = container_of(work, struct slsi_skb_work, work);
	struct slsi_dev *sdev = w->sdev;
	struct net_device *dev = w->dev;
	struct sk_buff *skb = NULL;
	if (WARN_ON(!dev))
		return;
	skb = slsi_skb_work_dequeue(w);

	slsi_wake_lock(&sdev->wlan_wl);
	while (skb) {
		slsi_debug_frame(sdev, dev, skb, "RX");
		slsi_rx_netdev_mlme(sdev, dev, skb);
		skb = slsi_skb_work_dequeue(w);
	}
	slsi_wake_unlock(&sdev->wlan_wl);
}

int slsi_rx_enqueue_netdev_mlme(struct slsi_dev *sdev, struct sk_buff *skb, u16 vif)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);

	/* in case of del_vif failure, we may get mlme signals for a netdev which is already removed */
	if (!dev) {
		SLSI_WARN(sdev, "dev is NULL");
		kfree_skb(skb);
		rcu_read_unlock();
		return 0;
	}

	ndev_vif = netdev_priv(dev);

	if (unlikely(ndev_vif->is_fw_test)) {
		kfree_skb(skb);
		rcu_read_unlock();
		return 0;
	}

	slsi_skb_work_enqueue(&ndev_vif->rx_mlme, skb);
	rcu_read_unlock();
	return 0;
}

static int slsi_rx_action_enqueue_netdev_mlme(struct slsi_dev *sdev, struct sk_buff *skb, u16 vif)
{
	struct net_device *dev;
	struct netdev_vif *ndev_vif;

	rcu_read_lock();
	dev = slsi_get_netdev_rcu(sdev, vif);
	if (WARN_ON(!dev)) {
		rcu_read_unlock();
		return -ENODEV;
	}

	ndev_vif = netdev_priv(dev);

	if (unlikely(ndev_vif->is_fw_test)) {
		rcu_read_unlock();
		return 0;
	}

	if (ndev_vif->iftype == NL80211_IFTYPE_P2P_GO || ndev_vif->iftype == NL80211_IFTYPE_P2P_CLIENT) {
		struct ieee80211_mgmt *mgmt = fapi_get_mgmt(skb);
		/*  DA of action frame is GO interface address?*/
		if (memcmp(mgmt->da, dev->dev_addr, ETH_ALEN) != 0) {
			struct net_device *p2pdev = slsi_get_netdev_rcu(sdev, SLSI_NET_INDEX_P2P);

			if (WARN_ON(!p2pdev)) {
				rcu_read_unlock();
				return -ENODEV;
			}
			if (memcmp(mgmt->da, p2pdev->dev_addr, ETH_ALEN) == 0) {
				/* If destination address is P2P DEV ADDR, then
				 * action frame is received on GO interface.
				 * Hence indicate action frames on P2P DEV
				 */
				ndev_vif = netdev_priv(p2pdev);

				if (unlikely(ndev_vif->is_fw_test)) {
					kfree_skb(skb);
					rcu_read_unlock();
					return 0;
				}
			}
		}
	}

	slsi_skb_work_enqueue(&ndev_vif->rx_mlme, skb);

	rcu_read_unlock();
	return 0;
}

static int sap_mlme_rx_handler(struct slsi_dev *sdev, struct sk_buff *skb)
{
	u16 scan_id;
	u16 vif = fapi_get_vif(skb);

	if (slsi_rx_blocking_signals(sdev, skb) == 0)
		return 0;

	if (fapi_is_ind(skb)) {

		switch (fapi_get_sigid(skb)) {
		case MLME_SCAN_DONE_IND:
			scan_id = fapi_get_u16(skb, u.mlme_scan_done_ind.scan_id);
			return slsi_rx_enqueue_netdev_mlme(sdev, skb, (scan_id >> 8));
		case MLME_SCAN_IND:
			if (vif)
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			scan_id = fapi_get_u16(skb, u.mlme_scan_ind.scan_id);
			return slsi_rx_enqueue_netdev_mlme(sdev, skb,  (scan_id >> 8));
		case MLME_RECEIVED_FRAME_IND:
			if (vif == 0) {
				SLSI_WARN(sdev, "MLME_RECEIVED_FRAME_IND VIF 0\n");
				goto err;
			}
			return slsi_rx_action_enqueue_netdev_mlme(sdev, skb, vif);
		case MLME_START_DETECT_IND:
			return slsi_rx_enqueue_netdev_mlme(sdev, skb, SLSI_NET_INDEX_WLAN);
		case MLME_ROAMED_IND:
			if (vif == 0) {
				SLSI_WARN(sdev, "MLME_ROAMED_IND VIF 0\n");
				goto err;
			} else {
				struct net_device *dev;
				struct netdev_vif *ndev_vif;

				rcu_read_lock();
				dev = slsi_get_netdev_rcu(sdev, vif);
				if (WARN_ON(!dev)) {
					rcu_read_unlock();
					return -ENODEV;
				}
				ndev_vif = netdev_priv(dev);
				if (atomic_read(&ndev_vif->sta.drop_roamed_ind)) {
					/* If roam cfm is not received for the
					 * req, ignore this roamed indication.
					 */
					kfree_skb(skb);
					rcu_read_unlock();
					return 0;
				}
				rcu_read_unlock();
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			}
		default:
			if (vif == 0) {
				SLSI_WARN(sdev, "Received signal 0x%04x on VIF 0, return error\n", fapi_get_sigid(skb));
				goto err;
			} else {
				return slsi_rx_enqueue_netdev_mlme(sdev, skb, vif);
			}
		}
	}

	if (fapi_is_cfm(skb) && fapi_get_sigid(skb) == MLME_SEND_FRAME_CFM && vif != 0) {
		kfree_skb(skb);
		return 0;
	}

	if (WARN_ON(fapi_is_req(skb)))
		goto err;

	if (slsi_is_test_mode_enabled()) {
		kfree_skb(skb);
		return 0;
	}

	WARN_ON(1);

err:
	return -EINVAL;
}

void slsi_get_fapi_version_string(char *res)
{
	int count = 0;

	count = snprintf(res, 100, "FAPI_CONTROL_SAP_VERSION : %d.%d.%d\n",
			 FAPI_MAJOR_VERSION(FAPI_CONTROL_SAP_VERSION),
			 FAPI_MINOR_VERSION(FAPI_CONTROL_SAP_VERSION),
			 FAPI_CONTROL_SAP_ENG_VERSION);
	res[count] = '\0';
}

int sap_mlme_init(void)
{
	SLSI_INFO_NODEV("Registering SAP\n");
	slsi_hip_sap_register(&sap_mlme);
	return 0;
}

int sap_mlme_deinit(void)
{
	SLSI_INFO_NODEV("Unregistering SAP\n");
	slsi_hip_sap_unregister(&sap_mlme);
	return 0;
}
