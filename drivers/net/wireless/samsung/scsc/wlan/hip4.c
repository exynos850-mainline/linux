// SPDX-License-Identifier: GPL-2.0
/******************************************************************************
 *
 * Copyright (c) 2014 - 2020 Samsung Electronics Co., Ltd. All rights reserved
 *
 *****************************************************************************/

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <scsc/scsc_mx.h>
#include <scsc/scsc_mifram.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <scsc/scsc_logring.h>
#include <linux/cpu.h>
#include <linux/bitmap.h>

#include "hip4.h"
#include "mbulk.h"
#include "dev.h"

#include "debug.h"

/* Global spinlock to serialize napi context with hip4_deinit*/
static DEFINE_SPINLOCK(in_napi_context);


static bool hip4_system_wq;
module_param(hip4_system_wq, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hip4_system_wq, "Use system wq instead of named workqueue. (default: N)");




#define SLSI_HIP_NAPI_STATE_ENABLED (0x1)
/* run NAPI poll on a specific CPU (preferably a big CPU if online) */
static int napi_select_cpu; /* CPU number */
module_param(napi_select_cpu, int, 0644);
MODULE_PARM_DESC(napi_select_cpu, "select a specific CPU to execute NAPI poll");

static uint hip4_napi_num_rx_pkts = 80;
module_param(hip4_napi_num_rx_pkts, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hip4_napi_num_rx_pkts, "number of packets in Rx queue to deem it as full");

static uint hip4_napi_num_rx_full = 3;
module_param(hip4_napi_num_rx_full, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hip4_napi_num_rx_full, "number of iterations of Rx queue full that signal to move to high performance");

static int max_buffered_frames = 10000;
module_param(max_buffered_frames, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_buffered_frames, "Maximum number of frames to buffer in the driver");

static ktime_t intr_received_fb;
static ktime_t bh_init_fb;
static ktime_t bh_end_fb;
static ktime_t intr_received_ctrl;
static ktime_t bh_init_ctrl;
static ktime_t bh_end_ctrl;
static ktime_t intr_received_data;
static ktime_t bh_init_data;
static ktime_t bh_end_data;
static ktime_t wdt;
static ktime_t send;
static ktime_t closing;

enum rw {
	widx,
	ridx,
};

static u8 hip4_read_index(struct slsi_hip4 *hip, u32 q, enum rw r_w);

/* Q mapping V3 - V4 */
/*offset of F/W owned indices */
#define FW_OWN_OFS      (64)
/**
 * HIP queue indices layout in the scoreboard (SC-505612-DD). v3
 *
 *             3        2        1        0
 *         +-----------------------------------+
 *     +0  |  Q3R   |   Q2R  |  Q1W   |  Q0W   |  Owned by the host
 *         +-----------------------------------+
 *     +4  |        |        |  Q5W   |  Q4R   |  Owned by the host
 *         +-----------------------------------+
 *
 *         +-----------------------------------+
 *    +64  |  Q3W   |   Q2W  |  Q1R   |  Q0R   |  Owned by the F/W
 *         +-----------------------------------+
 *    +68  |        |        |  Q5R   |  Q4W   |  Owned by the F/W
 *         +-----------------------------------+
 *
 * The queue indcies which owned by the host are only writable by the host.
 * F/W can only read them. And vice versa.
 */
static int q_idx_layout[6][2] = {
	{               0,  FW_OWN_OFS + 0},	/* mif_q_fh_ctl : 0 */
	{               1,  FW_OWN_OFS + 1},	/* mif_q_fh_dat : 1 */
	{  FW_OWN_OFS + 2,               2},	/* mif_q_fh_rfb : 2 */
	{  FW_OWN_OFS + 3,               3},    /* mif_q_th_ctl : 3 */
	{  FW_OWN_OFS + 4,               4},	/* mif_q_th_dat : 4 */
	{               5,  FW_OWN_OFS + 5}	/* mif_q_th_rfb : 5 */
};

/*offset of F/W owned VIF Status */
#define FW_OWN_VIF      (96)
/**
 * HIP Pause state VIF. v4. 2 bits per PEER
 *
 *         +-----------------------------------+
 *    +96  |        VIF[0] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +100 |        VIF[0] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +104 |        VIF[1] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +108 |        VIF[1] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +112 |        VIF[2] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +116 |        VIF[2] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +120 |        VIF[3] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +124 |        VIF[3] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *
 */

/* MAX_STORM. Max Interrupts allowed when platform is in suspend */
#define MAX_STORM            5

/* Timeout for Wakelocks in HIP  */
#define SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS   (1000)


#define FB_NO_SPC_NUM_RET    100
#define FB_NO_SPC_SLEEP_MS   10
#define FB_NO_SPC_DELAY_US   1000

/* Update scoreboard index */
/* Function can be called from BH context */
static void hip4_update_index(struct slsi_hip4 *hip, u32 q, enum rw r_w, u8 value)
{
	struct hip4_priv    *hip_priv = hip->hip_priv;

	write_lock_bh(&hip_priv->rw_scoreboard);
	if (hip->hip_priv->version == 5 || hip->hip_priv->version == 4) {
		*((u8 *)(hip->hip_priv->scbrd_base + q_idx_layout[q][r_w])) = value;
	} else {
		SLSI_ERR_NODEV("Incorrect version\n");
		goto error;
	}

	/* Memory barrier when updating shared mailbox/memory */
	smp_wmb();
error:
	write_unlock_bh(&hip_priv->rw_scoreboard);
}

/* Read scoreboard index */
/* Function can be called from BH context */
static u8 hip4_read_index(struct slsi_hip4 *hip, u32 q, enum rw r_w)
{
	struct hip4_priv    *hip_priv = hip->hip_priv;
	u32                 value = 0;

	read_lock_bh(&hip_priv->rw_scoreboard);
	if (hip->hip_priv->version == 5 || hip->hip_priv->version == 4) {
		value = *((u8 *)(hip->hip_priv->scbrd_base + q_idx_layout[q][r_w]));
	} else {
		SLSI_ERR_NODEV("Incorrect version\n");
		goto error;
	}

	/* Memory barrier when reading shared mailbox/memory */
	smp_rmb();
error:
	read_unlock_bh(&hip_priv->rw_scoreboard);
	return value;
}

static void hip4_dump_dbg(struct slsi_hip4 *hip, struct mbulk *m, struct sk_buff *skb, struct scsc_service *service)
{
	unsigned int        i = 0;
	scsc_mifram_ref     ref;
	SLSI_ERR_NODEV("intr_tohost_fb 0x%x\n", hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_RFB]);
	SLSI_ERR_NODEV("intr_tohost_ctrl 0x%x\n", hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_CTRL]);
	SLSI_ERR_NODEV("intr_tohost_dat 0x%x\n", hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
	SLSI_ERR_NODEV("intr_fromhost 0x%x\n", hip->hip_priv->intr_fromhost);

	/* Print scoreboard */
	for (i = 0; i < 6; i++) {
		SLSI_ERR_NODEV("Q%dW 0x%x\n", i, hip4_read_index(hip, i, widx));
		SLSI_ERR_NODEV("Q%dR 0x%x\n", i, hip4_read_index(hip, i, ridx));
	}

	if (service)
		scsc_mx_service_mif_dump_registers(service);

	if (m && service) {
		if (scsc_mx_service_mif_ptr_to_addr(service, m, &ref))
			return;
		SLSI_ERR_NODEV("m: %p 0x%x\n", m, ref);
		print_hex_dump(KERN_ERR, SCSC_PREFIX "mbulk ", DUMP_PREFIX_NONE, 16, 1, m, sizeof(struct mbulk), 0);
	}
	if (m && mbulk_has_signal(m))
		print_hex_dump(KERN_ERR, SCSC_PREFIX "sig   ", DUMP_PREFIX_NONE, 16, 1, mbulk_get_signal(m),
			       MBULK_SEG_SIG_BUFSIZE(m), 0);
	if (skb)
		print_hex_dump(KERN_ERR, SCSC_PREFIX "skb   ", DUMP_PREFIX_NONE, 16, 1, skb->data, skb->len > 0xff ? 0xff : skb->len, 0);

	SLSI_ERR_NODEV("time: wdt     %lld\n", ktime_to_ns(wdt));
	SLSI_ERR_NODEV("time: send    %lld\n", ktime_to_ns(send));
	SLSI_ERR_NODEV("time: intr_fb      %lld\n", ktime_to_ns(intr_received_fb));
	SLSI_ERR_NODEV("time: bh_init_fb   %lld\n", ktime_to_ns(bh_init_fb));
	SLSI_ERR_NODEV("time: bh_end_fb    %lld\n", ktime_to_ns(bh_end_fb));
	SLSI_ERR_NODEV("time: intr_ctrl    %lld\n", ktime_to_ns(intr_received_ctrl));
	SLSI_ERR_NODEV("time: bh_init_ctrl %lld\n", ktime_to_ns(bh_init_ctrl));
	SLSI_ERR_NODEV("time: bh_end_ctrl  %lld\n", ktime_to_ns(bh_end_ctrl));
	SLSI_ERR_NODEV("time: intr_data    %lld\n", ktime_to_ns(intr_received_data));
	SLSI_ERR_NODEV("time: bh_init_data %lld\n", ktime_to_ns(bh_init_data));
	SLSI_ERR_NODEV("time: bh_end_data  %lld\n", ktime_to_ns(bh_end_data));
	SLSI_ERR_NODEV("time: closing %lld\n", ktime_to_ns(closing));
}

/* Transform skb to mbulk (fapi_signal + payload) */
static struct mbulk *hip4_skb_to_mbulk(struct hip4_priv *hip, struct sk_buff *skb, bool ctrl_packet, mbulk_colour colour)
{
	struct mbulk        *m = NULL;
	void                *sig = NULL, *b_data = NULL;
	size_t              payload = 0;
	u8                  pool_id = ctrl_packet ? MBULK_POOL_ID_CTRL : MBULK_POOL_ID_DATA;
	u8                  headroom = 0, tailroom = 0;
	enum mbulk_class    clas = ctrl_packet ? MBULK_CLASS_FROM_HOST_CTL : MBULK_CLASS_FROM_HOST_DAT;
	struct slsi_skb_cb *cb = slsi_skb_cb_get(skb);

	payload = skb->len - cb->sig_length;

	/* Get headroom/tailroom */
	headroom = hip->unidat_req_headroom;
	tailroom = hip->unidat_req_tailroom;

	/* Allocate mbulk */
	if (payload > 0) {
		/* If signal include payload, add headroom and tailroom */
		m = mbulk_with_signal_alloc_by_pool(pool_id, colour, clas, cb->sig_length + 4,
						    payload + headroom + tailroom);
		if (!m)
			return NULL;
		if (!mbulk_reserve_head(m, headroom))
			return NULL;
	} else {
		/* If it is only a signal do not add headroom */
		m = mbulk_with_signal_alloc_by_pool(pool_id, colour, clas, cb->sig_length + 4, 0);
		if (!m)
			return NULL;
	}

	/* Get signal handler */
	sig = mbulk_get_signal(m);
	if (!sig) {
		SLSI_ERR_NODEV("no sig in mbulk\n");
		mbulk_free_virt_host(m);
		return NULL;
	}

	/* Copy signal */
	/* 4Bytes offset is required for FW fapi header */
	memcpy(sig + 4, skb->data, cb->sig_length);

	/* Copy payload */
	/* If the signal has payload memcpy the data */
	if (payload > 0) {
		/* Get head pointer */
		b_data = mbulk_dat_rw(m);
		if (!b_data) {
			SLSI_ERR_NODEV("head pointer is NULL\n");
			mbulk_free_virt_host(m);
			return NULL;
		}

		/* Copy payload skipping the signal data */
		memcpy(b_data, skb->data + cb->sig_length, payload);
		mbulk_append_tail(m, payload);
	}
	m->flag |= MBULK_F_OBOUND;

	return m;
}

/* Transform mbulk to skb (fapi_signal + payload) */
static struct sk_buff *hip4_mbulk_to_skb(struct scsc_service *service, struct hip4_priv *hip_priv, struct mbulk *m, scsc_mifram_ref *to_free, bool atomic)
{
	struct slsi_skb_cb        *cb;
	struct mbulk              *next_mbulk[MBULK_MAX_CHAIN];
	struct sk_buff            *skb = NULL;
	scsc_mifram_ref           ref;
	scsc_mifram_ref           m_chain_next;
	u8                        free = 0;
	u8                        i = 0, j = 0;
	u8                        *p;
	size_t                    bytes_to_alloc = 0;

	/* Get the mif ref pointer, check for incorrect mbulk */
	if (scsc_mx_service_mif_ptr_to_addr(service, m, &ref)) {
		SLSI_ERR_NODEV("mbulk address conversion failed\n");
		return NULL;
	}

	/* Track mbulk that should be freed */
	to_free[free++] = ref;

	bytes_to_alloc += m->sig_bufsz - 4;
	bytes_to_alloc += m->len;

	/* Detect Chained mbulk to start building the chain */
	if ((MBULK_SEG_IS_CHAIN_HEAD(m)) && (MBULK_SEG_IS_CHAINED(m))) {
		m_chain_next = mbulk_chain_next(m);
		if (!m_chain_next) {
			SLSI_ERR_NODEV("Mbulk is set MBULK_F_CHAIN_HEAD and MBULK_F_CHAIN but m_chain_next is NULL\n");
			goto cont;
		}
		while (1) {
			/* increase number mbulks in chain */
			i++;
			/* Get next_mbulk kernel address space pointer  */
			next_mbulk[i - 1] = scsc_mx_service_mif_addr_to_ptr(service, m_chain_next);
			if (!next_mbulk[i - 1]) {
				SLSI_ERR_NODEV("First Mbulk is set as MBULK_F_CHAIN but next_mbulk is NULL\n");
				return NULL;
			}
			/* Track mbulk to be freed */
			to_free[free++] = m_chain_next;
			bytes_to_alloc += next_mbulk[i - 1]->len;
			if (MBULK_SEG_IS_CHAINED(next_mbulk[i - 1])) {
				/* continue traversing the chain */
				m_chain_next = mbulk_chain_next(next_mbulk[i - 1]);
				if (!m_chain_next)
					break;

				if (i >= MBULK_MAX_CHAIN) {
					SLSI_ERR_NODEV("Max number of chained MBULK reached\n");
					return NULL;
				}
			} else {
				break;
			}
		}
	}

cont:
	if (atomic) {
		skb = alloc_skb(bytes_to_alloc, GFP_ATOMIC);
	} else {
		spin_unlock_bh(&hip_priv->rx_lock);
		skb = alloc_skb(bytes_to_alloc, GFP_KERNEL);
		spin_lock_bh(&hip_priv->rx_lock);
	}
	if (!skb) {
		SLSI_ERR_NODEV("Error allocating skb %zu bytes\n", bytes_to_alloc);
		return NULL;
	}

	cb = slsi_skb_cb_init(skb);
	cb->sig_length = m->sig_bufsz - 4;
	/* fapi_data_append adds to the data_length */
	cb->data_length = cb->sig_length;

	p = mbulk_get_signal(m);
	if (!p) {
		SLSI_ERR_NODEV("No signal in Mbulk\n");
		print_hex_dump(KERN_ERR, SCSC_PREFIX "mbulk ", DUMP_PREFIX_NONE, 16, 1, m, sizeof(struct mbulk), 0);
		kfree_skb(skb);
		return NULL;
	}
	/* Remove 4Bytes offset coming from FW */
	p += 4;

	/* Don't need to copy the 4Bytes header coming from the FW */
	memcpy(skb_put(skb, cb->sig_length), p, cb->sig_length);

	if (m->len)
		fapi_append_data(skb, mbulk_dat_r(m), m->len);
	for (j = 0; j < i; j++)
		fapi_append_data(skb, mbulk_dat_r(next_mbulk[j]), next_mbulk[j]->len);

	return skb;
}

/* Add signal reference (offset in shared memory) in the selected queue */
/* This function should be called in atomic context. Callers should supply proper locking mechanism */
static int hip4_q_add_signal(struct slsi_hip4 *hip, enum hip4_hip_q_conf conf, scsc_mifram_ref phy_m, struct scsc_service *service)
{
	struct hip4_hip_control *ctrl = hip->hip_control;
	struct hip4_priv        *hip_priv = hip->hip_priv;
	u8                      idx_w;
	u8                      idx_r;

	/* Read the current q write pointer */
	idx_w = hip4_read_index(hip, conf, widx);
	/* Read the current q read pointer */
	idx_r = hip4_read_index(hip, conf, ridx);

	/* Queueu is full */
	if (idx_r == ((idx_w + 1) & (MAX_NUM - 1)))
		return -ENOSPC;

	/* Update array */
	ctrl->q[conf].array[idx_w] = phy_m;
	/* Memory barrier before updating shared mailbox */
	smp_wmb();

	/* Increase index */
	idx_w++;
	idx_w &= (MAX_NUM - 1);

	/* Update the scoreboard */
	hip4_update_index(hip, conf, widx, idx_w);

	send = ktime_get();
	scsc_service_mifintrbit_bit_set(service, hip_priv->intr_fromhost, SCSC_MIFINTR_TARGET_R4);

	return 0;
}

static void hip4_watchdog(struct timer_list *t)
{
	struct hip4_priv        *priv = timer_container_of(priv, t, watchdog);
	struct slsi_hip4        *hip = priv->hip;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service     *service;
	ktime_t                 intr_ov;
	unsigned long           flags;
	bool retrigger_watchdog = true;
	u8 i;

	if (!hip || !sdev || !sdev->service || !hip->hip_priv)
		return;

	spin_lock_irqsave(&hip->hip_priv->watchdog_lock, flags);
	if (!atomic_read(&hip->hip_priv->watchdog_timer_active))
		goto exit;

	wdt = ktime_get();

	/* if intr_received > wdt skip as intr has been unblocked */
	if (test_and_clear_bit(HIP4_MIF_Q_FH_RFB, hip->hip_priv->irq_bitmap)) {
		intr_ov = ktime_add_ms(intr_received_fb, jiffies_to_msecs(HZ));
		if ((ktime_compare(intr_ov, wdt) < 0))
			retrigger_watchdog = false;
	}
	if (test_and_clear_bit(HIP4_MIF_Q_TH_CTRL, hip->hip_priv->irq_bitmap)) {
		intr_ov = ktime_add_ms(intr_received_ctrl, jiffies_to_msecs(HZ));
		if ((ktime_compare(intr_ov, wdt) < 0))
			retrigger_watchdog = false;
	}
	if (test_and_clear_bit(HIP4_MIF_Q_TH_DAT, hip->hip_priv->irq_bitmap)) {
		intr_ov = ktime_add_ms(intr_received_data, jiffies_to_msecs(HZ));
		if ((ktime_compare(intr_ov, wdt) < 0))
			retrigger_watchdog = false;
	}
	if (retrigger_watchdog) {
		wdt = ktime_set(0, 0);
		/* Retrigger WDT to check flags again in the future */
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ / 2);
		goto exit;
	}

	/* Unlock irq to avoid __local_bh_enable_ip warning */
	spin_unlock_irqrestore(&hip->hip_priv->watchdog_lock, flags);
	hip4_dump_dbg(hip, NULL, NULL, sdev->service);
	spin_lock_irqsave(&hip->hip_priv->watchdog_lock, flags);

	service = sdev->service;

	SLSI_INFO_NODEV("Hip4 watchdog triggered\n");

	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		if (hip->hip_priv->intr_tohost_mul[i] == MIF_NO_IRQ)
			continue;
		if (scsc_service_mifintrbit_bit_mask_status_get(service) & (1 << hip->hip_priv->intr_tohost_mul[i])) {
			/* Interrupt might be pending! */
			SLSI_INFO_NODEV("%d: Interrupt Masked. Unmask to restart Interrupt processing\n", i);
			scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[i]);
		}
	}
exit:
	spin_unlock_irqrestore(&hip->hip_priv->watchdog_lock, flags);
}

void hip4_set_napi_cpu(struct slsi_hip4 *hip, u8 napi_cpu, bool perf_mode)
{
	struct hip4_priv        *hip_priv;
	struct slsi_dev         *sdev;
	unsigned long flags;

	if (!hip)
		return;

	sdev = container_of(hip, struct slsi_dev, hip4_inst);
	if (!sdev)
		return;

	hip_priv = hip->hip_priv;

	if (!hip_priv)
		return;

	if (!cpu_online(napi_cpu)) {
		SLSI_ERR_NODEV("CPU%d is offline.\n", napi_cpu);
		scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
		return;
	}

	slsi_wake_lock(&hip->hip_priv->hip4_wake_lock_data);
	if (napi_cpu != napi_select_cpu) {
		spin_lock_irqsave(&hip->hip_priv->napi_cpu_lock, flags);
		scsc_service_mifintrbit_bit_mask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
		if (!test_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state)) {
			slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_data);
			SLSI_INFO_NODEV("NAPI is already disabled on CPU%d\n", napi_select_cpu);
			spin_unlock_irqrestore(&hip->hip_priv->napi_cpu_lock, flags);
			return;
		}

		SLSI_INFO_NODEV("disable NAPI on CPU%d\n", napi_select_cpu);
		clear_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state);
		napi_select_cpu = napi_cpu;

		/* napi_disable may sleep, so release the lock */
		spin_unlock_irqrestore(&hip->hip_priv->napi_cpu_lock, flags);
		napi_disable(&hip->hip_priv->napi);


		local_bh_disable();
		spin_lock_irqsave(&hip->hip_priv->napi_cpu_lock, flags);

		hip_priv->napi_perf_mode = perf_mode;
		if (!test_and_set_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state)) {
			SLSI_INFO_NODEV("enable NAPI on CPU%d\n", napi_select_cpu);
			napi_enable(&hip->hip_priv->napi);

			if (napi_select_cpu == smp_processor_id()) {
				napi_schedule(&hip->hip_priv->napi);
			} else {
				if (napi_select_cpu && cpu_online(napi_select_cpu))
					schedule_work_on(napi_select_cpu, &hip->hip_priv->intr_wq_napi_cpu_switch);
				else
					napi_schedule(&hip->hip_priv->napi);
			}
		}

		spin_unlock_irqrestore(&hip->hip_priv->napi_cpu_lock, flags);
		local_bh_enable();
	}
	scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
	slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_data);
}

static void hip4_tl_fb(unsigned long data)

{
	struct slsi_hip4        *hip = (void *)data;
	struct hip4_priv        *hip_priv = hip->hip_priv;
	struct hip4_hip_control *ctrl;
	struct scsc_service     *service;
	struct slsi_dev         *sdev;
	bool                    no_change = true;
	u8                      idx_r;
	u8                      idx_w;
	scsc_mifram_ref         ref;
	void                    *mem;

	if (!hip_priv || !hip_priv->hip) {
		SLSI_ERR_NODEV("hip_priv or hip_priv->hip is Null\n");
		return;
	}

	hip = hip_priv->hip;
	ctrl = hip->hip_control;

	if (!ctrl) {
		SLSI_ERR_NODEV("hip->hip_control is Null\n");
		return;
	}
	sdev = container_of(hip, struct slsi_dev, hip4_inst);

	if (!sdev || !sdev->service) {
		SLSI_ERR_NODEV("sdev or sdev->service is Null\n");
		return;
	}

	spin_lock_bh(&hip_priv->rx_lock);
	service = sdev->service;
	bh_init_fb = ktime_get();
	clear_bit(HIP4_MIF_Q_FH_RFB, hip->hip_priv->irq_bitmap);

	idx_r = hip4_read_index(hip, HIP4_MIF_Q_FH_RFB, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_FH_RFB, widx);

	while (idx_r != idx_w) {
		struct mbulk *m;
		mbulk_colour colour;

		no_change = false;
		ref = ctrl->q[HIP4_MIF_Q_FH_RFB].array[idx_r];
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)mem;

		if (!m) {
			SLSI_ERR_NODEV("FB: Mbulk is NULL\n");
			goto consume_fb_mbulk;
		}
		/* Account ONLY for data RFB */
		if ((m->pid & 0x1) == MBULK_POOL_ID_DATA) {
			colour = mbulk_get_colour(MBULK_POOL_ID_DATA, m);
			/* Ignore return value */
			slsi_hip_tx_done(sdev,
					 SLSI_MBULK_COLOUR_GET_VIF(colour),
					 SLSI_MBULK_COLOUR_GET_PEER_IDX(colour),
					 SLSI_MBULK_COLOUR_GET_AC(colour));
		}
		mbulk_free_virt_host(m);
consume_fb_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);
		hip4_update_index(hip, HIP4_MIF_Q_FH_RFB, ridx, idx_r);
	}

	if (no_change)
		atomic_inc(&hip->hip_priv->stats.spurious_irqs);

	if (!atomic_read(&hip->hip_priv->closing)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
		scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_RFB]);
	}

	if (slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_tx)) {
		slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_tx);
	}

	bh_end_fb = ktime_get();
	spin_unlock_bh(&hip_priv->rx_lock);
}

static void hip4_irq_handler_fb(int irq, void *data)
{
	struct slsi_hip4    *hip = (struct slsi_hip4 *)data;
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);

	intr_received_fb = ktime_get();

	if (!slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_tx)) {
		slsi_wake_lock_timeout(&hip->hip_priv->hip4_wake_lock_tx, msecs_to_jiffies(SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS));
	}

	if (!atomic_read(&hip->hip_priv->watchdog_timer_active)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 1);
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ);
	}
	set_bit(HIP4_MIF_Q_FH_RFB, hip->hip_priv->irq_bitmap);

	scsc_service_mifintrbit_bit_mask(sdev->service, irq);
	tasklet_hi_schedule(&hip->hip_priv->intr_tl_fb);

	/* Clear interrupt */
	scsc_service_mifintrbit_bit_clear(sdev->service, irq);
}

static void hip4_wq_ctrl(struct work_struct *data)
{
	struct hip4_priv        *hip_priv = container_of(data, struct hip4_priv, intr_wq_ctrl);
	struct slsi_hip4        *hip;
	struct hip4_hip_control *ctrl;
	struct scsc_service     *service;
	struct slsi_dev         *sdev;
	u8						retry;
	bool                    no_change = true;
	u8                      idx_r;
	u8                      idx_w;
	scsc_mifram_ref         ref;
	void                    *mem;
	struct mbulk            *m;

	if (!hip_priv || !hip_priv->hip) {
		SLSI_ERR_NODEV("hip_priv or hip_priv->hip is Null\n");
		return;
	}

	hip = hip_priv->hip;
	ctrl = hip->hip_control;

	if (!ctrl) {
		SLSI_ERR_NODEV("hip->hip_control is Null\n");
		return;
	}
	sdev = container_of(hip, struct slsi_dev, hip4_inst);

	if (!sdev || !sdev->service) {
		SLSI_ERR_NODEV("sdev or sdev->service is Null\n");
		return;
	}

	spin_lock_bh(&hip_priv->rx_lock);
	service = sdev->service;
	bh_init_ctrl = ktime_get();
	clear_bit(HIP4_MIF_Q_TH_CTRL, hip->hip_priv->irq_bitmap);

	idx_r = hip4_read_index(hip, HIP4_MIF_Q_TH_CTRL, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_TH_CTRL, widx);

	while (idx_r != idx_w) {
		struct sk_buff *skb;
		/* TODO: currently the max number to be freed is 2. In future
		 * implementations (i.e. AMPDU) this number may be bigger
		 * list of mbulks to be freedi
		 */
		scsc_mifram_ref to_free[MBULK_MAX_CHAIN + 1] = { 0 };
		u8              i = 0;

		no_change = false;
		/* Catch-up with idx_w */
		ref = ctrl->q[HIP4_MIF_Q_TH_CTRL].array[idx_r];
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)(mem);
		/* Process Control Signal */
		skb = hip4_mbulk_to_skb(service, hip_priv, m, to_free, false);
		if (!skb) {
			SLSI_ERR_NODEV("Ctrl: Error parsing or allocating skb\n");
			hip4_dump_dbg(hip, m, skb, service);
			goto consume_ctl_mbulk;
		}

		if (m->flag & MBULK_F_WAKEUP) {
			SLSI_INFO(sdev, "WIFI wakeup by MLME frame 0x%x:\n", fapi_get_sigid(skb));
			SCSC_BIN_TAG_INFO(BINARY, skb->data, skb->len > 128 ? 128 : skb->len);
			slsi_skb_cb_get(skb)->wakeup = true;
		}

		if (slsi_hip_rx(sdev, skb) < 0) {
			SLSI_ERR_NODEV("Ctrl: Error detected slsi_hip_rx\n");
			hip4_dump_dbg(hip, m, skb, service);
			kfree_skb(skb);
		}
consume_ctl_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);

		/* Go through the list of references to free */
		while ((ref = to_free[i++])) {
			/* Set the number of retries */
			retry = FB_NO_SPC_NUM_RET;
			/* return to the firmware */
			while (hip4_q_add_signal(hip, HIP4_MIF_Q_TH_RFB, ref, service) && (!atomic_read(&hip->hip_priv->closing)) && (retry > 0)) {
				SLSI_WARN_NODEV("Ctrl: Not enough space in FB, retry: %d/%d\n", retry, FB_NO_SPC_NUM_RET);
				spin_unlock_bh(&hip_priv->rx_lock);
				msleep(FB_NO_SPC_SLEEP_MS);
				spin_lock_bh(&hip_priv->rx_lock);
				retry--;
				if (retry == 0)
					SLSI_ERR_NODEV("Ctrl: FB has not been freed for %d ms\n", FB_NO_SPC_NUM_RET * FB_NO_SPC_SLEEP_MS);
			}
		}
		hip4_update_index(hip, HIP4_MIF_Q_TH_CTRL, ridx, idx_r);
	}

	if (no_change)
		atomic_inc(&hip->hip_priv->stats.spurious_irqs);

	if (!atomic_read(&hip->hip_priv->closing)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
		scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_CTRL]);
	}

	if (slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_ctrl)) {
		slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_ctrl);
	}

	bh_end_ctrl = ktime_get();
	spin_unlock_bh(&hip_priv->rx_lock);
}

static void hip4_irq_handler_ctrl(int irq, void *data)
{
	struct slsi_hip4    *hip = (struct slsi_hip4 *)data;
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);

	intr_received_ctrl = ktime_get();

	if (!slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_ctrl)) {
		slsi_wake_lock_timeout(&hip->hip_priv->hip4_wake_lock_ctrl, msecs_to_jiffies(SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS));
	}

	if (!atomic_read(&hip->hip_priv->watchdog_timer_active)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 1);
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ);
	}
	set_bit(HIP4_MIF_Q_TH_CTRL, hip->hip_priv->irq_bitmap);

	scsc_service_mifintrbit_bit_mask(sdev->service, irq);

	if (hip4_system_wq)
		schedule_work(&hip->hip_priv->intr_wq_ctrl);
	else
		queue_work(hip->hip_priv->hip4_workq, &hip->hip_priv->intr_wq_ctrl);

	/* Clear interrupt */
	scsc_service_mifintrbit_bit_clear(sdev->service, irq);
}

void hip4_sched_wq_ctrl(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);

	if (!hip || !sdev || !sdev->service || !hip->hip_priv)
		return;

	if (atomic_read(&hip->hip_priv->closing))
		return;

	hip4_dump_dbg(hip, NULL, NULL, sdev->service);

	if (!slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_ctrl)) {
		slsi_wake_lock_timeout(&hip->hip_priv->hip4_wake_lock_ctrl, msecs_to_jiffies(SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS));
	}

	SLSI_DBG1(sdev, SLSI_HIP, "Trigger wq for skipped ctrl BH\n");

	if (hip4_system_wq)
		schedule_work(&hip->hip_priv->intr_wq_ctrl);
	else
		if(!queue_work(hip->hip_priv->hip4_workq, &hip->hip_priv->intr_wq_ctrl))
			SLSI_DBG1(sdev, SLSI_HIP, "hip4_wq_ctrl is already scheduled\n");
}

static int hip4_napi_poll(struct napi_struct *napi, int budget)
{
	struct hip4_priv        *hip_priv = container_of(napi, struct hip4_priv, napi);
	struct slsi_hip4        *hip;
	struct hip4_hip_control *ctrl;
	struct scsc_service     *service;
	struct slsi_dev         *sdev;

	u8                      idx_r;
	u8                      idx_w;
	scsc_mifram_ref         ref;
	void                    *mem;
	struct mbulk            *m;
	u8                      retry;
	u8                      todo = 0;
	int work_done = 0;

	spin_lock_bh(&in_napi_context);
	if (!hip_priv || !hip_priv->hip) {
		SLSI_ERR_NODEV("hip_priv or hip_priv->hip is Null\n");
		spin_unlock_bh(&in_napi_context);
		return 0;
	}

	hip = hip_priv->hip;
	if (!hip || !hip->hip_priv) {
		SLSI_ERR_NODEV("either hip or hip->hip_priv is Null\n");
		spin_unlock_bh(&in_napi_context);
		return 0;
	}

	ctrl = hip->hip_control;

	if (!ctrl) {
		SLSI_ERR_NODEV("hip->hip_control is Null\n");
		spin_unlock_bh(&in_napi_context);
		return 0;
	}
	sdev = container_of(hip, struct slsi_dev, hip4_inst);

	if (!sdev || !sdev->service) {
		SLSI_ERR_NODEV("sdev or sdev->service is Null\n");
		spin_unlock_bh(&in_napi_context);
		return 0;
	}

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED) {
		napi_complete_done(napi, 0);
		spin_unlock_bh(&in_napi_context);
		return 0;
	}

	spin_lock_bh(&hip_priv->rx_lock);
	if (ktime_compare(bh_init_data, bh_end_data) <= 0) {
		bh_init_data = ktime_get();
		if (!atomic_read(&hip->hip_priv->closing))
			atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	}
	clear_bit(HIP4_MIF_Q_TH_DAT, hip->hip_priv->irq_bitmap);

	idx_r = hip4_read_index(hip, HIP4_MIF_Q_TH_DAT, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_TH_DAT, widx);

	service = sdev->service;

	if (idx_r == idx_w) {
		SLSI_DBG3(sdev, SLSI_RX, "nothing to do, NAPI Complete\n");
		hip->hip_priv->napi_rx_full_cnt = 0;
		hip_priv->napi_rx_saturated = 0;
		bh_end_data = ktime_get();
		napi_complete_done(napi, 0);
		if (!atomic_read(&hip->hip_priv->closing)) {
			/* Nothing more to drain, unmask interrupt */
			scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
		}

		if (slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_data)) {
			slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_data);
		}

		goto end;
	}


	todo = ((idx_w - idx_r) & 0xff);
	SLSI_DBG3(sdev, SLSI_RX, "todo:%hhu\n", todo);

	/* check if Rx queue is saturating and if so move to performance mode */
	if (!napi_select_cpu && !hip_priv->napi_perf_mode && !hip_priv->napi_rx_saturated) {
		if (todo > hip4_napi_num_rx_pkts)
		{
			if (++hip_priv->napi_rx_full_cnt >= hip4_napi_num_rx_full) {
				SLSI_INFO(sdev, "Rx queue saturating.. move to performance mode\n");
				hip_priv->napi_rx_saturated = 1;

				/* signal to traffic monitor to notify clients */
				slsi_traffic_mon_override(sdev);
			}
		} else {
			hip_priv->napi_rx_full_cnt = 0;
		}
	}

	while (idx_r != idx_w) {
		struct sk_buff *skb;
		/* TODO: currently the max number to be freed is 2. In future
		 * implementations (i.e. AMPDU) this number may be bigger
		 */
		/* list of mbulks to be freed */
		scsc_mifram_ref to_free[MBULK_MAX_CHAIN + 1] = { 0 };
		u8              i = 0;

		/* Catch-up with idx_w */
		ref = ctrl->q[HIP4_MIF_Q_TH_DAT].array[idx_r];
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)(mem);
		skb = hip4_mbulk_to_skb(service, hip_priv, m, to_free, true);
		if (!skb) {
			SLSI_ERR_NODEV("Dat: Error parsing or allocating skb\n");
			hip4_dump_dbg(hip, m, skb, service);
			goto consume_dat_mbulk;
		}

		if (m->flag & MBULK_F_WAKEUP) {
			SLSI_INFO(sdev, "WIFI wakeup by DATA frame:\n");
			SCSC_BIN_TAG_INFO(BINARY, fapi_get_data(skb), fapi_get_datalen(skb) > 54 ? 54 : fapi_get_datalen(skb));
			slsi_skb_cb_get(skb)->wakeup = true;
		}

		if (slsi_hip_rx(sdev, skb) < 0) {
			SLSI_ERR_NODEV("Dat: Error detected slsi_hip_rx\n");
			hip4_dump_dbg(hip, m, skb, service);
			kfree_skb(skb);
		}
consume_dat_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);

		while ((ref = to_free[i++])) {
			/* Set the number of retries */
			retry = FB_NO_SPC_NUM_RET;
			while (hip4_q_add_signal(hip, HIP4_MIF_Q_TH_RFB, ref, service) && (!atomic_read(&hip->hip_priv->closing)) && (retry > 0)) {
				SLSI_WARN_NODEV("Dat: Not enough space in FB, retry: %d/%d\n", retry, FB_NO_SPC_NUM_RET);
				udelay(FB_NO_SPC_DELAY_US);
				retry--;

				if (retry == 0)
					SLSI_ERR_NODEV("Dat: FB has not been freed for %d us\n", FB_NO_SPC_NUM_RET * FB_NO_SPC_DELAY_US);
			}
		}

		work_done++;
		if (budget == work_done) {
			/* We have consumed all the bugdet */
			break;
		}
	}

	hip4_update_index(hip, HIP4_MIF_Q_TH_DAT, ridx, idx_r);

	if (work_done < budget) {
		SLSI_DBG3(sdev, SLSI_RX, "NAPI complete (work_done:%d)\n", work_done);
		bh_end_data = ktime_get();
		hip->hip_priv->napi_rx_full_cnt = 0;
		hip_priv->napi_rx_saturated = 0;
		napi_complete_done(napi, work_done);
		if (!atomic_read(&hip->hip_priv->closing)) {
			/* Nothing more to drain, unmask interrupt */
			scsc_service_mifintrbit_bit_unmask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
		}
		if (slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_data)) {
			slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_data);
		}
	}
end:
	SLSI_DBG3(sdev, SLSI_RX, "work done:%d\n", work_done);
	spin_unlock_bh(&hip_priv->rx_lock);
	spin_unlock_bh(&in_napi_context);
	return work_done;
}

static void hip4_irq_data_napi_switch_work(struct work_struct *work)
{
	struct hip4_priv *hip_priv = container_of(work, struct hip4_priv, intr_wq_napi_cpu_switch);
	local_bh_disable();
	napi_schedule(&hip_priv->napi);
	local_bh_enable();
}

static void hip4_irq_handler_dat(int irq, void *data)
{
	struct slsi_hip4    *hip = (struct slsi_hip4 *)data;
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	unsigned long flags;

	if (!hip || !sdev || !sdev->service || !hip->hip_priv)
		return;

	intr_received_data = ktime_get();

	if (!slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_data)) {
		slsi_wake_lock_timeout(&hip->hip_priv->hip4_wake_lock_data, msecs_to_jiffies(SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS));
	}

	if (!atomic_read(&hip->hip_priv->watchdog_timer_active)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 1);
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ);
	}
	set_bit(HIP4_MIF_Q_TH_DAT, hip->hip_priv->irq_bitmap);

	/* Mask interrupt to avoid interrupt storm and let BH run */
	scsc_service_mifintrbit_bit_mask(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);

	spin_lock_irqsave(&hip->hip_priv->napi_cpu_lock, flags);
	if (test_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state)) {
		if (napi_select_cpu && (napi_select_cpu != smp_processor_id()) && cpu_online(napi_select_cpu))
			/* queue work on system_wq. Do not use hip4_workq as
			 * it is single thread wq and WQ_UNBOUND wouldnt be
			 * set. What it means? its not garunteed to run on
			 * intended CPU if wq is created as single threaded
			 * wq.
			 */
			schedule_work_on(napi_select_cpu, &hip->hip_priv->intr_wq_napi_cpu_switch);
		else
			napi_schedule(&hip->hip_priv->napi);

		scsc_service_mifintrbit_bit_clear(sdev->service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
	}
	spin_unlock_irqrestore(&hip->hip_priv->napi_cpu_lock, flags);
}



int hip4_init(struct slsi_hip4 *hip)
{
	void                    *hip_ptr;
	struct hip4_hip_control *hip_control;
	struct scsc_service     *service;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	scsc_mifram_ref         ref, ref_scoreboard;
	int                     i;
	int                     ret;
	u32                     total_mib_len;
	u32                     mib_file_offset;
	struct net_device       *dev;

	if (!sdev || !sdev->service)
		return -EINVAL;

	hip->hip_priv = kzalloc(sizeof(*hip->hip_priv), GFP_ATOMIC);
	if (!hip->hip_priv)
		return -ENOMEM;

	SLSI_INFO_NODEV("HIP4_WLAN_CONFIG_SIZE (%d)\n", HIP4_WLAN_CONFIG_SIZE);
	SLSI_INFO_NODEV("HIP4_WLAN_MIB_SIZE (%d)\n", HIP4_WLAN_MIB_SIZE);
	SLSI_INFO_NODEV("HIP4_WLAN_TX_DAT_SIZE (%d)\n", HIP4_WLAN_TX_DAT_SIZE);
	SLSI_INFO_NODEV("HIP4_WLAN_TX_CTL_SIZE (%d)\n", HIP4_WLAN_TX_CTL_SIZE);
	SLSI_INFO_NODEV("HIP4_WLAN_RX_SIZE (%d)\n", HIP4_WLAN_RX_SIZE);
	SLSI_INFO_NODEV("HIP4_WLAN_TOTAL_MEM (%d)\n", HIP4_WLAN_TOTAL_MEM);
	SLSI_INFO_NODEV("HIP4_DAT_SLOTS (%d)\n", HIP4_DAT_SLOTS);
	SLSI_INFO_NODEV("HIP4_CTL_SLOTS (%d)\n", HIP4_CTL_SLOTS);

	/* Used in the workqueue */
	hip->hip_priv->hip = hip;

	service = sdev->service;

	hip->hip_priv->host_pool_id_dat = MBULK_POOL_ID_DATA;
	hip->hip_priv->host_pool_id_ctl = MBULK_POOL_ID_CTRL;

	/* hip_ref contains the reference of the start of shared memory allocated for WLAN */
	/* hip_ptr is the kernel address of hip_ref*/
	hip_ptr = scsc_mx_service_mif_addr_to_ptr(service, hip->hip_ref);

	/* Configure mbulk allocator - Data QUEUES */
	ret = mbulk_pool_add(MBULK_POOL_ID_DATA, hip_ptr + HIP4_WLAN_TX_DAT_OFFSET,
			     hip_ptr + HIP4_WLAN_TX_DAT_OFFSET + HIP4_WLAN_TX_DAT_SIZE,
			     (HIP4_WLAN_TX_DAT_SIZE / HIP4_DAT_SLOTS) - sizeof(struct mbulk), 5);
	if (ret)
		return ret;

	/* Configure mbulk allocator - Control QUEUES */
	ret = mbulk_pool_add(MBULK_POOL_ID_CTRL, hip_ptr + HIP4_WLAN_TX_CTL_OFFSET,
			     hip_ptr + HIP4_WLAN_TX_CTL_OFFSET + HIP4_WLAN_TX_CTL_SIZE,
			    (HIP4_WLAN_TX_CTL_SIZE / HIP4_CTL_SLOTS) - sizeof(struct mbulk), 0);
	if (ret)
		return ret;

	/* Reset hip_control table */
	memset(hip_ptr, 0, sizeof(struct hip4_hip_control));

	/* Reset Sample q values sending 0xff */

	/***** VERSION 4 *******/
	/* TOHOST Handler allocator */
	/* Q0 FH CTRL */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_CTRL] = MIF_NO_IRQ;
	/* Q1 FH DATA */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_DAT] = MIF_NO_IRQ;
	/* Q5 TH RFB */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_RFB] = MIF_NO_IRQ;
	/* Q2 FH FB */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_RFB] =
		scsc_service_mifintrbit_register_tohost(service, hip4_irq_handler_fb, hip);
	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_RFB]);
	/* Q3 TH CTRL */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_CTRL] =
		scsc_service_mifintrbit_register_tohost(service, hip4_irq_handler_ctrl, hip);
	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_CTRL]);
	/* Q4 TH DAT */
	hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT] =
		scsc_service_mifintrbit_register_tohost(service, hip4_irq_handler_dat, hip);
	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);

	rcu_read_lock();
	/* one NAPI instance is ok for multiple netdev devices */
	dev = slsi_get_netdev_rcu(sdev, SLSI_NET_INDEX_WLAN);
	if (!dev) {
		SLSI_ERR(sdev, "netdev No longer exists\n");
		rcu_read_unlock();
		return -EINVAL;
	}
	netif_napi_add(dev, &hip->hip_priv->napi, hip4_napi_poll);
	rcu_read_unlock();

	/* FROMHOST Handler allocator */
	hip->hip_priv->intr_fromhost =
		scsc_service_mifintrbit_alloc_fromhost(service, SCSC_MIFINTR_TARGET_R4);

	/* Get hip_control pointer on shared memory  */
	hip_control = (struct hip4_hip_control *)(hip_ptr +
		      HIP4_WLAN_CONFIG_OFFSET);

	/* Initialize scoreboard */
	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->scoreboard, &ref_scoreboard))
		return -EFAULT;

	/* Calculate total space used by wlan*.hcf files */
	for (i = 0, total_mib_len = 0; i < SLSI_WLAN_MAX_MIB_FILE; i++)
		total_mib_len += sdev->mib[i].mib_len;

	/* Copy MIB content in shared memory if any */
	/* Clear the area to avoid picking up old values */
	memset(hip_ptr + HIP4_WLAN_MIB_OFFSET, 0, HIP4_WLAN_MIB_SIZE);

	if (total_mib_len > HIP4_WLAN_MIB_SIZE) {
		SLSI_ERR_NODEV("MIB size (%d), is bigger than the MIB AREA (%d). Aborting memcpy\n", total_mib_len, HIP4_WLAN_MIB_SIZE);
		hip_control->config_v4.mib_loc      = 0;
		hip_control->config_v4.mib_sz       = 0;
		hip_control->config_v5.mib_loc      = 0;
		hip_control->config_v5.mib_sz       = 0;
		total_mib_len = 0;
	} else if (total_mib_len) {
		SLSI_INFO_NODEV("Loading MIB into shared memory, size (%d)\n", total_mib_len);
		/* Load each MIB file into shared DRAM region */
		for (i = 0, mib_file_offset = 0;
		     i < SLSI_WLAN_MAX_MIB_FILE;
		     i++) {
			SLSI_INFO_NODEV("Loading MIB %d into shared memory, offset (%d), size (%d), total (%d)\n", i, mib_file_offset, sdev->mib[i].mib_len, total_mib_len);
			if (sdev->mib[i].mib_len) {
				memcpy((u8 *)hip_ptr + HIP4_WLAN_MIB_OFFSET + mib_file_offset, sdev->mib[i].mib_data, sdev->mib[i].mib_len);
				mib_file_offset += sdev->mib[i].mib_len;
			}
		}
		hip_control->config_v4.mib_loc      = hip->hip_ref + HIP4_WLAN_MIB_OFFSET;
		hip_control->config_v4.mib_sz       = total_mib_len;
		hip_control->config_v5.mib_loc      = hip->hip_ref + HIP4_WLAN_MIB_OFFSET;
		hip_control->config_v5.mib_sz       = total_mib_len;
	} else {
		hip_control->config_v4.mib_loc      = 0;
		hip_control->config_v4.mib_sz       = 0;
		hip_control->config_v5.mib_loc      = 0;
		hip_control->config_v5.mib_sz       = 0;
	}

	/* Initialize hip_control table for version 4 */
	/***** VERSION 4 *******/
	hip_control->config_v4.magic_number = 0xcaba0401;
	hip_control->config_v4.hip_config_ver = 4;
	hip_control->config_v4.config_len = sizeof(struct hip4_hip_config_version_4);
	hip_control->config_v4.host_cache_line = 64;
	hip_control->config_v4.host_buf_loc = hip->hip_ref + HIP4_WLAN_TX_OFFSET;
	hip_control->config_v4.host_buf_sz  = HIP4_WLAN_TX_SIZE;
	hip_control->config_v4.fw_buf_loc   = hip->hip_ref + HIP4_WLAN_RX_OFFSET;
	hip_control->config_v4.fw_buf_sz    = HIP4_WLAN_RX_SIZE;
	hip_control->config_v4.log_config_loc = 0;

	hip_control->config_v4.mif_fh_int_n = hip->hip_priv->intr_fromhost;
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		hip_control->config_v4.mif_th_int_n[i] = hip->hip_priv->intr_tohost_mul[i];
	}

	hip_control->config_v4.scbrd_loc = (u32)ref_scoreboard;
	hip_control->config_v4.q_num = 6;
	hip_control->config_v4.q_len = 256;
	hip_control->config_v4.q_idx_sz = 1;
	/* Initialize q relative positions */
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->q[i].array, &ref))
			return -EFAULT;
		hip_control->config_v4.q_loc[i] = (u32)ref;
	}
	/***** END VERSION 4 *******/

	/* Initialize hip_control table for version 5 */
	/***** VERSION 5 *******/
	hip_control->config_v5.magic_number = 0xcaba0401;
	hip_control->config_v5.hip_config_ver = 5;
	hip_control->config_v5.config_len = sizeof(struct hip4_hip_config_version_5);
	hip_control->config_v5.host_cache_line = 64;
	hip_control->config_v5.host_buf_loc = hip->hip_ref + HIP4_WLAN_TX_OFFSET;
	hip_control->config_v5.host_buf_sz  = HIP4_WLAN_TX_SIZE;
	hip_control->config_v5.fw_buf_loc   = hip->hip_ref + HIP4_WLAN_RX_OFFSET;
	hip_control->config_v5.fw_buf_sz    = HIP4_WLAN_RX_SIZE;
	hip_control->config_v5.log_config_loc = 0;
	hip_control->config_v5.mif_fh_int_n = hip->hip_priv->intr_fromhost;
	hip_control->config_v5.mif_th_int_n = hip->hip_priv->intr_tohost;
	hip_control->config_v5.q_num = 6;
	hip_control->config_v5.q_len = 256;
	hip_control->config_v5.q_idx_sz = 1;
	hip_control->config_v5.scbrd_loc = (u32)ref_scoreboard; /* scoreborad location */

	/* Initialize q relative positions */
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->q[i].array, &ref))
			return -EFAULT;
		hip_control->config_v5.q_loc[i] = (u32)ref;
	}
	/***** END VERSION 5 *******/

	/* Initialzie hip_init configuration */
	hip_control->init.magic_number = 0xcaaa0400;
	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->config_v4, &ref))
		return -EFAULT;
	hip_control->init.version_a_ref = ref;

	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->config_v5, &ref))
		return -EFAULT;
	hip_control->init.version_b_ref = ref;
	/* End hip_init configuration */

	hip->hip_control = hip_control;
	hip->hip_priv->scbrd_base = &hip_control->scoreboard;

	spin_lock_init(&hip->hip_priv->rx_lock);
	atomic_set(&hip->hip_priv->in_rx, 0);
	spin_lock_init(&hip->hip_priv->tx_lock);
	atomic_set(&hip->hip_priv->in_tx, 0);

	slsi_wake_lock_init(NULL, &hip->hip_priv->hip4_wake_lock_tx.ws, "hip4_wake_lock_tx");
	slsi_wake_lock_init(NULL, &hip->hip_priv->hip4_wake_lock_ctrl.ws, "hip4_wake_lock_ctrl");
	slsi_wake_lock_init(NULL, &hip->hip_priv->hip4_wake_lock_data.ws, "hip4_wake_lock_data");
	slsi_wake_lock_init(NULL, &hip->hip_priv->hip4_wake_lock.ws, "hip4_wake_lock");

	/* Init work structs */
	hip->hip_priv->hip4_workq = create_singlethread_workqueue("hip4_work");
	if (!hip->hip_priv->hip4_workq) {
		SLSI_ERR_NODEV("Error creating singlethread_workqueue\n");
		return -ENOMEM;
	}
	spin_lock_init(&hip->hip_priv->napi_cpu_lock);
	INIT_WORK(&hip->hip_priv->intr_wq_napi_cpu_switch, hip4_irq_data_napi_switch_work);
	INIT_WORK(&hip->hip_priv->intr_wq_ctrl, hip4_wq_ctrl);
	tasklet_init(&hip->hip_priv->intr_tl_fb, hip4_tl_fb, (unsigned long)hip);
	rwlock_init(&hip->hip_priv->rw_scoreboard);

	/* Setup watchdog timer */
	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	spin_lock_init(&hip->hip_priv->watchdog_lock);
	timer_setup(&hip->hip_priv->watchdog, hip4_watchdog, 0);

	atomic_set(&hip->hip_priv->gmod, HIP4_DAT_SLOTS);
	atomic_set(&hip->hip_priv->gactive, 1);
	spin_lock_init(&hip->hip_priv->gbot_lock);
	hip->hip_priv->saturated = 0;

	return 0;
}

/**
 * This function returns the number of free slots available to
 * transmit control packet.
 */
int hip4_free_ctrl_slots_count(struct slsi_hip4 *hip)
{
	return mbulk_pool_get_free_count(MBULK_POOL_ID_CTRL);
}

static bool slsi_hip_validatesize(struct sk_buff *skb, bool ctrl_packet, u8 head_tail_room)
{
	size_t payload, total_size;
	//u8 headroom, tailroom;
	struct slsi_skb_cb *cb = slsi_skb_cb_get(skb);

	payload = skb->len - cb->sig_length;
//	headroom = hip_priv->unidat_req_headroom;
//	tailroom = hip_priv->unidat_req_tailroom;
	if (payload)
		total_size = cb->sig_length + head_tail_room;
		//total_size = cb->sig_length + headroom + tailroom;
	else
		total_size = cb->sig_length;
	if (mbulk_pool_seg_size(ctrl_packet ? MBULK_POOL_ID_CTRL : MBULK_POOL_ID_DATA) < total_size)
		return false;
	return true;
}

/**
 * This function is in charge to transmit a frame through the HIP.
 * It does NOT take ownership of the SKB unless it successfully transmit it;
 * as a consequence skb is NOT freed on error.
 * We return ENOSPC on queue related troubles in order to trigger upper
 * layers of kernel to requeue/retry.
 * We free ONLY locally-allocated stuff.
 *
 * the vif_index, peer_index, priority fields are valid for data packets only
 */
int scsc_wifi_transmit_frame(struct slsi_hip4 *hip, struct sk_buff *skb, bool ctrl_packet, u8 vif_index, u8 peer_index, u8 priority)
{
	struct scsc_service       *service;
	scsc_mifram_ref           offset;
	struct mbulk              *m;
	mbulk_colour              colour = 0;
	struct slsi_dev           *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct fapi_signal_header *fapi_header;
	int                       ret = 0;

	if (!hip || !sdev || !sdev->service || !skb || !hip->hip_priv)
		return -EINVAL;

	spin_lock_bh(&hip->hip_priv->tx_lock);
	atomic_set(&hip->hip_priv->in_tx, 1);

	if (!slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_tx)) {
		slsi_wake_lock_timeout(&hip->hip_priv->hip4_wake_lock_tx, msecs_to_jiffies(SLSI_HIP_WAKELOCK_TIME_OUT_IN_MS));
	}

	service = sdev->service;
	fapi_header = (struct fapi_signal_header *)skb->data;

	if (fapi_is_ma(skb))
		SLSI_MBULK_COLOUR_SET(colour, vif_index, peer_index, priority);

	m = hip4_skb_to_mbulk(hip->hip_priv, skb, ctrl_packet, colour);
	if (!m) {
		if (!slsi_hip_validatesize(skb, ctrl_packet, 
					   hip->hip_priv->unidat_req_headroom + hip->hip_priv->unidat_req_tailroom)) {
			ret = -ENOSPC;
			SLSI_ERR_NODEV("mbulk is NULL\n");
		} else {
			ret = -EINVAL;
			SLSI_ERR_NODEV("High payload length\n");
		}
		goto error;
	}

	if (scsc_mx_service_mif_ptr_to_addr(service, m, &offset) < 0) {
		mbulk_free_virt_host(m);
		ret = -EFAULT;
		SLSI_ERR_NODEV("Incorrect reference memory\n");
		goto error;
	}

	if (hip4_q_add_signal(hip, ctrl_packet ? HIP4_MIF_Q_FH_CTRL : HIP4_MIF_Q_FH_DAT, offset, service)) {
		mbulk_free_virt_host(m);
		ret = -ENOSPC;
		SLSI_ERR_NODEV("No space\n");
		goto error;
	}


	/* Here we push a copy of the bare skb TRANSMITTED data also to the logring
	 * as a binary record. Note that bypassing UDI subsystem as a whole
	 * means we are losing:
	 *   UDI filtering / UDI Header INFO / UDI QueuesFrames Throttling /
	 *   UDI Skb Asynchronous processing
	 * We keep separated DATA/CTRL paths.
	 */
	if (ctrl_packet)
		SCSC_BIN_TAG_DEBUG(BIN_WIFI_CTRL_TX, skb->data, skb_headlen(skb));
	else
		SCSC_BIN_TAG_DEBUG(BIN_WIFI_DATA_TX, skb->data, skb_headlen(skb));
	consume_skb(skb);
	atomic_set(&hip->hip_priv->in_tx, 0);
	spin_unlock_bh(&hip->hip_priv->tx_lock);
	return 0;

error:
	if (slsi_wake_lock_active(&hip->hip_priv->hip4_wake_lock_tx)) {
		slsi_wake_unlock(&hip->hip_priv->hip4_wake_lock_tx);
	}
	atomic_set(&hip->hip_priv->in_tx, 0);
	spin_unlock_bh(&hip->hip_priv->tx_lock);
	return ret;
}

/* HIP4 has been initialize, setup with values
 * provided by FW
 */
int hip4_setup(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service *service;
	u32 conf_hip4_ver = 0;

	if (!sdev || !sdev->service)
		return -EIO;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return -EIO;

	service = sdev->service;

	/* Get the Version reported by the FW */
	conf_hip4_ver = scsc_wifi_get_hip_config_version(&hip->hip_control->init);
	/* Check if the version is supported. And get the index */
	/* This is hardcoded and may change in future versions */
	if (conf_hip4_ver != 4 && conf_hip4_ver != 5) {
		SLSI_ERR_NODEV("FW HIP config version %d not supported\n", conf_hip4_ver);
		return -EIO;
	}

	if (conf_hip4_ver == 4) {
		hip->hip_priv->unidat_req_headroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_headroom, 4);
		hip->hip_priv->unidat_req_tailroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_tailroom, 4);
		hip->hip_priv->version = 4;

		hip->hip_priv->napi_rx_full_cnt = 0;
		hip->hip_priv->napi_rx_saturated = 0;
		if (!test_and_set_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state))
			napi_enable(&hip->hip_priv->napi);
	} else {
		/* version 5 */
		hip->hip_priv->unidat_req_headroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_headroom, 5);
		hip->hip_priv->unidat_req_tailroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_tailroom, 5);
		hip->hip_priv->version = 5;
	}
	/* Unmask interrupts - now host should handle them */
	atomic_set(&hip->hip_priv->stats.irqs, 0);
	atomic_set(&hip->hip_priv->stats.spurious_irqs, 0);
	atomic_set(&sdev->debug_inds, 0);

	atomic_set(&hip->hip_priv->closing, 0);

	scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_FH_RFB]);
	scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_CTRL]);
	scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[HIP4_MIF_Q_TH_DAT]);
	return 0;
}

/* On suspend hip4 needs to ensure that TH interrupts *are* unmasked */
void hip4_suspend(struct slsi_hip4 *hip)
{
	struct slsi_dev *sdev;
	struct scsc_service *service;
	u8 i;

	if (!hip || !hip->hip_priv)
		return;

	sdev = container_of(hip, struct slsi_dev, hip4_inst);
	if (!sdev || !sdev->service)
		return;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return;

	service = sdev->service;


	atomic_set(&hip->hip_priv->in_suspend, 1);

	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
		if (hip->hip_priv->intr_tohost_mul[i] != MIF_NO_IRQ)
			scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[i]);
}

void hip4_resume(struct slsi_hip4 *hip)
{
	struct slsi_dev *sdev;
	struct scsc_service *service;
	u8 i;

	if (!hip || !hip->hip_priv)
		return;

	sdev = container_of(hip, struct slsi_dev, hip4_inst);
	if (!sdev || !sdev->service)
		return;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return;

	service = sdev->service;

	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
		if (hip->hip_priv->intr_tohost_mul[i] != MIF_NO_IRQ)
			scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->intr_tohost_mul[i]);
	atomic_set(&hip->hip_priv->in_suspend, 0);
}

void hip4_freeze(struct slsi_hip4 *hip)
{
	struct slsi_dev *sdev;
	struct scsc_service *service;
	u8 i;

	if (!hip || !hip->hip_priv)
		return;

	sdev = container_of(hip, struct slsi_dev, hip4_inst);
	if (!sdev || !sdev->service)
		return;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return;

	service = sdev->service;

	closing = ktime_get();
	atomic_set(&hip->hip_priv->closing, 1);

	hip4_dump_dbg(hip, NULL, NULL, service);
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
		if (hip->hip_priv->intr_tohost_mul[i] != MIF_NO_IRQ)
			scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->intr_tohost_mul[i]);

	if (test_and_clear_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state))
		napi_disable(&hip->hip_priv->napi);
	hip->hip_priv->napi_rx_full_cnt = 0;
	hip->hip_priv->napi_rx_saturated = 0;
	cancel_work_sync(&hip->hip_priv->intr_wq_napi_cpu_switch);
	cancel_work_sync(&hip->hip_priv->intr_wq_ctrl);
	tasklet_kill(&hip->hip_priv->intr_tl_fb);
	flush_workqueue(hip->hip_priv->hip4_workq);
	destroy_workqueue(hip->hip_priv->hip4_workq);
	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);

	/* Deactive the wd timer prior its expiration */
	timer_delete_sync(&hip->hip_priv->watchdog);
}

void hip4_deinit(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev;
	struct scsc_service *service;
	u8 i;

	if (!hip || !hip->hip_priv)
		return;

	sdev = container_of(hip, struct slsi_dev, hip4_inst);
	if (!sdev || !sdev->service)
		return;

	service = sdev->service;



	closing = ktime_get();
	atomic_set(&hip->hip_priv->closing, 1);

	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
		if (hip->hip_priv->intr_tohost_mul[i] != MIF_NO_IRQ)
			scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->intr_tohost_mul[i]);

	if (test_and_clear_bit(SLSI_HIP_NAPI_STATE_ENABLED, &hip->hip_priv->napi_state))
		napi_disable(&hip->hip_priv->napi);
	hip->hip_priv->napi_rx_full_cnt = 0;
	hip->hip_priv->napi_rx_saturated = 0;
	cancel_work_sync(&hip->hip_priv->intr_wq_napi_cpu_switch);
	cancel_work_sync(&hip->hip_priv->intr_wq_ctrl);
	tasklet_kill(&hip->hip_priv->intr_tl_fb);

	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
		if (hip->hip_priv->intr_tohost_mul[i] != MIF_NO_IRQ)
			scsc_service_mifintrbit_unregister_tohost(service, hip->hip_priv->intr_tohost_mul[i]);

	netif_napi_del(&hip->hip_priv->napi);
	flush_workqueue(hip->hip_priv->hip4_workq);
	destroy_workqueue(hip->hip_priv->hip4_workq);

	scsc_service_mifintrbit_free_fromhost(service, hip->hip_priv->intr_fromhost, SCSC_MIFINTR_TARGET_R4);

	slsi_wake_lock_destroy(&hip->hip_priv->hip4_wake_lock_tx);
	slsi_wake_lock_destroy(&hip->hip_priv->hip4_wake_lock_ctrl);
	slsi_wake_lock_destroy(&hip->hip_priv->hip4_wake_lock_data);
	slsi_wake_lock_destroy(&hip->hip_priv->hip4_wake_lock);

	/* If we get to that point with rx_lock/tx_lock claimed, trigger BUG() */
	WARN_ON(atomic_read(&hip->hip_priv->in_tx));
	WARN_ON(atomic_read(&hip->hip_priv->in_rx));

	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	/* Deactive the wd timer prior its expiration */
	timer_delete_sync(&hip->hip_priv->watchdog);


	spin_lock_bh(&in_napi_context);
	kfree(hip->hip_priv);

	hip->hip_priv = NULL;
	spin_unlock_bh(&in_napi_context);

	/* remove the pools */
	mbulk_pool_remove(MBULK_POOL_ID_DATA);
	mbulk_pool_remove(MBULK_POOL_ID_CTRL);
}
