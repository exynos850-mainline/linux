// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos s5000ap modem boot and IPC driver.
 */

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/blkdev.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/firmware.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kdev_t.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/sizes.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>

/* mailbox */
#define MBOX_MCUCTLR		0x00
#define  MBOX_MCUCTLR_SWRST	BIT(0)
#define MBOX_INTGR0		0x08
#define MBOX_INTCR0		0x0c
#define MBOX_INTMR0		0x10
#define MBOX_INTSR0		0x14
#define MBOX_INTGR1		0x1c	/* AP2CP interrupt generate */
#define MBOX_INTCR1		0x20
#define MBOX_ISSR(n)		(0x80 + 4 * (n))

#define MBOX_RX_SHIFT		16	/* CP2AP interrupt n is INTSR0 bit 16+n */

/* interrupt numbers */
#define INT_AP2CP_MSG		0
#define INT_AP2CP_WAKEUP	1
#define INT_AP2CP_ACTIVE	3
#define IRQ_CP2AP_MSG		0
#define IRQ_CP2AP_STATUS	2
#define IRQ_CP2AP_ACTIVE	3
#define IRQ_CP2AP_WAKELOCK	8
#define MBOX_RX_IRQS		(BIT(IRQ_CP2AP_MSG) | BIT(IRQ_CP2AP_STATUS) | \
				 BIT(IRQ_CP2AP_ACTIVE) | BIT(IRQ_CP2AP_WAKELOCK))

/* shared status registers */
#define SR_AP2CP_MSG		0
#define SR_CP2AP_MSG		1
#define SR_AP2CP_STATUS		2	/* "ap2cp_united_status" */
#define SR_CP2AP_STATUS		3	/* "cp2ap_united_status" */
#define SR_AP2CP_KERNELTIME	9

/* ap2cp_united_status fields */
#define AP_ST_AP_STATUS		GENMASK(4, 1)
#define AP_ST_PDA_ACTIVE	BIT(5)
#define AP_ST_SYS_REV		GENMASK(13, 6)
#define AP_ST_DS_DET		GENMASK(15, 14)
/* cp2ap_united_status fields */
#define CP_ST_CP_STATUS		GENMASK(4, 1)
#define CP_ST_LTE_ACTIVE	BIT(5)
#define CP_ST_WAKELOCK		BIT(6)
#define CP_ST_FLOWCTL		BIT(2)	/* SHM_FLOWCTL_BIT, status IRQ */
/* ap2cp_kerneltime */
#define KT_SEC			GENMASK(31, 20)
#define KT_USEC			GENMASK(19, 0)

/* control word in ap2cp_msg / cp2ap_msg */
#define MASK_INT_VALID		0x0080
#define MASK_CMD_VALID		0x0040
#define MASK_CMD_FIELD		0x003f
#define MASK_SEND_DATA		0x0001
#define CMD_INIT_START		0x01
#define CMD_INIT_END		0x02
#define CMD_CRASH_RESET		0x07
#define CMD_PHONE_START		0x08
#define CMD_CRASH_EXIT		0x09
#define CMD_PIF_INIT_DONE	0x0d
#define CMD(x)			(MASK_INT_VALID | MASK_CMD_VALID | (x))

/* memory map */
#define CP_OFF			0x0
#define CP_SIZE			0x6900000
#define VSS_OFF			0x6900000
#define VSS_SIZE		0x700000
#define IPC_OFF_DEFAULT		0x7000000
#define IPC_SIZE_DEFAULT	0x800000
#define BTL_OFF			0x7800000
#define BTL_SIZE		0x1000000
#define RMEM_SIZE_MIN		(BTL_OFF + BTL_SIZE)

/* boot-time control block at start of IPC region */
#define LEG_MAGIC		0x00
#define LEG_ACCESS		0x04
#define LEG_QPTRS		0x08	/* FMT/RAW head+tail, 0x08-0x27 */
#define LEG_QPTRS_END		0x28
#define MAGIC_BOOT		0x424f4f54	/* "BOOT" */
#define MAGIC_IPC		0xaa

/* SBD layout */
#define SBD_CTRL_SIZE		SZ_1K
#define SBD_CMD_SIZE		SZ_1K
#define SBD_DESC_OFF		(SBD_CTRL_SIZE + SBD_CMD_SIZE)
#define SBD_DESC_SIZE		(SZ_64K - SBD_DESC_OFF)
#define SBD_BUFF_OFF		SZ_64K
#define SBD_BOOTLOG_SIZE	SZ_4K	/* CONFIG_SBD_BOOTLOG tail, kept free */
#define SBD_MAX_IDS		32
#define SBD_VERSION		1
#define S5K_UL			0
#define S5K_DL			1
#define S5K_ULDL			2

struct s5k_sbd_rb_desc {
	__le16 ch, reserved, direction, signaling;
	__le32 sig_mask;
	__le16 length, id, buff_size, payload_offset;
} __packed;

struct s5k_sbd_rb_ch {
	__le32 rb_desc_offset;
	__le32 buff_pos_array_offset;
} __packed;

struct s5k_sbd_gdesc {
	__le32 version, num_channels, rpwp_array_offset;
	struct s5k_sbd_rb_ch rb_ch[SBD_MAX_IDS][S5K_ULDL];
	struct s5k_sbd_rb_desc rb_desc[SBD_MAX_IDS][S5K_ULDL];
} __packed;

/* SIPC5 link header */
#define SIPC5_START_MASK	0xf8
#define SIPC5_EXT_FIELD		0x02
#define SIPC5_CFG_MASK_EXT	0x03
#define SIPC5_MULTI_FRAME	0x03
#define SIPC5_MIN_HDR		4

/* ring config */
enum s5k_port_kind { PK_NONE, PK_AT, PK_SIT, PK_RFS, PK_OEM, PK_PS };

struct s5k_ring_cfg {
	const char	*name;
	u8		ch;		/* SIPC5 channel */
	enum s5k_port_kind kind;
	bool		lnk_hdr;
	u16		ul_n, ul_sz, dl_n, dl_sz;
};

static const struct s5k_ring_cfg s5k_rings[] = {
	{ "umts_cass",       35, PK_OEM, true,   16, 2048,   16, 2048 },
	{ "umts_ipc0",      235, PK_SIT, true,   16, 4096,   32, 4096 },
	{ "umts_ipc1",      236, PK_SIT, true,   16, 4096,   32, 4096 },
	{ "umts_rfs0",      245, PK_RFS, true,   16, 2048,  512, 2048 },
	{ "umts_csd",         1, PK_OEM, true,   32, 2048,   64, 2048 },
	{ "umts_router",     25, PK_AT,  true,   16, 2048,   16, 2048 },
	{ "umts_dm0",        28, PK_OEM, true,   16, 2048,  128, 2048 },
	{ "ipc_loopback0",  244, PK_NONE, true,  16, 2048,   32, 2048 },
	/* multipdp_hiprio / multipdp: shared rings for every rmnetN */
	{ "multipdp_hiprio", 10, PK_PS, false, 256, 2048, 1024, 2048 },
	{ "multipdp",        11, PK_PS, false, 512, 2048, 1024, 2048 },
};
#define S5K_NRINGS		ARRAY_SIZE(s5k_rings)
#define S5K_RING_PS_HI		8
#define S5K_RING_PS_NORM	9

#define S5K_PDP_FIRST		10	/* SIPC_CH_ID_PDP_0 */
#define S5K_PDP_COUNT		8	/* rmnet0..7 */
#define S5K_PS_SLOT		2048

#define S5K_MAX_PORTS		S5K_NRINGS

/* drv data */
enum s5k_state { S5K_OFFLINE, S5K_BOOTING, S5K_IPC, S5K_ONLINE, S5K_CRASH };

struct s5k_rb {
	u16		*rp, *wp;
	__le32		*pos, *lens;
	u8		*buf;
	u16		n, sz;
	struct sk_buff	*pend;		/* multi-slot frame being reassembled */
	u32		pend_left;
};

struct s5k_img { const u8 *data; size_t size; };

struct s5k_ring {
	struct s5k_modem	*sm;
	const struct s5k_ring_cfg *cfg;
	u16			id;
	struct s5k_rb		rb[S5K_ULDL];
	spinlock_t		tx_lock;
	struct wwan_port	*port;
};

/*
 * SEQ_SMC_*: CP_CTRL_S / CP_CTRL_NS (PMU 0x3014 / 0x3090 on Exynos3830) are
 * not writable from the non-secure world so downstream pmucal reaches them with
 * SMC(SMC_ID, READ_CTRL/WRITE_CTRL, ...).  For those ops the "addr" cell is
 * the register selector: 0 = CP_CTRL_S, 1 = CP_CTRL_NS.
 */
enum s5k_seq_op { SEQ_UPDATE, SEQ_WRITE, SEQ_WAIT, SEQ_DELAY_US,
		  SEQ_SMC_UPDATE, SEQ_SMC_WRITE };

struct s5k_seq_op_desc {
	u32		op, mask, val, sel;
	void __iomem	*reg;
};

struct s5k_seq {
	struct s5k_seq_op_desc	*ops;
	int			n;
};

struct s5k_rmnet {
	struct s5k_modem	*sm;
	u8			ch;
};

struct s5k_modem {
	struct device		*dev;

	void __iomem		*mbox;
	void __iomem		*ap2cp_cfg;
	int			mbox_irq, wdt_irq;
	spinlock_t		mbox_lock;	/* SR read-modify-write */
	atomic_t		pending;	/* CP2AP IRQs seen by the hard IRQ */

	/* reserved memory */
	phys_addr_t		rmem_base;
	resource_size_t		rmem_size;
	u32			ipc_off, ipc_size;
	u8			*ipc;		/* memremap(WC) of the IPC region */

	/* firmware */
	const char		*fw_name;
	const char		*fw_part;
	u32			fw_devt[2];
	bool			fw_devt_set;

	struct s5k_seq		seq_init, seq_assert, seq_release;
	struct s5k_seq		seq_dump_on, seq_dump_off;
	void __iomem		*cp_status_reg;	/* NULL: read via SMC */
	u32			cp_status_sel;	/* SMC CP_CTRL selector */
	u32			cp_status_mask;

	u32			hw_rev;

	/* boot state machine */
	struct mutex		boot_lock;
	struct delayed_work	boot_work;
	unsigned int		fw_retries;
	struct completion	init_cmpl;
	enum s5k_state		state;
	bool			sbd_active;
	bool			dev_created;
	u32			mem_map_found;

	/* SBD */
	struct s5k_sbd_gdesc	*gdesc;
	struct s5k_ring		ring[S5K_NRINGS];
	u16			*rpwp;
	wait_queue_head_t	txq_wq;

	/* data */
	struct net_device	*ndev[S5K_PDP_COUNT];
	bool			tx_suspend;
	bool			wdt_on;		/* irq depth tracking */
	bool			wake_held;

	struct dentry		*dbgfs;
};

static int ds_detect = 2;
module_param(ds_detect, int, 0644);
MODULE_PARM_DESC(ds_detect, "Dual-SIM detect: 1 = single SIM, 2 = dual SIM");

static bool verify_vss = true;
module_param(verify_vss, bool, 0644);
MODULE_PARM_DESC(verify_vss, "treat a failed VSS signature check as fatal");

static bool smc_pass_base = true;
module_param(smc_pass_base, bool, 0644);
MODULE_PARM_DESC(smc_pass_base,
		 "OR the CP base address into the NORMAL-boot SMC mode when BOOT carries a MEM map");

static int hw_rev = -1;
module_param(hw_rev, int, 0644);
MODULE_PARM_DESC(hw_rev, "override the board revision sent to the CP (-1 = auto)");

/* mailbox helpers */
static u32 sr_get(struct s5k_modem *sm, unsigned int n)
{
	return readl(sm->mbox + MBOX_ISSR(n));
}

static void sr_set(struct s5k_modem *sm, unsigned int n, u32 v)
{
	writel(v, sm->mbox + MBOX_ISSR(n));
}

static void sr_update(struct s5k_modem *sm, unsigned int n, u32 mask, u32 v)
{
	unsigned long flags;
	u32 r;

	spin_lock_irqsave(&sm->mbox_lock, flags);
	r = sr_get(sm, n);
	r &= ~mask;
	r |= field_prep(mask, v);
	sr_set(sm, n, r);
	spin_unlock_irqrestore(&sm->mbox_lock, flags);
}

static u32 sr_field(struct s5k_modem *sm, unsigned int n, u32 mask)
{
	return field_get(mask, sr_get(sm, n));
}

static void mbox_kick(struct s5k_modem *sm, unsigned int n)
{
	writel(BIT(n), sm->mbox + MBOX_INTGR1);
}

static void send_ipc_irq(struct s5k_modem *sm, u16 val)
{
	sr_set(sm, SR_AP2CP_MSG, val);
	mbox_kick(sm, INT_AP2CP_MSG);
}

static void mbox_unmask(struct s5k_modem *sm)
{
	writel(~(MBOX_RX_IRQS << MBOX_RX_SHIFT) & 0xffff0000,
	       sm->mbox + MBOX_INTMR0);
}

static void mbox_sw_reset(struct s5k_modem *sm)
{
	u32 v = readl(sm->mbox + MBOX_MCUCTLR);

	writel(v | MBOX_MCUCTLR_SWRST, sm->mbox + MBOX_MCUCTLR);
	udelay(5);
	mbox_unmask(sm);
}

/* PMU sequences */
static int s5k_parse_seq(struct s5k_modem *sm, const char *prop,
			 struct s5k_seq *seq, bool required)
{
	struct device_node *np = sm->dev->of_node;
	u32 *cells;
	int n, i, ret;

	n = of_property_count_u32_elems(np, prop);
	if (n < 0)
		return required ? dev_err_probe(sm->dev, n, "missing %s\n", prop) : 0;
	if (n % 4)
		return dev_err_probe(sm->dev, -EINVAL,
				     "%s: need <op addr mask value> tuples\n", prop);

	cells = kcalloc(n, sizeof(*cells), GFP_KERNEL);
	seq->ops = devm_kcalloc(sm->dev, n / 4, sizeof(*seq->ops), GFP_KERNEL);
	if (!cells || !seq->ops) {
		kfree(cells);
		return -ENOMEM;
	}
	ret = of_property_read_u32_array(np, prop, cells, n);
	if (ret)
		goto out;

	seq->n = n / 4;
	for (i = 0; i < seq->n; i++) {
		struct s5k_seq_op_desc *o = &seq->ops[i];

		o->op = cells[4 * i];
		o->mask = cells[4 * i + 2];
		o->val = cells[4 * i + 3];
		if (o->op > SEQ_SMC_WRITE) {
			ret = dev_err_probe(sm->dev, -EINVAL, "%s: bad op %u\n",
					    prop, o->op);
			goto out;
		}
		if (o->op == SEQ_SMC_UPDATE || o->op == SEQ_SMC_WRITE) {
			o->sel = cells[4 * i + 1];
			if (o->sel > 1) {
				ret = dev_err_probe(sm->dev, -EINVAL,
						    "%s: SMC selector must be 0 or 1\n", prop);
				goto out;
			}
		} else if (o->op != SEQ_DELAY_US) {
			o->reg = devm_ioremap(sm->dev, cells[4 * i + 1], 4);
			if (!o->reg) {
				ret = -ENOMEM;
				goto out;
			}
		}
	}
out:
	kfree(cells);
	return ret;
}

#define SMC_ID			0x82000700
#define SMC_ID_CLK		0x82001011
#define SMC_READ_CTRL		0x3
#define SMC_WRITE_CTRL		0x4
#define SSS_CLK_ENABLE		0
#define SSS_CLK_DISABLE		1

static unsigned long s5k_smc(unsigned long a0, unsigned long a1,
			     unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;

	arm_smccc_smc(a0, a1, a2, a3, 0, 0, 0, 0, &res);
	return res.a0;
}

/* pmucal_smc_read(): the secure side returns 16 bits per call, result in [31:16] */
static int s5k_smc_ctrl_read(struct s5k_modem *sm, u32 sel, u32 *val)
{
	u32 lo = s5k_smc(SMC_ID, SMC_READ_CTRL, 0, sel);
	u32 hi;

	if (lo & 0xffff)
		return -EIO;
	hi = s5k_smc(SMC_ID, SMC_READ_CTRL, 1, sel);
	if (hi & 0xffff)
		return -EIO;
	*val = (hi & 0xffff0000) | (lo >> 16);
	return 0;
}

static int s5k_smc_ctrl_write(struct s5k_modem *sm, u32 sel, u32 val)
{
	return (int)s5k_smc(SMC_ID, SMC_WRITE_CTRL, val, sel) > 0 ? -EIO : 0;
}

static int s5k_run_seq(struct s5k_modem *sm, const struct s5k_seq *seq)
{
	int i;

	for (i = 0; i < seq->n; i++) {
		const struct s5k_seq_op_desc *o = &seq->ops[i];
		u32 v;

		switch (o->op) {
		case SEQ_UPDATE:
			v = readl(o->reg);
			v = (v & ~o->mask) | (o->val & o->mask);
			writel(v, o->reg);
			break;
		case SEQ_WRITE:
			writel(o->val, o->reg);
			break;
		case SEQ_WAIT:
			if (readl_poll_timeout(o->reg, v,
					       (v & o->mask) == o->val, 10, 100000)) {
				dev_err(sm->dev, "PMU wait step %d timed out\n", i);
				return -ETIMEDOUT;
			}
			break;
		case SEQ_DELAY_US:
			usleep_range(o->val, o->val + 10);
			break;
		case SEQ_SMC_UPDATE:
		case SEQ_SMC_WRITE:
			v = 0;
			if (o->op == SEQ_SMC_UPDATE || o->mask != U32_MAX) {
				if (s5k_smc_ctrl_read(sm, o->sel, &v)) {
					dev_err(sm->dev, "SMC CP_CTRL read failed (step %d)\n", i);
					return -EIO;
				}
			}
			v = (v & ~o->mask) | (o->val & o->mask);
			if (s5k_smc_ctrl_write(sm, o->sel, v)) {
				dev_err(sm->dev, "SMC CP_CTRL write failed (step %d)\n", i);
				return -EIO;
			}
			break;
		}
	}
	return 0;
}

/* pmucal_cp_status(): "powered" when (reg & mask) == PMU_CP_STATUS_BIT (1) */
static bool s5k_cp_on(struct s5k_modem *sm)
{
	u32 v = 0;

	if (sm->cp_status_reg)
		v = readl(sm->cp_status_reg);
	else if (s5k_smc_ctrl_read(sm, sm->cp_status_sel, &v))
		return false;
	return (v & sm->cp_status_mask) == 1;
}

/* SIPC5 helpers */
static unsigned int sipc5_hdr_len(const u8 *f)
{
	if (f[0] & SIPC5_EXT_FIELD)
		return (f[0] & SIPC5_CFG_MASK_EXT) == SIPC5_MULTI_FRAME ? 5 : 6;
	return SIPC5_MIN_HDR;
}

static unsigned int sipc5_frame_len(const u8 *f)
{
	if ((f[0] & SIPC5_EXT_FIELD) &&
	    (f[0] & SIPC5_CFG_MASK_EXT) != SIPC5_MULTI_FRAME)
		return get_unaligned_le32(f + 2);
	return get_unaligned_le16(f + 2);
}

/* SBD ring buffers */
static u32 sbd_off(struct s5k_modem *sm, const void *p)
{
	return (const u8 *)p - sm->ipc;
}

static unsigned int circ_space(unsigned int q, unsigned int in, unsigned int out)
{
	return in < out ? out - in - 1 : q + out - in - 1;
}

static int s5k_sbd_layout(struct s5k_modem *sm)
{
	u8 *desc = sm->ipc + SBD_DESC_OFF;
	u8 *desc_end = desc + SBD_DESC_SIZE;
	u8 *buf = sm->ipc + SBD_BUFF_OFF;
	u8 *buf_end = sm->ipc + sm->ipc_size - SBD_BOOTLOG_SIZE;
	u16 *rpwp;
	int i, d;

	sm->gdesc = (struct s5k_sbd_gdesc *)desc;
	desc += sizeof(struct s5k_sbd_gdesc);

	rpwp = (u16 *)desc;
	desc += sizeof(u16) * S5K_ULDL * 2 * S5K_NRINGS;
	sm->rpwp = rpwp;

	for (i = 0; i < S5K_NRINGS; i++) {
		struct s5k_ring *r = &sm->ring[i];

		r->sm = sm;
		r->cfg = &s5k_rings[i];
		r->id = i;
		spin_lock_init(&r->tx_lock);

		for (d = 0; d < S5K_ULDL; d++) {
			struct s5k_rb *rb = &r->rb[d];

			rb->n = d == S5K_UL ? r->cfg->ul_n : r->cfg->dl_n;
			rb->sz = d == S5K_UL ? r->cfg->ul_sz : r->cfg->dl_sz;
			/* rp[S5K_UL] wp[S5K_UL] rp[S5K_DL] wp[S5K_DL], each S5K_NRINGS u16 wide */
			rb->rp = rpwp + (2 * d + 0) * S5K_NRINGS + i;
			rb->wp = rpwp + (2 * d + 1) * S5K_NRINGS + i;
			rb->pos = (__le32 *)desc;
			desc += rb->n * sizeof(u32);
			rb->lens = (__le32 *)desc;
			desc += rb->n * sizeof(u32);
			rb->buf = buf;
			buf += (size_t)rb->n * rb->sz;
		}
	}

	if (desc > desc_end || buf > buf_end)
		return dev_err_probe(sm->dev, -ENOSPC,
				     "SBD layout overflows IPC region (desc %zu buf %#zx)\n",
				     (size_t)(desc - (sm->ipc + SBD_DESC_OFF)),
				     (size_t)(buf - sm->ipc));
	return 0;
}

static void s5k_sbd_init(struct s5k_modem *sm)
{
	struct s5k_sbd_gdesc *g = sm->gdesc;
	int i, d, k;

	memset(sm->ipc + SBD_DESC_OFF, 0, SBD_DESC_SIZE);

	g->version = cpu_to_le32(SBD_VERSION);
	g->num_channels = cpu_to_le32(S5K_NRINGS);
	g->rpwp_array_offset = cpu_to_le32(sbd_off(sm, sm->rpwp));

	for (i = 0; i < S5K_NRINGS; i++) {
		struct s5k_ring *r = &sm->ring[i];

		for (d = 0; d < S5K_ULDL; d++) {
			struct s5k_rb *rb = &r->rb[d];
			struct s5k_sbd_rb_desc *rd = &g->rb_desc[i][d];

			rb->pend = NULL;
			rb->pend_left = 0;
			for (k = 0; k < rb->n; k++)
				rb->pos[k] = cpu_to_le32(sbd_off(sm, rb->buf + k * rb->sz));

			rd->ch = cpu_to_le16(r->cfg->ch);
			rd->direction = cpu_to_le16(d);
			rd->signaling = cpu_to_le16(1);
			rd->sig_mask = cpu_to_le32(MASK_INT_VALID | MASK_SEND_DATA);
			rd->length = cpu_to_le16(rb->n);
			rd->id = cpu_to_le16(i);
			rd->buff_size = cpu_to_le16(rb->sz);
			rd->payload_offset = 0;

			g->rb_ch[i][d].rb_desc_offset = cpu_to_le32(sbd_off(sm, rd));
			g->rb_ch[i][d].buff_pos_array_offset =
				cpu_to_le32(sbd_off(sm, rb->pos));
		}
	}
	dma_wmb();
}

static int s5k_ring_send(struct s5k_ring *r, const void *data, u32 len,
			 u8 ps_ch)
{
	struct s5k_modem *sm = r->sm;
	struct s5k_rb *rb = &r->rb[S5K_UL];
	unsigned long flags;
	u32 in, out, total = len;
	u8 *dst;

	if (READ_ONCE(sm->state) != S5K_ONLINE || !smp_load_acquire(&sm->sbd_active))
		return -ENODEV;

	if (r->cfg->lnk_hdr)
		total += SIPC5_MIN_HDR;
	if (!len || total > rb->sz)
		return -EMSGSIZE;

	spin_lock_irqsave(&r->tx_lock, flags);
	in = READ_ONCE(*rb->wp);
	out = READ_ONCE(*rb->rp);
	if (in >= rb->n || out >= rb->n) {
		spin_unlock_irqrestore(&r->tx_lock, flags);
		return -EIO;
	}
	if (!circ_space(rb->n, in, out)) {
		spin_unlock_irqrestore(&r->tx_lock, flags);
		return -ENOSPC;
	}

	dst = rb->buf + in * rb->sz;
	if (r->cfg->lnk_hdr) {
		dst[0] = SIPC5_START_MASK;
		dst[1] = r->cfg->ch;
		put_unaligned_le16(total, dst + 2);
		memcpy(dst + SIPC5_MIN_HDR, data, len);
		rb->lens[in] = cpu_to_le32(total);
	} else {
		memcpy(dst, data, len);
		rb->lens[in] = cpu_to_le32((len & 0xffff) | ((u32)ps_ch << 16));
	}
	dma_wmb();
	WRITE_ONCE(*rb->wp, in + 1 >= rb->n ? 0 : in + 1);
	spin_unlock_irqrestore(&r->tx_lock, flags);

	send_ipc_irq(sm, MASK_INT_VALID | MASK_SEND_DATA);
	return 0;
}

static bool s5k_ring_has_space(struct s5k_ring *r)
{
	struct s5k_rb *rb = &r->rb[S5K_UL];

	return circ_space(rb->n, READ_ONCE(*rb->wp), READ_ONCE(*rb->rp)) > 0;
}

/* rx */
static void s5k_net_rx(struct s5k_modem *sm, u8 ch, const u8 *src, u32 len)
{
	struct net_device *ndev;
	struct sk_buff *skb;

	if (ch < S5K_PDP_FIRST || ch >= S5K_PDP_FIRST + S5K_PDP_COUNT)
		return;
	ndev = READ_ONCE(sm->ndev[ch - S5K_PDP_FIRST]);
	if (!ndev || !len)
		return;

	skb = netdev_alloc_skb(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}
	skb_put_data(skb, src, len);
	skb->protocol = htons((skb->data[0] >> 4) == 6 ? ETH_P_IPV6 : ETH_P_IP);
	skb->dev = ndev;
	skb_reset_mac_header(skb);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	netif_rx(skb);
}

static void s5k_deliver(struct s5k_ring *r, struct sk_buff *skb)
{
	if (r->port)
		wwan_port_rx(r->port, skb);
	else
		kfree_skb(skb);
}

static void s5k_rx_slot(struct s5k_ring *r, struct s5k_rb *rb, u16 out)
{
	struct s5k_modem *sm = r->sm;
	u32 lw = le32_to_cpu(rb->lens[out]);
	u32 len = lw & 0xffff, hdr, flen, plen, first;
	const u8 *src = rb->buf + out * rb->sz;
	struct sk_buff *skb;

	if (len > rb->sz) {
		dev_warn_ratelimited(sm->dev, "ring %s: slot len %u > %u\n",
				     r->cfg->name, len, rb->sz);
		return;
	}

	if (!r->cfg->lnk_hdr) {
		s5k_net_rx(sm, lw >> 16, src, len);
		return;
	}

	if (rb->pend) {			/* continuation of a long frame */
		u32 n = min(len, rb->pend_left);

		skb_put_data(rb->pend, src, n);
		rb->pend_left -= n;
		if (!rb->pend_left) {
			s5k_deliver(r, rb->pend);
			rb->pend = NULL;
		}
		return;
	}

	if (len < SIPC5_MIN_HDR || (src[0] & SIPC5_START_MASK) != SIPC5_START_MASK) {
		dev_warn_ratelimited(sm->dev, "ring %s: bad SIPC5 start %#x\n",
				     r->cfg->name, len ? src[0] : 0);
		return;
	}
	if (src[1] != r->cfg->ch) {
		dev_warn_ratelimited(sm->dev, "ring %s: frame ch %u != ring ch %u\n",
				     r->cfg->name, src[1], r->cfg->ch);
		return;
	}
	hdr = sipc5_hdr_len(src);
	flen = sipc5_frame_len(src);
	if (flen < hdr || len < hdr || flen > SZ_1M)
		return;
	plen = flen - hdr;
	if (!plen)
		return;

	skb = alloc_skb(plen, GFP_KERNEL);
	if (!skb)
		return;
	first = min(len - hdr, plen);
	skb_put_data(skb, src + hdr, first);
	if (first < plen) {
		rb->pend = skb;
		rb->pend_left = plen - first;
	} else {
		s5k_deliver(r, skb);
	}
}

static void s5k_ring_rx(struct s5k_ring *r)
{
	struct s5k_rb *rb = &r->rb[S5K_DL];
	u32 in = READ_ONCE(*rb->wp), out = READ_ONCE(*rb->rp);
	unsigned int guard = rb->n;

	if (in >= rb->n || out >= rb->n) {
		dev_err_ratelimited(r->sm->dev, "ring %s: S5K_DL ptr oob %u/%u\n",
				    r->cfg->name, in, out);
		return;
	}
	if (in == out)
		return;
	dma_rmb();

	while (in != out && guard--) {
		s5k_rx_slot(r, rb, out);
		out = out + 1 >= rb->n ? 0 : out + 1;
		WRITE_ONCE(*rb->rp, out);
		in = READ_ONCE(*rb->wp);
	}
}

static void s5k_rx_all(struct s5k_modem *sm)
{
	int i;

	if (!smp_load_acquire(&sm->sbd_active))
		return;
	for (i = 0; i < S5K_NRINGS; i++)
		s5k_ring_rx(&sm->ring[i]);
}

/* wwan ports */
static int s5k_port_start(struct wwan_port *port) { return 0; }
static void s5k_port_stop(struct wwan_port *port) { }

static int s5k_port_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5k_ring *r = wwan_port_get_drvdata(port);
	int ret = s5k_ring_send(r, skb->data, skb->len, 0);

	if (ret == -ENOSPC)
		ret = -EAGAIN;
	if (ret)
		return ret;
	consume_skb(skb);
	return 0;
}

static int s5k_port_tx_blocking(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5k_ring *r = wwan_port_get_drvdata(port);
	int ret;

	for (;;) {
		ret = s5k_ring_send(r, skb->data, skb->len, 0);
		if (ret != -ENOSPC)
			break;
		ret = wait_event_interruptible_timeout(r->sm->txq_wq,
						       s5k_ring_has_space(r), HZ);
		if (!ret)
			ret = -ETIMEDOUT;
		if (ret < 0)
			break;
	}
	if (ret)
		return ret;
	consume_skb(skb);
	return 0;
}

static const struct wwan_port_ops s5k_port_ops = {
	.start		= s5k_port_start,
	.stop		= s5k_port_stop,
	.tx		= s5k_port_tx,
	.tx_blocking	= s5k_port_tx_blocking,
};

static enum wwan_port_type s5k_wwan_type(enum s5k_port_kind k)
{
	switch (k) {
	case PK_AT:	return WWAN_PORT_AT;
	case PK_SIT:	return WWAN_PORT_SIT;
	case PK_RFS:	return WWAN_PORT_RFS;
	default:	return WWAN_PORT_OEM;
	}
}

/* raw-ip netdevs (rmnet0-7, SBD PS rings) */
static int s5k_ndo_open(struct net_device *ndev)
{
	netif_tx_start_all_queues(ndev);
	return 0;
}

static int s5k_ndo_stop(struct net_device *ndev)
{
	netif_tx_stop_all_queues(ndev);
	return 0;
}

static netdev_tx_t s5k_ndo_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct s5k_rmnet *p = netdev_priv(ndev);
	struct s5k_modem *sm = p->sm;
	struct s5k_ring *r;
	int ret;

	if (READ_ONCE(sm->tx_suspend)) {
		netif_tx_stop_all_queues(ndev);
		return NETDEV_TX_BUSY;
	}
	if (skb_linearize(skb) || skb->len > S5K_PS_SLOT) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	r = &sm->ring[skb_get_queue_mapping(skb) == 1 ? S5K_RING_PS_HI :
						       S5K_RING_PS_NORM];
	ret = s5k_ring_send(r, skb->data, skb->len, p->ch);
	if (ret == -ENOSPC) {
		netif_tx_stop_all_queues(ndev);
		return NETDEV_TX_BUSY;
	}
	if (ret) {
		ndev->stats.tx_dropped++;
	} else {
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
	}
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops s5k_netdev_ops = {
	.ndo_open	= s5k_ndo_open,
	.ndo_stop	= s5k_ndo_stop,
	.ndo_start_xmit	= s5k_ndo_xmit,
};

static void s5k_netdev_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &s5k_netdev_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->mtu = ETH_DATA_LEN;
	ndev->min_mtu = 68;
	ndev->max_mtu = ETH_DATA_LEN;
	ndev->tx_queue_len = 1000;
	ndev->needs_free_netdev = true;
}

static void s5k_wake_queues(struct s5k_modem *sm)
{
	int i;

	if (READ_ONCE(sm->tx_suspend))
		return;
	if (!s5k_ring_has_space(&sm->ring[S5K_RING_PS_NORM]))
		return;
	for (i = 0; i < S5K_PDP_COUNT; i++) {
		struct net_device *ndev = READ_ONCE(sm->ndev[i]);

		if (ndev && netif_running(ndev))
			netif_tx_wake_all_queues(ndev);
	}
}

static void s5k_stop_queues(struct s5k_modem *sm)
{
	int i;

	for (i = 0; i < S5K_PDP_COUNT; i++)
		if (sm->ndev[i])
			netif_tx_stop_all_queues(sm->ndev[i]);
}

/* Ports and netdevs outlive CP reboots, they are created on the first ONLINE. */
static void s5k_create_devices(struct s5k_modem *sm)
{
	int i;

	if (sm->dev_created)
		return;
	sm->dev_created = true;

	for (i = 0; i < S5K_NRINGS; i++) {
		struct s5k_ring *r = &sm->ring[i];
		struct wwan_port *port;

		if (r->cfg->kind == PK_NONE || r->cfg->kind == PK_PS)
			continue;
		port = wwan_create_port(sm->dev, s5k_wwan_type(r->cfg->kind),
					&s5k_port_ops, NULL, r);
		if (IS_ERR(port)) {
			dev_err(sm->dev, "port %s: %ld\n", r->cfg->name, PTR_ERR(port));
			continue;
		}
		smp_store_release(&r->port, port);
		dev_info(sm->dev, "port up: %s (ch %u)\n", r->cfg->name, r->cfg->ch);
	}

	for (i = 0; i < S5K_PDP_COUNT; i++) {
		struct s5k_rmnet *p;
		struct net_device *ndev;
		char name[IFNAMSIZ];

		snprintf(name, sizeof(name), "rmnet%d", i);
		ndev = alloc_netdev_mqs(sizeof(*p), name, NET_NAME_PREDICTABLE,
					s5k_netdev_setup, 2, 1);
		if (!ndev)
			continue;
		p = netdev_priv(ndev);
		p->sm = sm;
		p->ch = S5K_PDP_FIRST + i;
		SET_NETDEV_DEV(ndev, sm->dev);
		if (register_netdev(ndev)) {
			free_netdev(ndev);
			continue;
		}
		WRITE_ONCE(sm->ndev[i], ndev);
	}
}

/* cmd / status interrupts */
static void s5k_wdt_enable(struct s5k_modem *sm, bool on)
{
	if (sm->wdt_irq <= 0 || sm->wdt_on == on)
		return;
	sm->wdt_on = on;
	if (on)
		enable_irq(sm->wdt_irq);
	else
		disable_irq(sm->wdt_irq);
}

static void s5k_set_state(struct s5k_modem *sm, enum s5k_state st)
{
	if (sm->state != st)
		dev_info(sm->dev, "state %d -> %d\n", sm->state, st);
	WRITE_ONCE(sm->state, st);
}

static bool s5k_cmdline_rev(u32 *rev)
{
	static const char key[] = "modem.bl.revision=";
	const char *p = strstr(saved_command_line, key);

	return p && sscanf(p + sizeof(key) - 1, "%u", rev) == 1;
}

static void s5k_handle_cmd(struct s5k_modem *sm, u16 cmd)
{
	switch (cmd) {
	case CMD_INIT_START:
		dev_info(sm->dev, "CP INIT_START\n");
		if (READ_ONCE(sm->state) != S5K_BOOTING)
			dev_warn(sm->dev, "INIT_START outside BOOTING\n");
		s5k_sbd_init(sm);
		smp_store_release(&sm->sbd_active, true);
		send_ipc_irq(sm, CMD(CMD_PIF_INIT_DONE));
		break;

	case CMD_PHONE_START:
		dev_info(sm->dev, "CP PHONE_START\n");
		if (READ_ONCE(sm->state) == S5K_BOOTING) {
			/* init_legacy_link(): clear q pointers, arm IPC magic */
			memset(sm->ipc + LEG_QPTRS, 0, LEG_QPTRS_END - LEG_QPTRS);
			writel(MAGIC_IPC, sm->ipc + LEG_MAGIC);
			writel(1, sm->ipc + LEG_ACCESS);
			s5k_set_state(sm, S5K_IPC);
			complete_all(&sm->init_cmpl);
		}
		/* The CP repeats PHONE_START until it sees INIT_END. */
		send_ipc_irq(sm, CMD(CMD_INIT_END));
		break;

	case CMD_CRASH_RESET:
	case CMD_CRASH_EXIT:
		dev_err(sm->dev, "CP crash notification %#x\n", cmd);
		s5k_set_state(sm, S5K_CRASH);
		s5k_stop_queues(sm);
		break;

	default:
		dev_warn(sm->dev, "unknown CP command %#x\n", cmd);
	}
}

static irqreturn_t s5k_mbox_hardirq(int irq, void *data)
{
	struct s5k_modem *sm = data;
	u32 st = readl(sm->mbox + MBOX_INTSR0);

	st &= ~readl(sm->mbox + MBOX_INTMR0) & (0xffffu << MBOX_RX_SHIFT);
	if (!st)
		return IRQ_NONE;
	writel(st, sm->mbox + MBOX_INTCR0);
	atomic_or(st >> MBOX_RX_SHIFT, &sm->pending);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t s5k_mbox_thread(int irq, void *data)
{
	struct s5k_modem *sm = data;
	u32 p = atomic_xchg(&sm->pending, 0);

	if (p & BIT(IRQ_CP2AP_MSG)) {
		u32 v = sr_get(sm, SR_CP2AP_MSG);

		if (v & MASK_CMD_VALID)
			s5k_handle_cmd(sm, v & MASK_CMD_FIELD);
	}

	if (p & BIT(IRQ_CP2AP_STATUS)) {
		bool susp = !!(sr_get(sm, SR_CP2AP_STATUS) & CP_ST_FLOWCTL);

		if (susp != READ_ONCE(sm->tx_suspend)) {
			WRITE_ONCE(sm->tx_suspend, susp);
			if (susp)
				s5k_stop_queues(sm);
			else
				s5k_wake_queues(sm);
		}
	}

	if (p & BIT(IRQ_CP2AP_WAKELOCK)) {
		bool hold = sr_field(sm, SR_CP2AP_STATUS, CP_ST_WAKELOCK);

		if (hold && !sm->wake_held)
			pm_stay_awake(sm->dev);
		else if (!hold && sm->wake_held)
			pm_relax(sm->dev);
		sm->wake_held = hold;
	}

	if (p & BIT(IRQ_CP2AP_ACTIVE)) {
		bool act = sr_field(sm, SR_CP2AP_STATUS, CP_ST_LTE_ACTIVE);

		dev_dbg(sm->dev, "cp_active %d (state %d)\n", act, sm->state);
		if (!act && s5k_cp_on(sm) && sm->state == S5K_ONLINE)
			s5k_set_state(sm, S5K_OFFLINE);
	}

	/* Rings are polled on every wake: the CP only says "data" via MSG. */
	s5k_rx_all(sm);
	wake_up_interruptible(&sm->txq_wq);
	if (sm->sbd_active)
		s5k_wake_queues(sm);
	return IRQ_HANDLED;
}

/* CP_WDT / RESET_REQ */
static irqreturn_t s5k_wdt_irq(int irq, void *data)
{
	struct s5k_modem *sm = data;

	disable_irq_nosync(irq);
	sm->wdt_on = false;
	dev_err(sm->dev, "CP watchdog / reset request\n");
	s5k_set_state(sm, S5K_CRASH);
	s5k_stop_queues(sm);
	return IRQ_HANDLED;
}

/* firmware */
/* TOC, load, sigcheck */
struct s5k_toc {
	char	name[12];
	__le32	img_off;
	__le32	mem_off;
	__le32	size;
	__le32	crc;
	__le32	misc;
};

#define MEM_MAP_OFF		0xa0
#define MEM_MAP_NS_MAX		10
#define MEM_MAP_MAGIC		0x004d454d	/* "MEM\0" after swab32 */
#define MEM_NAME_CPI		0x00495043	/* "CPI\0" (IPC) */

static const struct s5k_toc *s5k_find(const struct s5k_img *fw, const char *name)
{
	const struct s5k_toc *t = (const void *)fw->data;
	size_t i, n;

	n = min_t(size_t, le32_to_cpu(t[0].size) / sizeof(*t), 64);
	for (i = 0; i < n; i++)
		if (!strncmp(t[i].name, name, sizeof(t[i].name)))
			return &t[i];
	return NULL;
}

static void s5k_parse_mem_map(struct s5k_modem *sm, const struct s5k_img *fw,
			      const struct s5k_toc *boot)
{
	size_t off = le32_to_cpu(boot->img_off) + MEM_MAP_OFF;
	const u8 *m;
	u32 count, i;

	sm->mem_map_found = 0;
	if (off + 16 + 12 * MEM_MAP_NS_MAX > fw->size)
		return;
	m = fw->data + off;
	if (swab32(get_unaligned_le32(m)) != MEM_MAP_MAGIC) {
		dev_info(sm->dev, "no MEM map in BOOT; using DT region layout\n");
		return;
	}
	count = min_t(u32, get_unaligned_le32(m + 12), MEM_MAP_NS_MAX);
	for (i = 0; i < count; i++) {
		const u8 *e = m + 16 + 12 * i;

		if (get_unaligned_le32(e) != MEM_NAME_CPI)
			continue;
		sm->ipc_off = get_unaligned_le32(e + 4);
		sm->ipc_size = get_unaligned_le32(e + 8);
		sm->mem_map_found = 1;
		dev_info(sm->dev, "MEM map: IPC at +%#x size %#x\n",
			 sm->ipc_off, sm->ipc_size);
	}
}

#define TOC_ADDR_MASK		0x3fffffff
#define NV_FW_NAME		"samsung/s5000ap/nv_data.bin"

static int s5k_copy_section(struct s5k_modem *sm, u8 *dst, size_t dst_size,
			    const struct s5k_img *fw, const struct s5k_toc *t,
			    bool toc_self)
{
	u32 io = le32_to_cpu(t->img_off), mo = le32_to_cpu(t->mem_off) & TOC_ADDR_MASK;
	u32 sz = le32_to_cpu(t->size);

	if (!sz)
		return 0;
	if ((u64)io + sz > fw->size || (u64)mo + sz > dst_size)
		return dev_err_probe(sm->dev, -EINVAL,
				     "TOC entry %.12s out of range (img %#x mem %#x size %#x)\n",
				     t->name, io, mo, sz);
	dev_info(sm->dev, "load %-8.12s img %#010x -> CP+%#010x size %#x%s\n",
		 t->name, io, mo, sz, toc_self ? " (TOC)" : "");
	memcpy(dst + mo, fw->data + io, sz);
	return 0;
}

static void *s5k_read_partition(struct s5k_modem *sm, const char *path,
				size_t *size)
{
	static const char * const want[] = { "BOOT", "MAIN", "VSS" };
	struct s5k_toc toc[32];
	struct file *f;
	size_t total = 0, i, n;
	void *buf;
	loff_t pos = 0;
	int ret;

	if (sm->fw_devt_set)
		f = bdev_file_open_by_dev(MKDEV(sm->fw_devt[0], sm->fw_devt[1]),
					  BLK_OPEN_READ, sm, NULL);
	else
		f = bdev_file_open_by_path(path, BLK_OPEN_READ, sm, NULL);

	if (IS_ERR(f))
		return ERR_CAST(f);

	ret = kernel_read(f, toc, sizeof(toc), &pos);
	dev_info(sm->dev, "partition %u:%u: read %d, name '%.4s', toc size %#x\n",
		 MAJOR(f->f_inode->i_rdev), MINOR(f->f_inode->i_rdev), ret,
		 toc[0].name, le32_to_cpu(toc[0].size));
	if (ret != sizeof(toc) || strncmp(toc[0].name, "TOC", 4)) {
		dev_err(sm->dev, "A: toc check ret %d name '%.4s'\n", ret, toc[0].name);
		buf = ERR_PTR(-EINVAL);
		goto out;
	}
	n = min_t(size_t, le32_to_cpu(toc[0].size) / sizeof(*toc), 32);
	for (i = 0; i < n; i++) {
		size_t end = (size_t)le32_to_cpu(toc[i].img_off) +
			     le32_to_cpu(toc[i].size);
		int k;

		for (k = 0; k < ARRAY_SIZE(want); k++)
			if (!strncmp(toc[i].name, want[k], sizeof(toc[i].name)))
				total = max(total, end);
	}
	dev_info(sm->dev, "B: n %zu total %#zx\n", n, total);
	if (!total || total > SZ_128M) {
		dev_err(sm->dev, "B: bad total\n");
		buf = ERR_PTR(-EINVAL);
		goto out;
	}

	buf = vmalloc(total);
	if (!buf) {
		buf = ERR_PTR(-ENOMEM);
		goto out;
	}
	pos = 0;
	for (i = 0; i < total; ) {
		ret = kernel_read(f, buf + i, min_t(size_t, total - i, SZ_1M), &pos);
		if (ret <= 0) {
			dev_err(sm->dev, "C: read at %zu of %zu returned %d (pos %lld)\n",
				i, total, ret, pos);
			vfree(buf);
			buf = ERR_PTR(ret ?: -EIO);
			goto out;
		}
		i += ret;
	}
	*size = total;
out:
	fput(f);
	return buf;
}

static int s5k_load_image(struct s5k_modem *sm, const struct s5k_img *fw)
{
	const struct s5k_toc *t = (const void *)fw->data;
	static const char * const names[] = { "TOC", "BOOT", "MAIN", "VSS" };
	const struct s5k_toc *nv = s5k_find(fw, "NV");
	size_t boot_total = CP_SIZE + VSS_SIZE;
	const struct firmware *nvfw = NULL;
	u8 *dst;
	int i, ret = 0;

	if (fw->size < sizeof(*t) || strncmp(t[0].name, "TOC", 4))
		return dev_err_probe(sm->dev, -EINVAL, "%s has no TOC\n", sm->fw_name);

	dst = memremap(sm->rmem_base + CP_OFF, boot_total, MEMREMAP_WC);
	if (!dst)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		const struct s5k_toc *e = s5k_find(fw, names[i]);

		if (!e) {
			if (i < 3) {
				ret = dev_err_probe(sm->dev, -ENOENT, "TOC lacks %s\n", names[i]);
				goto out;
			}
			continue;	/* VSS is optional */
		}
		ret = s5k_copy_section(sm, dst, boot_total, fw, e, i == 0);
		if (ret)
			goto out;
	}

	if (nv && le32_to_cpu(nv->size)) {
		u32 mo = le32_to_cpu(nv->mem_off) & TOC_ADDR_MASK;
		u32 sz = le32_to_cpu(nv->size);

		if ((u64)mo + sz > boot_total) {
			ret = -EINVAL;
			goto out;
		}
		memset(dst + mo, 0, sz);
		if (!request_firmware_direct(&nvfw, NV_FW_NAME, sm->dev)) {
			memcpy(dst + mo, nvfw->data, min_t(size_t, nvfw->size, sz));
			dev_info(sm->dev, "NV: %zu bytes from %s\n", nvfw->size, NV_FW_NAME);
			release_firmware(nvfw);
		} else {
			dev_info(sm->dev, "NV: empty (no %s)\n", NV_FW_NAME);
		}
	}
	wmb();
out:
	memunmap(dst);
	return ret;
}

#define MODE_CP_CHECK_CONTINUE	0x8
#define CP_CHECK_SIGN_NOT_FINISH 0x100
/* enum cp_boot_mode */
#define CP_BOOT_MODE_NORMAL	0
#define CP_BOOT_RE_INIT		2
#define CP_BOOT_MODE_MANUAL	7

static int s5k_security_req(struct s5k_modem *sm, u32 mode, unsigned long p2,
			    unsigned long p3)
{
	unsigned long ret;
	unsigned int tries = 0;

	s5k_smc(SMC_ID_CLK, SSS_CLK_ENABLE, 0, 0);
	ret = s5k_smc(SMC_ID, mode, p2, p3);
	while (ret == CP_CHECK_SIGN_NOT_FINISH && tries++ < 0x1000)
		ret = s5k_smc(SMC_ID, MODE_CP_CHECK_CONTINUE, 0, 0);
	s5k_smc(SMC_ID_CLK, SSS_CLK_DISABLE, 0, 0);

	dev_info(sm->dev, "security req mode %#x p2 %#lx p3 %#lx -> %lu\n",
		 mode, p2, p3, ret);
	return ret ? -EACCES : 0;
}

static int s5k_re_init(struct s5k_modem *sm)
{
	return s5k_security_req(sm, CP_BOOT_RE_INIT, 0, 0);
}

static int s5k_verify(struct s5k_modem *sm, const struct s5k_img *fw)
{
	const struct s5k_toc *toc = s5k_find(fw, "TOC"), *boot = s5k_find(fw, "BOOT"),
			     *main_ = s5k_find(fw, "MAIN"), *vss = s5k_find(fw, "VSS");
	u32 mode = CP_BOOT_MODE_NORMAL;
	int ret;

	if (!toc || !boot || !main_)
		return dev_err_probe(sm->dev, -ENOENT, "TOC lacks BOOT/MAIN\n");

	ret = s5k_security_req(sm, CP_BOOT_MODE_MANUAL,
			       sm->rmem_base + CP_OFF + (le32_to_cpu(toc->mem_off) & TOC_ADDR_MASK),
			       le32_to_cpu(toc->size));
	if (ret) {
		dev_err(sm->dev, "security check fail for TOC\n");
		return ret;
	}

	if (vss && le32_to_cpu(vss->size)) {
		ret = s5k_security_req(sm, CP_BOOT_MODE_MANUAL,
				       sm->rmem_base + CP_OFF +
				       (le32_to_cpu(vss->mem_off) & TOC_ADDR_MASK),
				       le32_to_cpu(vss->size));
		if (ret) {
			dev_err(sm->dev, "security check fail for VSS\n");
			if (verify_vss)
				return ret;
		}
	}

	if (sm->mem_map_found && smc_pass_base)
		mode |= lower_32_bits(sm->rmem_base + CP_OFF);
	ret = s5k_security_req(sm, mode, le32_to_cpu(boot->size), le32_to_cpu(main_->size));
	if (ret)
		dev_err(sm->dev, "secure err: invalid MAIN/BOOT image\n");
	return ret;
}

/* boot */
static void s5k_init_control_messages(struct s5k_modem *sm)
{
	int ds = clamp(ds_detect, 1, 2) - 1;
	u32 rev = hw_rev >= 0 ? hw_rev : sm->hw_rev;

	sr_set(sm, SR_AP2CP_STATUS, 0);
	sr_set(sm, SR_CP2AP_STATUS, 0);
	sr_set(sm, SR_AP2CP_MSG, 0);
	sr_set(sm, SR_CP2AP_MSG, 0);
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_DS_DET, ds);
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_SYS_REV, rev <= 0xff ? rev : 0);
}

static void s5k_publish_kerneltime(struct s5k_modem *sm)
{
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	sr_update(sm, SR_AP2CP_KERNELTIME, KT_SEC, ts.tv_sec % 3600);
	sr_update(sm, SR_AP2CP_KERNELTIME, KT_USEC, ts.tv_nsec / NSEC_PER_USEC);
}

static int s5k_map_ipc(struct s5k_modem *sm)
{
	if (sm->ipc) {
		memunmap(sm->ipc);
		sm->ipc = NULL;
	}
	if ((u64)sm->ipc_off + sm->ipc_size > sm->rmem_size)
		return -EINVAL;
	sm->ipc = memremap(sm->rmem_base + sm->ipc_off, sm->ipc_size, MEMREMAP_WC);
	if (!sm->ipc)
		return -ENOMEM;
	return s5k_sbd_layout(sm);
}

static int s5k_boot(struct s5k_modem *sm)
{
	struct s5k_img img = { };
	const struct s5k_img *fw = &img;
	const struct firmware *fwf = NULL;
	const struct s5k_toc *boot;
	void *pbuf = NULL;
	int cnt, ret;

	mutex_lock(&sm->boot_lock);

	s5k_set_state(sm, S5K_OFFLINE);
	smp_store_release(&sm->sbd_active, false);
	s5k_stop_queues(sm);
	s5k_wdt_enable(sm, false);

	if (sm->fw_part || sm->fw_devt_set) {
		pbuf = s5k_read_partition(sm, sm->fw_part, &img.size);
		if (IS_ERR(pbuf)) {
			ret = PTR_ERR(pbuf);
			pbuf = NULL;
			dev_err(sm->dev, "cannot read partition %s: %d\n",
				sm->fw_part, ret);
			goto out;
		}
		img.data = pbuf;
	} else {
		ret = request_firmware(&fwf, sm->fw_name, sm->dev);
		if (ret) {
			dev_err(sm->dev, "cannot load %s: %d\n", sm->fw_name, ret);
			goto out;
		}
		img.data = fwf->data;
		img.size = fwf->size;
	}

	boot = s5k_find(fw, "BOOT");
	if (!boot) {
		ret = -ENOENT;
		dev_err(sm->dev, "no BOOT in TOC\n");
		goto out_fw;
	}

	/* 1. power / reset (power_on_cp, power_reset_cp) */
	if (!s5k_cp_on(sm)) {
		dev_info(sm->dev, "CP domain down: init\n");
		s5k_run_seq(sm, &sm->seq_dump_off);
		ret = s5k_run_seq(sm, &sm->seq_init);
	} else {
		dev_info(sm->dev, "CP already powered: reset\n");
		mbox_kick(sm, INT_AP2CP_WAKEUP);
		usleep_range(5000, 10000);
		s5k_run_seq(sm, &sm->seq_dump_off);
		s5k_run_seq(sm, &sm->seq_assert);
		usleep_range(5000, 10000);
		ret = s5k_run_seq(sm, &sm->seq_release);
		mbox_sw_reset(sm);
	}
	if (ret)
		goto out_fw;

	/* 2. RE_INIT security mode (cbd runs this before any image load) */
	ret = s5k_re_init(sm);
	if (ret)
		goto out_fw;

	/* 3. image */
	s5k_parse_mem_map(sm, fw, boot);
	if (!sm->mem_map_found) {
		sm->ipc_off = IPC_OFF_DEFAULT;
		sm->ipc_size = IPC_SIZE_DEFAULT;
	}
	ret = s5k_load_image(sm, fw);
	if (ret)
		goto out_fw;

	/* 4. TOC/VSS/BOOT+MAIN authentication, opens the CP windows */
	ret = s5k_verify(sm, fw);
	if (ret)
		goto out_fw;

	ret = s5k_map_ipc(sm);
	if (ret)
		goto out_fw;

	/* 5. start_normal_boot() */
	reinit_completion(&sm->init_cmpl);
	s5k_init_control_messages(sm);
	s5k_set_state(sm, S5K_BOOTING);

	memset(sm->ipc + LEG_QPTRS, 0, LEG_QPTRS_END - LEG_QPTRS);
	writel(1, sm->ipc + LEG_ACCESS);
	writel(MAGIC_BOOT, sm->ipc + LEG_MAGIC);

	if (sm->ap2cp_cfg) {
		writel(1, sm->ap2cp_cfg);
		if (readl(sm->ap2cp_cfg) != 1) {
			dev_err(sm->dev, "AP2CP_CFG did not latch\n");
			ret = -EIO;
			goto out_fw;
		}
	}

	for (cnt = 200; !sr_field(sm, SR_CP2AP_STATUS, CP_ST_CP_STATUS); cnt--) {
		if (!cnt) {
			dev_err(sm->dev, "CP never reported alive (status %#x)\n",
				sr_get(sm, SR_CP2AP_STATUS));
			ret = -EACCES;
			goto out_fw;
		}
		usleep_range(10000, 20000);
	}
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_AP_STATUS, 1);
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_PDA_ACTIVE, 1);

	/* 6. INIT_START / PHONE_START are handled in the IRQ thread */
	if (!wait_for_completion_timeout(&sm->init_cmpl, 15 * HZ)) {
		dev_err(sm->dev, "timeout waiting for PHONE_START (cp2ap %#x)\n",
			sr_get(sm, SR_CP2AP_MSG));
		ret = -ETIMEDOUT;
		goto out_fw;
	}

	s5k_set_state(sm, S5K_ONLINE);
	s5k_wdt_enable(sm, true);
	s5k_create_devices(sm);
	s5k_wake_queues(sm);
	dev_info(sm->dev, "CP is ONLINE\n");
	ret = 0;
out_fw:
	vfree(pbuf);
	release_firmware(fwf);
out:
	if (ret)
		s5k_set_state(sm, S5K_OFFLINE);
	mutex_unlock(&sm->boot_lock);
	return ret;
}

static void s5k_boot_work(struct work_struct *w)
{
	struct s5k_modem *sm = container_of(to_delayed_work(w),
					    struct s5k_modem, boot_work);
	int ret = s5k_boot(sm);

	/* rootfs / eMMC may not be ready yet: try again in 5 s */
	if ((ret == -ENOENT || ret == -ENODEV || ret == -ENXIO) &&
	    sm->fw_retries++ < 120)
		schedule_delayed_work(&sm->boot_work, 5 * HZ);
}

static int s5k_dbg_boot_set(void *data, u64 val)
{
	struct s5k_modem *sm = data;

	if (val)
		schedule_delayed_work(&sm->boot_work, 0);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(s5k_dbg_boot_fops, NULL, s5k_dbg_boot_set, "%llu\n");

/* probe / remove / PM */
static int s5k_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *rn;
	struct s5k_modem *sm;
	struct resource *res, rmem;
	u32 cs[3];
	int ret;

	sm = devm_kzalloc(dev, sizeof(*sm), GFP_KERNEL);
	if (!sm)
		return -ENOMEM;
	sm->dev = dev;
	platform_set_drvdata(pdev, sm);
	spin_lock_init(&sm->mbox_lock);
	mutex_init(&sm->boot_lock);
	init_completion(&sm->init_cmpl);
	init_waitqueue_head(&sm->txq_wq);
	INIT_DELAYED_WORK(&sm->boot_work, s5k_boot_work);
	device_init_wakeup(dev, true);

	sm->mbox = devm_platform_ioremap_resource_byname(pdev, "mailbox");
	if (IS_ERR(sm->mbox))
		return PTR_ERR(sm->mbox);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ap2cp-cfg");
	if (res) {
		/* shared with the soc MIF sysreg block, map but do not request */
		sm->ap2cp_cfg = devm_ioremap(dev, res->start, 4);
		if (!sm->ap2cp_cfg)
			return -ENOMEM;
	}

	rn = of_parse_phandle(np, "memory-region", 0);
	if (!rn)
		return dev_err_probe(dev, -EINVAL, "missing memory-region\n");
	ret = of_address_to_resource(rn, 0, &rmem);
	of_node_put(rn);
	if (ret)
		return ret;
	sm->rmem_base = rmem.start;
	sm->rmem_size = resource_size(&rmem);
	if (sm->rmem_size < RMEM_SIZE_MIN)
		return dev_err_probe(dev, -EINVAL, "memory-region %pa too small\n",
				     &sm->rmem_size);

	of_property_read_string(np, "samsung,firmware-partition", &sm->fw_part);
	if (!sm->fw_part) {
		/* no partition given: fall back to request_firmware() */
		if (of_property_read_string(np, "firmware-name", &sm->fw_name))
			sm->fw_name = "samsung/s5000ap/modem.bin";
	}
	of_property_read_u32(np, "samsung,hw-revision", &sm->hw_rev);
	s5k_cmdline_rev(&sm->hw_rev);

	sm->fw_devt_set = !of_property_read_u32_array(np, "samsung,firmware-devt",
						      sm->fw_devt, 2);

	/* <kind addr mask>: kind 0 = MMIO phys addr, kind 1 = SMC CP_CTRL (addr 0 S / 1 NS) */
	ret = of_property_read_u32_array(np, "samsung,cp-status", cs, 3);
	if (ret)
		return dev_err_probe(dev, ret, "missing samsung,cp-status\n");
	if (cs[0] == 0) {
		sm->cp_status_reg = devm_ioremap(dev, cs[1], 4);
		if (!sm->cp_status_reg)
			return -ENOMEM;
	} else if (cs[1] > 1) {
		return dev_err_probe(dev, -EINVAL, "bad cp-status selector\n");
	}
	sm->cp_status_sel = cs[1];
	sm->cp_status_mask = cs[2];

	ret = s5k_parse_seq(sm, "samsung,cp-init-seq", &sm->seq_init, true);
	if (!ret)
		ret = s5k_parse_seq(sm, "samsung,cp-reset-assert-seq", &sm->seq_assert, true);
	if (!ret)
		ret = s5k_parse_seq(sm, "samsung,cp-reset-release-seq", &sm->seq_release, true);
	if (!ret)
		ret = s5k_parse_seq(sm, "samsung,cp-dump-disable-seq", &sm->seq_dump_off, false);
	if (!ret)
		ret = s5k_parse_seq(sm, "samsung,cp-dump-enable-seq", &sm->seq_dump_on, false);
	if (ret)
		return ret;

	sm->mbox_irq = platform_get_irq_byname(pdev, "mailbox");
	if (sm->mbox_irq < 0)
		return sm->mbox_irq;
	sm->wdt_irq = platform_get_irq_byname_optional(pdev, "cp-reset-req");

	writel(0xffff, sm->mbox + MBOX_INTCR1);
	writel(0xffff0000, sm->mbox + MBOX_INTMR0);

	ret = devm_request_threaded_irq(dev, sm->mbox_irq, s5k_mbox_hardirq,
					s5k_mbox_thread, IRQF_NO_SUSPEND,
					"s5000ap-mbox", sm);
	if (ret)
		return ret;
	if (sm->wdt_irq > 0) {
		ret = devm_request_irq(dev, sm->wdt_irq, s5k_wdt_irq,
				       IRQF_NO_SUSPEND | IRQF_NO_AUTOEN,
				       "s5000ap-wdt", sm);
		if (ret)
			return ret;
	}
	mbox_unmask(sm);
	enable_irq_wake(sm->mbox_irq);

	sm->dbgfs = debugfs_create_dir("s5000ap-modem", NULL);
	debugfs_create_file("boot", 0200, sm->dbgfs, sm, &s5k_dbg_boot_fops);

	schedule_delayed_work(&sm->boot_work, 0);
	return 0;
}

static void s5k_remove(struct platform_device *pdev)
{
	struct s5k_modem *sm = platform_get_drvdata(pdev);
	int i;

	debugfs_remove_recursive(sm->dbgfs);
	writel(0xffff0000, sm->mbox + MBOX_INTMR0);
	disable_irq(sm->mbox_irq);
	cancel_delayed_work_sync(&sm->boot_work);
	smp_store_release(&sm->sbd_active, false);

	for (i = 0; i < S5K_PDP_COUNT; i++)
		if (sm->ndev[i])
			unregister_netdev(sm->ndev[i]);
	for (i = 0; i < S5K_NRINGS; i++)
		if (sm->ring[i].port)
			wwan_remove_port(sm->ring[i].port);
	if (sm->wake_held)
		pm_relax(sm->dev);
	if (sm->ipc)
		memunmap(sm->ipc);
}

/* suspend_cp()/resume_cp(): tell the CP whether the AP is alive */
static int s5k_suspend(struct device *dev)
{
	struct s5k_modem *sm = dev_get_drvdata(dev);

	if (READ_ONCE(sm->state) != S5K_ONLINE)
		return 0;
	s5k_publish_kerneltime(sm);
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_PDA_ACTIVE, 0);
	mbox_kick(sm, INT_AP2CP_ACTIVE);
	return 0;
}

static int s5k_resume(struct device *dev)
{
	struct s5k_modem *sm = dev_get_drvdata(dev);

	if (READ_ONCE(sm->state) != S5K_ONLINE)
		return 0;
	s5k_publish_kerneltime(sm);
	sr_update(sm, SR_AP2CP_STATUS, AP_ST_PDA_ACTIVE, 1);
	mbox_kick(sm, INT_AP2CP_ACTIVE);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s5k_pm_ops, s5k_suspend, s5k_resume);

static const struct of_device_id s5k_of_match[] = {
	{ .compatible = "samsung,s5000ap-modem" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k_of_match);

static struct platform_driver s5k_driver = {
	.probe	= s5k_probe,
	.remove	= s5k_remove,
	.driver	= {
		.name		= "exynos-s5000ap-modem",
		.of_match_table	= s5k_of_match,
		.pm		= pm_sleep_ptr(&s5k_pm_ops),
	},
};
module_platform_driver(s5k_driver);

MODULE_FIRMWARE("samsung/s5000ap/modem.bin");
MODULE_DESCRIPTION("Samsung Exynos S5000AP modem driver");
MODULE_LICENSE("GPL");
