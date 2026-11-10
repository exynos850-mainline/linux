// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos850 platform interface of the Maxwell WLBT subsystem
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 * The WLBT block is powered through the PMU. When it comes out of reset it
 * raises CFG_REQ, and the host then sets up the bus windows (BAAW) through
 * which the WLBT processors see the shared DRAM area and the mailboxes,
 * patches the WLBT PMU boot ROM, and acknowledges the request. Host and
 * firmware then talk through the AP2WLBT mailbox and the shared memory.
 */

#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <scsc/scsc_logring.h>

#include "mif_reg_S5E3830.h"
#include "platform_mif.h"
#include "platform_mif_module.h"

/* Time to wait for the CFG_REQ IRQ */
#define WLBT_BOOT_TIMEOUT	HZ

/* Chip ID register holding the main/sub revision in [23:16] */
#define EXYNOS850_CHIPID_REV	0x10

enum wlbt_boot_state {
	WLBT_BOOT_IN_RESET = 0,
	WLBT_BOOT_WAIT_CFG_REQ,
	WLBT_BOOT_ACK_CFG_REQ,
	WLBT_BOOT_CFG_DONE,
	WLBT_BOOT_CFG_ERROR,
};

struct platform_mif {
	struct scsc_mif_abs interface;
	struct platform_device *pdev;
	struct device *dev;

	int irq_mbox;
	int irq_wdog;
	int irq_cfg_req;
	bool wdog_disabled;

	/* AP2WLBT mailbox */
	void __iomem *base;
	/* WLBT bus configuration blocks */
	void __iomem *dbus_baaw;
	void __iomem *pbus_baaw;
	void __iomem *wlbt_remap;
	void __iomem *boot_cfg;
	/* APM PMIC interface, only read to poll the VGPIO handshake */
	void __iomem *apm_pmic;

	struct regmap *pmureg;
	u32 soc_rev;

	/* Signalled when the CFG_REQ IRQ was handled */
	struct completion cfg_ack;
	enum wlbt_boot_state boot_state;
	bool init_done;

	/* MIF registers preserved during suspend */
	u32 irq_bit_mask;

	/* Shared memory area */
	phys_addr_t mem_start;
	size_t mem_size;
	void *mem;

	/* Serializes mailbox register access */
	spinlock_t mif_spinlock;

	void (*r4_handler)(int irq, void *data);
	void *irq_dev;
	void (*reset_request_handler)(int irq, void *data);
	void *irq_reset_request_dev;
	bool (*recovery_disabled)(void);

	int (*suspend_handler)(struct scsc_mif_abs *abs, void *data);
	void (*resume_handler)(struct scsc_mif_abs *abs, void *data);
	void *suspendresume_data;
};

#define platform_mif_from_mif_abs(abs) container_of(abs, struct platform_mif, interface)

static inline void mbox_write(struct platform_mif *platform, u32 offset, u32 value)
{
	writel(value, platform->base + offset);
}

static inline u32 mbox_read(struct platform_mif *platform, u32 offset)
{
	return readl(platform->base + offset);
}

static void platform_mif_irq_default_handler(int irq, void *data)
{
}

static void platform_mif_irq_reset_request_default_handler(int irq, void *data)
{
}

static void wlbt_regdump(struct platform_mif *platform)
{
	static const struct {
		const char *name;
		u32 offset;
	} regs[] = {
		{ "WLBT_CTRL_S", WLBT_CTRL_S },
		{ "WLBT_CONFIGURATION", WLBT_CONFIGURATION },
		{ "WLBT_CTRL_NS", WLBT_CTRL_NS },
		{ "WLBT_IN", WLBT_IN },
		{ "WLBT_OUT", WLBT_OUT },
		{ "WLBT_STATUS", WLBT_STATUS },
		{ "WLBT_STATES", WLBT_STATES },
		{ "WLBT_DEBUG", WLBT_DEBUG },
	};
	u32 val;
	int i;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		regmap_read(platform->pmureg, regs[i].offset, &val);
		dev_info(platform->dev, "%s 0x%08x\n", regs[i].name, val);
	}
}

static void platform_mif_dump_register(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	unsigned long flags;

	spin_lock_irqsave(&platform->mif_spinlock, flags);
	dev_info(platform->dev,
		 "INTGR0 0x%08x INTCR0 0x%08x INTMR0 0x%08x INTSR0 0x%08x INTMSR0 0x%08x\n",
		 mbox_read(platform, INTGR0), mbox_read(platform, INTCR0),
		 mbox_read(platform, INTMR0), mbox_read(platform, INTSR0),
		 mbox_read(platform, INTMSR0));
	dev_info(platform->dev,
		 "INTGR1 0x%08x INTCR1 0x%08x INTMR1 0x%08x INTSR1 0x%08x INTMSR1 0x%08x\n",
		 mbox_read(platform, INTGR1), mbox_read(platform, INTCR1),
		 mbox_read(platform, INTMR1), mbox_read(platform, INTSR1),
		 mbox_read(platform, INTMSR1));
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);
}

static irqreturn_t platform_mif_isr(int irq, void *data)
{
	struct platform_mif *platform = data;

	platform->r4_handler(irq, platform->irq_dev);

	return IRQ_HANDLED;
}

static irqreturn_t platform_wdog_isr(int irq, void *data)
{
	struct platform_mif *platform = data;

	dev_info(platform->dev, "WDOG/reset request IRQ\n");
	platform_mif_dump_register(&platform->interface);

	if (platform->reset_request_handler == platform_mif_irq_reset_request_default_handler) {
		dev_info(platform->dev, "no reset request handler, disabling WDOG IRQ\n");
		disable_irq_nosync(irq);
		platform->wdog_disabled = true;
	} else if (platform->boot_state == WLBT_BOOT_WAIT_CFG_REQ) {
		/* Spurious interrupt from the SoC during the CFG_REQ phase */
		dev_info(platform->dev, "spurious WDOG IRQ during CFG_REQ phase\n");
		return IRQ_HANDLED;
	} else {
		disable_irq_nosync(irq);
		platform->reset_request_handler(irq, platform->irq_reset_request_dev);
	}

	/*
	 * The wakeup source is not cleared until WLBT is reset, so make it
	 * edge triggered while it stays in that state.
	 */
	if (platform->recovery_disabled && platform->recovery_disabled())
		regmap_update_bits(platform->pmureg, WAKEUP_INT_TYPE, RESETREQ_WLBT, 0);

	return IRQ_HANDLED;
}

/*
 * Replacement for the WLBT PMU+PROC boot ROM (Maxwell142 PMU ROM v2), written
 * to the KA RAM during the configuration phase.
 */
static const u32 ka_patch[] = {
	0x80750002, 0x11a4c218, 0x755a118a, 0x8075e080, 0x30b3e5e7, 0xa230f8e0,
	0xf5077411, 0xfdb2b5b1, 0x7400b153, 0xb5acf507, 0xb475fdad, 0x20b3e501,
	0xb475fbe0, 0x75a3d200, 0xd2801880, 0x7907a075, 0xe6b07837, 0x07b40754,
	0xd90b8002, 0xf5c404f6, 0x00af7580, 0x80750380, 0xf7532290, 0xeff753f7,
	0x53dff753, 0x0479fece, 0xce53fed9, 0xd90c79fd, 0xfbce53fe, 0x53159275,
	0xce53fd91, 0x02f943f7, 0x22fef953, 0xd8fed9f9, 0x9e7522fb, 0xcfc17501,
	0x75a4c275, 0xc4754ac3, 0xa4c57547, 0x7561c675, 0xd27542c7, 0x80d37503,
	0x7500c975, 0xcb75d0ca, 0x00cc7500, 0x75009a75, 0x9c75c09b, 0x009d7500,
	0x82740278, 0x43848012, 0x057802c6, 0x8012d074, 0x20c34384, 0x75d08075,
	0x93750291, 0x029e7501, 0x00000022,
};

#define KA_PATCH_ADDR		0x1000
#define BOOT_SOURCE		0x0
#define BOOT_SOURCE_ROM		0x0
#define BOOT_SOURCE_CFG		0x1
#define BOOT_CFG_ACK		0x4

static void wlbt_set_baaw(void __iomem *baaw, u32 start, u32 end, u32 target)
{
	writel(start >> 12, baaw + 0x0);
	writel(end >> 12, baaw + 0x4);
	writel(target >> 12, baaw + 0x8);
	writel(WLBT_BAAW_ACCESS_CTRL, baaw + 0xc);
}

static irqreturn_t platform_cfg_req_isr(int irq, void *data)
{
	struct platform_mif *platform = data;
	struct arm_smccc_res res;
	u32 id;
	int i;

	disable_irq_nosync(irq);

	/*
	 * A CFG_REQ before we expected it typically means that WLBT did not
	 * return to reset properly.
	 */
	if (platform->boot_state != WLBT_BOOT_WAIT_CFG_REQ) {
		dev_err(platform->dev, "spurious CFG_REQ IRQ\n");
		wlbt_regdump(platform);
		return IRQ_HANDLED;
	}

	/* Let the secure monitor set the WLBT TZPC to non-secure */
	arm_smccc_smc(SMC_CMD_CONN_IF, (1ULL << 32) | EXYNOS_SET_CONN_TZPC,
		      0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(platform->dev, "failed to set TZPC to non-secure: %lu\n", res.a0);

	/* PMU remap */
	writel(WLBT_DBUS_BAAW_0_START >> 12, platform->wlbt_remap + 0x400);

	/* Update CHIP_VERSION_ID with the SoC revision */
	id = readl(platform->wlbt_remap + CHIP_VERSION_ID_OFFSET);
	id &= ~(CHIP_VERSION_ID_IP_MAJOR | CHIP_VERSION_ID_IP_MINOR);
	id |= (platform->soc_rev << CHIP_VERSION_ID_IP_MINOR_SHIFT) &
	      (CHIP_VERSION_ID_IP_MAJOR | CHIP_VERSION_ID_IP_MINOR);
	writel(id, platform->wlbt_remap + CHIP_VERSION_ID_OFFSET);
	dev_info(platform->dev, "CHIP_VERSION_ID 0x%08x\n", id);

	/* Shared DRAM window, mapped to the reserved memory area */
	wlbt_set_baaw(platform->dbus_baaw, WLBT_DBUS_BAAW_0_START,
		      WLBT_DBUS_BAAW_0_END, platform->mem_start);

	/* CP2WLBT mailbox */
	wlbt_set_baaw(platform->pbus_baaw, WLBT_CBUS_BAAW_0_START,
		      WLBT_CBUS_BAAW_0_END, WLBT_PBUS_MBOX_CP2WLBT_BASE);
	/* AP2WLBT, APM2WLBT, GNSS2WLBT... mailboxes */
	wlbt_set_baaw(platform->pbus_baaw + 0x10, WLBT_CBUS_BAAW_1_START,
		      WLBT_CBUS_BAAW_1_END, WLBT_PBUS_MBOX_GNSS2WLBT_BASE);

	/* Patch the PMU boot ROM through the configuration bus */
	writel(BOOT_SOURCE_CFG, platform->boot_cfg + BOOT_SOURCE);
	for (i = 0; i < ARRAY_SIZE(ka_patch); i++)
		writel(ka_patch[i], platform->boot_cfg + KA_PATCH_ADDR + i * 4);
	writel(BOOT_SOURCE_ROM, platform->boot_cfg + BOOT_SOURCE);

	/* The firmware may panic as soon as CFG_ACK is written */
	platform->boot_state = WLBT_BOOT_ACK_CFG_REQ;
	writel(1, platform->boot_cfg + BOOT_CFG_ACK);

	platform->boot_state = WLBT_BOOT_CFG_DONE;
	complete(&platform->cfg_ack);

	return IRQ_HANDLED;
}

static void platform_mif_unregister_irq(struct platform_mif *platform)
{
	devm_free_irq(platform->dev, platform->irq_mbox, platform);
	devm_free_irq(platform->dev, platform->irq_wdog, platform);
	devm_free_irq(platform->dev, platform->irq_cfg_req, platform);
	platform->wdog_disabled = false;
}

static int platform_mif_register_irq(struct platform_mif *platform)
{
	struct device *dev = platform->dev;
	int ret;

	ret = devm_request_irq(dev, platform->irq_mbox, platform_mif_isr, 0,
			       DRV_NAME "-mbox", platform);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, platform->irq_wdog, platform_wdog_isr, 0,
			       DRV_NAME "-wdog", platform);
	if (ret)
		goto err_mbox;

	/* Guard against a spurious CFG_REQ before enabling it */
	platform->boot_state = WLBT_BOOT_IN_RESET;
	smp_wmb();

	/* Left disabled until WLBT is released from reset */
	ret = devm_request_irq(dev, platform->irq_cfg_req, platform_cfg_req_isr,
			       IRQF_NO_AUTOEN, DRV_NAME "-cfg-req", platform);
	if (ret)
		goto err_wdog;

	return 0;

err_wdog:
	devm_free_irq(dev, platform->irq_wdog, platform);
err_mbox:
	devm_free_irq(dev, platform->irq_mbox, platform);
	return ret;
}

static void platform_mif_destroy(struct scsc_mif_abs *interface)
{
}

static char *platform_mif_get_uid(struct scsc_mif_abs *interface)
{
	return "0";
}

static int wlbt_wait_power_state(struct platform_mif *platform, bool on)
{
	u32 val;
	int ret;

	ret = regmap_read_poll_timeout(platform->pmureg, WLBT_STATUS, val,
				       !!(val & WLBT_STATUS_BIT0) == on,
				       10, 500 * USEC_PER_MSEC);
	if (ret)
		dev_err(platform->dev, "timeout waiting for power %s, WLBT_STATUS 0x%x\n",
			on ? "on" : "off", val);

	return ret;
}

static int platform_mif_pmu_reset_release(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	u32 val;
	int ret;

	/* We are now ready for the CFG_REQ IRQ */
	reinit_completion(&platform->cfg_ack);
	platform->boot_state = WLBT_BOOT_WAIT_CFG_REQ;
	smp_wmb();

	/*
	 * The first boot is a cold reset of the AP power sequencer, which
	 * also needs WLBT_START. Later boots are warm resets for the AP
	 * power sequencer but still cold resets for WLBT.
	 */
	if (!platform->init_done) {
		ret = regmap_set_bits(platform->pmureg, WLBT_CTRL_S, WLBT_START);
		if (ret)
			return ret;
	}

	ret = regmap_set_bits(platform->pmureg, WLBT_OPTION, WLBT_OPTION_DATA);
	if (ret)
		return ret;

	udelay(3);

	ret = regmap_set_bits(platform->pmureg, TOP_OUT, PWRRGTON_CP);
	if (ret)
		return ret;

	/* Wait for the APM to report that the WLBT supplies are up */
	if (readl_poll_timeout(platform->apm_pmic + VGPIO_TX_MONITOR, val,
			       val & VGPIO_TX_MON_BIT29, 10, 500 * USEC_PER_MSEC))
		dev_warn(platform->dev, "timeout waiting for VGPIO_TX_MONITOR (0x%x)\n", val);

	usleep_range(1000, 1500);

	ret = regmap_set_bits(platform->pmureg, WLBT_CONFIGURATION, LOCAL_PWR_CFG);
	if (ret)
		return ret;

	wlbt_wait_power_state(platform, true);

	ret = regmap_clear_bits(platform->pmureg, WLBT_CTRL_NS, WLBT_ACTIVE_CLR);
	if (ret)
		return ret;

	ret = regmap_set_bits(platform->pmureg, WLBT_CTRL_NS, WLBT_ACTIVE_EN);
	if (ret)
		return ret;

	platform->init_done = true;

	/* WLBT should now raise CFG_REQ */
	enable_irq(platform->irq_cfg_req);

	if (!wait_for_completion_timeout(&platform->cfg_ack, WLBT_BOOT_TIMEOUT)) {
		dev_err(platform->dev, "timeout waiting for CFG_REQ IRQ\n");
		wlbt_regdump(platform);
		return -ETIMEDOUT;
	}

	if (platform->boot_state == WLBT_BOOT_CFG_ERROR) {
		dev_err(platform->dev, "CFG_REQ failed to configure WLBT\n");
		return -EIO;
	}

	return 0;
}

static int platform_mif_pmu_reset_assert(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	int ret;

	ret = regmap_clear_bits(platform->pmureg, WLBT_CTRL_NS, WLBT_ACTIVE_EN);
	if (ret)
		return ret;

	ret = regmap_clear_bits(platform->pmureg, WLBT_CONFIGURATION, LOCAL_PWR_CFG);
	if (ret)
		return ret;

	ret = wlbt_wait_power_state(platform, false);
	if (ret)
		wlbt_regdump(platform);

	return ret;
}

/* reset = false: release from reset, reset = true: hold in reset */
static int platform_mif_reset(struct scsc_mif_abs *interface, bool reset)
{
	if (reset)
		return platform_mif_pmu_reset_assert(interface);

	return platform_mif_pmu_reset_release(interface);
}

static void *platform_mif_map(struct scsc_mif_abs *interface, size_t *allocated)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	int i;

	if (allocated)
		*allocated = 0;

	platform->mem = memremap(platform->mem_start, platform->mem_size, MEMREMAP_WC);
	if (!platform->mem) {
		dev_err(platform->dev, "failed to map shared memory\n");
		return NULL;
	}

	/* Initialise the mailbox registers with their documented defaults */
	for (i = 0; i < NUM_MBOX_PLAT; i++)
		mbox_write(platform, ISSR(i), 0);

	/* Mask and clear all interrupts */
	mbox_write(platform, INTMR0, 0xffff0000);
	mbox_write(platform, INTMR1, 0x0000ffff);
	mbox_write(platform, INTCR0, 0xffff0000);
	mbox_write(platform, INTCR1, 0x0000ffff);

	if (platform_mif_register_irq(platform)) {
		dev_err(platform->dev, "failed to register IRQs\n");
		memunmap(platform->mem);
		platform->mem = NULL;
		return NULL;
	}

	if (allocated)
		*allocated = platform->mem_size;

	return platform->mem;
}

static void platform_mif_unmap(struct scsc_mif_abs *interface, void *mem)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	/* Mask all interrupts */
	mbox_write(platform, INTMR0, 0xffff0000);
	mbox_write(platform, INTMR1, 0x0000ffff);

	platform_mif_unregister_irq(platform);

	/* Clear all interrupts */
	mbox_write(platform, INTCR0, 0xffff0000);
	mbox_write(platform, INTCR1, 0x0000ffff);

	memunmap(platform->mem);
	platform->mem = NULL;
}

/* Interrupts from WLBT are in INTxR0[31:16], interrupts to WLBT in INTxR1[15:0] */
static u32 platform_mif_irq_bit_mask_status_get(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	return mbox_read(platform, INTMR0) >> 16;
}

static u32 platform_mif_irq_get(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	/* Interrupts that are pending and not masked */
	return mbox_read(platform, INTMSR0) >> 16;
}

static void platform_mif_irq_bit_set(struct scsc_mif_abs *interface, int bit_num,
				     enum scsc_mif_abs_target target)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	if (WARN_ON(bit_num >= 16))
		return;

	mbox_write(platform, INTGR1, BIT(bit_num));
}

static void platform_mif_irq_bit_clear(struct scsc_mif_abs *interface, int bit_num)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	if (WARN_ON(bit_num >= 16))
		return;

	mbox_write(platform, INTCR0, BIT(bit_num) << 16);
}

static void platform_mif_irq_bit_update(struct platform_mif *platform, int bit_num, bool mask)
{
	unsigned long flags;
	u32 val;

	if (WARN_ON(bit_num >= 16))
		return;

	spin_lock_irqsave(&platform->mif_spinlock, flags);
	val = mbox_read(platform, INTMR0);
	if (mask)
		val |= BIT(bit_num) << 16;
	else
		val &= ~(BIT(bit_num) << 16);
	mbox_write(platform, INTMR0, val);
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);
}

static void platform_mif_irq_bit_mask(struct scsc_mif_abs *interface, int bit_num)
{
	platform_mif_irq_bit_update(platform_mif_from_mif_abs(interface), bit_num, true);
}

static void platform_mif_irq_bit_unmask(struct scsc_mif_abs *interface, int bit_num)
{
	platform_mif_irq_bit_update(platform_mif_from_mif_abs(interface), bit_num, false);
}

static void platform_mif_irq_reg_handler(struct scsc_mif_abs *interface,
					 void (*handler)(int irq, void *data), void *dev)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	unsigned long flags;

	spin_lock_irqsave(&platform->mif_spinlock, flags);
	platform->r4_handler = handler;
	platform->irq_dev = dev;
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);
}

static void platform_mif_irq_unreg_handler(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	unsigned long flags;

	spin_lock_irqsave(&platform->mif_spinlock, flags);
	platform->r4_handler = platform_mif_irq_default_handler;
	platform->irq_dev = NULL;
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);
}

static void platform_mif_irq_reg_reset_request_handler(struct scsc_mif_abs *interface,
						       void (*handler)(int irq, void *data),
						       void *dev)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	platform->reset_request_handler = handler;
	platform->irq_reset_request_dev = dev;

	if (platform->wdog_disabled) {
		enable_irq(platform->irq_wdog);
		platform->wdog_disabled = false;
	}
}

static void platform_mif_irq_unreg_reset_request_handler(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	platform->reset_request_handler = platform_mif_irq_reset_request_default_handler;
	platform->irq_reset_request_dev = NULL;
}

static void platform_mif_suspend_reg_handler(struct scsc_mif_abs *interface,
					     int (*suspend)(struct scsc_mif_abs *abs, void *data),
					     void (*resume)(struct scsc_mif_abs *abs, void *data),
					     void *data)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	platform->suspend_handler = suspend;
	platform->resume_handler = resume;
	platform->suspendresume_data = data;
}

static void platform_mif_suspend_unreg_handler(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	platform->suspend_handler = NULL;
	platform->resume_handler = NULL;
	platform->suspendresume_data = NULL;
}

static void platform_recovery_disabled_reg(struct scsc_mif_abs *interface, bool (*handler)(void))
{
	platform_mif_from_mif_abs(interface)->recovery_disabled = handler;
}

static void platform_recovery_disabled_unreg(struct scsc_mif_abs *interface)
{
	platform_mif_from_mif_abs(interface)->recovery_disabled = NULL;
}

static u32 *platform_mif_get_mbox_ptr(struct scsc_mif_abs *interface, u32 mbox_index)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	return (u32 __force *)(platform->base + ISSR(mbox_index));
}

static int platform_mif_get_mifram_ref(struct scsc_mif_abs *interface, void *ptr,
				       scsc_mifram_ref *ref)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	if (!platform->mem)
		return -ENOMEM;

	if (ptr < platform->mem || ptr >= platform->mem + platform->mem_size) {
		dev_err(platform->dev, "pointer %p out of the shared memory\n", ptr);
		return -ENOMEM;
	}

	*ref = (scsc_mifram_ref)(ptr - platform->mem);

	return 0;
}

static void *platform_mif_get_mifram_ptr(struct scsc_mif_abs *interface, scsc_mifram_ref ref)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	if (!platform->mem || ref < 0 || ref >= platform->mem_size)
		return NULL;

	return platform->mem + ref;
}

static void *platform_mif_get_mifram_phy_ptr(struct scsc_mif_abs *interface, scsc_mifram_ref ref)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	return (void *)(uintptr_t)(platform->mem_start + ref);
}

static uintptr_t platform_mif_get_mif_pfn(struct scsc_mif_abs *interface)
{
	return PHYS_PFN(platform_mif_from_mif_abs(interface)->mem_start);
}

static struct device *platform_mif_get_mif_device(struct scsc_mif_abs *interface)
{
	return platform_mif_from_mif_abs(interface)->dev;
}

static void platform_mif_irq_clear(void)
{
}

static int platform_mif_read_register(struct scsc_mif_abs *interface, u64 id, u32 *val)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);

	if (id == SCSC_REG_READ_WLBT_STAT)
		return regmap_read(platform->pmureg, WLBT_STAT, val);

	return -EIO;
}

static void platform_mif_cleanup(struct scsc_mif_abs *interface)
{
}

static void platform_mif_restart(struct scsc_mif_abs *interface)
{
}

static u32 platform_mif_soc_revision(struct device *dev)
{
	struct device_node *np;
	struct regmap *chipid;
	u32 val = 0;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos850-chipid");
	if (!np)
		return 0;

	chipid = device_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(chipid) || regmap_read(chipid, EXYNOS850_CHIPID_REV, &val)) {
		dev_warn(dev, "failed to read the SoC revision\n");
		return 0;
	}

	/* main revision in [23:20], sub revision in [19:16] */
	return (val >> 16) & 0xff;
}

struct scsc_mif_abs *platform_mif_create(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct scsc_mif_abs *platform_if;
	struct platform_mif *platform;
	struct reserved_mem *rmem;
	struct device_node *np;
	struct resource *res;

	platform = devm_kzalloc(dev, sizeof(*platform), GFP_KERNEL);
	if (!platform)
		return NULL;

	platform_if = &platform->interface;
	platform_if->destroy = platform_mif_destroy;
	platform_if->get_uid = platform_mif_get_uid;
	platform_if->reset = platform_mif_reset;
	platform_if->map = platform_mif_map;
	platform_if->unmap = platform_mif_unmap;
	platform_if->irq_bit_set = platform_mif_irq_bit_set;
	platform_if->irq_get = platform_mif_irq_get;
	platform_if->irq_bit_mask_status_get = platform_mif_irq_bit_mask_status_get;
	platform_if->irq_bit_clear = platform_mif_irq_bit_clear;
	platform_if->irq_bit_mask = platform_mif_irq_bit_mask;
	platform_if->irq_bit_unmask = platform_mif_irq_bit_unmask;
	platform_if->irq_reg_handler = platform_mif_irq_reg_handler;
	platform_if->irq_unreg_handler = platform_mif_irq_unreg_handler;
	platform_if->irq_reg_reset_request_handler = platform_mif_irq_reg_reset_request_handler;
	platform_if->irq_unreg_reset_request_handler = platform_mif_irq_unreg_reset_request_handler;
	platform_if->suspend_reg_handler = platform_mif_suspend_reg_handler;
	platform_if->suspend_unreg_handler = platform_mif_suspend_unreg_handler;
	platform_if->get_mbox_ptr = platform_mif_get_mbox_ptr;
	platform_if->get_mifram_ptr = platform_mif_get_mifram_ptr;
	platform_if->get_mifram_ref = platform_mif_get_mifram_ref;
	platform_if->get_mifram_pfn = platform_mif_get_mif_pfn;
	platform_if->get_mifram_phy_ptr = platform_mif_get_mifram_phy_ptr;
	platform_if->get_mif_device = platform_mif_get_mif_device;
	platform_if->irq_clear = platform_mif_irq_clear;
	platform_if->mif_dump_registers = platform_mif_dump_register;
	platform_if->mif_read_register = platform_mif_read_register;
	platform_if->mif_cleanup = platform_mif_cleanup;
	platform_if->mif_restart = platform_mif_restart;
	platform_if->recovery_disabled_reg = platform_recovery_disabled_reg;
	platform_if->recovery_disabled_unreg = platform_recovery_disabled_unreg;

	platform->pdev = pdev;
	platform->dev = dev;
	platform->r4_handler = platform_mif_irq_default_handler;
	platform->reset_request_handler = platform_mif_irq_reset_request_default_handler;
	spin_lock_init(&platform->mif_spinlock);
	init_completion(&platform->cfg_ack);
	platform->boot_state = WLBT_BOOT_IN_RESET;

	/* Shared memory area */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "no memory-region\n");
		return NULL;
	}
	rmem = of_reserved_mem_lookup(np);
	of_node_put(np);
	if (!rmem) {
		dev_err(dev, "memory-region is not a reserved memory area\n");
		return NULL;
	}
	platform->mem_start = rmem->base;
	platform->mem_size = rmem->size;
	if (!IS_ALIGNED(platform->mem_start, SZ_4K) ||
	    platform->mem_size < WLBT_DBUS_BAAW_0_END - WLBT_DBUS_BAAW_0_START) {
		dev_err(dev, "invalid shared memory area %pa (%zu bytes)\n",
			&platform->mem_start, platform->mem_size);
		return NULL;
	}

	platform->base = devm_platform_ioremap_resource_byname(pdev, "mbox");
	if (IS_ERR(platform->base))
		return NULL;
	platform->dbus_baaw = devm_platform_ioremap_resource_byname(pdev, "dbus-baaw");
	if (IS_ERR(platform->dbus_baaw))
		return NULL;
	platform->pbus_baaw = devm_platform_ioremap_resource_byname(pdev, "pbus-baaw");
	if (IS_ERR(platform->pbus_baaw))
		return NULL;
	platform->wlbt_remap = devm_platform_ioremap_resource_byname(pdev, "remap");
	if (IS_ERR(platform->wlbt_remap))
		return NULL;
	platform->boot_cfg = devm_platform_ioremap_resource_byname(pdev, "boot-cfg");
	if (IS_ERR(platform->boot_cfg))
		return NULL;

	/* Shared with the APM interface, so it is only mapped, not requested */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apm-pmic");
	if (!res)
		return NULL;
	platform->apm_pmic = devm_ioremap(dev, res->start, resource_size(res));
	if (!platform->apm_pmic)
		return NULL;

	platform->irq_mbox = platform_get_irq_byname(pdev, "mbox");
	if (platform->irq_mbox < 0)
		return NULL;
	platform->irq_wdog = platform_get_irq_byname(pdev, "wdog");
	if (platform->irq_wdog < 0)
		return NULL;
	platform->irq_cfg_req = platform_get_irq_byname(pdev, "cfg-req");
	if (platform->irq_cfg_req < 0)
		return NULL;

	platform->pmureg = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(platform->pmureg)) {
		dev_err(dev, "failed to get the PMU regmap\n");
		return NULL;
	}

	platform->soc_rev = platform_mif_soc_revision(dev);

	dev_info(dev, "shared memory %pa (%zu bytes), SoC revision 0x%02x\n",
		 &platform->mem_start, platform->mem_size, platform->soc_rev);

	return platform_if;
}

void platform_mif_destroy_platform(struct platform_device *pdev, struct scsc_mif_abs *interface)
{
}

struct platform_device *platform_mif_get_platform_dev(struct scsc_mif_abs *interface)
{
	return platform_mif_from_mif_abs(interface)->pdev;
}

struct device *platform_mif_get_dev(struct scsc_mif_abs *interface)
{
	return platform_mif_from_mif_abs(interface)->dev;
}

/*
 * The mailbox registers lose their value when every user of the MIF powers
 * down, so the interrupt mask is saved and restored around suspend.
 */
int platform_mif_suspend(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	unsigned long flags;
	int r = 0;

	if (platform->suspend_handler)
		r = platform->suspend_handler(interface, platform->suspendresume_data);

	/* After the suspend handler, which may still use the MIF */
	spin_lock_irqsave(&platform->mif_spinlock, flags);
	platform->irq_bit_mask = mbox_read(platform, INTMR0);
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);

	return r;
}

void platform_mif_resume(struct scsc_mif_abs *interface)
{
	struct platform_mif *platform = platform_mif_from_mif_abs(interface);
	unsigned long flags;

	spin_lock_irqsave(&platform->mif_spinlock, flags);
	mbox_write(platform, INTMR0, platform->irq_bit_mask);
	spin_unlock_irqrestore(&platform->mif_spinlock, flags);

	/* Clear the WLBT_ACTIVE request latched on wakeup */
	regmap_set_bits(platform->pmureg, WLBT_CTRL_NS, WLBT_ACTIVE_CLR);

	if (platform->resume_handler)
		platform->resume_handler(interface, platform->suspendresume_data);
}
