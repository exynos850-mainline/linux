/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register definitions for the Exynos850 MIPI DSI master and its D-PHY.
 *
 * Based on the Samsung DPU30 CAL for Exynos3830.
 */

#ifndef __REGS_DSIM850_H__
#define __REGS_DSIM850_H__

#include <linux/bits.h>

/* DSIM link */
#define DSIM_SWRST				0x0004
#define DSIM_SWRST_FUNCRST			BIT(8)
#define DSIM_SWRST_RESET			BIT(0)

#define DSIM_LINK_STATUS0			0x0008
#define DSIM_LINK_STATUS0_VIDEO_MODE_ACTIVE	BIT(24)
#define DSIM_LINK_STATUS3			0x0014
#define DSIM_LINK_STATUS3_PLL_STABLE		BIT(0)

#define DSIM_DPHY_STATUS			0x001c
#define DSIM_DPHY_STATUS_TX_READY_HS_CLK	BIT(10)
#define DSIM_DPHY_STATUS_STOP_STATE_CLK		BIT(8)

#define DSIM_CLK_CTRL				0x0020
#define DSIM_CLK_CTRL_CLOCK_SEL			BIT(26)
#define DSIM_CLK_CTRL_NONCONT_CLOCK_LANE	BIT(25)
#define DSIM_CLK_CTRL_CLKLANE_ONOFF		BIT(24)
#define DSIM_CLK_CTRL_TX_REQUEST_HSCLK		BIT(20)
#define DSIM_CLK_CTRL_WORDCLK_EN		BIT(17)
#define DSIM_CLK_CTRL_ESCCLK_EN			BIT(16)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN(_x)	((_x) << 8)
#define DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK	GENMASK(12, 8)
#define DSIM_CLK_CTRL_ESC_PRESCALER(_x)		((_x) << 0)
#define DSIM_CLK_CTRL_ESC_PRESCALER_MASK	GENMASK(7, 0)

#define DSIM_DESKEW_CTRL			0x0024
#define DSIM_DESKEW_CTRL_HW_EN			BIT(15)

#define DSIM_TIMEOUT				0x0028
#define DSIM_TIMEOUT_BTA_TOUT(_x)		((_x) << 16)
#define DSIM_TIMEOUT_LPDR_TOUT(_x)		((_x) << 0)

#define DSIM_ESCMODE				0x002c
#define DSIM_ESCMODE_STOP_STATE_CNT(_x)		((_x) << 21)
#define DSIM_ESCMODE_STOP_STATE_CNT_MASK	GENMASK(31, 21)
#define DSIM_ESCMODE_CMD_LPDT			BIT(7)

#define DSIM_NUM_OF_TRANSFER			0x0030
#define DSIM_THRESHOLD				0x0038

#define DSIM_RESOL				0x003c
#define DSIM_RESOL_VRESOL(_x)			(((_x) & 0x1fff) << 16)
#define DSIM_RESOL_HRESOL(_x)			(((_x) & 0x1fff) << 0)

#define DSIM_HPORCH				0x0044
#define DSIM_HPORCH_HFP(_x)			((_x) << 16)
#define DSIM_HPORCH_HBP(_x)			((_x) << 0)

#define DSIM_SYNC				0x0048
#define DSIM_SYNC_VSA(_x)			((_x) << 16)
#define DSIM_SYNC_HSA(_x)			((_x) << 0)

#define DSIM_CONFIG				0x004c
#define DSIM_CONFIG_SYNC_INFORM			BIT(27)
#define DSIM_CONFIG_BURST_MODE			BIT(26)
#define DSIM_CONFIG_HSE_DISABLE			BIT(23)
#define DSIM_CONFIG_HFP_DISABLE			BIT(22)
#define DSIM_CONFIG_HBP_DISABLE			BIT(21)
#define DSIM_CONFIG_HSA_DISABLE			BIT(20)
#define DSIM_CONFIG_CPRS_EN			BIT(19)
#define DSIM_CONFIG_VIDEO_MODE			BIT(18)
#define DSIM_CONFIG_VC_ID_MASK			GENMASK(16, 15)
#define DSIM_CONFIG_PIXEL_FORMAT(_x)		((_x) << 9)
#define DSIM_CONFIG_PIXEL_FORMAT_MASK		GENMASK(14, 9)
#define DSIM_CONFIG_PER_FRAME_READ_EN		BIT(8)
#define DSIM_CONFIG_EOTP_EN			BIT(7)
#define DSIM_CONFIG_NUM_OF_DATA_LANE(_x)	((_x) << 5)
#define DSIM_CONFIG_NUM_OF_DATA_LANE_MASK	GENMASK(6, 5)
#define DSIM_CONFIG_LANES_EN(_x)		(((_x) & 0x1f) << 0)
#define DSIM_CONFIG_LANES_EN_MASK		GENMASK(4, 0)

#define DSIM_PIXEL_FORMAT_RGB24			0x3e
#define DSIM_PIXEL_FORMAT_RGB18_LOOSE		0x2e
#define DSIM_PIXEL_FORMAT_RGB18_PACKED		0x1e
#define DSIM_PIXEL_FORMAT_RGB16			0x0e

#define DSIM_INTSRC				0x0050
#define DSIM_INTMSK				0x0054
#define DSIM_INT_SFR_PL_FIFO_EMPTY		BIT(29)
#define DSIM_INT_SFR_PH_FIFO_EMPTY		BIT(28)
#define DSIM_INT_INVALID_SFR_VALUE		BIT(23)
#define DSIM_INT_UNDER_RUN			BIT(19)
#define DSIM_INT_RX_DATA_DONE			BIT(18)
#define DSIM_INT_ERR_RX_ECC			BIT(15)
#define DSIM_INT_ALL				GENMASK(31, 13)

#define DSIM_PKTHDR				0x0058
#define DSIM_PKTHDR_BTA_TYPE(_x)		((_x) << 24)
#define DSIM_PKTHDR_DATA1(_x)			((_x) << 16)
#define DSIM_PKTHDR_DATA0(_x)			((_x) << 8)
#define DSIM_PKTHDR_ID(_x)			((_x) << 0)
#define DSIM_PAYLOAD				0x005c
#define DSIM_RXFIFO				0x0060

#define DSIM_SFR_CTRL				0x0064
#define DSIM_SFR_CTRL_SHADOW_REG_READ_EN	BIT(1)
#define DSIM_SFR_CTRL_SHADOW_EN			BIT(0)

#define DSIM_FIFOCTRL				0x0068
#define DSIM_FIFOCTRL_NUMBER_OF_PH_SFR_GET(_x)	(((_x) >> 16) & 0x3f)
#define DSIM_FIFOCTRL_EMPTY_RX			BIT(12)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR		BIT(10)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR		BIT(8)

#define DSIM_CMD_CONFIG				0x0080
#define DSIM_CMD_CONFIG_PKT_GO_EN		BIT(16)
#define DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN	BIT(8)
#define DSIM_CMD_CONFIG_MULTI_PKT_CNT(_x)	((_x) << 0)
#define DSIM_CMD_CONFIG_MULTI_PKT_CNT_MASK	GENMASK(5, 0)

#define DSIM_VIDEO_TIMER			0x0090
#define DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL(_x)	((_x) << 1)
#define DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL_MASK	GENMASK(2, 1)
#define DSIM_VSTATUS_VFP			3

#define DSIM_PLLCTRL				0x00a0
#define DSIM_PLLCTRL_PLL_EN			BIT(23)
#define DSIM_PLLTMR				0x00ac

#define DSIM_VPORCH				0x0104
#define DSIM_VPORCH_VFP_TOTAL(_x)		((_x) << 16)
#define DSIM_VPORCH_VBP(_x)			((_x) << 0)

#define DSIM_VFP_DETAIL				0x0108
#define DSIM_VFP_DETAIL_CMD_ALLOW(_x)		((_x) << 16)
#define DSIM_VFP_DETAIL_STABLE_VFP(_x)		((_x) << 0)

#define DSIM_VT_HTIMING0			0x0110
#define DSIM_VT_HTIMING0_HSA_PERIOD(_x)		((_x) << 16)
#define DSIM_VT_HTIMING0_HACT_PERIOD(_x)	((_x) << 0)
#define DSIM_VT_HTIMING1			0x0114
#define DSIM_VT_HTIMING1_HFP_PERIOD(_x)		((_x) << 16)
#define DSIM_VT_HTIMING1_HBP_PERIOD(_x)		((_x) << 0)

/* Lane bits for DSIM_CONFIG_LANES_EN */
#define DSIM_LANE_CLOCK				BIT(0)
#define DSIM_LANE_DATA(_n)			BIT((_n) + 1)

/* D-PHY, relative to the PHY region */
#define DSIM_PHY_BIAS_CTRL(_n)			(0x0400 + 4 * (_n))
#define DSIM_PHY_DTB_CTRL(_n)			(0x0800 + 4 * (_n))
#define DSIM_PHY_PLL_CTRL(_n)			(0x0c00 + 4 * (_n))
#define DSIM_PHY_ACTRL_MC(_n)			(0x1000 + 4 * (_n))
#define DSIM_PHY_ACTRL_MD(_l, _n)		(0x1400 + 0x400 * (_l) + 4 * (_n))

/* Digital control registers of the clock lane and of data lane _l */
#define DSIM_PHY_DCTRL_MC(_n)			(0x1080 + 4 * ((_n) - 1))
#define DSIM_PHY_DCTRL_MD(_l, _n)		(0x1480 + 0x400 * (_l) + 4 * ((_n) - 1))

#define DSIM_PHY_PMS_P(_x)			(((_x) & 0x3f) << 2)
#define DSIM_PHY_PMS_M_9_8(_x)			(((_x) & 0x300) >> 8)
#define DSIM_PHY_PMS_M_7_0(_x)			((_x) & 0xff)
#define DSIM_PHY_PMS_S(_x)			(((_x) & 0x7) << 5)
#define DSIM_PHY_PMS_S_MASK			GENMASK(7, 5)
#define DSIM_PHY_PMS_K_15_8(_x)			(((_x) & 0xff00) >> 8)
#define DSIM_PHY_PMS_K_7_0(_x)			((_x) & 0xff)

#define DSIM_PHY_ULPS_EXIT_CNT_7_0(_x)		((_x) & 0xff)
#define DSIM_PHY_ULPS_EXIT_CNT_9_8(_x)		(((_x) & 0x300) >> 5)
#define DSIM_PHY_ULPS_EXIT_CNT_9_8_MASK		GENMASK(4, 3)
#define DSIM_PHY_HS_MODE_SEL			BIT(2)
#define DSIM_PHY_SKEWCAL_EN			BIT(4)

/* DPU sysreg */
#define DPU_SYSREG_MIPI_PHY_CON			0x1008
#define DPU_SYSREG_SEL_RESET_DPHY(_n)		BIT(4 + (_n))
#define DPU_SYSREG_M_RESETN_M4S4_TOP		BIT(0)

/* PMU */
#define EXYNOS850_MIPI_PHY_CONTROL0		0x070c
#define EXYNOS850_MIPI_PHY_ENABLE		BIT(0)

#endif /* __REGS_DSIM850_H__ */
