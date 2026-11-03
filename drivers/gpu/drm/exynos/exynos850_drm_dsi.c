// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos850 MIPI DSI master driver.
 *
 * This DSIM generation has a different register layout from the one handled
 * by the samsung-dsim bridge, and drives its own D-PHY whose reset and
 * isolation are controlled through the DPU sysreg and the PMU.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "regs-dsim850.h"

#define DSIM_MAX_LANES			4
#define DSIM_POLL_TIMEOUT_US		10000
#define DSIM_XFER_TIMEOUT_US		50000
#define DSIM_RX_TIMEOUT_US		100000
#define DSIM_PH_FIFO_DEPTH		32
#define DSIM_PL_FIFO_SIZE		2048

/* Values from the vendor CAL, without further documentation */
#define DSIM_STOP_STATE_CNT		0xa
#define DSIM_BTA_TIMEOUT		0xff
#define DSIM_LP_RX_TIMEOUT		0xffff
#define DSIM_CMD_ALLOW			4
#define DSIM_STABLE_VFP			2
#define DSIM_PLL_LOCK_CNT_MULT		500

/* D-PHY analog settings from the vendor CAL */
static const u32 dsim_phy_bias[] = { 0x10, 0x00, 0x05, 0x00, 0x68, 0x4c };
static const u32 dsim_phy_dtb[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u32 dsim_phy_actrl_mc[] = { 0x88, 0x00, 0x20, 0x00 };
static const u32 dsim_phy_actrl_md0[] = { 0x88, 0x00, 0x20, 0x00, 0xc0, 0x00 };
static const u32 dsim_phy_actrl_md[] = { 0x88, 0x00, 0x20 };

/*
 * D-PHY timings by lane rate (Mbps): clk_prepare, clk_zero, clk_post,
 * clk_trail, hs_prepare, hs_zero, hs_trail, lpx, hs_exit.
 */
static const u16 dsim_dphy_timings[][10] = {
	{  500, 19,  6, 15, 35, 21,  1, 28, 14,  2 },
	{  510, 20,  6, 15, 35, 21,  1, 28, 15,  2 },
	{  520, 20,  6, 15, 35, 22,  1, 28, 15,  2 },
	{  530, 21,  6, 15, 36, 22,  1, 29, 15,  2 },
	{  540, 21,  6, 15, 36, 23,  1, 29, 16,  2 },
	{  550, 22,  6, 15, 37, 23,  2, 30, 16,  2 },
	{  560, 22,  7, 15, 37, 23,  2, 30, 16,  2 },
	{  570, 22,  7, 15, 37, 24,  2, 30, 17,  2 },
	{  580, 23,  7, 15, 38, 24,  2, 31, 17,  2 },
	{  590, 23,  7, 15, 38, 25,  2, 31, 17,  3 },
	{  600, 24,  7, 15, 38, 25,  2, 31, 17,  3 },
	{  610, 24,  8, 15, 39, 25,  2, 32, 18,  3 },
	{  620, 24,  8, 15, 39, 26,  2, 32, 18,  3 },
	{  630, 25,  8, 15, 39, 26,  2, 32, 18,  3 },
	{  640, 25,  8, 15, 40, 26,  2, 33, 19,  3 },
	{  650, 26,  8, 15, 40, 27,  2, 33, 19,  3 },
	{  660, 26,  9, 15, 40, 27,  3, 33, 19,  3 },
	{  670, 26,  9, 15, 41, 28,  3, 34, 20,  3 },
	{  680, 27,  9, 15, 41, 28,  3, 34, 20,  3 },
	{  690, 27,  9, 15, 41, 28,  3, 34, 20,  3 },
	{  700, 28,  9, 15, 42, 29,  3, 35, 20,  3 },
	{  710, 28,  9, 15, 42, 29,  3, 35, 21,  3 },
	{  720, 29, 10, 15, 43, 30,  3, 36, 21,  3 },
	{  730, 29, 10, 15, 43, 30,  3, 36, 21,  4 },
	{  740, 29, 10, 15, 43, 30,  3, 36, 22,  4 },
	{  750, 30, 10, 15, 44, 31,  3, 37, 22,  4 },
	{  760, 30, 10, 15, 44, 31,  3, 37, 22,  4 },
	{  770, 31, 10, 15, 44, 32,  3, 37, 23,  4 },
	{  780, 31, 11, 15, 45, 32,  4, 38, 23,  4 },
	{  790, 31, 11, 15, 45, 32,  4, 38, 23,  4 },
	{  800, 32, 11, 15, 45, 33,  4, 38, 23,  4 },
	{  810, 32, 11, 15, 46, 33,  4, 39, 24,  4 },
	{  820, 33, 11, 15, 46, 33,  4, 39, 24,  4 },
	{  830, 33, 12, 15, 46, 34,  4, 39, 24,  4 },
	{  840, 33, 12, 15, 47, 34,  4, 40, 25,  4 },
	{  850, 34, 12, 15, 47, 35,  4, 40, 25,  4 },
	{  860, 34, 12, 15, 48, 35,  4, 41, 25,  4 },
	{  870, 35, 12, 15, 48, 35,  4, 41, 26,  4 },
	{  880, 35, 13, 15, 48, 36,  5, 41, 26,  5 },
	{  890, 35, 13, 15, 49, 36,  5, 42, 26,  5 },
	{  900, 36, 13, 15, 49, 37,  5, 42, 26,  5 },
	{  910, 36, 13, 15, 49, 37,  5, 42, 27,  5 },
	{  920, 37, 13, 15, 50, 37,  5, 43, 27,  5 },
	{  930, 37, 13, 15, 50, 38,  5, 43, 27,  5 },
	{  940, 38, 14, 15, 50, 38,  5, 43, 28,  5 },
	{  950, 38, 14, 15, 51, 39,  5, 44, 28,  5 },
	{  960, 38, 14, 15, 51, 39,  5, 44, 28,  5 },
	{  970, 39, 14, 15, 51, 39,  5, 44, 29,  5 },
	{  980, 39, 14, 15, 52, 40,  5, 45, 29,  5 },
	{  990, 40, 14, 15, 52, 40,  5, 45, 29,  5 },
	{ 1000, 40, 15, 15, 52, 40,  6, 45, 29,  5 },
	{ 1010, 40, 15, 15, 53, 41,  6, 46, 30,  5 },
	{ 1020, 41, 15, 15, 53, 41,  6, 46, 30,  6 },
	{ 1030, 41, 15, 15, 54, 34,  6, 47, 25,  6 },
	{ 1040, 42, 15, 15, 54, 42,  6, 47, 31,  6 },
	{ 1050, 42, 16, 15, 54, 42,  6, 47, 31,  6 },
	{ 1060, 42, 16, 15, 55, 43,  6, 48, 31,  6 },
	{ 1070, 43, 16, 15, 55, 43,  6, 48, 32,  6 },
	{ 1080, 43, 16, 15, 55, 44,  6, 48, 32,  6 },
	{ 1090, 44, 16, 15, 56, 44,  6, 49, 32,  6 },
	{ 1100, 44, 17, 15, 56, 44,  6, 49, 32,  6 },
	{ 1110, 45, 17, 15, 56, 45,  6, 49, 33,  6 },
	{ 1120, 45, 17, 15, 57, 45,  7, 50, 33,  6 },
	{ 1130, 45, 17, 15, 57, 46,  7, 50, 33,  6 },
	{ 1140, 46, 17, 15, 57, 46,  7, 50, 34,  6 },
	{ 1150, 46, 17, 15, 58, 46,  7, 51, 34,  6 },
	{ 1160, 47, 18, 15, 58, 47,  7, 51, 34,  6 },
	{ 1170, 47, 18, 15, 59, 47,  7, 52, 35,  7 },
	{ 1180, 47, 18, 15, 59, 48,  7, 52, 35,  7 },
	{ 1190, 48, 18, 15, 59, 48,  7, 52, 35,  7 },
	{ 1200, 48, 18, 15, 60, 48,  7, 53, 35,  7 },
	{ 1210, 49, 18, 15, 60, 49,  7, 53, 36,  7 },
	{ 1220, 49, 19, 15, 60, 49,  7, 53, 36,  7 },
	{ 1230, 49, 19, 15, 61, 49,  8, 54, 36,  7 },
	{ 1240, 50, 19, 15, 61, 50,  8, 54, 37,  7 },
	{ 1250, 50, 19, 15, 61, 50,  8, 54, 37,  7 },
	{ 1260, 51, 19, 15, 62, 51,  8, 55, 37,  7 },
	{ 1270, 51, 20, 15, 62, 51,  8, 55, 38,  7 },
	{ 1280, 51, 20, 15, 62, 51,  8, 55, 38,  7 },
	{ 1290, 52, 20, 15, 63, 52,  8, 56, 38,  7 },
	{ 1300, 52, 20, 15, 63, 52,  8, 56, 38,  7 },
	{ 1310, 53, 20, 15, 64, 53,  8, 57, 39,  8 },
	{ 1320, 53, 21, 15, 64, 53,  8, 57, 39,  8 },
	{ 1330, 54, 21, 15, 64, 53,  8, 57, 39,  8 },
	{ 1340, 54, 21, 15, 65, 54,  9, 58, 40,  8 },
	{ 1350, 54, 21, 15, 65, 54,  9, 58, 40,  8 },
	{ 1360, 55, 21, 15, 65, 55,  9, 58, 40,  8 },
	{ 1370, 55, 21, 15, 66, 55,  9, 59, 41,  8 },
	{ 1380, 56, 22, 15, 66, 55,  9, 59, 41,  8 },
	{ 1390, 56, 22, 15, 66, 56,  9, 59, 41,  8 },
	{ 1400, 56, 22, 15, 67, 56,  9, 60, 41,  8 },
	{ 1410, 57, 22, 15, 67, 56,  9, 60, 42,  8 },
	{ 1420, 57, 22, 15, 67, 57,  9, 60, 42,  8 },
	{ 1430, 58, 22, 15, 68, 57,  9, 61, 42,  8 },
	{ 1440, 58, 23, 15, 68, 58,  9, 61, 43,  8 },
	{ 1450, 58, 23, 15, 69, 58, 10, 61, 43,  8 },
	{ 1460, 59, 23, 15, 69, 58, 10, 62, 43,  9 },
	{ 1470, 59, 23, 15, 69, 59, 10, 62, 44,  9 },
	{ 1480, 60, 23, 15, 70, 59, 10, 63, 44,  9 },
	{ 1490, 60, 24, 15, 70, 60, 10, 63, 44,  9 },
	{ 1500,  7, 24, 15,  6,  7, 10,  5,  5,  9 },
	{ 1510,  7, 24, 15,  6,  7, 10,  5,  5,  9 },
	{ 1520,  7, 24, 15,  6,  7, 10,  5,  5,  9 },
	{ 1530,  7, 24, 15,  6,  7, 10,  5,  5,  9 },
	{ 1540,  7, 24, 15,  6,  7, 10,  5,  5,  9 },
	{ 1550,  7, 25, 15,  6,  7, 10,  5,  5,  9 },
	{ 1560,  7, 25, 15,  6,  7, 10,  5,  5,  9 },
	{ 1570,  7, 25, 15,  6,  7, 11,  5,  5,  9 },
	{ 1580,  7, 25, 15,  6,  7, 11,  5,  5,  9 },
	{ 1590,  7, 26, 15,  6,  7, 11,  5,  5,  9 },
	{ 1600,  7, 26, 15,  6,  7, 11,  5,  5,  9 },
	{ 1610,  7, 26, 15,  6,  7, 11,  5,  6, 10 },
	{ 1620,  7, 26, 15,  6,  7, 11,  5,  6, 10 },
	{ 1630,  7, 26, 15,  6,  7, 11,  5,  6, 10 },
	{ 1640,  7, 27, 15,  6,  7, 11,  5,  6, 10 },
	{ 1650,  7, 27, 15,  6,  7, 12,  6,  6, 10 },
	{ 1660,  8, 27, 15,  6,  7, 12,  6,  6, 10 },
	{ 1670,  8, 27, 15,  6,  7, 12,  6,  6, 10 },
	{ 1680,  8, 27, 15,  7,  8, 12,  6,  6, 10 },
	{ 1690,  8, 27, 15,  7,  8, 12,  6,  6, 10 },
	{ 1700,  8, 27, 15,  7,  8, 12,  6,  6, 10 },
	{ 1710,  8, 27, 15,  7,  8, 12,  6,  6, 10 },
	{ 1720,  8, 28, 15,  7,  8, 12,  6,  6, 10 },
	{ 1730,  8, 28, 15,  7,  8, 12,  6,  6, 10 },
	{ 1740,  8, 28, 15,  7,  8, 12,  6,  6, 10 },
	{ 1750,  8, 28, 15,  7,  8, 12,  6,  6, 11 },
	{ 1760,  8, 29, 15,  7,  8, 12,  6,  6, 11 },
	{ 1770,  8, 29, 15,  7,  8, 12,  6,  6, 11 },
	{ 1780,  8, 29, 15,  7,  8, 12,  6,  6, 11 },
	{ 1790,  8, 29, 15,  7,  8, 13,  6,  6, 11 },
	{ 1800,  8, 29, 15,  7,  8, 13,  6,  6, 11 },
	{ 1810,  8, 30, 15,  7,  8, 13,  6,  6, 11 },
	{ 1820,  8, 30, 15,  7,  8, 13,  6,  6, 11 },
	{ 1830,  8, 30, 15,  7,  8, 13,  6,  6, 11 },
	{ 1840,  8, 30, 15,  7,  8, 13,  6,  6, 11 },
	{ 1850,  8, 31, 15,  7,  8, 13,  6,  6, 11 },
	{ 1860,  9, 31, 15,  7,  8, 13,  6,  6, 11 },
	{ 1870,  9, 31, 15,  7,  8, 14,  6,  7, 11 },
	{ 1880,  9, 31, 15,  7,  8, 14,  7,  7, 11 },
	{ 1890,  9, 31, 15,  7,  9, 14,  7,  7, 11 },
	{ 1900,  9, 31, 15,  7,  9, 14,  7,  7, 12 },
	{ 1910,  9, 31, 15,  8,  9, 14,  7,  7, 12 },
	{ 1920,  9, 31, 15,  8,  9, 14,  7,  7, 12 },
	{ 1930,  9, 31, 15,  8,  9, 14,  7,  7, 12 },
	{ 1940,  9, 32, 15,  8,  9, 14,  7,  7, 12 },
	{ 1950,  9, 32, 15,  8,  9, 14,  7,  7, 12 },
	{ 1960,  9, 32, 15,  8,  9, 14,  7,  7, 12 },
	{ 1970,  9, 32, 15,  8,  9, 14,  7,  7, 12 },
	{ 1980,  9, 33, 15,  8,  9, 14,  7,  7, 12 },
	{ 1990,  9, 33, 15,  8,  9, 14,  7,  7, 12 },
	{ 2000,  9, 33, 15,  8,  9, 14,  7,  7, 12 },
	{ 2010,  9, 33, 15,  8,  9, 15,  7,  7, 12 },
	{ 2020,  9, 34, 15,  8,  9, 15,  7,  7, 12 },
	{ 2030,  9, 34, 15,  8,  9, 15,  7,  7, 12 },
	{ 2040,  9, 34, 15,  8,  9, 15,  7,  7, 13 },
	{ 2050, 10, 34, 15,  8,  9, 15,  7,  7, 13 },
	{ 2060, 10, 34, 15,  8,  9, 15,  7,  7, 13 },
	{ 2070, 10, 34, 15,  8,  9, 15,  7,  7, 13 },
	{ 2080, 10, 34, 15,  8,  9, 15,  7,  7, 13 },
	{ 2090, 10, 34, 15,  8, 10, 15,  7,  7, 13 },
	{ 2100, 10, 34, 15,  8, 10, 15,  8,  7, 13 },
	{ 2110, 10, 35, 15,  8, 10, 15,  8,  7, 13 },
	{ 2120, 10, 35, 15,  8, 10, 15,  8,  7, 13 },
	{ 2130, 10, 35, 15,  9, 10, 15,  8,  7, 13 },
	{ 2140, 10, 35, 15,  9, 10, 15,  8,  8, 13 },
	{ 2150, 10, 36, 15,  9, 10, 15,  8,  8, 13 },
	{ 2160, 10, 36, 15,  9, 10, 16,  8,  8, 13 },
	{ 2170, 10, 36, 15,  9, 10, 16,  8,  8, 13 },
	{ 2180, 10, 36, 15,  9, 10, 16,  8,  8, 13 },
	{ 2190, 10, 37, 15,  9, 10, 16,  8,  8, 14 },
	{ 2200, 10, 37, 15,  9, 10, 16,  8,  8, 14 },
	{ 2210, 10, 37, 15,  9, 10, 16,  8,  8, 14 },
	{ 2220, 10, 37, 15,  9, 10, 16,  8,  8, 14 },
	{ 2230, 10, 37, 15,  9, 10, 17,  8,  8, 14 },
	{ 2240, 10, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2250, 11, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2260, 11, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2270, 11, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2280, 11, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2290, 11, 38, 15,  9, 10, 17,  8,  8, 14 },
	{ 2300, 11, 38, 15,  9, 11, 17,  8,  8, 14 },
	{ 2310, 11, 38, 15,  9, 11, 17,  8,  8, 14 },
	{ 2320, 11, 39, 15,  9, 11, 17,  8,  8, 14 },
	{ 2330, 11, 39, 15,  9, 11, 17,  9,  8, 15 },
	{ 2340, 11, 39, 15,  9, 11, 17,  9,  8, 15 },
	{ 2350, 11, 39, 15,  9, 11, 17,  9,  8, 15 },
	{ 2360, 11, 40, 15, 10, 11, 17,  9,  8, 15 },
	{ 2370, 11, 40, 15, 10, 11, 17,  9,  8, 15 },
	{ 2380, 11, 40, 15, 10, 11, 18,  9,  8, 15 },
	{ 2390, 11, 40, 15, 10, 11, 18,  9,  8, 15 },
	{ 2400, 11, 40, 15, 10, 11, 18,  9,  8, 15 },
	{ 2410, 11, 41, 15, 10, 11, 18,  9,  9, 15 },
	{ 2420, 11, 41, 15, 10, 11, 18,  9,  9, 15 },
	{ 2430, 11, 41, 15, 10, 11, 18,  9,  9, 15 },
	{ 2440, 12, 41, 15, 10, 11, 18,  9,  9, 15 },
	{ 2450, 12, 41, 15, 10, 11, 19,  9,  9, 15 },
	{ 2460, 12, 41, 15, 10, 11, 19,  9,  9, 15 },
	{ 2470, 12, 41, 15, 10, 11, 19,  9,  9, 15 },
	{ 2480, 12, 41, 15, 10, 11, 19,  9,  9, 16 },
	{ 2490, 12, 42, 15, 10, 11, 19,  9,  9, 16 },
	{ 2500, 12, 42, 15, 10, 11, 19,  9,  9, 16 },
};

/* ULPS exit counter, by escape clock from 7 to 20 MHz */
static const u16 dsim_dphy_ulps_exit[] = {
	0x0af, 0x0c8, 0x0e1, 0x0fa, 0x113, 0x12c, 0x145,
	0x15e, 0x177, 0x190, 0x1a9, 0x1c2, 0x1db, 0x1f4,
};

struct dsim850_pll {
	u32 p, m, k;
	int s;
};

struct dsim850 {
	struct device *dev;
	void __iomem *regs;
	void __iomem *phy_regs;
	struct regmap *sysreg;
	struct regmap *pmu;
	struct clk *bus_clk;
	struct clk *pll_clk;
	int irq;

	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct drm_encoder encoder;
	struct mipi_dsi_host host;
	struct drm_display_mode mode;

	u32 lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;

	u32 hs_rate_mhz;
	u32 esc_rate_mhz;
	struct dsim850_pll pll;

	/* serialises command transfers */
	struct mutex lock;
	/* link is up, commands can be sent */
	bool enabled;
	/* left running by the bootloader, adopt it on the first enable */
	bool boot_on;
};

static inline struct dsim850 *bridge_to_dsim(struct drm_bridge *bridge)
{
	return container_of(bridge, struct dsim850, bridge);
}

static inline struct dsim850 *host_to_dsim(struct mipi_dsi_host *host)
{
	return container_of(host, struct dsim850, host);
}

static inline void dsim_write(struct dsim850 *dsim, u32 reg, u32 val)
{
	writel(val, dsim->regs + reg);
}

static inline u32 dsim_read(struct dsim850 *dsim, u32 reg)
{
	return readl(dsim->regs + reg);
}

static inline void dsim_set_bits(struct dsim850 *dsim, u32 reg, u32 mask,
				 u32 val)
{
	val = (val & mask) | (dsim_read(dsim, reg) & ~mask);
	dsim_write(dsim, reg, val);
}

static inline void phy_write(struct dsim850 *dsim, u32 reg, u32 val)
{
	writel(val, dsim->phy_regs + reg);
}

static inline u32 phy_read(struct dsim850 *dsim, u32 reg)
{
	return readl(dsim->phy_regs + reg);
}

static inline void phy_set_bits(struct dsim850 *dsim, u32 reg, u32 mask,
				u32 val)
{
	val = (val & mask) | (phy_read(dsim, reg) & ~mask);
	phy_write(dsim, reg, val);
}

static u32 dsim850_lane_mask(struct dsim850 *dsim)
{
	return DSIM_LANE_CLOCK | GENMASK(dsim->lanes, 1);
}

static void dsim850_dphy_reset(struct dsim850 *dsim, bool assert)
{
	regmap_update_bits(dsim->sysreg, DPU_SYSREG_MIPI_PHY_CON,
			   DPU_SYSREG_M_RESETN_M4S4_TOP,
			   assert ? 0 : DPU_SYSREG_M_RESETN_M4S4_TOP);
}

static void dsim850_sw_reset(struct dsim850 *dsim)
{
	u32 val;

	dsim_set_bits(dsim, DSIM_SWRST, DSIM_SWRST_RESET, ~0);
	if (readl_poll_timeout(dsim->regs + DSIM_SWRST, val,
			       !(val & DSIM_SWRST_RESET), 10,
			       DSIM_POLL_TIMEOUT_US))
		dev_err(dsim->dev, "software reset timed out\n");
}

static void dsim850_set_lanes(struct dsim850 *dsim, bool enable)
{
	u32 lanes = dsim850_lane_mask(dsim);

	dsim_set_bits(dsim, DSIM_CONFIG, DSIM_CONFIG_LANES_EN_MASK,
		      enable ? DSIM_CONFIG_LANES_EN(lanes) : 0);
	usleep_range(400, 500);
}

static void dsim850_set_esc_clk_on_lanes(struct dsim850 *dsim, bool enable)
{
	/* data lanes in bits 3:0, clock lane in bit 4 */
	u32 lanes = (dsim850_lane_mask(dsim) >> 1) | BIT(4);

	dsim_set_bits(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_LANE_ESCCLK_EN_MASK,
		      enable ? DSIM_CLK_CTRL_LANE_ESCCLK_EN(lanes) : 0);
}

static void dsim850_set_link_clock(struct dsim850 *dsim, bool pll)
{
	dsim_set_bits(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_CLOCK_SEL,
		      pll ? ~0 : 0);
}

static bool dsim850_pll_stable(struct dsim850 *dsim)
{
	return dsim_read(dsim, DSIM_LINK_STATUS3) & DSIM_LINK_STATUS3_PLL_STABLE;
}

static int dsim850_calc_pll(struct dsim850 *dsim, struct dsim850_pll *pll)
{
	unsigned long fin = clk_get_rate(dsim->pll_clk);
	u64 vco, mk;

	if (!fin)
		return -EINVAL;

	/*
	 * Fout = (M + K / 65536) * Fin / (P * 2^S). Like the vendor settings,
	 * use a reference of Fin / 2 and the highest VCO up to about 3.4 GHz,
	 * e.g. P=2 M=255 S=2 K=25206 for 830 Mbps from 26 MHz.
	 */
	pll->p = 2;
	for (pll->s = 5; pll->s >= 0; pll->s--) {
		vco = (u64)dsim->hs_rate_mhz * 1000000 << pll->s;
		if (vco <= 3400000000ULL)
			break;
	}
	if (pll->s < 0 || vco < 1300000000ULL)
		return -ERANGE;

	mk = div64_ul(vco * pll->p * 65536, fin);
	pll->m = mk >> 16;
	pll->k = mk & 0xffff;

	return 0;
}

static void dsim850_read_pll(struct dsim850 *dsim, struct dsim850_pll *pll)
{
	pll->p = (phy_read(dsim, DSIM_PHY_PLL_CTRL(6)) >> 2) & 0x3f;
	pll->m = (phy_read(dsim, DSIM_PHY_PLL_CTRL(6)) & 0x3) << 8 |
		 (phy_read(dsim, DSIM_PHY_PLL_CTRL(7)) & 0xff);
	pll->s = (phy_read(dsim, DSIM_PHY_PLL_CTRL(8)) >> 5) & 0x7;
	pll->k = (phy_read(dsim, DSIM_PHY_PLL_CTRL(4)) & 0xff) << 8 |
		 (phy_read(dsim, DSIM_PHY_PLL_CTRL(5)) & 0xff);
}

static void dsim850_set_pll_freq(struct dsim850 *dsim)
{
	const struct dsim850_pll *pll = &dsim->pll;

	phy_write(dsim, DSIM_PHY_PLL_CTRL(4), DSIM_PHY_PMS_K_15_8(pll->k));
	phy_write(dsim, DSIM_PHY_PLL_CTRL(5), DSIM_PHY_PMS_K_7_0(pll->k));
	phy_write(dsim, DSIM_PHY_PLL_CTRL(6),
		  DSIM_PHY_PMS_P(pll->p) | DSIM_PHY_PMS_M_9_8(pll->m));
	phy_write(dsim, DSIM_PHY_PLL_CTRL(7), DSIM_PHY_PMS_M_7_0(pll->m));
	phy_set_bits(dsim, DSIM_PHY_PLL_CTRL(8), DSIM_PHY_PMS_S_MASK,
		     DSIM_PHY_PMS_S(pll->s));

	phy_write(dsim, DSIM_PHY_PLL_CTRL(1), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(2), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(3), 0xa1);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(9), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(10), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(11), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(12), 0x00);
	phy_write(dsim, DSIM_PHY_PLL_CTRL(13), 0x00);
}

static void dsim850_set_lane_timing(struct dsim850 *dsim, u32 reg04,
				    u32 reg05, u32 ulps_exit, bool hs_mode)
{
	phy_write(dsim, reg04, DSIM_PHY_ULPS_EXIT_CNT_7_0(ulps_exit));
	phy_set_bits(dsim, reg05,
		     DSIM_PHY_ULPS_EXIT_CNT_9_8_MASK | DSIM_PHY_HS_MODE_SEL,
		     DSIM_PHY_ULPS_EXIT_CNT_9_8(ulps_exit) |
		     (hs_mode ? DSIM_PHY_HS_MODE_SEL : 0));
}

static int dsim850_set_dphy_timing(struct dsim850 *dsim, u32 esc_mhz)
{
	const u16 *t = NULL;
	bool hs_mode = dsim->hs_rate_mhz < 1500;
	unsigned int i, lane;
	u32 ulps_exit;

	for (i = 0; i < ARRAY_SIZE(dsim_dphy_timings); i++) {
		if (dsim_dphy_timings[i][0] >= dsim->hs_rate_mhz) {
			t = dsim_dphy_timings[i];
			break;
		}
	}
	if (!t || esc_mhz < 7 || esc_mhz > 20)
		return -EINVAL;

	ulps_exit = dsim_dphy_ulps_exit[esc_mhz - 7];

	/* Clock lane */
	dsim850_set_lane_timing(dsim, DSIM_PHY_DCTRL_MC(4), DSIM_PHY_DCTRL_MC(5),
				ulps_exit, hs_mode);
	phy_set_bits(dsim, DSIM_PHY_DCTRL_MC(6), DSIM_PHY_SKEWCAL_EN, 0);
	phy_write(dsim, DSIM_PHY_DCTRL_MC(8), t[8]);	/* lpx */
	phy_write(dsim, DSIM_PHY_DCTRL_MC(9), t[9]);	/* hs_exit */
	phy_write(dsim, DSIM_PHY_DCTRL_MC(11), t[1]);	/* clk_prepare */
	phy_write(dsim, DSIM_PHY_DCTRL_MC(12), t[2]);	/* clk_zero */
	phy_write(dsim, DSIM_PHY_DCTRL_MC(13), t[3]);	/* clk_post */
	phy_write(dsim, DSIM_PHY_DCTRL_MC(14), t[4]);	/* clk_trail */

	/* Data lanes; all four are set up regardless of how many are used. */
	for (lane = 0; lane < DSIM_MAX_LANES; lane++) {
		dsim850_set_lane_timing(dsim, DSIM_PHY_DCTRL_MD(lane, 4),
					DSIM_PHY_DCTRL_MD(lane, 5), ulps_exit,
					hs_mode);
		phy_write(dsim, DSIM_PHY_DCTRL_MD(lane, 7), t[5]);	/* hs_prepare */
		phy_write(dsim, DSIM_PHY_DCTRL_MD(lane, 8), t[8]);	/* lpx */
		phy_write(dsim, DSIM_PHY_DCTRL_MD(lane, 9), t[9]);	/* hs_exit */
		phy_write(dsim, DSIM_PHY_DCTRL_MD(lane, 11), t[6]);	/* hs_zero */
		phy_write(dsim, DSIM_PHY_DCTRL_MD(lane, 12), t[7]);	/* hs_trail */
	}

	return 0;
}

static int dsim850_enable_clocks(struct dsim850 *dsim)
{
	u32 word_mhz = dsim->hs_rate_mhz / 16;
	u32 esc_div, esc_mhz, val;
	unsigned int i, lane;
	int ret;

	esc_div = DIV_ROUND_UP(word_mhz, dsim->esc_rate_mhz);
	esc_mhz = word_mhz / esc_div;

	dsim_set_bits(dsim, DSIM_CLK_CTRL,
		      DSIM_CLK_CTRL_ESCCLK_EN | DSIM_CLK_CTRL_ESC_PRESCALER_MASK,
		      DSIM_CLK_CTRL_ESCCLK_EN |
		      DSIM_CLK_CTRL_ESC_PRESCALER(esc_div));

	ret = dsim850_set_dphy_timing(dsim, esc_mhz);
	if (ret) {
		dev_err(dsim->dev, "no D-PHY timing for %u Mbps / %u MHz\n",
			dsim->hs_rate_mhz, esc_mhz);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(dsim_phy_bias); i++)
		phy_write(dsim, DSIM_PHY_BIAS_CTRL(i + 1), dsim_phy_bias[i]);
	for (i = 0; i < ARRAY_SIZE(dsim_phy_dtb); i++)
		phy_write(dsim, DSIM_PHY_DTB_CTRL(i + 1), dsim_phy_dtb[i]);
	for (i = 0; i < ARRAY_SIZE(dsim_phy_actrl_mc); i++)
		phy_write(dsim, DSIM_PHY_ACTRL_MC(i + 1), dsim_phy_actrl_mc[i]);
	for (i = 0; i < ARRAY_SIZE(dsim_phy_actrl_md0); i++)
		phy_write(dsim, DSIM_PHY_ACTRL_MD(0, i + 1),
			  dsim_phy_actrl_md0[i]);
	for (lane = 1; lane < DSIM_MAX_LANES; lane++)
		for (i = 0; i < ARRAY_SIZE(dsim_phy_actrl_md); i++)
			phy_write(dsim, DSIM_PHY_ACTRL_MD(lane, i + 1),
				  dsim_phy_actrl_md[i]);

	dsim850_set_pll_freq(dsim);
	dsim_write(dsim, DSIM_PLLTMR, DSIM_PLL_LOCK_CNT_MULT * dsim->pll.p);

	dsim_set_bits(dsim, DSIM_PLLCTRL, DSIM_PLLCTRL_PLL_EN, ~0);
	ret = readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS3, val,
				 val & DSIM_LINK_STATUS3_PLL_STABLE, 10,
				 DSIM_POLL_TIMEOUT_US);
	if (ret)
		dev_err(dsim->dev, "D-PHY PLL did not lock\n");

	return ret;
}

static void dsim850_disable_clocks(struct dsim850 *dsim)
{
	u32 val;

	dsim_set_bits(dsim, DSIM_CLK_CTRL,
		      DSIM_CLK_CTRL_ESCCLK_EN | DSIM_CLK_CTRL_ESC_PRESCALER_MASK,
		      DSIM_CLK_CTRL_ESC_PRESCALER(0xff));

	dsim_set_bits(dsim, DSIM_PLLCTRL, DSIM_PLLCTRL_PLL_EN, 0);
	if (readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS3, val,
			       !(val & DSIM_LINK_STATUS3_PLL_STABLE), 10,
			       DSIM_POLL_TIMEOUT_US))
		dev_warn(dsim->dev, "D-PHY PLL did not stop\n");
}

static u32 dsim850_pixel_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB666:
		return DSIM_PIXEL_FORMAT_RGB18_LOOSE;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return DSIM_PIXEL_FORMAT_RGB18_PACKED;
	case MIPI_DSI_FMT_RGB565:
		return DSIM_PIXEL_FORMAT_RGB16;
	case MIPI_DSI_FMT_RGB888:
	default:
		return DSIM_PIXEL_FORMAT_RGB24;
	}
}

/* Convert pixels to word clock cycles (HS clock / 16). */
static u32 dsim850_to_wclk(struct dsim850 *dsim, u32 pixels)
{
	u64 wclk_khz = DIV_ROUND_CLOSEST(dsim->hs_rate_mhz * 1000, 16);

	return DIV64_U64_ROUND_CLOSEST((u64)pixels * wclk_khz, dsim->mode.clock);
}

static void dsim850_set_hperiod(struct dsim850 *dsim)
{
	const struct drm_display_mode *m = &dsim->mode;
	u32 hsa = m->hsync_end - m->hsync_start;
	u32 hbp = m->htotal - m->hsync_end;
	u32 hfp = m->hsync_start - m->hdisplay;

	dsim_write(dsim, DSIM_VT_HTIMING0,
		   DSIM_VT_HTIMING0_HSA_PERIOD(dsim850_to_wclk(dsim, hsa)) |
		   DSIM_VT_HTIMING0_HACT_PERIOD(dsim850_to_wclk(dsim, m->hdisplay)));
	dsim_write(dsim, DSIM_VT_HTIMING1,
		   DSIM_VT_HTIMING1_HFP_PERIOD(dsim850_to_wclk(dsim, hfp)) |
		   DSIM_VT_HTIMING1_HBP_PERIOD(dsim850_to_wclk(dsim, hbp)));
}

static void dsim850_set_config(struct dsim850 *dsim)
{
	const struct drm_display_mode *m = &dsim->mode;
	unsigned long flags = dsim->mode_flags;
	u32 vfp = m->vsync_start - m->vdisplay;
	u32 vsa = m->vsync_end - m->vsync_start;
	u32 vbp = m->vtotal - m->vsync_end;
	u32 hfp = m->hsync_start - m->hdisplay;
	u32 hsa = m->hsync_end - m->hsync_start;
	u32 hbp = m->htotal - m->hsync_end;
	u32 val;

	dsim_set_bits(dsim, DSIM_SFR_CTRL, DSIM_SFR_CTRL_SHADOW_REG_READ_EN, ~0);
	dsim_set_bits(dsim, DSIM_CLK_CTRL,
		      DSIM_CLK_CTRL_CLKLANE_ONOFF |
		      DSIM_CLK_CTRL_NONCONT_CLOCK_LANE,
		      (flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) ?
		      DSIM_CLK_CTRL_NONCONT_CLOCK_LANE : 0);
	dsim_set_bits(dsim, DSIM_DESKEW_CTRL, DSIM_DESKEW_CTRL_HW_EN, 0);

	dsim_write(dsim, DSIM_TIMEOUT,
		   DSIM_TIMEOUT_BTA_TOUT(DSIM_BTA_TIMEOUT) |
		   DSIM_TIMEOUT_LPDR_TOUT(DSIM_LP_RX_TIMEOUT));
	dsim_set_bits(dsim, DSIM_ESCMODE,
		      DSIM_ESCMODE_CMD_LPDT | DSIM_ESCMODE_STOP_STATE_CNT_MASK,
		      DSIM_ESCMODE_STOP_STATE_CNT(DSIM_STOP_STATE_CNT));

	/* One line per transfer */
	dsim_write(dsim, DSIM_THRESHOLD, m->hdisplay);
	dsim_write(dsim, DSIM_RESOL,
		   DSIM_RESOL_VRESOL(m->vdisplay) |
		   DSIM_RESOL_HRESOL(m->hdisplay));
	dsim_write(dsim, DSIM_VPORCH,
		   DSIM_VPORCH_VFP_TOTAL(vfp) | DSIM_VPORCH_VBP(vbp));
	dsim_write(dsim, DSIM_VFP_DETAIL,
		   DSIM_VFP_DETAIL_CMD_ALLOW(DSIM_CMD_ALLOW) |
		   DSIM_VFP_DETAIL_STABLE_VFP(DSIM_STABLE_VFP));
	dsim_write(dsim, DSIM_HPORCH, DSIM_HPORCH_HFP(hfp) | DSIM_HPORCH_HBP(hbp));
	dsim_write(dsim, DSIM_SYNC, DSIM_SYNC_VSA(vsa) | DSIM_SYNC_HSA(hsa));
	dsim_write(dsim, DSIM_NUM_OF_TRANSFER, m->vdisplay);

	val = DSIM_CONFIG_NUM_OF_DATA_LANE(dsim->lanes - 1) |
	      DSIM_CONFIG_PIXEL_FORMAT(dsim850_pixel_format(dsim->format));
	if (!(flags & MIPI_DSI_MODE_NO_EOT_PACKET))
		val |= DSIM_CONFIG_EOTP_EN;
	dsim_set_bits(dsim, DSIM_CONFIG,
		      DSIM_CONFIG_NUM_OF_DATA_LANE_MASK | DSIM_CONFIG_EOTP_EN |
		      DSIM_CONFIG_PER_FRAME_READ_EN |
		      DSIM_CONFIG_PIXEL_FORMAT_MASK | DSIM_CONFIG_VC_ID_MASK,
		      val);

	/* The mode and the horizontal periods only latch with shadowing on. */
	dsim_set_bits(dsim, DSIM_SFR_CTRL, DSIM_SFR_CTRL_SHADOW_EN, ~0);

	val = DSIM_CONFIG_VIDEO_MODE;
	if (flags & MIPI_DSI_MODE_VIDEO_NO_HSA)
		val |= DSIM_CONFIG_HSA_DISABLE;
	if (flags & MIPI_DSI_MODE_VIDEO_NO_HBP)
		val |= DSIM_CONFIG_HBP_DISABLE;
	if (flags & MIPI_DSI_MODE_VIDEO_NO_HFP)
		val |= DSIM_CONFIG_HFP_DISABLE;
	if (!(flags & MIPI_DSI_MODE_VIDEO_HSE))
		val |= DSIM_CONFIG_HSE_DISABLE;
	if (flags & MIPI_DSI_MODE_VIDEO_BURST)
		val |= DSIM_CONFIG_BURST_MODE;
	if (flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		val |= DSIM_CONFIG_SYNC_INFORM;
	dsim_set_bits(dsim, DSIM_CONFIG,
		      DSIM_CONFIG_VIDEO_MODE | DSIM_CONFIG_CPRS_EN |
		      DSIM_CONFIG_HSA_DISABLE | DSIM_CONFIG_HBP_DISABLE |
		      DSIM_CONFIG_HFP_DISABLE | DSIM_CONFIG_HSE_DISABLE |
		      DSIM_CONFIG_BURST_MODE | DSIM_CONFIG_SYNC_INFORM, val);

	dsim_set_bits(dsim, DSIM_CMD_CONFIG,
		      DSIM_CMD_CONFIG_MULTI_PKT_CNT_MASK |
		      DSIM_CMD_CONFIG_MULTI_CMD_PKT_EN |
		      DSIM_CMD_CONFIG_PKT_GO_EN,
		      DSIM_CMD_CONFIG_MULTI_PKT_CNT(DSIM_PH_FIFO_DEPTH));

	dsim850_set_hperiod(dsim);
	dsim_set_bits(dsim, DSIM_VIDEO_TIMER, DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL_MASK,
		      DSIM_VIDEO_TIMER_VSTATUS_INTR_SEL(DSIM_VSTATUS_VFP));
}

/* Bring the link up in LP mode, ready for commands. */
static int dsim850_init_link(struct dsim850 *dsim)
{
	int ret;

	regmap_update_bits(dsim->pmu, EXYNOS850_MIPI_PHY_CONTROL0,
			   EXYNOS850_MIPI_PHY_ENABLE, EXYNOS850_MIPI_PHY_ENABLE);
	/* Let the sysreg, not the DSIM, control the D-PHY reset. */
	regmap_update_bits(dsim->sysreg, DPU_SYSREG_MIPI_PHY_CON,
			   DPU_SYSREG_SEL_RESET_DPHY(0), 0);

	dsim850_set_link_clock(dsim, false);
	dsim850_sw_reset(dsim);
	dsim850_set_lanes(dsim, true);
	dsim850_set_esc_clk_on_lanes(dsim, true);
	dsim_set_bits(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_WORDCLK_EN, ~0);

	dsim850_dphy_reset(dsim, true);
	ret = dsim850_enable_clocks(dsim);
	dsim850_dphy_reset(dsim, false);
	if (ret)
		return ret;

	dsim850_set_link_clock(dsim, true);
	dsim850_set_config(dsim);

	dsim_write(dsim, DSIM_INTSRC, ~0);

	return 0;
}

static void dsim850_set_irqs(struct dsim850 *dsim, bool enable)
{
	u32 errors = DSIM_INT_UNDER_RUN | DSIM_INT_INVALID_SFR_VALUE |
		     DSIM_INT_ERR_RX_ECC;

	dsim_write(dsim, DSIM_INTSRC, ~0);
	dsim_write(dsim, DSIM_INTMSK, enable ? DSIM_INT_ALL & ~errors :
						DSIM_INT_ALL);
}

static void dsim850_start(struct dsim850 *dsim)
{
	u32 val;

	dsim_set_bits(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_TX_REQUEST_HSCLK, ~0);
	if (!(dsim->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) &&
	    readl_poll_timeout(dsim->regs + DSIM_DPHY_STATUS, val,
			       val & DSIM_DPHY_STATUS_TX_READY_HS_CLK, 10,
			       DSIM_POLL_TIMEOUT_US))
		dev_err(dsim->dev, "clock lane did not enter HS mode\n");

	dsim850_set_irqs(dsim, true);
}

static void dsim850_stop(struct dsim850 *dsim)
{
	u32 val;
	int ret;

	dsim850_set_irqs(dsim, false);

	dsim_set_bits(dsim, DSIM_CLK_CTRL, DSIM_CLK_CTRL_TX_REQUEST_HSCLK, 0);
	if (readl_poll_timeout(dsim->regs + DSIM_DPHY_STATUS, val,
			       val & DSIM_DPHY_STATUS_STOP_STATE_CLK, 10,
			       DSIM_POLL_TIMEOUT_US))
		dev_warn(dsim->dev, "clock lane did not enter LP mode\n");

	ret = readl_poll_timeout(dsim->regs + DSIM_LINK_STATUS0, val,
				 !(val & DSIM_LINK_STATUS0_VIDEO_MODE_ACTIVE),
				 10, DSIM_POLL_TIMEOUT_US);
	if (ret)
		dev_warn(dsim->dev, "video stream did not stop\n");

	dsim850_set_link_clock(dsim, false);
	dsim850_dphy_reset(dsim, true);
	dsim850_set_lanes(dsim, false);
	dsim850_set_esc_clk_on_lanes(dsim, false);
	dsim850_disable_clocks(dsim);

	if (!ret)
		dsim850_sw_reset(dsim);

	regmap_update_bits(dsim->pmu, EXYNOS850_MIPI_PHY_CONTROL0,
			   EXYNOS850_MIPI_PHY_ENABLE, 0);
}

static irqreturn_t dsim850_irq_handler(int irq, void *dev_id)
{
	struct dsim850 *dsim = dev_id;
	u32 status = dsim_read(dsim, DSIM_INTSRC) &
		     ~dsim_read(dsim, DSIM_INTMSK);

	if (!status)
		return IRQ_NONE;

	dsim_write(dsim, DSIM_INTSRC, status);

	if (status & DSIM_INT_UNDER_RUN)
		dev_err_ratelimited(dsim->dev, "underrun\n");
	if (status & DSIM_INT_INVALID_SFR_VALUE)
		dev_err_ratelimited(dsim->dev, "invalid SFR value\n");
	if (status & DSIM_INT_ERR_RX_ECC)
		dev_err_ratelimited(dsim->dev, "RX ECC error\n");

	return IRQ_HANDLED;
}

static void dsim850_write_payload(struct dsim850 *dsim, const u8 *buf,
				  size_t len)
{
	u32 word;
	size_t i;

	for (i = 0; i < len; i += 4) {
		word = buf[i];
		if (i + 1 < len)
			word |= buf[i + 1] << 8;
		if (i + 2 < len)
			word |= buf[i + 2] << 16;
		if (i + 3 < len)
			word |= buf[i + 3] << 24;
		dsim_write(dsim, DSIM_PAYLOAD, word);
	}
}

static ssize_t dsim850_read_response(struct dsim850 *dsim,
				     const struct mipi_dsi_msg *msg)
{
	u8 *buf = msg->rx_buf;
	size_t len, i;
	u32 word, val;
	int ret;

	ret = readl_poll_timeout(dsim->regs + DSIM_INTSRC, val,
				 val & DSIM_INT_RX_DATA_DONE, 10,
				 DSIM_RX_TIMEOUT_US);
	if (ret) {
		dev_err(dsim->dev, "read response timed out\n");
		return ret;
	}
	dsim_write(dsim, DSIM_INTSRC, DSIM_INT_RX_DATA_DONE);

	word = dsim_read(dsim, DSIM_RXFIFO);
	switch (word & 0xff) {
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		dev_err(dsim->dev, "peripheral reported error 0x%04x\n",
			(word >> 8) & 0xffff);
		return -EIO;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		len = min_t(size_t, msg->rx_len, 1);
		break;
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
		len = min_t(size_t, msg->rx_len, 2);
		break;
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE: {
		size_t count = (word >> 8) & 0xffff;

		for (i = 0; i < count; i += 4) {
			word = dsim_read(dsim, DSIM_RXFIFO);
			for (len = 0; len < 4 && i + len < count; len++)
				if (i + len < msg->rx_len)
					buf[i + len] = word >> (8 * len);
		}
		return min(count, msg->rx_len);
	}
	default:
		dev_err(dsim->dev, "unexpected response 0x%02x\n", word & 0xff);
		return -EIO;
	}

	for (i = 0; i < len; i++)
		buf[i] = word >> (8 * (i + 1));

	return len;
}

static ssize_t dsim850_host_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	struct dsim850 *dsim = host_to_dsim(host);
	struct mipi_dsi_packet packet;
	bool read = msg->rx_len && msg->rx_buf;
	u32 val;
	ssize_t ret;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	if (packet.payload_length > DSIM_PL_FIFO_SIZE)
		return -EINVAL;

	guard(mutex)(&dsim->lock);

	if (!dsim->enabled)
		return -EINVAL;

	if (readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
			       DSIM_FIFOCTRL_NUMBER_OF_PH_SFR_GET(val) <
			       DSIM_PH_FIFO_DEPTH, 10, DSIM_XFER_TIMEOUT_US))
		return -EBUSY;

	dsim_set_bits(dsim, DSIM_ESCMODE, DSIM_ESCMODE_CMD_LPDT,
		      (msg->flags & MIPI_DSI_MSG_USE_LPM) ? ~0 : 0);
	dsim_write(dsim, DSIM_INTSRC,
		   DSIM_INT_SFR_PH_FIFO_EMPTY | DSIM_INT_RX_DATA_DONE);

	/* Hold the packet until the header is complete. */
	dsim_set_bits(dsim, DSIM_CMD_CONFIG, DSIM_CMD_CONFIG_PKT_GO_EN, ~0);
	dsim850_write_payload(dsim, packet.payload, packet.payload_length);
	dsim_write(dsim, DSIM_PKTHDR,
		   DSIM_PKTHDR_BTA_TYPE(read ? 1 : 0) |
		   DSIM_PKTHDR_DATA1(packet.header[2]) |
		   DSIM_PKTHDR_DATA0(packet.header[1]) |
		   DSIM_PKTHDR_ID(packet.header[0]));
	dsim_set_bits(dsim, DSIM_CMD_CONFIG, DSIM_CMD_CONFIG_PKT_GO_EN, 0);

	ret = readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				 (val & DSIM_FIFOCTRL_EMPTY_PH_SFR) &&
				 (val & DSIM_FIFOCTRL_EMPTY_PL_SFR), 10,
				 DSIM_XFER_TIMEOUT_US);
	if (ret) {
		dev_err(dsim->dev, "command 0x%02x timed out\n", msg->type);
		return ret;
	}

	if (read)
		return dsim850_read_response(dsim, msg);

	return packet.payload_length;
}

static const struct component_ops dsim850_component_ops;

static int dsim850_host_attach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct dsim850 *dsim = host_to_dsim(host);
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	int ret;

	if (device->lanes < 1 || device->lanes > DSIM_MAX_LANES)
		return -EINVAL;

	if (!(device->mode_flags & MIPI_DSI_MODE_VIDEO)) {
		dev_err(dsim->dev, "command mode panels are not supported\n");
		return -EINVAL;
	}

	panel = of_drm_find_panel(device->dev.of_node);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	bridge = devm_drm_panel_bridge_add(dsim->dev, panel);
	drm_panel_put(panel);
	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	dsim->next_bridge = bridge;
	dsim->lanes = device->lanes;
	dsim->format = device->format;
	dsim->mode_flags = device->mode_flags;

	ret = component_add(dsim->dev, &dsim850_component_ops);
	if (ret)
		dsim->next_bridge = NULL;

	return ret;
}

static int dsim850_host_detach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct dsim850 *dsim = host_to_dsim(host);

	component_del(dsim->dev, &dsim850_component_ops);
	dsim->next_bridge = NULL;

	return 0;
}

static const struct mipi_dsi_host_ops dsim850_host_ops = {
	.attach = dsim850_host_attach,
	.detach = dsim850_host_detach,
	.transfer = dsim850_host_transfer,
};

static void dsim850_atomic_pre_enable(struct drm_bridge *bridge,
				      struct drm_atomic_commit *state)
{
	struct dsim850 *dsim = bridge_to_dsim(bridge);
	int ret;

	if (dsim->boot_on) {
		/* The link was left up by the bootloader. */
		dsim->enabled = true;
		return;
	}

	ret = pm_runtime_resume_and_get(dsim->dev);
	if (ret < 0) {
		dev_err(dsim->dev, "failed to resume DSIM: %d\n", ret);
		return;
	}

	ret = dsim850_init_link(dsim);
	if (ret) {
		dev_err(dsim->dev, "failed to initialize link: %d\n", ret);
		pm_runtime_put_sync(dsim->dev);
		return;
	}

	dsim->enabled = true;
}

static void dsim850_atomic_enable(struct drm_bridge *bridge,
				  struct drm_atomic_commit *state)
{
	struct dsim850 *dsim = bridge_to_dsim(bridge);

	if (!dsim->enabled)
		return;

	if (dsim->boot_on) {
		dsim->boot_on = false;
		dsim850_set_irqs(dsim, true);
		return;
	}

	dsim850_start(dsim);
}

static void dsim850_atomic_post_disable(struct drm_bridge *bridge,
					struct drm_atomic_commit *state)
{
	struct dsim850 *dsim = bridge_to_dsim(bridge);

	if (!dsim->enabled)
		return;

	scoped_guard(mutex, &dsim->lock)
		dsim->enabled = false;

	dsim850_stop(dsim);
	pm_runtime_put_sync(dsim->dev);
}

static void dsim850_mode_set(struct drm_bridge *bridge,
			     const struct drm_display_mode *mode,
			     const struct drm_display_mode *adjusted_mode)
{
	struct dsim850 *dsim = bridge_to_dsim(bridge);

	drm_mode_copy(&dsim->mode, adjusted_mode);
}

static int dsim850_attach(struct drm_bridge *bridge,
			  struct drm_encoder *encoder,
			  enum drm_bridge_attach_flags flags)
{
	struct dsim850 *dsim = bridge_to_dsim(bridge);

	return drm_bridge_attach(encoder, dsim->next_bridge, bridge, flags);
}

static const struct drm_bridge_funcs dsim850_bridge_funcs = {
	.atomic_duplicate_state	= drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_bridge_destroy_state,
	.atomic_create_state	= drm_atomic_helper_bridge_create_state,
	.atomic_pre_enable	= dsim850_atomic_pre_enable,
	.atomic_enable		= dsim850_atomic_enable,
	.atomic_post_disable	= dsim850_atomic_post_disable,
	.mode_set		= dsim850_mode_set,
	.attach			= dsim850_attach,
};

static const struct drm_encoder_funcs dsim850_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int dsim850_bind(struct device *dev, struct device *master, void *data)
{
	struct dsim850 *dsim = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dsim->encoder;
	struct drm_device *drm = data;
	struct drm_connector *connector;
	int ret;

	ret = drm_encoder_init(drm, encoder, &dsim850_encoder_funcs,
			       DRM_MODE_ENCODER_DSI, NULL);
	if (ret)
		return ret;

	ret = exynos_drm_set_possible_crtcs(encoder, EXYNOS_DISPLAY_TYPE_LCD);
	if (ret)
		return ret;

	ret = drm_bridge_attach(encoder, &dsim->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector))
		return dev_err_probe(dev, PTR_ERR(connector),
				     "failed to create connector\n");

	return drm_connector_attach_encoder(connector, encoder);
}

static void dsim850_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct dsim850 *dsim = dev_get_drvdata(dev);

	dsim850_atomic_post_disable(&dsim->bridge, NULL);
}

static const struct component_ops dsim850_component_ops = {
	.bind	= dsim850_bind,
	.unbind	= dsim850_unbind,
};

static void dsim850_check_boot_state(struct dsim850 *dsim)
{
	struct dsim850_pll boot_pll;

	dsim->boot_on = (dsim_read(dsim, DSIM_CLK_CTRL) &
			 DSIM_CLK_CTRL_CLOCK_SEL) && dsim850_pll_stable(dsim);
	if (!dsim->boot_on)
		return;

	dsim850_read_pll(dsim, &boot_pll);
	dev_info(dsim->dev,
		 "running from bootloader: PLL P=%u M=%u S=%d K=%u, computed P=%u M=%u S=%d K=%u\n",
		 boot_pll.p, boot_pll.m, boot_pll.s, boot_pll.k,
		 dsim->pll.p, dsim->pll.m, dsim->pll.s, dsim->pll.k);

	/* Reuse the known good settings when the link has to be restarted. */
	dsim->pll = boot_pll;

	dsim_write(dsim, DSIM_INTMSK, DSIM_INT_ALL);
	dsim_write(dsim, DSIM_INTSRC, ~0);
}

static int dsim850_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dsim850 *dsim;
	u32 rate;
	int ret;

	dsim = devm_drm_bridge_alloc(dev, struct dsim850, bridge,
				     &dsim850_bridge_funcs);
	if (IS_ERR(dsim))
		return PTR_ERR(dsim);

	dsim->dev = dev;
	ret = devm_mutex_init(dev, &dsim->lock);
	if (ret)
		return ret;

	dsim->regs = devm_platform_ioremap_resource_byname(pdev, "dsim");
	if (IS_ERR(dsim->regs))
		return PTR_ERR(dsim->regs);

	dsim->phy_regs = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(dsim->phy_regs))
		return PTR_ERR(dsim->phy_regs);

	dsim->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,disp-sysreg");
	if (IS_ERR(dsim->sysreg))
		return dev_err_probe(dev, PTR_ERR(dsim->sysreg),
				     "failed to get DPU sysreg\n");

	dsim->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						    "samsung,pmu-syscon");
	if (IS_ERR(dsim->pmu))
		return dev_err_probe(dev, PTR_ERR(dsim->pmu),
				     "failed to get PMU\n");

	dsim->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(dsim->bus_clk))
		return dev_err_probe(dev, PTR_ERR(dsim->bus_clk),
				     "failed to get bus clock\n");

	dsim->pll_clk = devm_clk_get(dev, "pll");
	if (IS_ERR(dsim->pll_clk))
		return dev_err_probe(dev, PTR_ERR(dsim->pll_clk),
				     "failed to get PLL reference clock\n");

	ret = of_property_read_u32(dev->of_node,
				   "samsung,burst-clock-frequency", &rate);
	if (ret)
		return dev_err_probe(dev, ret, "missing burst clock frequency\n");
	dsim->hs_rate_mhz = rate / 1000000;

	ret = of_property_read_u32(dev->of_node,
				   "samsung,esc-clock-frequency", &rate);
	if (ret)
		return dev_err_probe(dev, ret, "missing escape clock frequency\n");
	dsim->esc_rate_mhz = rate / 1000000;
	if (!dsim->esc_rate_mhz)
		return -EINVAL;

	ret = dsim850_calc_pll(dsim, &dsim->pll);
	if (ret)
		return dev_err_probe(dev, ret, "unsupported burst clock\n");

	dsim->irq = platform_get_irq(pdev, 0);
	if (dsim->irq < 0)
		return dsim->irq;

	/* Only enabled while the registers are clocked */
	ret = devm_request_irq(dev, dsim->irq, dsim850_irq_handler,
			       IRQF_NO_AUTOEN, dev_name(dev), dsim);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	platform_set_drvdata(pdev, dsim);

	ret = clk_prepare_enable(dsim->bus_clk);
	if (ret)
		return ret;

	dsim850_check_boot_state(dsim);
	if (dsim->boot_on) {
		enable_irq(dsim->irq);
		pm_runtime_set_active(dev);
		pm_runtime_get_noresume(dev);
	} else {
		clk_disable_unprepare(dsim->bus_clk);
	}
	pm_runtime_enable(dev);

	dsim->bridge.of_node = dev->of_node;
	dsim->bridge.type = DRM_MODE_CONNECTOR_DSI;
	dsim->bridge.pre_enable_prev_first = true;

	ret = devm_drm_bridge_add(dev, &dsim->bridge);
	if (ret)
		goto err_disable_pm_runtime;

	dsim->host.dev = dev;
	dsim->host.ops = &dsim850_host_ops;
	ret = mipi_dsi_host_register(&dsim->host);
	if (ret)
		goto err_disable_pm_runtime;

	return 0;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	if (dsim->boot_on) {
		disable_irq(dsim->irq);
		pm_runtime_put_noidle(dev);
		pm_runtime_set_suspended(dev);
		clk_disable_unprepare(dsim->bus_clk);
	}

	return ret;
}

static void dsim850_remove(struct platform_device *pdev)
{
	struct dsim850 *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
	pm_runtime_disable(&pdev->dev);
}

static int dsim850_runtime_suspend(struct device *dev)
{
	struct dsim850 *dsim = dev_get_drvdata(dev);

	disable_irq(dsim->irq);
	clk_disable_unprepare(dsim->bus_clk);

	return 0;
}

static int dsim850_runtime_resume(struct device *dev)
{
	struct dsim850 *dsim = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dsim->bus_clk);
	if (ret)
		return ret;

	enable_irq(dsim->irq);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(dsim850_pm_ops, dsim850_runtime_suspend,
				 dsim850_runtime_resume, NULL);

static const struct of_device_id dsim850_of_match[] = {
	{ .compatible = "samsung,exynos850-mipi-dsi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dsim850_of_match);

struct platform_driver exynos850_dsi_driver = {
	.probe		= dsim850_probe,
	.remove		= dsim850_remove,
	.driver		= {
		.name	= "exynos850-dsi",
		.pm	= pm_ptr(&dsim850_pm_ops),
		.of_match_table = dsim850_of_match,
	},
};
