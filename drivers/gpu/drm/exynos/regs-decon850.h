/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Register definitions for the Exynos850 DECON and DPU_DMA (IDMA) blocks.
 *
 * Based on the Samsung DPU30 CAL for Exynos3830.
 */

#ifndef __REGS_DECON850_H__
#define __REGS_DECON850_H__

#include <linux/bits.h>

/* DECON */
#define DECON_GLOBAL_CONTROL			0x0000
#define GLOBAL_CONTROL_SRESET			BIT(28)
#define GLOBAL_CONTROL_OPERATION_MODE_CMD	BIT(8)
#define GLOBAL_CONTROL_IDLE_STATUS		BIT(5)
#define GLOBAL_CONTROL_RUN_STATUS		BIT(4)
#define GLOBAL_CONTROL_DECON_EN			BIT(1)
#define GLOBAL_CONTROL_DECON_EN_F		BIT(0)

#define DECON_SRAM_SHARE_ENABLE_MAIN		0x0030
#define FF0_SRAM_SHARE_ENABLE_F			BIT(12)

#define DECON_INTERRUPT_ENABLE			0x0040
#define DPU_FRAME_DONE_INT_EN			BIT(13)
#define DPU_FRAME_START_INT_EN			BIT(12)
#define DPU_EXTRA_INT_EN			BIT(4)
#define DPU_INT_EN				BIT(0)

#define DECON_EXTRA_INTERRUPT_ENABLE		0x0044
#define DPU_RESOURCE_CONFLICT_INT_EN		BIT(8)
#define DPU_TIME_OUT_INT_EN			BIT(4)

#define DECON_INTERRUPT_PENDING			0x004c
#define DPU_FRAME_DONE_INT_PEND			BIT(13)
#define DPU_FRAME_START_INT_PEND		BIT(12)
#define DPU_EXTRA_INT_PEND			BIT(4)

#define DECON_EXTRA_INTERRUPT_PENDING		0x0050
#define DPU_RESOURCE_CONFLICT_INT_PEND		BIT(8)
#define DPU_TIME_OUT_INT_PEND			BIT(4)

#define DECON_SHADOW_REG_UPDATE_REQ		0x0060
#define SHADOW_REG_UPDATE_REQ_GLOBAL		BIT(31)
#define SHADOW_REG_UPDATE_REQ_WIN(_w)		BIT(_w)
#define SHADOW_REG_UPDATE_REQ_ALL_WIN		GENMASK(3, 0)

#define DECON_CLOCK_CONTROL_0			0x00f0
#define CLOCK_CONTROL_0_CG_MASK			0x00511001
#define CLOCK_CONTROL_0_QACTIVE_MASK		BIT(24)

#define DECON_FRAME_FIFO_0_SIZE_CONTROL_0	0x0120
#define FRAME_FIFO_HEIGHT(_v)			((_v) << 16)
#define FRAME_FIFO_WIDTH(_v)			((_v) << 0)
#define DECON_FRAME_FIFO_0_SIZE_CONTROL_1	0x0124
#define DECON_FRAME_FIFO_TH_CONTROL_0		0x0130

#define DECON_FORMATTER_CONTROL			0x0190
#define FORMATTER_OUT_RGB_ORDER(_v)		((_v) << 4)
#define FORMATTER_OUT_RGB_ORDER_MASK		GENMASK(6, 4)
#define FORMATTER_OUT_RGB_ORDER_BGR		0x4

#define DECON_BLENDER_BG_IMAGE_SIZE_0		0x0200
#define BLENDER_BG_HEIGHT(_v)			((_v) << 16)
#define BLENDER_BG_WIDTH(_v)			((_v) << 0)
#define DECON_BLENDER_BG_IMAGE_SIZE_1		0x0204

#define DECON_DATA_PATH_CONTROL_0		0x0214
#define WIN_MAPCOLOR_EN(_w)			BIT(4 * (_w) + 1)
#define WIN_EN(_w)				BIT(4 * (_w) + 0)

#define DECON_DATA_PATH_CONTROL_1		0x0218
#define WIN_CHMAP(_w, _ch)			(((_ch) & 0x7) << (4 * (_w)))

#define DECON_DATA_PATH_CONTROL_2		0x0230
/* No compression, frame FIFO 0 -> formatter 0 -> DSIM interface 0 */
#define DATA_PATH_NOCOMP_FF0_FORMATTER0_DSIMIF0	0x001

#define DECON_WIN_CONTROL_0(_w)			(0x1000 + (_w) * 0x30)
#define WIN_ALPHA1(_v)				(((_v) & 0xff) << 24)
#define WIN_ALPHA0(_v)				(((_v) & 0xff) << 16)
#define WIN_ALPHA_MASK				GENMASK(31, 16)
#define WIN_FUNC(_v)				(((_v) & 0xf) << 8)
#define WIN_FUNC_MASK				GENMASK(11, 8)
#define WIN_ALPHA_MULT_SRC_SEL(_v)		(((_v) & 0x3) << 0)
#define WIN_ALPHA_MULT_SRC_SEL_MASK		GENMASK(1, 0)

#define DECON_WIN_CONTROL_1(_w)			(0x1004 + (_w) * 0x30)
#define WIN_FG_ALPHA_D_SEL(_v)			(((_v) & 0xf) << 24)
#define WIN_BG_ALPHA_D_SEL(_v)			(((_v) & 0xf) << 16)
#define WIN_FG_ALPHA_A_SEL(_v)			(((_v) & 0xf) << 8)
#define WIN_BG_ALPHA_A_SEL(_v)			(((_v) & 0xf) << 0)

#define DECON_WIN_START_POSITION(_w)		(0x1008 + (_w) * 0x30)
#define DECON_WIN_END_POSITION(_w)		(0x100c + (_w) * 0x30)
#define WIN_POS_Y(_v)				(((_v) & 0x3fff) << 16)
#define WIN_POS_X(_v)				(((_v) & 0x3fff) << 0)

#define DECON_WIN_START_TIME_CONTROL(_w)	(0x1018 + (_w) * 0x30)

/* Blending functions (WIN_FUNC) */
#define PD_FUNC_COPY				0x1
#define PD_FUNC_USER_DEFINED			0xd

/* Blending coefficients (WIN_*_ALPHA_*_SEL) */
#define BND_COEF_ZERO				0x0
#define BND_COEF_ONE				0x1
#define BND_COEF_AF				0x2
#define BND_COEF_1_M_AF				0x3
#define BND_COEF_PLANE_ALPHA0			0x6
#define BND_COEF_ALPHA_MULT			0xa
#define BND_COEF_1_M_ALPHA_MULT			0xb

/* Alpha multiplier sources (WIN_ALPHA_MULT_SRC_SEL) */
#define ALPHA_MULT_SRC_SEL_ALPHA0		0x0
#define ALPHA_MULT_SRC_SEL_AF			0x2

/* DPU_DMA channel (IDMA), relative to the channel base */
#define IDMA_ENABLE				0x0000
#define IDMA_SRESET				BIT(24)

#define IDMA_IRQ				0x0004
#define IDMA_CONFIG_ERROR			BIT(21)
#define IDMA_ALL_IRQ_CLEAR			(0x7b << 16)
#define IDMA_ALL_IRQ_MASK			(0x7b << 1)
#define IDMA_IRQ_ENABLE				BIT(0)

#define IDMA_IN_CON				0x0008
#define IDMA_IMG_FORMAT(_v)			((_v) << 11)
#define IDMA_IMG_FORMAT_MASK			GENMASK(15, 11)
#define IDMA_IMG_FORMAT_GET(_v)			(((_v) >> 11) & 0x1f)
#define IDMA_BLOCK_EN				BIT(3)

#define IDMA_OUT_CON				0x000c
#define IDMA_OUT_FRAME_ALPHA(_v)		((_v) << 24)
#define IDMA_OUT_FRAME_ALPHA_MASK		GENMASK(31, 24)

#define IDMA_SRC_SIZE				0x0010
#define IDMA_SRC_HEIGHT(_v)			((_v) << 16)
#define IDMA_SRC_WIDTH(_v)			((_v) << 0)

#define IDMA_SRC_OFFSET				0x0014
#define IDMA_SRC_OFFSET_Y(_v)			((_v) << 16)
#define IDMA_SRC_OFFSET_X(_v)			((_v) << 0)

#define IDMA_IMG_SIZE				0x0018
#define IDMA_IMG_HEIGHT(_v)			((_v) << 16)
#define IDMA_IMG_WIDTH(_v)			((_v) << 0)

#define IDMA_IN_BASE_ADDR_Y			0x0040
#define IDMA_DYNAMIC_GATING_EN			0x0058
#define IDMA_CFG_ERR_STATE			0x0870

/* IDMA_IMG_FORMAT values; the names follow the DRM fourcc convention */
#define IDMA_FMT_ARGB8888			0
#define IDMA_FMT_ABGR8888			1
#define IDMA_FMT_RGBA8888			2
#define IDMA_FMT_BGRA8888			3
#define IDMA_FMT_XRGB8888			4
#define IDMA_FMT_XBGR8888			5
#define IDMA_FMT_RGBX8888			6
#define IDMA_FMT_BGRX8888			7
#define IDMA_FMT_RGB565				8

#endif /* __REGS_DECON850_H__ */
