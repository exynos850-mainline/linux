// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos850 DECON (display controller) driver.
 *
 * The DECON blends up to four windows and feeds the result to the DSIM. Each
 * window is fetched from memory by one DPU_DMA (IDMA) channel. Only the three
 * graphics (G) channels are used, one per window, so the planes have a fixed
 * order. The VG channel additionally routes through a DPP for colour
 * conversion, which is not supported. None of the channels can scale.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "regs-decon850.h"

#define DECON_WINDOWS_NR	3
#define DECON_PRIMARY_WIN	0
/* The fourth window has no G channel left and stays off */
#define DECON_HW_WINDOWS_NR	4

/* Register offsets of the DPU_DMA channels from the start of the DMA block */
#define IDMA_CH_VG0_OFFSET	0x4000
#define IDMA_CH_G0_OFFSET	0x1000
#define IDMA_CH_G1_OFFSET	0x2000
#define IDMA_CH_G2_OFFSET	0x3000

/* DECON channel numbers, as programmed into WIN_CHMAP */
#define DECON_CH_VG0		0
#define DECON_CH_G1		1
#define DECON_CH_G2		2
#define DECON_CH_G0		3

#define DECON_RUN_TIMEOUT_US	20000

/* Smallest window an IDMA channel fetches */
#define DECON_WIN_MIN_SIZE	16

struct decon850_win {
	unsigned int ch;
	unsigned int dma_offset;
};

/* Ordered bottom to top: DECON blends a window over the ones below it. */
static const struct decon850_win decon850_wins[DECON_WINDOWS_NR] = {
	[DECON_PRIMARY_WIN] = { .ch = DECON_CH_G1, .dma_offset = IDMA_CH_G1_OFFSET },
	{ .ch = DECON_CH_G0, .dma_offset = IDMA_CH_G0_OFFSET },
	{ .ch = DECON_CH_G2, .dma_offset = IDMA_CH_G2_OFFSET },
};

struct decon850_context {
	struct device			*dev;
	struct drm_device		*drm_dev;
	void				*dma_priv;
	struct exynos_drm_crtc		*crtc;
	struct exynos_drm_plane		planes[DECON_WINDOWS_NR];
	struct exynos_drm_plane_config	configs[DECON_WINDOWS_NR];
	void __iomem			*regs;
	void __iomem			*dma_regs;
	struct clk_bulk_data		*clks;
	int				num_clks;
	int				irq_frame_start;
	int				irq_extra;

	/* protects event and the shadow update request */
	spinlock_t			lock;
	struct drm_pending_vblank_event	*event;

	/* left running by the bootloader, adopt it on the first enable */
	bool				boot_on;
	/* configured but not started, start it on the next flush */
	bool				start_pending;
};

/*
 * The IDMA also has codes for the formats with alpha in the low byte, but
 * those show wrong colours, so only the formats seen working are exposed.
 */
static const u32 decon850_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGB565,
};

static inline void decon_write(struct decon850_context *ctx, u32 reg, u32 val)
{
	writel(val, ctx->regs + reg);
}

static inline u32 decon_read(struct decon850_context *ctx, u32 reg)
{
	return readl(ctx->regs + reg);
}

static inline void decon_set_bits(struct decon850_context *ctx, u32 reg,
				  u32 mask, u32 val)
{
	val = (val & mask) | (decon_read(ctx, reg) & ~mask);
	decon_write(ctx, reg, val);
}

static inline void idma_write(struct decon850_context *ctx, unsigned int win,
			      u32 reg, u32 val)
{
	writel(val, ctx->dma_regs + decon850_wins[win].dma_offset + reg);
}

static inline u32 idma_read(struct decon850_context *ctx, unsigned int win,
			    u32 reg)
{
	return readl(ctx->dma_regs + decon850_wins[win].dma_offset + reg);
}

static inline void idma_set_bits(struct decon850_context *ctx, unsigned int win,
				 u32 reg, u32 mask, u32 val)
{
	val = (val & mask) | (idma_read(ctx, win, reg) & ~mask);
	idma_write(ctx, win, reg, val);
}

static int decon850_idma_format(u32 fourcc)
{
	switch (fourcc) {
	case DRM_FORMAT_ARGB8888:
		return IDMA_FMT_ARGB8888;
	case DRM_FORMAT_ABGR8888:
		return IDMA_FMT_ABGR8888;
	case DRM_FORMAT_XRGB8888:
		return IDMA_FMT_XRGB8888;
	case DRM_FORMAT_XBGR8888:
		return IDMA_FMT_XBGR8888;
	case DRM_FORMAT_RGB565:
		return IDMA_FMT_RGB565;
	default:
		return -EINVAL;
	}
}

static void decon850_clear_irqs(struct decon850_context *ctx)
{
	decon_write(ctx, DECON_INTERRUPT_PENDING,
		    DPU_FRAME_DONE_INT_PEND | DPU_FRAME_START_INT_PEND |
		    DPU_EXTRA_INT_PEND);
	decon_write(ctx, DECON_EXTRA_INTERRUPT_PENDING,
		    DPU_RESOURCE_CONFLICT_INT_PEND | DPU_TIME_OUT_INT_PEND);
}

static int decon850_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon850_context *ctx = crtc->ctx;

	decon_set_bits(ctx, DECON_INTERRUPT_ENABLE, DPU_FRAME_START_INT_EN, ~0);

	return 0;
}

static void decon850_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon850_context *ctx = crtc->ctx;

	decon_set_bits(ctx, DECON_INTERRUPT_ENABLE, DPU_FRAME_START_INT_EN, 0);
}

static void decon850_idma_init(struct decon850_context *ctx, unsigned int win)
{
	/* Errors are reported through DECON, keep the channel IRQs masked. */
	idma_write(ctx, win, IDMA_IRQ, IDMA_ALL_IRQ_CLEAR | IDMA_ALL_IRQ_MASK);
	idma_write(ctx, win, IDMA_DYNAMIC_GATING_EN, 0);
	idma_set_bits(ctx, win, IDMA_OUT_CON, IDMA_OUT_FRAME_ALPHA_MASK,
		      IDMA_OUT_FRAME_ALPHA(0xff));
	idma_set_bits(ctx, win, IDMA_IN_CON, IDMA_BLOCK_EN, 0);
}

static void decon850_set_chmap(struct decon850_context *ctx)
{
	u32 val = 0;
	unsigned int win;

	for (win = 0; win < DECON_WINDOWS_NR; win++)
		val |= WIN_CHMAP(win, decon850_wins[win].ch);
	/* Point the unused window at the remaining channel, not a used one. */
	for (; win < DECON_HW_WINDOWS_NR; win++)
		val |= WIN_CHMAP(win, DECON_CH_VG0);

	decon_write(ctx, DECON_DATA_PATH_CONTROL_1, val);
}

static void decon850_set_size(struct decon850_context *ctx,
			      const struct drm_display_mode *mode)
{
	u32 w = mode->hdisplay, h = mode->vdisplay;

	decon_write(ctx, DECON_BLENDER_BG_IMAGE_SIZE_0,
		    BLENDER_BG_HEIGHT(h) | BLENDER_BG_WIDTH(w));
	decon_write(ctx, DECON_BLENDER_BG_IMAGE_SIZE_1, w * h);

	decon_write(ctx, DECON_FRAME_FIFO_0_SIZE_CONTROL_0,
		    FRAME_FIFO_HEIGHT(h) | FRAME_FIFO_WIDTH(w));
	decon_write(ctx, DECON_FRAME_FIFO_0_SIZE_CONTROL_1, w * h);
	decon_write(ctx, DECON_FRAME_FIFO_TH_CONTROL_0, w);
}

static void decon850_init(struct decon850_context *ctx,
			  const struct drm_display_mode *mode)
{
	decon_set_bits(ctx, DECON_CLOCK_CONTROL_0,
		       CLOCK_CONTROL_0_CG_MASK | CLOCK_CONTROL_0_QACTIVE_MASK, 0);
	decon_write(ctx, DECON_SRAM_SHARE_ENABLE_MAIN, FF0_SRAM_SHARE_ENABLE_F);

	/* Only video mode panels are supported. */
	decon_set_bits(ctx, DECON_GLOBAL_CONTROL,
		       GLOBAL_CONTROL_OPERATION_MODE_CMD, 0);

	decon_set_bits(ctx, DECON_FORMATTER_CONTROL, FORMATTER_OUT_RGB_ORDER_MASK,
		       FORMATTER_OUT_RGB_ORDER(FORMATTER_OUT_RGB_ORDER_BGR));
	decon_write(ctx, DECON_DATA_PATH_CONTROL_2,
		    DATA_PATH_NOCOMP_FF0_FORMATTER0_DSIMIF0);

	decon850_set_size(ctx, mode);

	decon_set_bits(ctx, DECON_GLOBAL_CONTROL, GLOBAL_CONTROL_DECON_EN_F, 0);

	decon850_clear_irqs(ctx);
}

static void decon850_start(struct decon850_context *ctx)
{
	u32 val;
	int ret;

	decon_set_bits(ctx, DECON_GLOBAL_CONTROL,
		       GLOBAL_CONTROL_DECON_EN | GLOBAL_CONTROL_DECON_EN_F, ~0);
	decon_set_bits(ctx, DECON_SHADOW_REG_UPDATE_REQ,
		       SHADOW_REG_UPDATE_REQ_GLOBAL, ~0);

	/* Without a TE trigger DECON runs as soon as the update is requested. */
	ret = readl_poll_timeout(ctx->regs + DECON_GLOBAL_CONTROL, val,
				 val & GLOBAL_CONTROL_RUN_STATUS, 10,
				 DECON_RUN_TIMEOUT_US);
	if (ret)
		dev_err(ctx->dev, "timed out waiting for DECON to run\n");
}

static void decon850_stop(struct decon850_context *ctx)
{
	const struct drm_display_mode *mode = &ctx->crtc->base.state->adjusted_mode;
	int vrefresh = drm_mode_vrefresh(mode) ?: 60;
	/* One frame with a 20% margin */
	unsigned long timeout_us = 1200000 / vrefresh + 5000;
	u32 val;
	int ret;

	/* Stop at the end of the current frame. */
	decon_set_bits(ctx, DECON_GLOBAL_CONTROL, GLOBAL_CONTROL_DECON_EN_F, 0);
	decon_set_bits(ctx, DECON_SHADOW_REG_UPDATE_REQ,
		       SHADOW_REG_UPDATE_REQ_GLOBAL, ~0);
	ret = readl_poll_timeout(ctx->regs + DECON_GLOBAL_CONTROL, val,
				 !(val & GLOBAL_CONTROL_RUN_STATUS), 10,
				 timeout_us);
	if (ret) {
		dev_warn(ctx->dev, "per-frame stop timed out, stopping now\n");
		decon_set_bits(ctx, DECON_GLOBAL_CONTROL,
			       GLOBAL_CONTROL_DECON_EN |
			       GLOBAL_CONTROL_DECON_EN_F, 0);
		decon_set_bits(ctx, DECON_SHADOW_REG_UPDATE_REQ,
			       SHADOW_REG_UPDATE_REQ_GLOBAL, ~0);
		ret = readl_poll_timeout(ctx->regs + DECON_GLOBAL_CONTROL, val,
					 !(val & GLOBAL_CONTROL_RUN_STATUS), 10,
					 timeout_us);
		if (ret)
			dev_err(ctx->dev, "failed to stop DECON\n");
	}

	if (!ret) {
		decon_set_bits(ctx, DECON_GLOBAL_CONTROL,
			       GLOBAL_CONTROL_SRESET, ~0);
		ret = readl_poll_timeout(ctx->regs + DECON_GLOBAL_CONTROL, val,
					 !(val & GLOBAL_CONTROL_SRESET), 10,
					 DECON_RUN_TIMEOUT_US);
		if (ret)
			dev_err(ctx->dev, "failed to reset DECON\n");
	}

	decon850_clear_irqs(ctx);
}

static void decon850_win_set_blend(struct decon850_context *ctx,
				   unsigned int win,
				   const struct drm_plane_state *state)
{
	unsigned int alpha = state->alpha >> 8;
	bool plane_alpha = alpha != 0xff;
	u32 fg_d, bg_d, fg_a, bg_a, func = PD_FUNC_USER_DEFINED;
	u32 alpha0 = 0xff, alpha1 = 0xff;
	u16 mode = state->pixel_blend_mode;

	if (!state->fb->format->has_alpha)
		mode = DRM_MODE_BLEND_PIXEL_NONE;

	if (plane_alpha) {
		alpha0 = alpha;
		alpha1 = 0;
	}

	switch (mode) {
	case DRM_MODE_BLEND_COVERAGE:
		fg_d = plane_alpha ? BND_COEF_ALPHA_MULT : BND_COEF_AF;
		bg_d = plane_alpha ? BND_COEF_1_M_ALPHA_MULT : BND_COEF_1_M_AF;
		break;
	case DRM_MODE_BLEND_PREMULTI:
		fg_d = plane_alpha ? BND_COEF_PLANE_ALPHA0 : BND_COEF_ONE;
		bg_d = plane_alpha ? BND_COEF_1_M_ALPHA_MULT : BND_COEF_1_M_AF;
		break;
	case DRM_MODE_BLEND_PIXEL_NONE:
	default:
		if (plane_alpha) {
			fg_d = BND_COEF_PLANE_ALPHA0;
			bg_d = BND_COEF_1_M_ALPHA_MULT;
		} else {
			func = PD_FUNC_COPY;
			fg_d = BND_COEF_ONE;
			bg_d = BND_COEF_ZERO;
		}
		break;
	}
	fg_a = fg_d;
	bg_a = bg_d;

	decon_set_bits(ctx, DECON_WIN_CONTROL_0(win),
		       WIN_ALPHA_MASK | WIN_FUNC_MASK |
		       WIN_ALPHA_MULT_SRC_SEL_MASK,
		       WIN_ALPHA1(alpha1) | WIN_ALPHA0(alpha0) | WIN_FUNC(func) |
		       WIN_ALPHA_MULT_SRC_SEL(mode == DRM_MODE_BLEND_PIXEL_NONE ?
					      ALPHA_MULT_SRC_SEL_ALPHA0 :
					      ALPHA_MULT_SRC_SEL_AF));
	decon_write(ctx, DECON_WIN_CONTROL_1(win),
		    WIN_FG_ALPHA_D_SEL(fg_d) | WIN_BG_ALPHA_D_SEL(bg_d) |
		    WIN_FG_ALPHA_A_SEL(fg_a) | WIN_BG_ALPHA_A_SEL(bg_a));
}

static void decon850_update_plane(struct exynos_drm_crtc *crtc,
				  struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct decon850_context *ctx = crtc->ctx;
	struct drm_framebuffer *fb = state->base.fb;
	unsigned int win = plane->index;
	unsigned int cpp = fb->format->cpp[0];
	dma_addr_t dma_addr = exynos_drm_fb_dma_addr(fb, 0);
	int fmt = decon850_idma_format(fb->format->format);

	if (WARN_ON(fmt < 0))
		return;

	if (upper_32_bits(dma_addr)) {
		dev_err_ratelimited(ctx->dev,
				    "buffer at %pad is outside the 32-bit range of IDMA\n",
				    &dma_addr);
		return;
	}

	/* IDMA: full buffer size, crop offset and size, all in pixels */
	idma_write(ctx, win, IDMA_SRC_SIZE,
		   IDMA_SRC_HEIGHT(fb->height) |
		   IDMA_SRC_WIDTH(fb->pitches[0] / cpp));
	idma_write(ctx, win, IDMA_SRC_OFFSET,
		   IDMA_SRC_OFFSET_Y(state->src.y) |
		   IDMA_SRC_OFFSET_X(state->src.x));
	idma_write(ctx, win, IDMA_IMG_SIZE,
		   IDMA_IMG_HEIGHT(state->src.h) | IDMA_IMG_WIDTH(state->src.w));
	idma_write(ctx, win, IDMA_IN_BASE_ADDR_Y, lower_32_bits(dma_addr));
	idma_set_bits(ctx, win, IDMA_IN_CON, IDMA_IMG_FORMAT_MASK,
		      IDMA_IMG_FORMAT(fmt));

	/* DECON window */
	decon_write(ctx, DECON_WIN_START_POSITION(win),
		    WIN_POS_Y(state->crtc.y) | WIN_POS_X(state->crtc.x));
	decon_write(ctx, DECON_WIN_END_POSITION(win),
		    WIN_POS_Y(state->crtc.y + state->crtc.h - 1) |
		    WIN_POS_X(state->crtc.x + state->crtc.w - 1));
	decon_write(ctx, DECON_WIN_START_TIME_CONTROL(win), 0);
	decon850_win_set_blend(ctx, win, &state->base);

	decon_set_bits(ctx, DECON_DATA_PATH_CONTROL_0,
		       WIN_EN(win) | WIN_MAPCOLOR_EN(win), WIN_EN(win));
}

static void decon850_disable_plane(struct exynos_drm_crtc *crtc,
				   struct exynos_drm_plane *plane)
{
	struct decon850_context *ctx = crtc->ctx;

	decon_set_bits(ctx, DECON_DATA_PATH_CONTROL_0, WIN_EN(plane->index), 0);
}

static void decon850_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon850_context *ctx = crtc->ctx;
	struct drm_crtc *drm_crtc = &crtc->base;
	unsigned long flags;

	if (ctx->start_pending) {
		/* The windows are configured now, DECON may start. */
		ctx->start_pending = false;
		decon850_start(ctx);
	}

	spin_lock_irqsave(&ctx->lock, flags);

	decon_set_bits(ctx, DECON_SHADOW_REG_UPDATE_REQ,
		       SHADOW_REG_UPDATE_REQ_GLOBAL |
		       SHADOW_REG_UPDATE_REQ_ALL_WIN, ~0);

	if (drm_crtc->state->event) {
		WARN_ON(ctx->event);
		WARN_ON(drm_crtc_vblank_get(drm_crtc) != 0);
		ctx->event = drm_crtc->state->event;
		drm_crtc->state->event = NULL;
	}

	spin_unlock_irqrestore(&ctx->lock, flags);
}

static void decon850_atomic_enable(struct exynos_drm_crtc *crtc)
{
	struct decon850_context *ctx = crtc->ctx;
	const struct drm_display_mode *mode = &crtc->base.state->adjusted_mode;
	unsigned int win;
	int ret;

	if (ctx->boot_on) {
		/* Clocks and the runtime PM reference are held since probe. */
		ctx->boot_on = false;
		decon850_set_size(ctx, mode);
	} else {
		ret = pm_runtime_resume_and_get(ctx->dev);
		if (ret < 0) {
			dev_err(ctx->dev, "failed to resume DECON: %d\n", ret);
			return;
		}
		decon850_init(ctx, mode);
		ctx->start_pending = true;
	}

	for (win = 0; win < DECON_WINDOWS_NR; win++)
		decon850_idma_init(ctx, win);

	/* Windows not managed here may still be on from the bootloader. */
	decon_write(ctx, DECON_DATA_PATH_CONTROL_0, 0);
	decon850_set_chmap(ctx);

	decon_write(ctx, DECON_EXTRA_INTERRUPT_ENABLE,
		    DPU_TIME_OUT_INT_EN | DPU_RESOURCE_CONFLICT_INT_EN);
	decon_set_bits(ctx, DECON_INTERRUPT_ENABLE,
		       DPU_INT_EN | DPU_EXTRA_INT_EN, ~0);
	enable_irq(ctx->irq_frame_start);
	enable_irq(ctx->irq_extra);
}

static void decon850_atomic_disable(struct exynos_drm_crtc *crtc)
{
	struct decon850_context *ctx = crtc->ctx;
	unsigned long flags;

	disable_irq(ctx->irq_frame_start);
	disable_irq(ctx->irq_extra);
	decon_write(ctx, DECON_INTERRUPT_ENABLE, 0);

	/* Do not scan out of buffers that are about to be released. */
	decon_write(ctx, DECON_DATA_PATH_CONTROL_0, 0);

	decon850_stop(ctx);
	ctx->start_pending = false;

	spin_lock_irqsave(&ctx->lock, flags);
	if (ctx->event) {
		spin_lock(&ctx->drm_dev->event_lock);
		drm_crtc_send_vblank_event(&crtc->base, ctx->event);
		spin_unlock(&ctx->drm_dev->event_lock);
		ctx->event = NULL;
		drm_crtc_vblank_put(&crtc->base);
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	pm_runtime_put_sync(ctx->dev);
}

static int decon850_atomic_check(struct exynos_drm_crtc *crtc,
				 struct drm_crtc_state *crtc_state)
{
	struct decon850_context *ctx = crtc->ctx;
	const struct drm_plane_state *plane_state;
	struct drm_plane *plane;

	drm_atomic_crtc_state_for_each_plane_state(plane, plane_state,
						   crtc_state) {
		const struct exynos_drm_plane_state *state =
			container_of(plane_state,
				     const struct exynos_drm_plane_state, base);

		if (!plane_state->fb)
			continue;

		if (state->src.w < DECON_WIN_MIN_SIZE ||
		    state->src.h < DECON_WIN_MIN_SIZE) {
			drm_dbg_atomic(ctx->drm_dev,
				       "[PLANE:%d] %ux%u is below the %ux%u minimum\n",
				       plane->base.id, state->src.w,
				       state->src.h, DECON_WIN_MIN_SIZE,
				       DECON_WIN_MIN_SIZE);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct exynos_drm_crtc_ops decon850_crtc_ops = {
	.atomic_enable		= decon850_atomic_enable,
	.atomic_disable		= decon850_atomic_disable,
	.enable_vblank		= decon850_enable_vblank,
	.disable_vblank		= decon850_disable_vblank,
	.atomic_check		= decon850_atomic_check,
	.update_plane		= decon850_update_plane,
	.disable_plane		= decon850_disable_plane,
	.atomic_flush		= decon850_atomic_flush,
};

static void decon850_handle_frame_start(struct decon850_context *ctx)
{
	struct drm_crtc *crtc = &ctx->crtc->base;

	drm_crtc_handle_vblank(crtc);

	spin_lock(&ctx->lock);
	/* The update has been latched once the request bits are cleared. */
	if (ctx->event && !decon_read(ctx, DECON_SHADOW_REG_UPDATE_REQ)) {
		spin_lock(&ctx->drm_dev->event_lock);
		drm_crtc_send_vblank_event(crtc, ctx->event);
		spin_unlock(&ctx->drm_dev->event_lock);
		ctx->event = NULL;
		drm_crtc_vblank_put(crtc);
	}
	spin_unlock(&ctx->lock);
}

static irqreturn_t decon850_irq_handler(int irq, void *dev_id)
{
	struct decon850_context *ctx = dev_id;
	u32 pend, extra;
	unsigned int win;

	pend = decon_read(ctx, DECON_INTERRUPT_PENDING);
	if (!pend)
		return IRQ_NONE;

	decon_write(ctx, DECON_INTERRUPT_PENDING, pend);

	if (pend & DPU_EXTRA_INT_PEND) {
		extra = decon_read(ctx, DECON_EXTRA_INTERRUPT_PENDING);
		decon_write(ctx, DECON_EXTRA_INTERRUPT_PENDING, extra);

		if (extra & DPU_TIME_OUT_INT_PEND)
			dev_err_ratelimited(ctx->dev, "DECON timeout\n");
		if (extra & DPU_RESOURCE_CONFLICT_INT_PEND)
			dev_err_ratelimited(ctx->dev, "DECON resource conflict\n");

		for (win = 0; win < DECON_WINDOWS_NR; win++) {
			u32 val = idma_read(ctx, win, IDMA_IRQ);

			if (val & IDMA_CONFIG_ERROR)
				dev_err_ratelimited(ctx->dev,
						    "IDMA%u config error 0x%x\n",
						    decon850_wins[win].ch,
						    idma_read(ctx, win,
							      IDMA_CFG_ERR_STATE));
			idma_write(ctx, win, IDMA_IRQ,
				   (val & ~IDMA_ALL_IRQ_CLEAR) |
				   IDMA_ALL_IRQ_CLEAR);
		}
	}

	if (pend & DPU_FRAME_START_INT_PEND)
		decon850_handle_frame_start(ctx);

	return IRQ_HANDLED;
}

static int decon850_bind(struct device *dev, struct device *master, void *data)
{
	struct decon850_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	unsigned int win;
	int ret;

	ctx->drm_dev = drm_dev;

	for (win = 0; win < DECON_WINDOWS_NR; win++) {
		ctx->configs[win].pixel_formats = decon850_formats;
		ctx->configs[win].num_pixel_formats = ARRAY_SIZE(decon850_formats);
		ctx->configs[win].zpos = win;
		ctx->configs[win].type = win == DECON_PRIMARY_WIN ?
					 DRM_PLANE_TYPE_PRIMARY :
					 DRM_PLANE_TYPE_OVERLAY;
		ctx->configs[win].capabilities = EXYNOS_DRM_PLANE_CAP_PIX_BLEND |
						 EXYNOS_DRM_PLANE_CAP_WIN_BLEND;

		ret = exynos_plane_init(drm_dev, &ctx->planes[win], win,
					&ctx->configs[win]);
		if (ret)
			return ret;
	}

	ctx->crtc = exynos_drm_crtc_create(drm_dev,
					   &ctx->planes[DECON_PRIMARY_WIN].base,
					   EXYNOS_DISPLAY_TYPE_LCD,
					   &decon850_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc))
		return PTR_ERR(ctx->crtc);

	return exynos_drm_register_dma(drm_dev, dev, &ctx->dma_priv);
}

static void decon850_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct decon850_context *ctx = dev_get_drvdata(dev);

	if (ctx->crtc->base.state && ctx->crtc->base.state->active)
		decon850_atomic_disable(ctx->crtc);

	exynos_drm_unregister_dma(ctx->drm_dev, ctx->dev, &ctx->dma_priv);
}

static const struct component_ops decon850_component_ops = {
	.bind	= decon850_bind,
	.unbind	= decon850_unbind,
};

static int decon850_request_irq(struct decon850_context *ctx, const char *name)
{
	struct platform_device *pdev = to_platform_device(ctx->dev);
	int irq, ret;

	irq = platform_get_irq_byname(pdev, name);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(ctx->dev, irq, decon850_irq_handler,
			       IRQF_NO_AUTOEN, dev_name(ctx->dev), ctx);
	if (ret)
		return dev_err_probe(ctx->dev, ret, "failed to request %s IRQ\n",
				     name);

	return irq;
}

static unsigned int decon850_ch_dma_offset(unsigned int ch)
{
	switch (ch) {
	case DECON_CH_G0:
		return IDMA_CH_G0_OFFSET;
	case DECON_CH_G1:
		return IDMA_CH_G1_OFFSET;
	case DECON_CH_G2:
		return IDMA_CH_G2_OFFSET;
	case DECON_CH_VG0:
	default:
		return IDMA_CH_VG0_OFFSET;
	}
}

static int decon850_check_boot_state(struct decon850_context *ctx)
{
	void __iomem *boot_dma;
	unsigned int win, ch;
	int ret;

	ret = clk_bulk_prepare_enable(ctx->num_clks, ctx->clks);
	if (ret)
		return ret;

	ctx->boot_on = decon_read(ctx, DECON_GLOBAL_CONTROL) &
		       GLOBAL_CONTROL_RUN_STATUS;
	if (!ctx->boot_on) {
		clk_bulk_disable_unprepare(ctx->num_clks, ctx->clks);
		return 0;
	}

	ch = decon_read(ctx, DECON_DATA_PATH_CONTROL_1) & 0x7;
	boot_dma = ctx->dma_regs + decon850_ch_dma_offset(ch);
	dev_info(ctx->dev,
		 "running from bootloader: win0 ch%u, format %u base 0x%08x\n",
		 ch, IDMA_IMG_FORMAT_GET(readl(boot_dma + IDMA_IN_CON)),
		 readl(boot_dma + IDMA_IN_BASE_ADDR_Y));
	for (win = 0; win < DECON_WINDOWS_NR; win++)
		dev_dbg(ctx->dev, "IDMA%u in_con 0x%08x\n",
			decon850_wins[win].ch, idma_read(ctx, win, IDMA_IN_CON));

	/* Interrupts may be pending from the bootloader. */
	decon_write(ctx, DECON_INTERRUPT_ENABLE, 0);
	decon850_clear_irqs(ctx);

	return 0;
}

static int decon850_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct decon850_context *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	spin_lock_init(&ctx->lock);

	ctx->regs = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);

	ctx->dma_regs = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(ctx->dma_regs))
		return PTR_ERR(ctx->dma_regs);

	ctx->num_clks = devm_clk_bulk_get_all(dev, &ctx->clks);
	if (ctx->num_clks < 0)
		return dev_err_probe(dev, ctx->num_clks, "failed to get clocks\n");

	ctx->irq_frame_start = decon850_request_irq(ctx, "frame-start");
	if (ctx->irq_frame_start < 0)
		return ctx->irq_frame_start;

	ctx->irq_extra = decon850_request_irq(ctx, "extra");
	if (ctx->irq_extra < 0)
		return ctx->irq_extra;

	platform_set_drvdata(pdev, ctx);

	ret = decon850_check_boot_state(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	if (ctx->boot_on) {
		pm_runtime_set_active(dev);
		pm_runtime_get_noresume(dev);
	}
	pm_runtime_enable(dev);

	ret = component_add(dev, &decon850_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	return 0;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	if (ctx->boot_on) {
		pm_runtime_put_noidle(dev);
		pm_runtime_set_suspended(dev);
		clk_bulk_disable_unprepare(ctx->num_clks, ctx->clks);
	}

	return ret;
}

static void decon850_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &decon850_component_ops);
	pm_runtime_disable(&pdev->dev);
}

static int decon850_runtime_suspend(struct device *dev)
{
	struct decon850_context *ctx = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ctx->num_clks, ctx->clks);

	return 0;
}

static int decon850_runtime_resume(struct device *dev)
{
	struct decon850_context *ctx = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ctx->num_clks, ctx->clks);
}

static DEFINE_RUNTIME_DEV_PM_OPS(decon850_pm_ops, decon850_runtime_suspend,
				 decon850_runtime_resume, NULL);

static const struct of_device_id decon850_of_match[] = {
	{ .compatible = "samsung,exynos850-decon" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, decon850_of_match);

struct platform_driver exynos850_decon_driver = {
	.probe		= decon850_probe,
	.remove		= decon850_remove,
	.driver		= {
		.name	= "exynos850-decon",
		.pm	= pm_ptr(&decon850_pm_ops),
		.of_match_table = decon850_of_match,
	},
};
