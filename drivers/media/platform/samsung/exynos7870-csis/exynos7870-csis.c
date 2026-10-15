// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung Exynos7870 MIPI CSI-2 receiver with DMA (CSIS v4.1)
 *
 * The receiver of this SoC writes the frames to memory by itself, one DMA
 * engine per virtual channel, without going through the ISP. This driver
 * uses that path only: the sensor is linked straight to a capture video
 * node, and the frames come out raw, as the sensor sends them.
 *
 * Register layout and programming sequence from the vendor driver of the
 * Galaxy Tab A 10.1 (2016), fimc-is-hw-csi-v4_0.[ch] and
 * fimc-is-device-csi_v2.c.
 *
 * Buffer handling: the DMA address registers are shadowed and latched at
 * frame start. An address written during frame N is used for frame N+1,
 * so at every DMA frame start the buffer programmed before it becomes the
 * active one and the next queued buffer is programmed; at DMA frame end
 * the active buffer is complete. With no buffer to program, the DMA of
 * the next frame is disabled and that frame is dropped; a buffer queued
 * while the DMA is off re-enables it at once.
 *
 * The DMA start and end interrupts are used, not the CSI-2 frame start:
 * under memory load the DMA drains the last lines of frame N after the
 * link has started frame N+1.
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#define CSIS_DRV_NAME			"exynos7870-csis"

#define CSIS_VERSION			0x0000
#define CSIS_CMN_CTRL			0x0004
#define CSIS_CMN_CTRL_UPDATE_SHADOW	GENMASK(19, 16)
#define CSIS_CMN_CTRL_DESKEW_EN		BIT(12)
#define CSIS_CMN_CTRL_INTERLEAVE	GENMASK(11, 10)
#define CSIS_CMN_CTRL_LANE_NUMBER	GENMASK(9, 8)
#define CSIS_CMN_CTRL_SW_RESET		BIT(1)
#define CSIS_CMN_CTRL_CSI_EN		BIT(0)
#define CSIS_INT_MSK0			0x0010
#define CSIS_INT_SRC0			0x0014
#define CSIS_INT_MSK1			0x0018
#define CSIS_INT_SRC1			0x001c
#define CSIS_DPHY_STATUS		0x0020
#define CSIS_DPHY_CMN_CTRL		0x0024
#define CSIS_DPHY_HSSETTLE		GENMASK(31, 24)
#define CSIS_DPHY_ENABLE_DAT		GENMASK(4, 1)
#define CSIS_DPHY_ENABLE_CLK		BIT(0)
#define CSIS_ISP_CONFIG_CH0		0x0040
#define CSIS_ISP_CONFIG_DOUBLE_CMPNT	BIT(12)
#define CSIS_ISP_CONFIG_PARALLEL	BIT(11)
#define CSIS_ISP_CONFIG_DATAFORMAT	GENMASK(7, 2)
#define CSIS_ISP_CONFIG_VC		GENMASK(1, 0)
#define CSIS_ISP_RESOL_CH0		0x0044
#define CSIS_FRM_CNT_CH0		0x0100
#define CSIS_DMA0_CTRL			0x1000
#define CSIS_DMA_CTRL_FRAMEPTR		GENMASK(4, 2)
#define CSIS_DMA_CTRL_UPDT_PTR_EN	BIT(1)
#define CSIS_DMA_CTRL_DISABLE		BIT(0)
#define CSIS_DMA0_FMT			0x1004
#define CSIS_DMA_FMT_1D			BIT(15)
#define CSIS_DMA_FMT_PACK12		BIT(14)
#define CSIS_DMA_FMT_BYTESWAP		BIT(12)
#define CSIS_DMA0_ADDR(n)		(0x1010 + 4 * (n))
#define CSIS_DMA0_ACT_CTRL		0x1030
#define CSIS_DMA0_BYTE_CNT		0x1040
#define CSIS_DMA_ERR_CODE		0x1404

/* Vendor defaults (CSIS_IRQ_MASK0/1): frame start/end, errors, DMA. */
#define CSIS_INT0_ENABLE		0x011fffff
#define CSIS_INT0_FRAMESTART_CH0	BIT(24)
#define CSIS_INT0_ERRORS		GENMASK(19, 0)
#define CSIS_INT1_ENABLE		0x00003ff0
#define CSIS_INT1_DMA_ERROR		BIT(12)
#define CSIS_INT1_DMA_FRM_END_CH0	BIT(8)
#define CSIS_INT1_DMA_FRM_START_CH0	BIT(4)

#define CSIS_DT_RAW8			0x2a
#define CSIS_DT_RAW10			0x2b

#define CSIS_MIN_SIZE			16
#define CSIS_MAX_SIZE			8192

/* D-PHY HS settle count, the vendor value */
#define CSIS_HS_SETTLE			15

struct csis_format {
	u32 fourcc;
	u32 mbus_code;
	u8 data_type;
	/* bytes per line for a line of 'width' pixels */
	u32 (*bpl)(u32 width);
};

static u32 csis_bpl_16(u32 width)
{
	return width * 2;
}

static const struct csis_format csis_formats[] = {
	{ V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, CSIS_DT_RAW10, csis_bpl_16 },
	{ V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, CSIS_DT_RAW10, csis_bpl_16 },
	{ V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, CSIS_DT_RAW10, csis_bpl_16 },
	{ V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, CSIS_DT_RAW10, csis_bpl_16 },
};

struct csis_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

#define CSIS_CHANNELS			2

struct csis_dev;

/* One receiver (CSIS0 or CSIS1) and its capture node. */
struct csis {
	struct csis_dev *parent;
	unsigned int id;
	bool present;
	struct device *dev;
	void __iomem *regs;
	int irq;
	struct phy *phy;
	unsigned int lanes;

	struct video_device vdev;
	struct media_pad vdev_pad;
	struct media_pipeline pipe;
	struct vb2_queue queue;
	struct mutex lock;

	struct v4l2_subdev *sensor;
	u32 sensor_pad;

	struct v4l2_pix_format fmt;
	const struct csis_format *cfmt;

	spinlock_t slock;
	struct list_head pending;
	struct csis_buffer *active;
	struct csis_buffer *next;
	u32 sequence;
	unsigned int errors;
	unsigned int done;
	unsigned int dropped;
	bool running;
};

static inline u32 csis_read(struct csis *csis, u32 reg)
{
	return readl(csis->regs + reg);
}

static inline void csis_write(struct csis *csis, u32 reg, u32 val)
{
	writel(val, csis->regs + reg);
}

static void csis_update(struct csis *csis, u32 reg, u32 mask, u32 val)
{
	csis_write(csis, reg, (csis_read(csis, reg) & ~mask) | (val & mask));
}

static const struct csis_format *csis_find_format(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csis_formats); i++)
		if (csis_formats[i].fourcc == fourcc)
			return &csis_formats[i];
	return NULL;
}

static inline struct csis_buffer *to_csis_buffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct csis_buffer, vb);
}

/* ------------------------------------------------------------------ DMA */

/* Called with slock held. */
static void csis_dma_program(struct csis *csis, struct csis_buffer *buf)
{
	u32 ctrl = csis_read(csis, CSIS_DMA0_CTRL);
	unsigned int i;

	if (!buf) {
		ctrl |= CSIS_DMA_CTRL_DISABLE;
		ctrl &= ~CSIS_DMA_CTRL_UPDT_PTR_EN;
		csis_write(csis, CSIS_DMA0_CTRL, ctrl);
		return;
	}

	/*
	 * The vendor writes the address to all eight frame pointers: if the
	 * frame pointer advances on its own, it never lands on a stale one.
	 */
	for (i = 0; i < 8; i++)
		csis_write(csis, CSIS_DMA0_ADDR(i),
			   vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0));

	ctrl &= ~(CSIS_DMA_CTRL_DISABLE | FIELD_PREP(CSIS_DMA_CTRL_FRAMEPTR, 1));
	ctrl |= CSIS_DMA_CTRL_UPDT_PTR_EN;
	csis_write(csis, CSIS_DMA0_CTRL, ctrl);
}

static struct csis_buffer *csis_pop_pending(struct csis *csis)
{
	struct csis_buffer *buf;

	buf = list_first_entry_or_null(&csis->pending, struct csis_buffer, list);
	if (buf)
		list_del(&buf->list);
	return buf;
}

static void csis_buffer_done(struct csis *csis, struct csis_buffer *buf,
			     enum vb2_buffer_state state)
{
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	/* The frame that ends here is the last one started, numbered from 0. */
	buf->vb.sequence = csis->sequence - 1;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, state);
}

static irqreturn_t csis_irq(int irq, void *data)
{
	struct csis *csis = data;
	u32 src0, src1;

	src0 = csis_read(csis, CSIS_INT_SRC0);
	src1 = csis_read(csis, CSIS_INT_SRC1);
	csis_write(csis, CSIS_INT_SRC0, src0);
	csis_write(csis, CSIS_INT_SRC1, src1);

	if (!src0 && !src1)
		return IRQ_NONE;

	spin_lock(&csis->slock);

	if ((src0 & CSIS_INT0_ERRORS) || (src1 & CSIS_INT1_DMA_ERROR)) {
		csis->errors++;
		dev_err_ratelimited(csis->dev,
				    "csis%u errors: src0 %08x src1 %08x dma %08x\n",
				    csis->id, src0, src1,
				    csis_read(csis, CSIS_DMA_ERR_CODE));
	}

	/* Frames seen on the link, captured or not. */
	if (src0 & CSIS_INT0_FRAMESTART_CH0) {
		struct v4l2_event ev = {
			.type = V4L2_EVENT_FRAME_SYNC,
		};

		/*
		 * The sequence the buffer of this frame will carry, from 0 as
		 * V4L2 wants: the sensor controls are applied at frame
		 * boundaries against it.
		 */
		ev.u.frame_sync.frame_sequence = csis->sequence++;
		v4l2_event_queue(&csis->vdev, &ev);
	}

	/* For the DMA engine, the end of frame N precedes the start of N+1. */
	if (src1 & CSIS_INT1_DMA_FRM_END_CH0) {
		if (csis->active) {
			csis_buffer_done(csis, csis->active, VB2_BUF_STATE_DONE);
			csis->active = NULL;
			csis->done++;
		}
	}

	if (src1 & CSIS_INT1_DMA_FRM_START_CH0) {
		if (csis->active) {
			/* Its end never came: give it back as broken. */
			csis_buffer_done(csis, csis->active, VB2_BUF_STATE_ERROR);
			csis->dropped++;
		}
		csis->active = csis->next;
		csis->next = csis_pop_pending(csis);
		csis_dma_program(csis, csis->next);
	}

	spin_unlock(&csis->slock);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------ hardware */

static int csis_hw_start(struct csis *csis)
{
	unsigned int i;
	u32 val;

	csis_update(csis, CSIS_CMN_CTRL, CSIS_CMN_CTRL_SW_RESET,
		    CSIS_CMN_CTRL_SW_RESET);
	for (i = 0; i < 10; i++) {
		udelay(10);
		if (!(csis_read(csis, CSIS_CMN_CTRL) & CSIS_CMN_CTRL_SW_RESET))
			break;
	}
	if (i == 10) {
		dev_err(csis->dev, "reset does not complete\n");
		return -ETIMEDOUT;
	}

	csis_update(csis, CSIS_DMA0_CTRL, CSIS_DMA_CTRL_DISABLE |
		    CSIS_DMA_CTRL_UPDT_PTR_EN, CSIS_DMA_CTRL_DISABLE);

	csis_update(csis, CSIS_DPHY_CMN_CTRL,
		    CSIS_DPHY_HSSETTLE | CSIS_DPHY_ENABLE_DAT,
		    FIELD_PREP(CSIS_DPHY_HSSETTLE, CSIS_HS_SETTLE) |
		    FIELD_PREP(CSIS_DPHY_ENABLE_DAT, (1 << csis->lanes) - 1));

	csis_update(csis, CSIS_CMN_CTRL,
		    CSIS_CMN_CTRL_LANE_NUMBER | CSIS_CMN_CTRL_DESKEW_EN |
		    CSIS_CMN_CTRL_INTERLEAVE,
		    FIELD_PREP(CSIS_CMN_CTRL_LANE_NUMBER, csis->lanes - 1));

	csis_update(csis, CSIS_ISP_CONFIG_CH0,
		    CSIS_ISP_CONFIG_VC | CSIS_ISP_CONFIG_DATAFORMAT |
		    CSIS_ISP_CONFIG_PARALLEL | CSIS_ISP_CONFIG_DOUBLE_CMPNT,
		    FIELD_PREP(CSIS_ISP_CONFIG_DATAFORMAT, csis->cfmt->data_type));
	csis_write(csis, CSIS_ISP_RESOL_CH0,
		   (csis->fmt.height << 16) | csis->fmt.width);

	val = csis_read(csis, CSIS_DMA0_FMT);
	val &= ~(CSIS_DMA_FMT_1D | CSIS_DMA_FMT_PACK12 | CSIS_DMA_FMT_BYTESWAP);
	csis_write(csis, CSIS_DMA0_FMT, val);

	csis_write(csis, CSIS_INT_SRC0, ~0);
	csis_write(csis, CSIS_INT_SRC1, ~0);
	csis_write(csis, CSIS_INT_MSK0, CSIS_INT0_ENABLE);
	csis_write(csis, CSIS_INT_MSK1, CSIS_INT1_ENABLE);

	/* The first buffer: latched at the first frame start. */
	spin_lock_irq(&csis->slock);
	csis->next = csis_pop_pending(csis);
	csis_dma_program(csis, csis->next);
	csis->running = true;
	spin_unlock_irq(&csis->slock);

	csis_update(csis, CSIS_CMN_CTRL, CSIS_CMN_CTRL_UPDATE_SHADOW,
		    CSIS_CMN_CTRL_UPDATE_SHADOW);
	csis_update(csis, CSIS_DPHY_CMN_CTRL, CSIS_DPHY_ENABLE_CLK,
		    CSIS_DPHY_ENABLE_CLK);
	csis_update(csis, CSIS_CMN_CTRL, CSIS_CMN_CTRL_CSI_EN,
		    CSIS_CMN_CTRL_CSI_EN);

	dev_dbg(csis->dev, "csis%u started: version %08x, %u lanes, settle %u\n",
		csis->id, csis_read(csis, CSIS_VERSION), csis->lanes, CSIS_HS_SETTLE);

	return 0;
}

static void csis_hw_stop(struct csis *csis)
{
	spin_lock_irq(&csis->slock);
	csis->running = false;
	spin_unlock_irq(&csis->slock);

	csis_write(csis, CSIS_INT_MSK0, 0);
	csis_write(csis, CSIS_INT_MSK1, 0);
	csis_update(csis, CSIS_DMA0_CTRL, CSIS_DMA_CTRL_DISABLE |
		    CSIS_DMA_CTRL_UPDT_PTR_EN, CSIS_DMA_CTRL_DISABLE);
	csis_update(csis, CSIS_DPHY_CMN_CTRL,
		    CSIS_DPHY_ENABLE_CLK | CSIS_DPHY_ENABLE_DAT, 0);
	csis_update(csis, CSIS_CMN_CTRL, CSIS_CMN_CTRL_CSI_EN, 0);
}

/* ---------------------------------------------------------------- vb2 */

static int csis_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct csis *csis = vb2_get_drv_priv(q);

	if (*nplanes)
		return sizes[0] < csis->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = csis->fmt.sizeimage;
	return 0;
}

static int csis_buf_prepare(struct vb2_buffer *vb)
{
	struct csis *csis = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < csis->fmt.sizeimage)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, csis->fmt.sizeimage);
	return 0;
}

static void csis_buf_queue(struct vb2_buffer *vb)
{
	struct csis *csis = vb2_get_drv_priv(vb->vb2_queue);
	struct csis_buffer *buf = to_csis_buffer(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	list_add_tail(&buf->list, &csis->pending);
	/*
	 * The DMA was switched off for lack of buffers: no DMA frame start
	 * will come to switch it on again, so program this one now. It is
	 * latched at the next frame start.
	 */
	if (csis->running && !csis->next) {
		csis->next = csis_pop_pending(csis);
		csis_dma_program(csis, csis->next);
	}
	spin_unlock_irqrestore(&csis->slock, flags);
}

static void csis_return_all(struct csis *csis, enum vb2_buffer_state state)
{
	struct csis_buffer *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&csis->slock, flags);
	if (csis->active)
		vb2_buffer_done(&csis->active->vb.vb2_buf, state);
	if (csis->next)
		vb2_buffer_done(&csis->next->vb.vb2_buf, state);
	csis->active = NULL;
	csis->next = NULL;
	list_for_each_entry_safe(buf, tmp, &csis->pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&csis->slock, flags);
}

static int csis_check_sensor_format(struct csis *csis)
{
	struct v4l2_subdev_format sfmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = csis->sensor_pad,
	};
	int ret;

	ret = v4l2_subdev_call_state_active(csis->sensor, pad, get_fmt, &sfmt);
	if (ret)
		return ret;

	if (sfmt.format.width != csis->fmt.width ||
	    sfmt.format.height != csis->fmt.height ||
	    sfmt.format.code != csis->cfmt->mbus_code) {
		dev_dbg(csis->dev, "sensor %ux%u %04x, node %ux%u %04x\n",
			sfmt.format.width, sfmt.format.height, sfmt.format.code,
			csis->fmt.width, csis->fmt.height,
			csis->cfmt->mbus_code);
		return -EPIPE;
	}

	return 0;
}

static int csis_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct csis *csis = vb2_get_drv_priv(q);
	int ret;

	if (!csis->sensor) {
		ret = -ENODEV;
		goto err_return;
	}

	ret = video_device_pipeline_start(&csis->vdev, &csis->pipe);
	if (ret)
		goto err_return;

	ret = csis_check_sensor_format(csis);
	if (ret)
		goto err_pipeline;

	ret = pm_runtime_resume_and_get(csis->dev);
	if (ret)
		goto err_pipeline;

	ret = phy_power_on(csis->phy);
	if (ret)
		goto err_pm;

	csis->sequence = 0;
	csis->errors = 0;
	csis->done = 0;
	csis->dropped = 0;

	ret = csis_hw_start(csis);
	if (ret)
		goto err_phy;

	ret = v4l2_subdev_enable_streams(csis->sensor, csis->sensor_pad, BIT(0));
	if (ret)
		goto err_hw;

	return 0;

err_hw:
	csis_hw_stop(csis);
err_phy:
	phy_power_off(csis->phy);
err_pm:
	pm_runtime_put(csis->dev);
err_pipeline:
	video_device_pipeline_stop(&csis->vdev);
err_return:
	csis_return_all(csis, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void csis_stop_streaming(struct vb2_queue *q)
{
	struct csis *csis = vb2_get_drv_priv(q);

	v4l2_subdev_disable_streams(csis->sensor, csis->sensor_pad, BIT(0));
	csis_hw_stop(csis);
	synchronize_irq(csis->irq);
	phy_power_off(csis->phy);
	pm_runtime_put(csis->dev);
	video_device_pipeline_stop(&csis->vdev);

	dev_dbg(csis->dev,
		"csis%u: %u frames on the link, %u captured, %u broken, %u error interrupts\n",
		csis->id, csis->sequence, csis->done, csis->dropped, csis->errors);

	csis_return_all(csis, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops csis_vb2_ops = {
	.queue_setup = csis_queue_setup,
	.buf_prepare = csis_buf_prepare,
	.buf_queue = csis_buf_queue,
	.start_streaming = csis_start_streaming,
	.stop_streaming = csis_stop_streaming,
};

/* --------------------------------------------------------------- V4L2 */

static void csis_try_fmt(struct v4l2_pix_format *pix,
			 const struct csis_format **cfmt_out)
{
	const struct csis_format *cfmt = csis_find_format(pix->pixelformat);

	if (!cfmt)
		cfmt = &csis_formats[0];

	pix->pixelformat = cfmt->fourcc;
	pix->width = clamp_t(u32, pix->width, CSIS_MIN_SIZE, CSIS_MAX_SIZE);
	pix->height = clamp_t(u32, pix->height, CSIS_MIN_SIZE, CSIS_MAX_SIZE);
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = cfmt->bpl(pix->width);
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;

	if (cfmt_out)
		*cfmt_out = cfmt;
}

static int csis_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	struct csis *csis = video_drvdata(file);

	strscpy(cap->driver, CSIS_DRV_NAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "Exynos7870 CSIS%u", csis->id);

	return 0;
}

static int csis_enum_fmt(struct file *file, void *priv,
			 struct v4l2_fmtdesc *f)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(csis_formats); i++) {
		if (f->mbus_code && csis_formats[i].mbus_code != f->mbus_code)
			continue;
		if (n++ == f->index) {
			f->pixelformat = csis_formats[i].fourcc;
			return 0;
		}
	}
	return -EINVAL;
}

static int csis_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct csis *csis = video_drvdata(file);

	f->fmt.pix = csis->fmt;
	return 0;
}

static int csis_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	csis_try_fmt(&f->fmt.pix, NULL);
	return 0;
}

static int csis_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct csis *csis = video_drvdata(file);

	if (vb2_is_busy(&csis->queue))
		return -EBUSY;

	csis_try_fmt(&f->fmt.pix, &csis->cfmt);
	csis->fmt = f->fmt.pix;
	return 0;
}

static int csis_enum_framesizes(struct file *file, void *priv,
				struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || !csis_find_format(fsize->pixel_format))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = CSIS_MIN_SIZE;
	fsize->stepwise.max_width = CSIS_MAX_SIZE;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = CSIS_MIN_SIZE;
	fsize->stepwise.max_height = CSIS_MAX_SIZE;
	fsize->stepwise.step_height = 1;
	return 0;
}

static int csis_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC || sub->id != 0)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 2, NULL);
}

static const struct v4l2_ioctl_ops csis_ioctl_ops = {
	.vidioc_querycap = csis_querycap,
	.vidioc_enum_fmt_vid_cap = csis_enum_fmt,
	.vidioc_g_fmt_vid_cap = csis_g_fmt,
	.vidioc_s_fmt_vid_cap = csis_s_fmt,
	.vidioc_try_fmt_vid_cap = csis_try_fmt_vid_cap,
	.vidioc_enum_framesizes = csis_enum_framesizes,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_subscribe_event = csis_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations csis_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/*
 * The two receivers sit behind one SysMMU, and exynos-iommu keeps one
 * master per SysMMU: they are one device, hence one IOMMU domain, with a
 * channel per receiver, one media graph and one camera per sensor.
 */
struct csis_dev {
	struct device *dev;
	struct clk_bulk_data clks[2];
	struct media_device mdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct csis chans[CSIS_CHANNELS];
};

struct csis_asc {
	struct v4l2_async_connection asc;
	struct csis *chan;
};

/* -------------------------------------------------------------- async */

static int csis_notify_bound(struct v4l2_async_notifier *notifier,
			     struct v4l2_subdev *sd,
			     struct v4l2_async_connection *asc)
{
	struct csis *csis = container_of(asc, struct csis_asc, asc)->chan;
	int pad;

	pad = media_entity_get_fwnode_pad(&sd->entity, asc->match.fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (pad < 0)
		return pad;

	csis->sensor = sd;
	csis->sensor_pad = pad;

	return media_create_pad_link(&sd->entity, pad, &csis->vdev.entity, 0,
				     MEDIA_LNK_FL_ENABLED |
				     MEDIA_LNK_FL_IMMUTABLE);
}

static int csis_notify_complete(struct v4l2_async_notifier *notifier)
{
	struct csis_dev *cdev = container_of(notifier, struct csis_dev, notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&cdev->v4l2_dev);
	if (ret)
		return ret;

	return media_device_register(&cdev->mdev);
}

static const struct v4l2_async_notifier_operations csis_notify_ops = {
	.bound = csis_notify_bound,
	.complete = csis_notify_complete,
};

/* The channel of port @id, if the device tree connects it to a sensor. */
static int csis_parse_port(struct csis_dev *cdev, unsigned int id)
{
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct platform_device *pdev = to_platform_device(cdev->dev);
	struct csis *csis = &cdev->chans[id];
	struct csis_asc *asc;
	struct fwnode_handle *ep;
	char name[8];
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(cdev->dev), id, 0, 0);
	if (!ep)
		return 0;

	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret)
		goto out;
	csis->lanes = vep.bus.mipi_csi2.num_data_lanes;
	if (csis->lanes < 1 || csis->lanes > 4) {
		ret = dev_err_probe(cdev->dev, -EINVAL, "port %u: %u lanes\n",
				    id, csis->lanes);
		goto out;
	}

	csis->regs = devm_platform_ioremap_resource(pdev, id);
	if (IS_ERR(csis->regs)) {
		ret = PTR_ERR(csis->regs);
		goto out;
	}

	csis->irq = platform_get_irq(pdev, id);
	if (csis->irq < 0) {
		ret = csis->irq;
		goto out;
	}

	snprintf(name, sizeof(name), "csis%u", id);
	csis->phy = devm_phy_get(cdev->dev, name);
	if (IS_ERR(csis->phy)) {
		ret = dev_err_probe(cdev->dev, PTR_ERR(csis->phy),
				    "failed to get PHY %s\n", name);
		goto out;
	}

	ret = devm_request_irq(cdev->dev, csis->irq, csis_irq, 0,
			       devm_kasprintf(cdev->dev, GFP_KERNEL, "%s:%s",
					      dev_name(cdev->dev), name),
			       csis);
	if (ret)
		goto out;

	asc = v4l2_async_nf_add_fwnode_remote(&cdev->notifier, ep,
					      struct csis_asc);
	if (IS_ERR(asc)) {
		ret = PTR_ERR(asc);
		goto out;
	}
	asc->chan = csis;
	csis->present = true;
	ret = 0;

out:
	fwnode_handle_put(ep);
	return ret;
}

/* -------------------------------------------------------------- probe */

static int csis_register_chan(struct csis *csis)
{
	struct video_device *vdev = &csis->vdev;
	struct vb2_queue *q = &csis->queue;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = csis;
	q->buf_struct_size = sizeof(struct csis_buffer);
	q->ops = &csis_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 1;
	/*
	 * The frames are read by the CPU (software ISP): cached buffers,
	 * synchronised by vb2 and by DMA_BUF_IOCTL_SYNC, instead of an
	 * uncached mapping that makes every read crawl.
	 */
	q->non_coherent_mem = 1;
	q->lock = &csis->lock;
	q->dev = csis->dev;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), CSIS_DRV_NAME "%u", csis->id);
	vdev->fops = &csis_fops;
	vdev->ioctl_ops = &csis_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->v4l2_dev = &csis->parent->v4l2_dev;
	vdev->queue = q;
	vdev->lock = &csis->lock;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(vdev, csis);

	csis->vdev_pad.flags = MEDIA_PAD_FL_SINK | MEDIA_PAD_FL_MUST_CONNECT;
	ret = media_entity_pads_init(&vdev->entity, 1, &csis->vdev_pad);
	if (ret)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		media_entity_cleanup(&vdev->entity);

	return ret;
}

static void csis_unregister_chans(struct csis_dev *cdev)
{
	unsigned int i;

	for (i = 0; i < CSIS_CHANNELS; i++) {
		struct csis *csis = &cdev->chans[i];

		if (!csis->present || !video_is_registered(&csis->vdev))
			continue;
		video_unregister_device(&csis->vdev);
		media_entity_cleanup(&csis->vdev.entity);
	}
}

static int csis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csis_dev *cdev;
	unsigned int i, present = 0;
	int ret;

	cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);
	if (!cdev)
		return -ENOMEM;

	cdev->dev = dev;
	platform_set_drvdata(pdev, cdev);

	cdev->clks[0].id = "aclk";
	cdev->clks[1].id = "pclk";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(cdev->clks), cdev->clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clocks\n");

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	cdev->mdev.dev = dev;
	strscpy(cdev->mdev.model, "Exynos7870 CSIS", sizeof(cdev->mdev.model));
	media_device_init(&cdev->mdev);

	cdev->v4l2_dev.mdev = &cdev->mdev;
	ret = v4l2_device_register(dev, &cdev->v4l2_dev);
	if (ret)
		goto err_mdev;

	v4l2_async_nf_init(&cdev->notifier, &cdev->v4l2_dev);
	cdev->notifier.ops = &csis_notify_ops;

	for (i = 0; i < CSIS_CHANNELS; i++) {
		struct csis *csis = &cdev->chans[i];

		csis->parent = cdev;
		csis->id = i;
		csis->dev = dev;
		mutex_init(&csis->lock);
		spin_lock_init(&csis->slock);
		INIT_LIST_HEAD(&csis->pending);
		csis->fmt.pixelformat = V4L2_PIX_FMT_SBGGR10;
		csis->fmt.width = 1632;
		csis->fmt.height = 1228;
		csis_try_fmt(&csis->fmt, &csis->cfmt);

		ret = csis_parse_port(cdev, i);
		if (ret)
			goto err_nf;
		present += csis->present;
	}
	if (!present) {
		ret = dev_err_probe(dev, -ENODEV, "no port connected\n");
		goto err_nf;
	}

	pm_runtime_enable(dev);

	for (i = 0; i < CSIS_CHANNELS; i++) {
		if (!cdev->chans[i].present)
			continue;
		ret = csis_register_chan(&cdev->chans[i]);
		if (ret)
			goto err_chans;
	}

	ret = v4l2_async_nf_register(&cdev->notifier);
	if (ret)
		goto err_chans;

	return 0;

err_chans:
	csis_unregister_chans(cdev);
	pm_runtime_disable(dev);
err_nf:
	v4l2_async_nf_cleanup(&cdev->notifier);
	v4l2_device_unregister(&cdev->v4l2_dev);
err_mdev:
	media_device_cleanup(&cdev->mdev);
	return ret;
}

static void csis_remove(struct platform_device *pdev)
{
	struct csis_dev *cdev = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&cdev->notifier);
	v4l2_async_nf_cleanup(&cdev->notifier);
	media_device_unregister(&cdev->mdev);
	csis_unregister_chans(cdev);
	v4l2_device_unregister(&cdev->v4l2_dev);
	media_device_cleanup(&cdev->mdev);
	pm_runtime_disable(&pdev->dev);
}

static int csis_runtime_suspend(struct device *dev)
{
	struct csis_dev *cdev = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(ARRAY_SIZE(cdev->clks), cdev->clks);
	return 0;
}

static int csis_runtime_resume(struct device *dev)
{
	struct csis_dev *cdev = dev_get_drvdata(dev);

	return clk_bulk_prepare_enable(ARRAY_SIZE(cdev->clks), cdev->clks);
}

static const struct dev_pm_ops csis_pm_ops = {
	RUNTIME_PM_OPS(csis_runtime_suspend, csis_runtime_resume, NULL)
};

static const struct of_device_id csis_of_match[] = {
	{ .compatible = "samsung,exynos7870-csis" },
	{ }
};
MODULE_DEVICE_TABLE(of, csis_of_match);

static struct platform_driver csis_driver = {
	.probe = csis_probe,
	.remove = csis_remove,
	.driver = {
		.name = CSIS_DRV_NAME,
		.of_match_table = csis_of_match,
		.pm = pm_ptr(&csis_pm_ops),
	},
};
module_platform_driver(csis_driver);

MODULE_DESCRIPTION("Samsung Exynos7870 MIPI CSI-2 receivers with DMA");
MODULE_LICENSE("GPL");
