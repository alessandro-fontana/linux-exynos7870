// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * linux/drivers/media/platform/samsung/s5p-mfc/s5p_mfc_dec.c
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 * Kamil Debski, <k.debski@samsung.com>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include "s5p_mfc_common.h"
#include "s5p_mfc_ctrl.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_dec.h"
#include "s5p_mfc_intr.h"
#include "s5p_mfc_opr.h"
#include "s5p_mfc_pm.h"

static const struct s5p_mfc_fmt formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_NV12MT_16X16,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6_BIT | MFC_V7_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12MT,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V5_BIT,
	},
	/*
	 * Contiguous NV12/NV21 first: it is what gralloc allocates, and
	 * clients pick the first format they support (Exynos7870).
	 */
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 1,
		.versions	= MFC_V10_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 1,
		.versions	= MFC_V10_BIT,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6PLUS_BITS,
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV21M,
		.codec_mode	= S5P_MFC_CODEC_NONE,
		.type		= MFC_FMT_RAW,
		.num_planes	= 2,
		.versions	= MFC_V6PLUS_BITS,
	},
	{
		.fourcc         = V4L2_PIX_FMT_YUV420M,
		.codec_mode     = S5P_MFC_CODEC_NONE,
		.type           = MFC_FMT_RAW,
		.num_planes     = 3,
		.versions       = MFC_V12_BIT,
	},
	{
		.fourcc         = V4L2_PIX_FMT_YVU420M,
		.codec_mode     = S5P_MFC_CODEC_NONE,
		.type           = MFC_FMT_RAW,
		.num_planes     = 3,
		.versions       = MFC_V12_BIT
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264,
		.codec_mode	= S5P_MFC_CODEC_H264_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H264_MVC,
		.codec_mode	= S5P_MFC_CODEC_H264_MVC_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_H263,
		.codec_mode	= S5P_MFC_CODEC_H263_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG1,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG2,
		.codec_mode	= S5P_MFC_CODEC_MPEG2_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_MPEG4,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_XVID,
		.codec_mode	= S5P_MFC_CODEC_MPEG4_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_G,
		.codec_mode	= S5P_MFC_CODEC_VC1_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VC1_ANNEX_L,
		.codec_mode	= S5P_MFC_CODEC_VC1RCV_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V5PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP8,
		.codec_mode	= S5P_MFC_CODEC_VP8_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V6PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
	{
		.fourcc		= V4L2_PIX_FMT_HEVC,
		.codec_mode	= S5P_FIMV_CODEC_HEVC_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V10PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION |
				  V4L2_FMT_FLAG_CONTINUOUS_BYTESTREAM,
	},
	{
		.fourcc		= V4L2_PIX_FMT_VP9,
		.codec_mode	= S5P_FIMV_CODEC_VP9_DEC,
		.type		= MFC_FMT_DEC,
		.num_planes	= 1,
		.versions	= MFC_V10PLUS_BITS,
		.flags		= V4L2_FMT_FLAG_DYN_RESOLUTION,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

/* Find selected format description */
static const struct s5p_mfc_fmt *find_format(struct v4l2_format *f, unsigned int t)
{
	unsigned int i;

	for (i = 0; i < NUM_FORMATS; i++) {
		if (formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    formats[i].type == t)
			return &formats[i];
	}
	return NULL;
}

static struct mfc_control controls[] = {
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H264 Display Delay",
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.minimum = 0,
		.maximum = 16383,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "H264 Display Delay Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.minimum = 0,
		.maximum = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Mpeg4 Loop Filter Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "Slice Interface Enable",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Minimum number of cap bufs",
		.minimum = 1,
		.maximum = 32,
		.step = 1,
		.default_value = 1,
		.is_volatile = 1,
	},
};

#define NUM_CTRLS ARRAY_SIZE(controls)

/* Largest stream header kept for a second SEQ_START (see reqbufs_capture()). */
#define MFC_DEC_HDR_MAX		SZ_64K

void s5p_mfc_dec_drop_header(struct s5p_mfc_ctx *ctx)
{
	if (ctx->hdr_buf.virt)
		s5p_mfc_release_generic_buf(ctx->dev, &ctx->hdr_buf);
	kfree(ctx->hdr);
	ctx->hdr = NULL;
	ctx->hdr_len = 0;
	ctx->reparse = false;
}

/* Check whether a context should be run on hardware */
static int s5p_mfc_ctx_ready(struct s5p_mfc_ctx *ctx)
{
	/* Context is to parse header */
	if (ctx->src_queue_cnt >= 1 && ctx->state == MFCINST_GOT_INST)
		return 1;
	/* Context is to decode a frame */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_RUNNING &&
	    ctx->dst_queue_cnt >= ctx->pb_count)
		return 1;
	/* Context is to return last frame */
	if (ctx->state == MFCINST_FINISHING &&
	    ctx->dst_queue_cnt >= ctx->pb_count)
		return 1;
	/* Context is to set buffers */
	if (ctx->src_queue_cnt >= 1 &&
	    ctx->state == MFCINST_HEAD_PARSED &&
	    ctx->capture_state == QUEUE_BUFS_MMAPED)
		return 1;
	/* Resolution change */
	if ((ctx->state == MFCINST_RES_CHANGE_INIT ||
		ctx->state == MFCINST_RES_CHANGE_FLUSH) &&
		ctx->dst_queue_cnt >= ctx->pb_count)
		return 1;
	if (ctx->state == MFCINST_RES_CHANGE_END &&
		ctx->src_queue_cnt >= 1)
		return 1;
	mfc_debug(2, "ctx is not ready\n");
	return 0;
}

static const struct s5p_mfc_codec_ops decoder_codec_ops = {
	.pre_seq_start		= NULL,
	.post_seq_start		= NULL,
	.pre_frame_start	= NULL,
	.post_frame_start	= NULL,
};

/* Query capabilities of the device */
static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);

	strscpy(cap->driver, S5P_MFC_NAME, sizeof(cap->driver));
	strscpy(cap->card, dev->vfd_dec->name, sizeof(cap->card));
	return 0;
}

/* Enumerate format */
static int vidioc_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
							bool out)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	int i, j = 0;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (out && formats[i].type != MFC_FMT_DEC)
			continue;
		else if (!out && formats[i].type != MFC_FMT_RAW)
			continue;
		else if ((dev->variant->version_bit & formats[i].versions) == 0)
			continue;

		if (j == f->index)
			break;
		++j;
	}
	if (i == ARRAY_SIZE(formats))
		return -EINVAL;
	f->pixelformat = formats[i].fourcc;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, false);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return vidioc_enum_fmt(file, f, true);
}

/* Get format */
static int vidioc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct v4l2_pix_format_mplane *pix_mp;

	mfc_debug_enter();
	pix_mp = &f->fmt.pix_mp;
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->state <= MFCINST_GOT_INST && ctx->src_queue_cnt == 0) {
		/*
		 * Nothing to parse yet (stateful API: the client sets up
		 * CAPTURE before queuing the header): report the format it
		 * asked for, the real one comes with V4L2_EVENT_SOURCE_CHANGE.
		 */
		unsigned int w = ctx->pre_width ? ctx->pre_width : 128;
		unsigned int h = ctx->pre_height ? ctx->pre_height : 128;

		pix_mp->width = w;
		pix_mp->height = h;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->pixelformat = ctx->dst_fmt->fourcc;
		pix_mp->num_planes = ctx->dst_fmt->num_planes;
		if (ctx->dst_contig) {
			pix_mp->pixelformat = ctx->dst_fmt->fourcc ==
					      V4L2_PIX_FMT_NV21M ?
					      V4L2_PIX_FMT_NV21 : V4L2_PIX_FMT_NV12;
			pix_mp->num_planes = 1;
		}
		pix_mp->plane_fmt[0].bytesperline = w;
		if (pix_mp->num_planes == 1) {
			pix_mp->plane_fmt[0].sizeimage = w * h * 3 / 2;
		} else {
			pix_mp->plane_fmt[0].sizeimage = w * h;
			pix_mp->plane_fmt[1].bytesperline = w;
			pix_mp->plane_fmt[1].sizeimage = w * h / 2;
		}
		return 0;
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    (ctx->state == MFCINST_GOT_INST || ctx->state ==
						MFCINST_RES_CHANGE_END)) {
		/* If the MFC is parsing the header,
		 * so wait until it is finished */
		s5p_mfc_wait_for_done_ctx(ctx, S5P_MFC_R2H_CMD_SEQ_DONE_RET,
									0);
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	    ctx->state >= MFCINST_HEAD_PARSED &&
	    ctx->state < MFCINST_ABORT) {
		/* This is run on CAPTURE (decode output) */
		/* Width and height are set to the dimensions
		   of the movie, the buffer is bigger and
		   further processing stages should crop to this
		   rectangle. */
		pix_mp->width = ctx->buf_width;
		pix_mp->height = ctx->buf_height;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->num_planes = ctx->dst_fmt->num_planes;
		/* Set pixelformat to the format in which MFC
		   outputs the decoded frame */
		pix_mp->pixelformat = ctx->dst_fmt->fourcc;
		pix_mp->plane_fmt[0].bytesperline = ctx->stride[0];
		pix_mp->plane_fmt[0].sizeimage = ctx->luma_size;
		pix_mp->plane_fmt[1].bytesperline = ctx->stride[1];
		pix_mp->plane_fmt[1].sizeimage = ctx->chroma_size;
		if (ctx->dst_contig) {
			/* Chroma right after the luma rows, as gralloc lays out NV12. */
			pix_mp->pixelformat = ctx->dst_fmt->fourcc ==
					      V4L2_PIX_FMT_NV21M ?
					      V4L2_PIX_FMT_NV21 : V4L2_PIX_FMT_NV12;
			pix_mp->num_planes = 1;
			pix_mp->height = ctx->contig_height;
			pix_mp->plane_fmt[0].sizeimage =
				ctx->stride[0] * ctx->contig_height * 3 / 2;
		}
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M) {
			pix_mp->plane_fmt[2].bytesperline = ctx->stride[2];
			pix_mp->plane_fmt[2].sizeimage = ctx->chroma_size_1;
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* This is run on OUTPUT
		   The buffer contains compressed image
		   so width and height have no meaning */
		pix_mp->width = 0;
		pix_mp->height = 0;
		pix_mp->field = V4L2_FIELD_NONE;
		pix_mp->plane_fmt[0].bytesperline = ctx->dec_src_buf_size;
		pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size;
		pix_mp->pixelformat = ctx->src_fmt->fourcc;
		pix_mp->num_planes = ctx->src_fmt->num_planes;
	} else {
		mfc_err("Format could not be read\n");
		mfc_debug(2, "%s-- with error\n", __func__);
		return -EINVAL;
	}
	mfc_debug_leave();
	return 0;
}

/* Try format */
static int vidioc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	const struct s5p_mfc_fmt *fmt;

	mfc_debug(2, "Type is %d\n", f->type);
	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		fmt = find_format(f, MFC_FMT_DEC);
		if (!fmt) {
			mfc_err("Unsupported format for source.\n");
			return -EINVAL;
		}
		if (fmt->codec_mode == S5P_FIMV_CODEC_NONE) {
			mfc_err("Unknown codec\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		fmt = find_format(f, MFC_FMT_RAW);
		if (!fmt) {
			mfc_err("Unsupported format for destination.\n");
			return -EINVAL;
		}
		if ((dev->variant->version_bit & fmt->versions) == 0) {
			mfc_err("Unsupported format by this MFC version.\n");
			return -EINVAL;
		}
	}

	return 0;
}

/* Set format */
static int vidioc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret = 0;
	struct v4l2_pix_format_mplane *pix_mp;
	const struct s5p_mfc_buf_size *buf_size = dev->variant->buf_size;

	mfc_debug_enter();
	ret = vidioc_try_fmt(file, priv, f);
	pix_mp = &f->fmt.pix_mp;
	if (ret)
		return ret;
	/* Only the queue being configured must be idle (stateful API). */
	if ((f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE &&
	     vb2_is_busy(&ctx->vq_dst)) ||
	    (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	     vb2_is_busy(&ctx->vq_src))) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy\n", __func__);
		ret = -EBUSY;
		goto out;
	}
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* dst_fmt is validated by call to vidioc_try_fmt */
		ctx->dst_fmt = find_format(f, MFC_FMT_RAW);
		ctx->dst_contig = ctx->dst_fmt->num_planes == 1;
		if (ctx->dst_contig) {
			/* Internally the MFC writes NV12M/NV21M planes. */
			struct v4l2_format f2 = *f;

			f2.fmt.pix_mp.pixelformat =
				ctx->dst_fmt->fourcc == V4L2_PIX_FMT_NV21 ?
				V4L2_PIX_FMT_NV21M : V4L2_PIX_FMT_NV12M;
			ctx->dst_fmt = find_format(&f2, MFC_FMT_RAW);
		}
		if (ctx->state < MFCINST_HEAD_PARSED) {
			ctx->pre_width = pix_mp->width;
			ctx->pre_height = pix_mp->height;
		} else if (ctx->state == MFCINST_HEAD_PARSED) {
			/* The layout depends on the format: recompute it. */
			s5p_mfc_hw_call(dev->mfc_ops, dec_calc_dpb_size, ctx);
		}
		ret = 0;
		goto out;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* src_fmt is validated by call to vidioc_try_fmt */
		ctx->src_fmt = find_format(f, MFC_FMT_DEC);
		ctx->codec_mode = ctx->src_fmt->codec_mode;
		mfc_debug(2, "The codec number is: %d\n", ctx->codec_mode);
		pix_mp->height = 0;
		pix_mp->width = 0;
		if (pix_mp->plane_fmt[0].sizeimage == 0)
			pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size =
								DEF_CPB_SIZE;
		else if (pix_mp->plane_fmt[0].sizeimage > buf_size->cpb)
			pix_mp->plane_fmt[0].sizeimage = ctx->dec_src_buf_size =
								buf_size->cpb;
		else
			ctx->dec_src_buf_size = pix_mp->plane_fmt[0].sizeimage;
		pix_mp->plane_fmt[0].bytesperline = 0;
		ctx->state = MFCINST_INIT;
		ret = 0;
		goto out;
	} else {
		mfc_err("Wrong type error for S_FMT : %d", f->type);
		ret = -EINVAL;
		goto out;
	}

out:
	mfc_debug_leave();
	return ret;
}

static int reqbufs_output(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on(dev);

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret)
			goto out;
		ctx->src_bufs_cnt = 0;
		ctx->output_state = QUEUE_FREE;
	} else if (ctx->output_state == QUEUE_FREE) {
		/* Can only request buffers when we have a valid format set. */
		WARN_ON(ctx->src_bufs_cnt != 0);
		if (ctx->state != MFCINST_INIT) {
			mfc_err("Reqbufs called in an invalid state\n");
			ret = -EINVAL;
			goto out;
		}

		mfc_debug(2, "Allocating %d buffers for OUTPUT queue\n",
				reqbufs->count);
		ret = vb2_reqbufs(&ctx->vq_src, reqbufs);
		if (ret)
			goto out;

		ret = s5p_mfc_open_mfc_inst(dev, ctx);
		if (ret) {
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_src, reqbufs);
			goto out;
		}

		ctx->output_state = QUEUE_BUFS_REQUESTED;
	} else {
		mfc_err("Buffers have already been requested\n");
		ret = -EINVAL;
	}
out:
	s5p_mfc_clock_off(dev);
	if (ret)
		mfc_err("Failed allocating buffers for OUTPUT queue\n");
	return ret;
}

static int reqbufs_capture(struct s5p_mfc_dev *dev, struct s5p_mfc_ctx *ctx,
				struct v4l2_requestbuffers *reqbufs)
{
	int ret = 0;

	s5p_mfc_clock_on(dev);

	if (reqbufs->count == 0) {
		mfc_debug(2, "Freeing buffers\n");
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;
		if (!ctx->dst_placeholder)
			s5p_mfc_hw_call(dev->mfc_ops, release_codec_buffers,
					ctx);
		ctx->dst_bufs_cnt = 0;
		ctx->dst_placeholder = false;
		ctx->capture_state = QUEUE_FREE;
		/*
		 * A new set of CAPTURE buffers while decoding, e.g. when the
		 * client moves to another surface and the pictures of the old
		 * one never come back: go back to setting up the DPB, so that
		 * the MFC gets the new addresses once they are all queued.
		 * The DPB was flushed by the streamoff, and dpb_flush_flag
		 * keeps the first OUTPUT buffer, a picture and not the
		 * header, from being dropped by the buffer setup.
		 */
		if (ctx->state == MFCINST_RUNNING) {
			/*
			 * The MFC only takes a new DPB after a new SEQ_START
			 * (INIT_BUFFERS alone fails with error 0xd), and the
			 * client does not send the header again: parse the
			 * copy kept from the first one.
			 */
			if (ctx->hdr_buf.virt)
				s5p_mfc_release_generic_buf(dev, &ctx->hdr_buf);
			ctx->reparse = false;
			if (ctx->hdr) {
				ctx->hdr_buf.size = ALIGN(ctx->dec_src_buf_size, 512);
				if (!s5p_mfc_alloc_generic_buf(dev, BANK_L_CTX,
							       &ctx->hdr_buf)) {
					memcpy(ctx->hdr_buf.virt, ctx->hdr,
					       ctx->hdr_len);
					ctx->reparse = true;
				}
			}
			if (!ctx->reparse)
				mfc_err("No stream header to set up a new DPB\n");
			ctx->state = MFCINST_HEAD_PARSED;
			ctx->dpb_flush_flag = 1;
		}
	} else if (ctx->capture_state == QUEUE_FREE &&
		   ctx->state < MFCINST_HEAD_PARSED) {
		/*
		 * Stateful API: CAPTURE buffers before the header is known.
		 * They only carry a possible early EOS; the MFC never sees
		 * them, the client reallocates on V4L2_EVENT_SOURCE_CHANGE.
		 */
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;
		ctx->dst_placeholder = true;
		ctx->capture_state = QUEUE_BUFS_REQUESTED;
	} else if (ctx->capture_state == QUEUE_FREE) {
		WARN_ON(ctx->dst_bufs_cnt != 0);
		mfc_debug(2, "Allocating %d buffers for CAPTURE queue\n",
				reqbufs->count);
		ret = vb2_reqbufs(&ctx->vq_dst, reqbufs);
		if (ret)
			goto out;

		ctx->capture_state = QUEUE_BUFS_REQUESTED;
		ctx->total_dpb_count = reqbufs->count;

		ret = s5p_mfc_hw_call(dev->mfc_ops, alloc_codec_buffers, ctx);
		if (ret) {
			mfc_err("Failed to allocate decoding buffers\n");
			reqbufs->count = 0;
			vb2_reqbufs(&ctx->vq_dst, reqbufs);
			ret = -ENOMEM;
			ctx->capture_state = QUEUE_FREE;
			goto out;
		}

		/*
		 * DMABUF buffers get their addresses when first queued
		 * (s5p_mfc_buf_init): the DPB is set up once all of them
		 * have been, asynchronously.
		 */
		if (reqbufs->memory == V4L2_MEMORY_DMABUF)
			goto out;

		WARN_ON(ctx->dst_bufs_cnt != ctx->total_dpb_count);
		ctx->capture_state = QUEUE_BUFS_MMAPED;

		if (s5p_mfc_ctx_ready(ctx))
			set_work_bit_irqsave(ctx);
		s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
		s5p_mfc_wait_for_done_ctx(ctx, S5P_MFC_R2H_CMD_INIT_BUFFERS_RET,
					  0);
	} else {
		mfc_err("Buffers have already been requested\n");
		ret = -EINVAL;
	}
out:
	s5p_mfc_clock_off(dev);
	if (ret)
		mfc_err("Failed allocating buffers for CAPTURE queue\n");
	return ret;
}

/* Request buffers */
static int vidioc_reqbufs(struct file *file, void *priv,
					  struct v4l2_requestbuffers *reqbufs)
{
	struct s5p_mfc_dev *dev = video_drvdata(file);
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (reqbufs->memory != V4L2_MEMORY_MMAP &&
	    reqbufs->memory != V4L2_MEMORY_DMABUF) {
		mfc_debug(2, "Only MMAP and DMABUF are supported\n");
		return -EINVAL;
	}

	if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		return reqbufs_output(dev, ctx, reqbufs);
	} else if (reqbufs->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		return reqbufs_capture(dev, ctx, reqbufs);
	} else {
		mfc_err("Invalid type requested\n");
		return -EINVAL;
	}
}

/* Query buffer */
static int vidioc_querybuf(struct file *file, void *priv,
						   struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret;
	int i;

	if (buf->memory != V4L2_MEMORY_MMAP &&
	    buf->memory != V4L2_MEMORY_DMABUF) {
		mfc_err("Only MMAP and DMABUF buffers can be used\n");
		return -EINVAL;
	}
	mfc_debug(2, "State: %d, buf->type: %d\n", ctx->state, buf->type);
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_src, buf);
	} else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		ret = vb2_querybuf(&ctx->vq_dst, buf);
		if (buf->memory == V4L2_MEMORY_MMAP)
			for (i = 0; i < buf->length; i++)
				buf->m.planes[i].m.mem_offset +=
							DST_QUEUE_OFF_BASE;
	} else {
		mfc_err("vidioc_querybuf called in an inappropriate state\n");
		ret = -EINVAL;
	}
	mfc_debug_leave();
	return ret;
}

/* Queue a buffer */
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (ctx->state == MFCINST_ERROR) {
		mfc_err("Call on QBUF after unrecoverable error\n");
		return -EIO;
	}
	if (buf->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_qbuf(&ctx->vq_src, NULL, buf);
	else if (buf->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_qbuf(&ctx->vq_dst, NULL, buf);
	return -EINVAL;
}

/* Dequeue a buffer */
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	const struct v4l2_event ev = {
		.type = V4L2_EVENT_EOS
	};
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret;

	if (ctx->state == MFCINST_ERROR) {
		mfc_err_limited("Call on DQBUF after unrecoverable error\n");
		return -EIO;
	}

	switch (buf->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return vb2_dqbuf(&ctx->vq_src, buf, file->f_flags & O_NONBLOCK);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		ret = vb2_dqbuf(&ctx->vq_dst, buf, file->f_flags & O_NONBLOCK);
		if (ret)
			return ret;

		if (ctx->state == MFCINST_FINISHED &&
		    (ctx->dst_bufs[buf->index].flags & MFC_BUF_FLAG_EOS))
			v4l2_event_queue_fh(&ctx->fh, &ev);
		return 0;
	default:
		return -EINVAL;
	}
}

/* Export DMA buffer */
static int vidioc_expbuf(struct file *file, void *priv,
	struct v4l2_exportbuffer *eb)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (eb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_expbuf(&ctx->vq_src, eb);
	if (eb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_expbuf(&ctx->vq_dst, eb);
	return -EINVAL;
}

/* Stream on */
static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	int ret = -EINVAL;

	mfc_debug_enter();
	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		ret = vb2_streamon(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		ret = vb2_streamon(&ctx->vq_dst, type);
	mfc_debug_leave();
	return ret;
}

/* Stream off, which equals to a pause */
static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return vb2_streamoff(&ctx->vq_src, type);
	else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return vb2_streamoff(&ctx->vq_dst, type);
	return -EINVAL;
}

/* Set controls - v4l2 control framework */
static int s5p_mfc_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY:
		ctx->display_delay = ctrl->val;
		break;
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE:
	case V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE:
		ctx->display_delay_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_MPEG4_DEBLOCK_FILTER:
		ctx->loop_filter_mpeg4 = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_DECODER_SLICE_INTERFACE:
		ctx->slice_interface = ctrl->val;
		break;
	default:
		mfc_err("Invalid control 0x%08x\n", ctrl->id);
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_dec_g_v_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5p_mfc_ctx *ctx = ctrl_to_ctx(ctrl);
	struct s5p_mfc_dev *dev = ctx->dev;

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->val = ctx->pb_count;
			break;
		} else if (ctx->state <= MFCINST_GOT_INST &&
			   ctx->src_queue_cnt == 0) {
			/* Nothing to parse yet: one placeholder buffer. */
			ctrl->val = 1;
			break;
		} else if (ctx->state != MFCINST_INIT &&
				ctx->state != MFCINST_RES_CHANGE_END) {
			v4l2_err(&dev->v4l2_dev, "Decoding not initialised\n");
			return -EINVAL;
		}
		/* Should wait for the header to be parsed */
		s5p_mfc_wait_for_done_ctx(ctx,
				S5P_MFC_R2H_CMD_SEQ_DONE_RET, 0);
		if (ctx->state >= MFCINST_HEAD_PARSED &&
		    ctx->state < MFCINST_ABORT) {
			ctrl->val = ctx->pb_count;
		} else {
			v4l2_err(&dev->v4l2_dev, "Decoding not initialised\n");
			return -EINVAL;
		}
		break;
	}
	return 0;
}


static const struct v4l2_ctrl_ops s5p_mfc_dec_ctrl_ops = {
	.s_ctrl = s5p_mfc_dec_s_ctrl,
	.g_volatile_ctrl = s5p_mfc_dec_g_v_ctrl,
};

/* Get compose information */
static int vidioc_g_selection(struct file *file, void *priv,
			      struct v4l2_selection *s)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;
	u32 left, right, top, bottom;
	u32 width, height;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (ctx->state != MFCINST_HEAD_PARSED &&
	    ctx->state != MFCINST_RUNNING &&
	    ctx->state != MFCINST_FINISHING &&
	    ctx->state != MFCINST_FINISHED) {
		mfc_err("Can not get compose information\n");
		return -EINVAL;
	}
	if (ctx->src_fmt->fourcc == V4L2_PIX_FMT_H264 ||
	    ctx->src_fmt->fourcc == V4L2_PIX_FMT_HEVC) {
		left = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_h, ctx);
		right = left >> S5P_FIMV_SHARED_CROP_RIGHT_SHIFT;
		left = left & S5P_FIMV_SHARED_CROP_LEFT_MASK;
		top = s5p_mfc_hw_call(dev->mfc_ops, get_crop_info_v, ctx);
		bottom = top >> S5P_FIMV_SHARED_CROP_BOTTOM_SHIFT;
		top = top & S5P_FIMV_SHARED_CROP_TOP_MASK;
		width = ctx->img_width - left - right;
		height = ctx->img_height - top - bottom;
		mfc_debug(2, "Composing info [h264]: l=%d t=%d w=%d h=%d (r=%d b=%d fw=%d fh=%d\n",
			  left, top, s->r.width, s->r.height, right, bottom,
			  ctx->buf_width, ctx->buf_height);
	} else {
		left = 0;
		top = 0;
		width = ctx->img_width;
		height = ctx->img_height;
		mfc_debug(2, "Composing info: w=%d h=%d fw=%d fh=%d\n",
			  s->r.width, s->r.height, ctx->buf_width,
			  ctx->buf_height);
	}

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = left;
		s->r.top = top;
		s->r.width = width;
		s->r.height = height;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_try_decoder_cmd(struct file *file, void *priv,
				  struct v4l2_decoder_cmd *cmd)
{
	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
	case V4L2_DEC_CMD_START:
		return cmd->flags ? -EINVAL : 0;
	default:
		return -EINVAL;
	}
}

static int vidioc_decoder_cmd(struct file *file, void *priv,
			      struct v4l2_decoder_cmd *cmd)
{
	struct s5p_mfc_ctx *ctx = file_to_ctx(file);
	struct s5p_mfc_dev *dev = ctx->dev;
	struct s5p_mfc_buf *buf;
	unsigned long flags;

	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		if (cmd->flags != 0)
			return -EINVAL;

		if (!vb2_is_streaming(&ctx->vq_src))
			return -EINVAL;

		spin_lock_irqsave(&dev->irqlock, flags);
		if (list_empty(&ctx->src_queue)) {
			mfc_err("EOS: empty src queue, entering finishing state");
			ctx->state = MFCINST_FINISHING;
			if (s5p_mfc_ctx_ready(ctx))
				set_work_bit_irqsave(ctx);
			spin_unlock_irqrestore(&dev->irqlock, flags);
			s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
		} else {
			mfc_err("EOS: marking last buffer of stream");
			buf = list_entry(ctx->src_queue.prev,
						struct s5p_mfc_buf, list);
			if (buf->flags & MFC_BUF_FLAG_USED)
				ctx->state = MFCINST_FINISHING;
			else
				buf->flags |= MFC_BUF_FLAG_EOS;
			spin_unlock_irqrestore(&dev->irqlock, flags);
		}
		break;
	case V4L2_DEC_CMD_START:
		/*
		 * Resume after a drain (V4L2 stateful decoder API): the
		 * instance was left finished once the last frame came out.
		 */
		if (cmd->flags != 0)
			return -EINVAL;

		spin_lock_irqsave(&dev->irqlock, flags);
		if (ctx->state == MFCINST_FINISHED ||
		    ctx->state == MFCINST_FINISHING)
			ctx->state = MFCINST_RUNNING;
		spin_unlock_irqrestore(&dev->irqlock, flags);
		vb2_clear_last_buffer_dequeued(&ctx->vq_dst);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				const struct  v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 2, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return -EINVAL;
	}
}


/* v4l2_ioctl_ops */
static const struct v4l2_ioctl_ops s5p_mfc_dec_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_cap_mplane = vidioc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vidioc_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vidioc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vidioc_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vidioc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vidioc_s_fmt,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_expbuf = vidioc_expbuf,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_g_selection = vidioc_g_selection,
	.vidioc_decoder_cmd = vidioc_decoder_cmd,
	.vidioc_try_decoder_cmd = vidioc_try_decoder_cmd,
	.vidioc_subscribe_event = vidioc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int s5p_mfc_queue_setup(struct vb2_queue *vq,
			unsigned int *buf_count,
			unsigned int *plane_count, unsigned int psize[],
			struct device *alloc_devs[])
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;
	const struct v4l2_format_info *format;

	/* Video output for decoding (source)
	 * this can be set after getting an instance */
	if (ctx->state == MFCINST_INIT &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* A single plane is required for input */
		*plane_count = 1;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	/* Video capture for decoding (destination)
	 * this can be set after the header was parsed */
	} else if (ctx->state < MFCINST_HEAD_PARSED &&
		   vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Placeholder CAPTURE buffers, see reqbufs_capture(). */
		unsigned int w = ctx->pre_width ? ctx->pre_width : 128;
		unsigned int h = ctx->pre_height ? ctx->pre_height : 128;

		*plane_count = ctx->dst_contig ? 1 : 2;
		if (*buf_count < 1)
			*buf_count = 1;
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
		if (ctx->dst_contig) {
			psize[0] = w * h * 3 / 2;
		} else {
			psize[0] = w * h;
			psize[1] = w * h / 2;
		}
		alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
		alloc_devs[1] = ctx->dev->mem_dev[BANK_L_CTX];
		return 0;
	} else if (ctx->state == MFCINST_HEAD_PARSED &&
		   vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* Output plane count is 2 - one for Y and one for CbCr */
		format = v4l2_format_info(ctx->dst_fmt->fourcc);
		if (!format) {
			mfc_err("invalid format\n");
			return -EINVAL;
		}
		*plane_count = ctx->dst_contig ? 1 : format->comp_planes;

		/* Setup buffer count */
		if (*buf_count < ctx->pb_count)
			*buf_count = ctx->pb_count;
		if (vq->memory == V4L2_MEMORY_DMABUF) {
			/*
			 * The DPB is given to the MFC once every buffer has
			 * been queued (their addresses come with the dmabufs),
			 * so the client must hold all of them at once: ask for
			 * the DPB plus the four the client adds to it, which
			 * fits in what an Android surface grants.
			 */
			*buf_count = clamp_t(unsigned int, *buf_count,
					     ctx->pb_count, ctx->pb_count + 4);
		} else if (*buf_count > ctx->pb_count + MFC_MAX_EXTRA_DPB) {
			*buf_count = ctx->pb_count + MFC_MAX_EXTRA_DPB;
		}
		if (*buf_count > MFC_MAX_BUFFERS)
			*buf_count = MFC_MAX_BUFFERS;
	} else {
		mfc_err("State seems invalid. State = %d, vq->type = %d\n",
							ctx->state, vq->type);
		return -EINVAL;
	}
	mfc_debug(2, "Buffer count=%d, plane count=%d\n",
						*buf_count, *plane_count);
	if (ctx->state == MFCINST_HEAD_PARSED &&
	    vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		psize[0] = ctx->luma_size;
		psize[1] = ctx->chroma_size;
		if (ctx->dst_contig)
			psize[0] = ctx->stride[0] * ctx->contig_height * 3 / 2;
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M)
			psize[2] = ctx->chroma_size_1;
		if (IS_MFCV6_PLUS(dev))
			alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
		else
			alloc_devs[0] = ctx->dev->mem_dev[BANK_R_CTX];
		alloc_devs[1] = ctx->dev->mem_dev[BANK_L_CTX];
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M)
			alloc_devs[2] = ctx->dev->mem_dev[BANK_L_CTX];
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE && ctx->state == MFCINST_INIT) {
		psize[0] = ctx->dec_src_buf_size;
		alloc_devs[0] = ctx->dev->mem_dev[BANK_L_CTX];
	} else {
		mfc_err("This video node is dedicated to decoding. Decoding not initialized\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	unsigned int i;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		if (ctx->dst_placeholder) {
			/* Never given to the MFC, see reqbufs_capture(). */
			ctx->dst_bufs[vb->index].b = vbuf;
			return 0;
		}
		if (ctx->capture_state == QUEUE_BUFS_MMAPED)
			return 0;
		if (ctx->dst_contig) {
			dma_addr_t luma = vb2_dma_contig_plane_dma_addr(vb, 0);
			unsigned long need = ctx->stride[0] * ctx->contig_height *
					     3 / 2;

			if (!luma || vb2_plane_size(vb, 0) < need) {
				mfc_err("CAPTURE buffer too small: %lu < %lu\n",
					vb2_plane_size(vb, 0), need);
				return -EINVAL;
			}
			i = vb->index;
			ctx->dst_bufs[i].b = vbuf;
			ctx->dst_bufs[i].cookie.raw.luma = luma;
			ctx->dst_bufs[i].cookie.raw.chroma =
				luma + ctx->stride[0] * ctx->contig_height;
			goto registered;
		}
		for (i = 0; i < ctx->dst_fmt->num_planes; i++) {
			if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, i)))) {
				mfc_err("Plane mem not allocated\n");
				return -EINVAL;
			}
		}
		if (vb2_plane_size(vb, 0) < ctx->luma_size ||
			vb2_plane_size(vb, 1) < ctx->chroma_size) {
			mfc_err("Plane buffer (CAPTURE) is too small\n");
			return -EINVAL;
		}
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M) {
			if (vb2_plane_size(vb, 2) < ctx->chroma_size_1) {
				mfc_err("Plane buffer (CAPTURE) is too small\n");
				return -EINVAL;
			}
		}
		i = vb->index;
		ctx->dst_bufs[i].b = vbuf;
		ctx->dst_bufs[i].cookie.raw.luma =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->dst_bufs[i].cookie.raw.chroma =
					vb2_dma_contig_plane_dma_addr(vb, 1);
		if (ctx->dst_fmt->fourcc == V4L2_PIX_FMT_YUV420M || ctx->dst_fmt->fourcc ==
				V4L2_PIX_FMT_YVU420M) {
			ctx->dst_bufs[i].cookie.raw.chroma_1 =
					vb2_dma_contig_plane_dma_addr(vb, 2);
		}
registered:
		ctx->dst_bufs_cnt++;
		/*
		 * DMABUF: the last buffer of the set has its address, the
		 * DPB can be given to the MFC (see reqbufs_capture()).
		 */
		if (vq->memory == V4L2_MEMORY_DMABUF &&
		    ctx->capture_state == QUEUE_BUFS_REQUESTED &&
		    ctx->dst_bufs_cnt == ctx->total_dpb_count)
			ctx->capture_state = QUEUE_BUFS_MMAPED;
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		if (IS_ERR_OR_NULL(ERR_PTR(
					vb2_dma_contig_plane_dma_addr(vb, 0)))) {
			mfc_err("Plane memory not allocated\n");
			return -EINVAL;
		}
		if (vb2_plane_size(vb, 0) < ctx->dec_src_buf_size) {
			mfc_err("Plane buffer (OUTPUT) is too small\n");
			return -EINVAL;
		}

		i = vb->index;
		ctx->src_bufs[i].b = vbuf;
		ctx->src_bufs[i].cookie.stream =
					vb2_dma_contig_plane_dma_addr(vb, 0);
		ctx->src_bufs_cnt++;
	} else {
		mfc_err("s5p_mfc_buf_init: unknown queue type\n");
		return -EINVAL;
	}
	return 0;
}

static int s5p_mfc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;

	v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (ctx->state == MFCINST_FINISHING ||
		ctx->state == MFCINST_FINISHED)
		ctx->state = MFCINST_RUNNING;
	/* If context is ready then dev = work->data;schedule it to run */
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
	return 0;
}

static void s5p_mfc_stop_streaming(struct vb2_queue *q)
{
	unsigned long flags;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(q);
	struct s5p_mfc_dev *dev = ctx->dev;
	int aborted = 0;

	spin_lock_irqsave(&dev->irqlock, flags);
	if ((ctx->state == MFCINST_FINISHING ||
		ctx->state ==  MFCINST_RUNNING) &&
		dev->curr_ctx == ctx->num && dev->hw_lock) {
		ctx->state = MFCINST_ABORT;
		spin_unlock_irqrestore(&dev->irqlock, flags);
		s5p_mfc_wait_for_done_ctx(ctx,
					S5P_MFC_R2H_CMD_FRAME_DONE_RET, 0);
		aborted = 1;
		spin_lock_irqsave(&dev->irqlock, flags);
	}
	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->dst_queue, &ctx->vq_dst);
		INIT_LIST_HEAD(&ctx->dst_queue);
		ctx->dst_queue_cnt = 0;
		/* A DPB flush only follows a streamoff while decoding. */
		if (ctx->state == MFCINST_RUNNING)
			ctx->dpb_flush_flag = 1;
		ctx->dec_dst_flag = 0;
		if (IS_MFCV6_PLUS(dev) && (ctx->state == MFCINST_RUNNING)) {
			ctx->state = MFCINST_FLUSH;
			set_work_bit_irqsave(ctx);
			s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
			spin_unlock_irqrestore(&dev->irqlock, flags);
			if (s5p_mfc_wait_for_done_ctx(ctx,
				S5P_MFC_R2H_CMD_DPB_FLUSH_RET, 0))
				mfc_err("Err flushing buffers\n");
			spin_lock_irqsave(&dev->irqlock, flags);
		}
	} else if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		s5p_mfc_cleanup_queue(&ctx->src_queue, &ctx->vq_src);
		INIT_LIST_HEAD(&ctx->src_queue);
		ctx->src_queue_cnt = 0;
	}
	if (aborted)
		ctx->state = MFCINST_RUNNING;
	spin_unlock_irqrestore(&dev->irqlock, flags);
}


static void s5p_mfc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct s5p_mfc_ctx *ctx = vb2_get_drv_priv(vq);
	struct s5p_mfc_dev *dev = ctx->dev;
	unsigned long flags;
	struct s5p_mfc_buf *mfc_buf;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* The first buffer is the header: keep it, see reqbufs_capture(). */
		if (ctx->state == MFCINST_GOT_INST && !ctx->hdr) {
			void *vaddr = vb2_plane_vaddr(vb, 0);
			unsigned int len = min_t(unsigned int, MFC_DEC_HDR_MAX,
					vb2_get_plane_payload(vb, 0));

			if (vaddr && len) {
				ctx->hdr = kmemdup(vaddr, len, GFP_KERNEL);
				if (ctx->hdr)
					ctx->hdr_len = len;
			}
		}
		mfc_buf = &ctx->src_bufs[vb->index];
		mfc_buf->flags &= ~MFC_BUF_FLAG_USED;
		spin_lock_irqsave(&dev->irqlock, flags);
		list_add_tail(&mfc_buf->list, &ctx->src_queue);
		ctx->src_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mfc_buf = &ctx->dst_bufs[vb->index];
		mfc_buf->flags &= ~MFC_BUF_FLAG_USED;
		/* Mark destination as available for use by MFC */
		spin_lock_irqsave(&dev->irqlock, flags);
		set_bit(vb->index, &ctx->dec_dst_flag);
		list_add_tail(&mfc_buf->list, &ctx->dst_queue);
		ctx->dst_queue_cnt++;
		spin_unlock_irqrestore(&dev->irqlock, flags);
	} else {
		mfc_err("Unsupported buffer type (%d)\n", vq->type);
	}
	if (s5p_mfc_ctx_ready(ctx))
		set_work_bit_irqsave(ctx);
	s5p_mfc_hw_call(dev->mfc_ops, try_run, dev);
}

static const struct vb2_ops s5p_mfc_dec_qops = {
	.queue_setup		= s5p_mfc_queue_setup,
	.buf_init		= s5p_mfc_buf_init,
	.start_streaming	= s5p_mfc_start_streaming,
	.stop_streaming		= s5p_mfc_stop_streaming,
	.buf_queue		= s5p_mfc_buf_queue,
};

const struct s5p_mfc_codec_ops *get_dec_codec_ops(void)
{
	return &decoder_codec_ops;
}

const struct vb2_ops *get_dec_queue_ops(void)
{
	return &s5p_mfc_dec_qops;
}

const struct v4l2_ioctl_ops *get_dec_v4l2_ioctl_ops(void)
{
	return &s5p_mfc_dec_ioctl_ops;
}

#define IS_MFC51_PRIV(x) ((V4L2_CTRL_ID2WHICH(x) == V4L2_CTRL_CLASS_CODEC) \
						&& V4L2_CTRL_DRIVER_PRIV(x))

int s5p_mfc_dec_ctrls_setup(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_ctrl_config cfg;
	int i;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, NUM_CTRLS);
	if (ctx->ctrl_handler.error) {
		mfc_err("v4l2_ctrl_handler_init failed\n");
		return ctx->ctrl_handler.error;
	}

	for (i = 0; i < NUM_CTRLS; i++) {
		if (IS_MFC51_PRIV(controls[i].id)) {
			memset(&cfg, 0, sizeof(struct v4l2_ctrl_config));
			cfg.ops = &s5p_mfc_dec_ctrl_ops;
			cfg.id = controls[i].id;
			cfg.min = controls[i].minimum;
			cfg.max = controls[i].maximum;
			cfg.def = controls[i].default_value;
			cfg.name = controls[i].name;
			cfg.type = controls[i].type;

			cfg.step = controls[i].step;
			cfg.menu_skip_mask = 0;

			ctx->ctrls[i] = v4l2_ctrl_new_custom(&ctx->ctrl_handler,
					&cfg, NULL);
		} else {
			ctx->ctrls[i] = v4l2_ctrl_new_std(&ctx->ctrl_handler,
					&s5p_mfc_dec_ctrl_ops,
					controls[i].id, controls[i].minimum,
					controls[i].maximum, controls[i].step,
					controls[i].default_value);
		}
		if (ctx->ctrl_handler.error) {
			mfc_err("Adding control (%d) failed\n", i);
			return ctx->ctrl_handler.error;
		}
		if (controls[i].is_volatile && ctx->ctrls[i])
			ctx->ctrls[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
	}
	return 0;
}

void s5p_mfc_dec_ctrls_delete(struct s5p_mfc_ctx *ctx)
{
	int i;

	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	for (i = 0; i < NUM_CTRLS; i++)
		ctx->ctrls[i] = NULL;
}

void s5p_mfc_dec_init(struct s5p_mfc_ctx *ctx)
{
	struct v4l2_format f;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	ctx->src_fmt = find_format(&f, MFC_FMT_DEC);
	if (IS_MFCV8_PLUS(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
	else if (IS_MFCV6_PLUS(ctx->dev))
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT_16X16;
	else
		f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12MT;
	ctx->dst_fmt = find_format(&f, MFC_FMT_RAW);
	mfc_debug(2, "Default src_fmt is %p, dest_fmt is %p\n",
			ctx->src_fmt, ctx->dst_fmt);
}

