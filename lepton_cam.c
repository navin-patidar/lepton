/*
 *      Copyright (c) 2016 Intel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the BSD Licence, GNU General Public License
 * as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/font.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/spi/spi.h>
#include <media/videobuf2-vmalloc.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>
#include <media/v4l2-common.h>
#include "palettes.h"

#define WIDTH 80
#define HEIGHT 60

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
#define FPS 27;

static unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

struct lepton_fmt {
	char  *name;
	u32   fourcc;          /* v4l2 format id */
	u8    depth;
};
static struct lepton_fmt format = {
		.name     = "RGB24 (LE)",
		.fourcc   = V4L2_PIX_FMT_RGB24,
		.depth    = 24,
};

/* buffer for one video frame */
struct lepton_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer	vb;
	struct list_head	list;
	struct lepton_fmt        *fmt;
};

struct lepton_queue {
	struct list_head       active;

	/* thread for generating video stream*/
	struct task_struct         *kthread;
	wait_queue_head_t          wq;
	/* Counters to control fps rate */
	int                        frame;
	int                        ini_jiffies;
};


struct lepton_data {
	struct list_head           vivi_devlist;
	struct v4l2_device 	   v4l2_dev;
	struct v4l2_ctrl_handler   ctrl_handler;
	struct video_device	   vdev;
	struct lepton_fmt *fmt;

	/* controls */
	struct v4l2_ctrl	   *brightness;
	struct v4l2_ctrl	   *contrast;
	struct v4l2_ctrl	   *saturation;
	struct v4l2_ctrl	   *hue;
	struct {
		/* autogain/gain cluster */
		struct v4l2_ctrl	   *autogain;
		struct v4l2_ctrl	   *gain;
	};
	struct v4l2_ctrl	   *volume;
	struct v4l2_ctrl	   *alpha;
	struct v4l2_ctrl	   *button;
	struct v4l2_ctrl	   *boolean;
	struct v4l2_ctrl	   *int32;
	struct v4l2_ctrl	   *int64;
	struct v4l2_ctrl	   *menu;
	struct v4l2_ctrl	   *string;
	struct v4l2_ctrl	   *bitmask;
	struct v4l2_ctrl	   *int_menu;
	struct spi_device *spi;
	spinlock_t                 slock;
	struct mutex		   mutex;

	struct lepton_queue       vidq;
	u8 result[PACKET_SIZE*PACKETS_PER_FRAME];

	/* Several counters */
	unsigned 		   ms;
	unsigned long              jiffies;
	unsigned		   button_pressed;

	int			   mv_count;	/* Controls bars movement */

	/* Input Number */
	int			   input;

	/* video capture */
	unsigned int               width, height;
	struct vb2_queue	   vb_vidq;
	unsigned int		   field_count;
	struct v4l2_fract    timeperframe;

//	u8			   bars[9][3];
//	u8			   line[MAX_WIDTH * 8];
	unsigned int		   pixelsize;
	u8			   alpha_component;
};

static const struct v4l2_file_operations lepton_fops = {
	.owner		= THIS_MODULE,
	.open           = v4l2_fh_open,
	.release        = vb2_fop_release,
	.read           = vb2_fop_read,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2, /* V4L2 ioctl handler */
	.mmap           = vb2_fop_mmap,
};

static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct lepton_data *dev = video_drvdata(file);

	strcpy(cap->driver, "lepton");
	strcpy(cap->card, "lepton");
	snprintf(cap->bus_info, sizeof(cap->bus_info),
			"platform:%s", dev->v4l2_dev.name);
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
//			   | V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{

	if (f->index > 0)
		return -EINVAL;

	strlcpy(f->description, format.name, sizeof(f->description));
	f->pixelformat = format.fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct lepton_data *dev = video_drvdata(file);

	f->fmt.pix.width        = dev->width;
	f->fmt.pix.height       = dev->height;
	f->fmt.pix.field        = V4L2_FIELD_INTERLACED;
	f->fmt.pix.pixelformat  = dev->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * dev->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct lepton_data *dev = video_drvdata(file);
	f->fmt.pix.width        = dev->width;
	f->fmt.pix.height       = dev->height;
	f->fmt.pix.field        = V4L2_FIELD_INTERLACED;
	f->fmt.pix.pixelformat  = dev->fmt->fourcc;
	f->fmt.pix.bytesperline =
		(f->fmt.pix.width * dev->fmt->depth) >> 3;
	f->fmt.pix.sizeimage =
		f->fmt.pix.height * f->fmt.pix.bytesperline;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{

	int ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return 0;
}




static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	pr_err( "Camera %d", inp->index );
	if (inp->index > 1)
		return -EINVAL;

//	pr_err("Camera 0");
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	sprintf(inp->name, "Camera %u", inp->index);
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct lepton_data *dev = video_drvdata(file);

	*i = dev->input;
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct lepton_data *dev = video_drvdata(file);

	if (i > 0)
		return -EINVAL;

	if (i == dev->input)
		return 0;

	dev->input = i;
	return 0;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
					 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width  = WIDTH;
	fsize->discrete.height = HEIGHT;
	return 0;
}


static int vidioc_enum_frameintervals(struct file *file, void *priv,
								struct v4l2_frmivalenum *fival)
{
	struct lepton_data *dev = video_drvdata(file);
	if (fival->index)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = dev->timeperframe;
	return 0;
}

static const struct v4l2_ioctl_ops lepton_ioctl_ops = {
	.vidioc_querycap      = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_enum_framesizes   = vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = vidioc_enum_frameintervals,
	.vidioc_reqbufs       = vb2_ioctl_reqbufs,
	.vidioc_create_bufs   = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
	.vidioc_querybuf      = vb2_ioctl_querybuf,
	.vidioc_qbuf          = vb2_ioctl_qbuf,
	.vidioc_dqbuf         = vb2_ioctl_dqbuf,
	.vidioc_enum_input    = vidioc_enum_input,
	.vidioc_g_input       = vidioc_g_input,
	.vidioc_s_input       = vidioc_s_input,
	.vidioc_streamon      = vb2_ioctl_streamon,
	.vidioc_streamoff     = vb2_ioctl_streamoff,
	.vidioc_log_status    = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static struct video_device lepton_template = {
	.name = "lepton",
	.fops = &lepton_fops,
	.ioctl_ops = &lepton_ioctl_ops,
	.release = video_device_release_empty,
};

//static u8 result[PACKET_SIZE*PACKETS_PER_FRAME];

static void vivi_fillbuff(struct lepton_data *dev, struct lepton_buffer *buf)
{
	char *vbuf = (char *) vb2_plane_vaddr(&buf->vb, 0);
	int ret, resets = 0,j;
    int row, column, i, temp;
	int packetNumber;
    u16 value;
    u16 minValue = 65535;
    u16 maxValue = 0;
	u16 *frameBuffer;
	u8 checkpkt;
//	u8 result[PACKET_SIZE];
	int idx,diff,scale;
	const u8 *colormap = colormap_ironblack;

	u8 *result = dev->result;

    for ( j = 0; j < PACKETS_PER_FRAME; j++) {
//       ret = spi_read(dev->spi, result + (PACKET_SIZE * j), PACKET_SIZE);
       ret = spi_read(dev->spi, result , PACKET_SIZE);

	   if(ret)
			pr_err("SPI read failed %d", ret);

//        packetNumber = result[(j * PACKET_SIZE) + 1];
//        checkpkt = result[j * PACKET_SIZE];
        packetNumber = result[1];
        checkpkt = result[0];

		result = result + PACKET_SIZE;
//		pr_err("%d = check pkt", checkpkt);
//		pr_err("%d = packet number", packetNumber);
        if (packetNumber != j) {
            j = -1;
			result = dev->result;
			pr_err("%d=bad packet number", packetNumber);
            resets += 1;
            msleep(1);
            if (resets == 75) {
                msleep(750);
            }
        }
    }

    if (resets >= 30) {
        pr_err("done reading, resets: \n" );
    }
	result = dev->result;
    frameBuffer = (u16 *)result;
    for (i = 0; i < FRAME_SIZE_UINT16; i++) {
        if (i % PACKET_SIZE_UINT16 < 2) {
            continue;
        }

//        temp = result[i * 2];
//        result[i * 2] = result[i * 2 + 1];
//        result[i * 2 + 1] = temp;

        value = frameBuffer[i];
        if (value > maxValue) {
            maxValue = value;
        }
        if (value < minValue) {
            minValue = value;
        }
        column = i % PACKET_SIZE_UINT16 - 2;
        row = i / PACKET_SIZE_UINT16;
    }

    diff = maxValue - minValue;
//    scale = 255 / diff;
    scale = diff/255;
	if ((diff%255>=128))
		scale++;

	pr_err("scale= %d, diff= %d",scale,diff);
	pr_err("maxValue= %d, minValue= %d",maxValue, minValue);
	for (i = 0; i < FRAME_SIZE_UINT16; i++) {
        if (i % PACKET_SIZE_UINT16 < 2) {
            continue;
        }
        value = (frameBuffer[i] - minValue) / scale;
//        value = frameBuffer[i]/257;
        column = (i % PACKET_SIZE_UINT16) - 2;
        row = i / PACKET_SIZE_UINT16;

        // Set video buffer pixel to scaled colormap value
        idx = row * WIDTH * 3 + column * 3;
//		vbuf[idx + 0] = value; // colormap[3 * value];
//		vbuf[idx + 1] = value; // colormap[3 * value + 1];
//	    vbuf[idx + 2] = value; // colormap[3 * value + 2];
		vbuf[idx + 0] = colormap[3 * value];
		vbuf[idx + 1] = colormap[3 * value + 1];
	    vbuf[idx + 2] = colormap[3 * value + 2];

    }
}



static void vivi_thread_tick(struct lepton_data *data)
{
	struct lepton_queue *dma_q = &data->vidq;
	struct lepton_buffer *buf;
	unsigned long flags = 0;

	pr_err("Thread tick\n");

	spin_lock_irqsave(&data->slock, flags);
	if (list_empty(&dma_q->active)) {
		pr_err("No active queue to serve\n");
		spin_unlock_irqrestore(&data->slock, flags);
		return;
	}

	buf = list_entry(dma_q->active.next, struct lepton_buffer, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&data->slock, flags);

//	do_gettimeofday(&buf->vb.v4l2_buf.timestamp);

	/* Fill buffer */
	vivi_fillbuff(data, buf);
	pr_err("filled buffer %p\n", buf);

	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
//	pr_err("[%p/%d] done\n", buf, buf->vb.v4l2_buf.index);
}


/* Wake up at about 30 fps */
#define WAKE_NUMERATOR 30
#define WAKE_DENOMINATOR 1001

#define frames_to_ms(frames)					\
	((frames * WAKE_NUMERATOR * 1000) / WAKE_DENOMINATOR)

static void vivi_sleep(struct lepton_data *data)
{
	struct lepton_queue *dma_q = &data->vidq;
	int timeout;
	DECLARE_WAITQUEUE(wait, current);

//	dprintk(dev, 1, "%s dma_q=0x%08lx\n", __func__,
//		(unsigned long)dma_q);

	add_wait_queue(&dma_q->wq, &wait);
	if (kthread_should_stop())
		goto stop_task;

	/* Calculate time to wake up */
	timeout = msecs_to_jiffies(frames_to_ms(1));

	vivi_thread_tick(data);

	schedule_timeout_interruptible(timeout);

stop_task:
	remove_wait_queue(&dma_q->wq, &wait);
	try_to_freeze();
}


/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/
static int queue_setup(struct vb2_queue *vq, const void  *parg,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct lepton_data *data = vb2_get_drv_priv(vq);
	unsigned long size;

	size = data->width * data->height * data->pixelsize;

	if (size == 0)
		return -EINVAL;

	if (0 == *nbuffers)
		*nbuffers = 32;

	while (size * *nbuffers > 8 * 1024 * 1024)
		(*nbuffers)--;

	*nplanes = 1;

	sizes[0] = size;

	/*
	 * videobuf2-vmalloc allocator is context-less so no need to set
	 * alloc_ctxs array.
	 */

	pr_err("%s, count=%d, size=%ld\n", __func__,
		*nbuffers, size);

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct lepton_data *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct lepton_buffer *buf = container_of(vb, struct lepton_buffer, vb);
	unsigned long size;

	BUG_ON(NULL == dev->fmt);

	size = dev->width * dev->height * dev->pixelsize;
	if (vb2_plane_size(vb, 0) < size) {
		pr_err("%s data will not fit into plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);

	buf->fmt = dev->fmt;

//	precalculate_bars(dev);
//	precalculate_line(dev);

	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct lepton_data *data = vb2_get_drv_priv(vb->vb2_queue);
	struct lepton_buffer *buf = container_of(vb, struct lepton_buffer, vb);
	struct lepton_queue *vidq = &data->vidq;
	unsigned long flags = 0;

	pr_err("%s\n", __func__);

	spin_lock_irqsave(&data->slock, flags);
	list_add_tail(&buf->list, &vidq->active);
	spin_unlock_irqrestore(&data->slock, flags);
}


static void lepton_lock(struct vb2_queue *vq)
{
	struct lepton_data *data = vb2_get_drv_priv(vq);
	mutex_lock(&data->mutex);
}

static void lepton_unlock(struct vb2_queue *vq)
{
	struct lepton_data *data = vb2_get_drv_priv(vq);
	mutex_unlock(&data->mutex);
}

static int vivi_thread(void *data)
{
	struct lepton_data *dev = data;

	pr_err("thread started\n");

	set_freezable();

	for (;;) {
		vivi_sleep(dev);

		if (kthread_should_stop())
			break;
	}
	pr_err( "thread: exit\n");
	return 0;
}


static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct lepton_data *dev = vb2_get_drv_priv(vq);
	struct lepton_queue *dma_q = &dev->vidq;

	pr_err("%s\n", __func__);

	/* Resets frame counters */
	dev->ms = 0;
	dev->mv_count = 0;
	dev->jiffies = jiffies;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;
	dma_q->kthread = kthread_run(vivi_thread, dev, dev->v4l2_dev.name);

	if (IS_ERR(dma_q->kthread)) {
		v4l2_err(&dev->v4l2_dev, "kernel_thread() failed\n");
		return PTR_ERR(dma_q->kthread);
	}
	/* Wakes thread */
	wake_up_interruptible(&dma_q->wq);

	pr_err("returning from %s\n", __func__);
	return 0;

}

void stop_streaming(struct vb2_queue *vq)
{
	struct lepton_data *dev = vb2_get_drv_priv(vq);
	struct lepton_queue *dma_q = &dev->vidq;

	pr_err("%s\n", __func__);
	/* shutdown control thread */
	if (dma_q->kthread) {
		kthread_stop(dma_q->kthread);
		dma_q->kthread = NULL;
	}

	/* Release all active buffers */
	while (!list_empty(&dma_q->active)) {
		struct lepton_buffer *buf;
		buf = list_entry(dma_q->active.next, struct lepton_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
//		pr_err("[%p/%d] done\n", buf, buf->vb.v4l2_buf.index);
	}

}



static struct vb2_ops vivi_video_qops = {
	.queue_setup		= queue_setup,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= lepton_unlock,
	.wait_finish		= lepton_lock,
};

static int skeleton_s_ctrl(struct v4l2_ctrl *ctrl)
{
	/*struct skeleton *skel =
		container_of(ctrl->handler, struct skeleton, ctrl_handler);*/

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		/* TODO: set brightness to ctrl->val */
		break;
	case V4L2_CID_CONTRAST:
		/* TODO: set contrast to ctrl->val */
		break;
	case V4L2_CID_SATURATION:
		/* TODO: set saturation to ctrl->val */
		break;
	case V4L2_CID_HUE:
		/* TODO: set hue to ctrl->val */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* ------------------------------------------------------------------
	File operations for the device
   ------------------------------------------------------------------*/

static const struct v4l2_ctrl_ops skel_ctrl_ops = {
	.s_ctrl = skeleton_s_ctrl,
};


static int lepton_probe(struct spi_device *spi)
{
	struct lepton_data *data;
	struct video_device *vfd;
	struct v4l2_ctrl_handler *hdl;
	struct vb2_queue *q;
	int ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spi_set_drvdata(spi, (void *) data);
	data->spi = spi;
	snprintf(data->v4l2_dev.name, sizeof(data->v4l2_dev.name),
			"%s", "lepton");
	ret = v4l2_device_register(&spi->dev, &data->v4l2_dev);
	if (ret)
		goto free_dev;

	data->input = 1;
	data->fmt = &format;
	data->width = WIDTH;
	data->height = HEIGHT;
	data->timeperframe.numerator = 1;
	data->timeperframe.denominator = 27;
	data->pixelsize = data->fmt->depth / 8;
	hdl = &data->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 8);

	v4l2_ctrl_new_std(hdl, &skel_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 255, 1, 127);
	v4l2_ctrl_new_std(hdl, &skel_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 255, 1, 16);
	v4l2_ctrl_new_std(hdl, &skel_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 255, 1, 127);
	v4l2_ctrl_new_std(hdl, &skel_ctrl_ops,
			  V4L2_CID_HUE, -128, 127, 1, 0);

//	data->volume = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_AUDIO_VOLUME, 0, 255, 1, 200);
//	data->brightness = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_BRIGHTNESS, 0, 255, 1, 127);
//	data->contrast = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_CONTRAST, 0, 255, 1, 16);
//	data->saturation = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_SATURATION, 0, 255, 1, 127);
//	data->hue = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_HUE, -128, 127, 1, 0);
//	data->autogain = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops//			V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
//			V4L2_CID_AUTOGAIN, 0, 1, 1, 1);
//	data->gain = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_GAIN, 0, 255, 1, 100);
//	data->alpha = v4l2_ctrl_new_std(hdl, &vivi_ctrl_ops,
//			V4L2_CID_ALPHA_COMPONENT, 0, 255, 1, 0);
//	data->button = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_button, NULL);
//	data->int32 = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_int32, NULL);
	printk(KERN_ERR "lepton: error %d after hld_reg\n", hdl->error);
/*
	data->int64 = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_int64, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
	data->boolean = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_boolean, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
	data->menu = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_menu, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
	data->string = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_string, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
	data->bitmask = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_bitmask, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
	data->int_menu = v4l2_ctrl_new_custom(hdl, &vivi_ctrl_int_menu, NULL);
	printk(KERN_ERR "vivi: error %d after hld_reg\n", hdl->error);
*/
	if (hdl->error) {
		ret = hdl->error;
		goto unreg_dev;
	}
//	v4l2_ctrl_auto_cluster(2, &data->autogain, 0, true);
	data->v4l2_dev.ctrl_handler = hdl;

	/* initialize locks */
	spin_lock_init(&data->slock);

	/* initialize queue */
	q = &data->vb_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->drv_priv = data;
	q->buf_struct_size = sizeof(struct lepton_buffer);
	q->ops = &vivi_video_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->min_buffers_needed = 8;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret)
		goto unreg_dev;

	mutex_init(&data->mutex);

	/* init video dma queues */
	INIT_LIST_HEAD(&data->vidq.active);
	init_waitqueue_head(&data->vidq.wq);

	vfd = &data->vdev;
	*vfd = lepton_template;
//	vfd->debug = debug;
	vfd->v4l2_dev = &data->v4l2_dev;
	vfd->queue = q;
//	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);

	/*
	 * Provide a mutex to v4l2 core. It will be used to protect
	 * all fops and v4l2 ioctls.
	 */
	vfd->lock = &data->mutex;
	video_set_drvdata(vfd, data);

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto unreg_dev;

	/* Now that everything is fine, let's add it to device list */
//	list_add_tail(&data->vivi_devlist, &vivi_devlist);

	v4l2_info(&data->v4l2_dev, "V4L2 device registered as %s\n",
		  video_device_node_name(vfd));
	return 0;

unreg_dev:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(&data->v4l2_dev);
free_dev:
	kfree(data);
	return ret;
}

static int lepton_remove(struct spi_device *spi)
{
	struct lepton_data *data;

	data = spi_get_drvdata(spi);
	v4l2_info(&data->v4l2_dev, "unregistering %s\n",
	video_device_node_name(&data->vdev));
	video_unregister_device(&data->vdev);
	v4l2_device_unregister(&data->v4l2_dev);
	v4l2_ctrl_handler_free(&data->ctrl_handler);
	kfree(data);
	return 0;
}

static const struct spi_device_id lepton_id[] = {
	{ "lepton", 0 },
	{},
};

static struct spi_driver lepton_driver = {
	.driver = {
		.name = "lepton",
	},
	.probe = lepton_probe,
	.remove = lepton_remove,
	.id_table = lepton_id,
};

module_spi_driver(lepton_driver);

MODULE_DESCRIPTION("Driver for Lepton thermal camera");
MODULE_AUTHOR("Navin Patidar <navin.patidar@gmail.com>");
MODULE_LICENSE("GPL");
