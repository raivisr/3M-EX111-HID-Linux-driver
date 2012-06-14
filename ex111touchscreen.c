/******************************************************************************
 * ex111touchscreen.c
 * Driver for 3M EX111 HID touchscreen
 *
 * Copyright (C) 2012 by Raivis Rengelis <raivis@rrkb.lv>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * This driver is based on usbtouchscreen.c by Daniel Ritz and Todd E. Johnson
 *****************************************************************************/

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/input.h>
#include <linux/hid.h>

#include "ex111-ids.h"

#define DRIVER_AUTHOR "Raivis Rengelis <raivis@rrkb.lv>"
#define DRIVER_DESC "3M EX111 HID touchscreen driver"
#define DRIVER_VERSION "0.1alpha"

static int swap_xy;
module_param(swap_xy, bool, 0644);
MODULE_PARM_DESC(swap_xy, "If set X and Y axes are swapped.");

#define REPT_SIZE 17
#define MIN_XC 0x0
#define MAX_XC 0x3FFF
#define MIN_YC 0x0
#define MAX_YC 0x3FFF

struct ex111_usb {
	unsigned char *data;
	dma_addr_t data_dma;
	unsigned char *buffer;
	int buf_len;
	struct urb *irq;
	struct usb_interface *interface;
	struct input_dev *input;
	char name[128];
	char phys[64];
	void *priv;
	unsigned int intrpipe;
	struct work_struct reset_pipe_work;

	int x, y;
	int touch, press;
};

static int ex111_init(struct ex111_usb *ex111)
{
	int ret;
	struct usb_device *udev = interface_to_usbdev(ex111->interface);
	char *buf;

	buf = (char *)kzalloc(32,GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = 0x10;
	buf[1] = 0x12;
	buf[2] = 0x01;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			HID_REQ_SET_REPORT,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0x0310, 0, (void *)buf, 0x8, USB_CTRL_SET_TIMEOUT);
	dbg("%s - usb_control_msg - EX111_RESET - bytes|err: %d",
	    __func__, ret);
	if (ret < 0)
		goto bailout;

	do {
		msleep(100);
		memset(buf,0,32);

		ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				0x0380, 0, (void *)buf, 0x14, USB_CTRL_SET_TIMEOUT);
		dbg("%s - usb_control_msg - EX111_STATUS_REQ - bytes|err: %d",
				__func__, ret);
	} while (buf[3] == 0x01);
	dbg("%s - EX111_STATUS_REQ buf[3]: %x", __func__, buf[3]);
	if (ret < 0)
		goto bailout;

	memset(buf,0,32);
	buf[0] = 0x10;
	buf[1] = 0x10;
	buf[2] = 0x01;
	buf[3] = 0x01;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			HID_REQ_SET_REPORT,
			USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
			0x0310, 0, (void *)buf, 0x8, USB_CTRL_SET_TIMEOUT);
	dbg("%s - usb_control_msg - EX111_SET_ASYNC - bytes|err: %d",
	    __func__, ret);
	if (ret < 0)
		goto bailout;

	do {
		memset(buf,0,32);

		ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
				HID_REQ_GET_REPORT,
				USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
				0x0380, 0, (void *)buf, 0x14, USB_CTRL_SET_TIMEOUT);
		dbg("%s - usb_control_msg - EX111_STATUS_REQ - bytes|err: %d",
				__func__, ret);
	} while (buf[3] == 0x01);
	dbg("%s - EX111_STATUS_REQ buf[3]: %x", __func__, buf[3]);

bailout:
	kfree(buf);

	if (ret < 0)
		return ret;

	return 0;
}

static void reset_halted_pipe(struct work_struct *ws)
{
	struct ex111_usb *ex111 =
			container_of(ws, struct ex111_usb, reset_pipe_work);
	struct usb_device *udev = interface_to_usbdev(ex111->interface);

	int rv = usb_clear_halt(udev, ex111->intrpipe);

	if (rv != 0)
		err("%s - usb_clear_halt failed with %d",__func__,rv);

	rv = ex111_init(ex111);
	if (rv != 0)
		err("%s - ex111_init failed with %d",__func__,rv);
}

static void ex111_process_paket(struct ex111_usb *ex111, unsigned char *pkt)
{
	ex111->x = (pkt[4] << 8) | pkt[3];
	ex111->y = (pkt[6] << 8) | pkt[5];
	ex111->touch = (pkt[2] & 0x01) ? 1 : 0;
	dbg("%s - cal_x %d, cal_y %d", __func__, (pkt[4] << 8) | pkt[3],(pkt[6] << 8) | pkt[5]);

	input_report_key(ex111->input, BTN_TOUCH, ex111->touch);

	if (swap_xy) {
		input_report_abs(ex111->input, ABS_X, ex111->y);
		input_report_abs(ex111->input, ABS_Y, ex111->x);
	} else {
		input_report_abs(ex111->input, ABS_X, ex111->x);
		input_report_abs(ex111->input, ABS_Y, ex111->y);
	}
	input_sync(ex111->input);
}

static void ex111_irq(struct urb *urb)
{
	struct ex111_usb *ex111 = urb->context;
	int retval;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ETIME:
		/* this urb is timing out */
		dbg("%s - urb timed out - was the device unplugged?",
		    __func__);
		return;
	case -EPIPE:
		dbg("%s - urb stalled with status: %d, trying to resurrect",
		    __func__, urb->status);
		schedule_work(&ex111->reset_pipe_work);
		goto exit;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	default:
		dbg("%s - nonzero urb status received: %d",
		    __func__, urb->status);
		goto exit;
	}

	ex111_process_paket(ex111, ex111->data);

exit:
	usb_mark_last_busy(interface_to_usbdev(ex111->interface));
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		err("%s - usb_submit_urb failed with result: %d", __func__, retval);
}


static int ex111_open(struct input_dev *input)
{
	struct ex111_usb *ex111 = input_get_drvdata(input);
	int r,i;

	ex111->irq->dev = interface_to_usbdev(ex111->interface);

	r = usb_autopm_get_interface(ex111->interface) ? -EIO : 0;
	dbg("%s - usb_autopm_get_interface returned %d",__func__, r);
	if (r < 0)
		goto out;

	i = usb_submit_urb(ex111->irq, GFP_KERNEL);
	if (i < 0) {
		dbg("%s - usb_submit_urb borked: %d",__func__, i);
		r = -EIO;
		goto out_put;
	}


	ex111->interface->needs_remote_wakeup = 1;

out_put:
	usb_autopm_put_interface(ex111->interface);
out:
	dbg("%s - finally returning %d",__func__, r);
	return r;
}

static void ex111_close(struct input_dev *input)
{
	struct ex111_usb *ex111 = input_get_drvdata(input);
	int r;

	usb_kill_urb(ex111->irq);
	r = usb_autopm_get_interface(ex111->interface);
	ex111->interface->needs_remote_wakeup = 0;
	if (!r)
		usb_autopm_put_interface(ex111->interface);
}

static const struct usb_device_id ex111_devices[] = {
	{USB_DEVICE(0x0596, 0x0003), },
	{},
};

static int ex111_suspend
(struct usb_interface *intf, pm_message_t message)
{
	struct ex111_usb *ex111 = usb_get_intfdata(intf);

	usb_kill_urb(ex111->irq);

	return 0;
}

static int ex111_resume(struct usb_interface *intf)
{
	struct ex111_usb *ex111 = usb_get_intfdata(intf);
	struct input_dev *input = ex111->input;
	int res = 0;

	mutex_lock(&input->mutex);
	if (input->users )
		res = usb_submit_urb(ex111->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return res;
}

static int ex111_reset_resume(struct usb_interface *intf)
{
	struct ex111_usb *ex111 = usb_get_intfdata(intf);
	struct input_dev *input = ex111->input;
	int err = 0;

	/* reinit the device */
	err = ex111_init(ex111);
	if (err) {
		dbg("%s - ex111_init() failed, err: %d",
				__func__, err);
		return err;
	}

	/* restart IO if needed */
	mutex_lock(&input->mutex);
	if (input->users)
		err = usb_submit_urb(ex111->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return err;
}

static struct usb_endpoint_descriptor *
ex111_get_input_endpoint(struct usb_host_interface *interface)
{
	int i;

	for (i = 0; i < interface->desc.bNumEndpoints; i++)
		if (usb_endpoint_dir_in(&interface->endpoint[i].desc))
			return &interface->endpoint[i].desc;

	return NULL;
}

static int ex111_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct ex111_usb *ex111;
	struct input_dev *input_dev;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device *udev = interface_to_usbdev(intf);
	int err = -ENOMEM;

	endpoint = ex111_get_input_endpoint(intf->cur_altsetting);
	if (!endpoint)
		return -ENXIO;

	ex111 = kzalloc(sizeof(struct ex111_usb), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ex111 || !input_dev)
		goto out_free;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
	ex111->data = usb_alloc_coherent(udev, REPT_SIZE, GFP_KERNEL, &ex111->data_dma);
#else
	ex111->data = usb_buffer_alloc(udev, REPT_SIZE, GFP_KERNEL, &ex111->data_dma);
#endif

	if (!ex111->data)
		goto out_free;

	ex111->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!ex111->irq) {
		dbg("%s - usb_alloc_urb failed: ex111->irq", __func__);
		goto out_free_buffers;
	}

	ex111->interface = intf;
	ex111->input = input_dev;

	if (udev->manufacturer)
		strlcpy(ex111->name, udev->manufacturer, sizeof(ex111->name));

	if (udev->product) {
		if (udev->manufacturer)
			strlcat(ex111->name, " ", sizeof(ex111->name));
		strlcat(ex111->name, udev->product, sizeof(ex111->name));
	}

	if (!strlen(ex111->name))
		snprintf(ex111->name, sizeof(ex111->name),
			"3M EX111 HID touchscreen %04x:%04x",
			 le16_to_cpu(udev->descriptor.idVendor),
			 le16_to_cpu(udev->descriptor.idProduct));

	usb_make_path(udev, ex111->phys, sizeof(ex111->phys));

	strlcat(ex111->phys, "/input0", sizeof(ex111->phys));

	input_dev->name = ex111->name;
	input_dev->phys = ex111->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->dev.parent = &intf->dev;

	input_set_drvdata(input_dev, ex111);

	input_dev->open = ex111_open;
	input_dev->close = ex111_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, MIN_XC, MAX_XC, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, MIN_YC, MAX_YC, 0, 0);

	ex111->intrpipe = usb_rcvintpipe(udev, endpoint->bEndpointAddress);
	usb_fill_int_urb(ex111->irq, udev,
			ex111->intrpipe,
			ex111->data, REPT_SIZE,
			ex111_irq, ex111, endpoint->bInterval);

	ex111->irq->dev = udev;
	ex111->irq->transfer_dma = ex111->data_dma;
	ex111->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	err = ex111_init(ex111);
	if (err) {
		dbg("%s - ex111_init() failed, err: %d", __func__, err);
		goto out_do_exit;
	}

	err = input_register_device(ex111->input);
	if (err) {
		dbg("%s - input_register_device() failed, err: %d", __func__, err);
		goto out_do_exit;
	}

	usb_set_intfdata(intf, ex111);

	INIT_WORK(&ex111->reset_pipe_work, reset_halted_pipe);

	return 0;

out_do_exit:
	usb_free_urb(ex111->irq);
out_free_buffers:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
	usb_free_coherent(interface_to_usbdev(intf), REPT_SIZE, ex111->data, ex111->data_dma);
#else
	usb_buffer_free(interface_to_usbdev(intf), REPT_SIZE, ex111->data, ex111->data_dma);
#endif
	kfree(ex111->buffer);
out_free:
	input_free_device(input_dev);
	kfree(ex111);
	return err;
}

static void ex111_disconnect(struct usb_interface *intf)
{
	struct ex111_usb *ex111 = usb_get_intfdata(intf);

	dbg("%s - called", __func__);

	if (!ex111)
		return;

	dbg("%s - ex111 is initialized, cleaning up", __func__);
	usb_set_intfdata(intf, NULL);
	/* this will stop IO via close */
	input_unregister_device(ex111->input);
	usb_free_urb(ex111->irq);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
	usb_free_coherent(interface_to_usbdev(intf), REPT_SIZE, ex111->data, ex111->data_dma);
#else
	usb_buffer_free(interface_to_usbdev(intf), REPT_SIZE, ex111->data, ex111->data_dma);
#endif
	kfree(ex111->buffer);
	kfree(ex111);
}

static struct usb_driver ex111_driver =
{
	.name		= "ex111touchscreen",
	.probe		= ex111_probe,
	.disconnect	= ex111_disconnect,
	.id_table	= ex111_devices,
	.suspend	= ex111_suspend,
	.resume		= ex111_resume,
	.reset_resume	= ex111_reset_resume,
	.supports_autosuspend = 1,
};

static int __init ex111_modinit(void)
{
	return usb_register(&ex111_driver);
}

static void __exit ex111_exit(void)
{
	usb_deregister(&ex111_driver);
}

module_init(ex111_modinit);
module_exit(ex111_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("usb:v0596p0003d*dc*dsc*dp*ic*isc*ip*");
