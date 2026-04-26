/*
 * Copyright © 2026
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <config.h>

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_ioctl.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "libusbi.h"

#define DEVPATH "/dev/"
#define USBCTL DEVPATH USB_DEVICE_NAME
#define USB_MAX_ENDPOINTS (UE_ADDR + 1)

struct device_priv {
	char *devname;
	int fd;
	uint8_t active_config_index;
	usb_config_descriptor_t *cdesc;
};

struct handle_priv {
	int claimed_interfaces[USB_IFACE_MAX];
};

static int freebsd_get_device_list(struct libusb_context *, struct discovered_devs **);
static int freebsd_open(struct libusb_device_handle *);
static void freebsd_close(struct libusb_device_handle *);
static int freebsd_get_active_config_descriptor(struct libusb_device *, void *, size_t);
static int freebsd_get_config_descriptor(struct libusb_device *, uint8_t, void *, size_t);
static int freebsd_get_configuration(struct libusb_device_handle *, uint8_t *);
static int freebsd_set_configuration(struct libusb_device_handle *, int);
static int freebsd_claim_interface(struct libusb_device_handle *, uint8_t);
static int freebsd_release_interface(struct libusb_device_handle *, uint8_t);
static int freebsd_set_interface_altsetting(struct libusb_device_handle *, uint8_t, uint8_t);
static int freebsd_clear_halt(struct libusb_device_handle *, unsigned char);
static void freebsd_destroy_device(struct libusb_device *);
static int freebsd_submit_transfer(struct usbi_transfer *);
static int freebsd_cancel_transfer(struct usbi_transfer *);
static int freebsd_handle_transfer_completion(struct usbi_transfer *);

static int errno_to_libusb(int);
static int cache_config_descriptor(struct libusb_device *, uint8_t);
static int sync_control_transfer(struct usbi_transfer *);

const struct usbi_os_backend usbi_backend = {
	.name = "Synchronous FreeBSD backend",
	.get_device_list = freebsd_get_device_list,
	.open = freebsd_open,
	.close = freebsd_close,

	.get_active_config_descriptor = freebsd_get_active_config_descriptor,
	.get_config_descriptor = freebsd_get_config_descriptor,

	.get_configuration = freebsd_get_configuration,
	.set_configuration = freebsd_set_configuration,

	.claim_interface = freebsd_claim_interface,
	.release_interface = freebsd_release_interface,
	.set_interface_altsetting = freebsd_set_interface_altsetting,
	.clear_halt = freebsd_clear_halt,
	.destroy_device = freebsd_destroy_device,

	.submit_transfer = freebsd_submit_transfer,
	.cancel_transfer = freebsd_cancel_transfer,
	.handle_transfer_completion = freebsd_handle_transfer_completion,

	.device_priv_size = sizeof(struct device_priv),
	.device_handle_priv_size = sizeof(struct handle_priv),
};

static int
freebsd_get_device_list(struct libusb_context *ctx, struct discovered_devs **discdevs)
{
	struct usb_device_info di;
	struct libusb_device *dev;
	struct discovered_devs *ddd;
	struct device_priv *dpriv;
	char devnode[32];
	unsigned long session_id;
	int fd, idx;

	fd = open(USBCTL, O_RDWR);
	if (fd < 0)
		return errno_to_libusb(errno);

	for (idx = 0; idx < 256; idx++) {
		memset(&di, 0, sizeof(di));
		di.udi_index = (uint8_t)idx;
		if (ioctl(fd, USB_DEVICEINFO, &di) < 0) {
			if (errno == ENXIO || errno == ENOENT)
				break;
			continue;
		}

		if (di.udi_addr == 0)
			continue;

		session_id = (di.udi_bus << 8) | di.udi_addr;
		dev = usbi_get_device_by_session_id(ctx, session_id);
		if (dev == NULL) {
			dev = usbi_alloc_device(ctx, session_id);
			if (dev == NULL) {
				close(fd);
				return LIBUSB_ERROR_NO_MEM;
			}

			dev->bus_number = di.udi_bus;
			dev->device_address = di.udi_addr;
			dev->speed = di.udi_speed;
			dev->port_number = di.udi_hubport;

			dpriv = usbi_get_device_priv(dev);
			dpriv->fd = -1;
			dpriv->active_config_index = di.udi_config_index;

			if (asprintf(&dpriv->devname, "ugen%u.%u", di.udi_bus, di.udi_addr) < 0) {
				libusb_unref_device(dev);
				close(fd);
				return LIBUSB_ERROR_NO_MEM;
			}

			snprintf(devnode, sizeof(devnode), DEVPATH "%s", dpriv->devname);
			int dfd = open(devnode, O_RDWR);
			if (dfd < 0) {
				libusb_unref_device(dev);
				continue;
			}

			if (ioctl(dfd, USB_GET_DEVICE_DESC, &dev->device_descriptor) < 0) {
				close(dfd);
				libusb_unref_device(dev);
				continue;
			}
			usbi_localize_device_descriptor(&dev->device_descriptor);
			close(dfd);

			if (cache_config_descriptor(dev, di.udi_config_index) != LIBUSB_SUCCESS) {
				libusb_unref_device(dev);
				continue;
			}

			if (usbi_sanitize_device(dev)) {
				libusb_unref_device(dev);
				continue;
			}
		}

		ddd = discovered_devs_append(*discdevs, dev);
		if (ddd == NULL) {
			libusb_unref_device(dev);
			close(fd);
			return LIBUSB_ERROR_NO_MEM;
		}
		libusb_unref_device(dev);
		*discdevs = ddd;
	}

	close(fd);
	return LIBUSB_SUCCESS;
}

static int
freebsd_open(struct libusb_device_handle *handle)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);
	char devnode[32];

	if (dpriv->fd >= 0)
		return LIBUSB_SUCCESS;

	snprintf(devnode, sizeof(devnode), DEVPATH "%s", dpriv->devname);
	dpriv->fd = open(devnode, O_RDWR);
	if (dpriv->fd < 0)
		return errno_to_libusb(errno);

	return LIBUSB_SUCCESS;
}

static void
freebsd_close(struct libusb_device_handle *handle)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);

	if (dpriv->fd >= 0) {
		close(dpriv->fd);
		dpriv->fd = -1;
	}
}

static int
freebsd_get_active_config_descriptor(struct libusb_device *dev, void *buf, size_t len)
{
	struct device_priv *dpriv = usbi_get_device_priv(dev);
	int desc_len;

	if (dpriv->cdesc == NULL)
		return LIBUSB_ERROR_NOT_FOUND;

	desc_len = UGETW(dpriv->cdesc->wTotalLength);
	if (len < (size_t)desc_len)
		return LIBUSB_ERROR_OVERFLOW;

	memcpy(buf, dpriv->cdesc, (size_t)desc_len);
	return desc_len;
}

static int
freebsd_get_config_descriptor(struct libusb_device *dev, uint8_t idx, void *buf, size_t len)
{
	struct device_priv *dpriv = usbi_get_device_priv(dev);
	char devnode[32];
	struct usb_gen_descriptor ugd;
	int fd;

	snprintf(devnode, sizeof(devnode), DEVPATH "%s", dpriv->devname);
	fd = open(devnode, O_RDWR);
	if (fd < 0)
		return errno_to_libusb(errno);

	memset(&ugd, 0, sizeof(ugd));
	ugd.ugd_data = buf;
	ugd.ugd_maxlen = (uint16_t)len;
	ugd.ugd_config_index = idx;
	ugd.ugd_offset = 0;

	if (ioctl(fd, USB_GET_FULL_DESC, &ugd) < 0) {
		int err = errno;
		close(fd);
		return errno_to_libusb(err);
	}

	close(fd);
	return ugd.ugd_actlen;
}

static int
freebsd_get_configuration(struct libusb_device_handle *handle, uint8_t *config)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);
	int cfg;

	if (ioctl(dpriv->fd, USB_GET_CONFIG, &cfg) < 0)
		return errno_to_libusb(errno);

	*config = (uint8_t)cfg;
	return LIBUSB_SUCCESS;
}

static int
freebsd_set_configuration(struct libusb_device_handle *handle, int config)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);

	if (ioctl(dpriv->fd, USB_SET_CONFIG, &config) < 0)
		return errno_to_libusb(errno);

	return cache_config_descriptor(handle->dev, (uint8_t)config);
}

static int
freebsd_claim_interface(struct libusb_device_handle *handle, uint8_t iface)
{
	struct handle_priv *hpriv = usbi_get_device_handle_priv(handle);

	if (iface >= USB_IFACE_MAX)
		return LIBUSB_ERROR_INVALID_PARAM;

	hpriv->claimed_interfaces[iface] = 1;
	return LIBUSB_SUCCESS;
}

static int
freebsd_release_interface(struct libusb_device_handle *handle, uint8_t iface)
{
	struct handle_priv *hpriv = usbi_get_device_handle_priv(handle);

	if (iface >= USB_IFACE_MAX)
		return LIBUSB_ERROR_INVALID_PARAM;

	hpriv->claimed_interfaces[iface] = 0;
	return LIBUSB_SUCCESS;
}

static int
freebsd_set_interface_altsetting(struct libusb_device_handle *handle, uint8_t iface, uint8_t altsetting)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);
	struct usb_alt_interface intf;

	memset(&intf, 0, sizeof(intf));
	intf.uai_interface_index = iface;
	intf.uai_alt_index = altsetting;

	if (ioctl(dpriv->fd, USB_SET_ALTINTERFACE, &intf) < 0)
		return errno_to_libusb(errno);

	return LIBUSB_SUCCESS;
}

static int
freebsd_clear_halt(struct libusb_device_handle *handle, unsigned char endpoint)
{
	struct device_priv *dpriv = usbi_get_device_priv(handle->dev);
	int stall = 0;

	if (UE_GET_DIR(endpoint) == UE_DIR_IN) {
		if (ioctl(dpriv->fd, USB_SET_RX_STALL_FLAG, &stall) < 0)
			return errno_to_libusb(errno);
	} else {
		if (ioctl(dpriv->fd, USB_SET_TX_STALL_FLAG, &stall) < 0)
			return errno_to_libusb(errno);
	}

	return LIBUSB_SUCCESS;
}

static void
freebsd_destroy_device(struct libusb_device *dev)
{
	struct device_priv *dpriv = usbi_get_device_priv(dev);

	free(dpriv->devname);
	free(dpriv->cdesc);
}

static int
freebsd_submit_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);

	if (transfer->type != LIBUSB_TRANSFER_TYPE_CONTROL)
		return LIBUSB_ERROR_NOT_SUPPORTED;

	return sync_control_transfer(itransfer);
}

static int
freebsd_cancel_transfer(struct usbi_transfer *itransfer)
{
	return LIBUSB_ERROR_NOT_SUPPORTED;
}

static int
freebsd_handle_transfer_completion(struct usbi_transfer *itransfer)
{
	usbi_handle_transfer_completion(itransfer, LIBUSB_TRANSFER_COMPLETED);
	return LIBUSB_SUCCESS;
}

static int
sync_control_transfer(struct usbi_transfer *itransfer)
{
	struct libusb_transfer *transfer = USBI_TRANSFER_TO_LIBUSB_TRANSFER(itransfer);
	struct device_priv *dpriv = usbi_get_device_priv(transfer->dev_handle->dev);
	struct libusb_control_setup *setup = (struct libusb_control_setup *)transfer->buffer;
	struct usb_ctl_request req;
	int timeout;

	memset(&req, 0, sizeof(req));
	req.ucr_addr = transfer->dev_handle->dev->device_address;
	req.ucr_request.bmRequestType = setup->bmRequestType;
	req.ucr_request.bRequest = setup->bRequest;
	(*(uint16_t *)req.ucr_request.wValue) = setup->wValue;
	(*(uint16_t *)req.ucr_request.wIndex) = setup->wIndex;
	(*(uint16_t *)req.ucr_request.wLength) = setup->wLength;
	req.ucr_data = transfer->buffer + LIBUSB_CONTROL_SETUP_SIZE;

	if ((transfer->flags & LIBUSB_TRANSFER_SHORT_NOT_OK) == 0)
		req.ucr_flags = USB_SHORT_XFER_OK;

	timeout = transfer->timeout;
	if (setup->bmRequestType & LIBUSB_ENDPOINT_IN) {
		if (ioctl(dpriv->fd, USB_SET_RX_TIMEOUT, &timeout) < 0)
			return errno_to_libusb(errno);
	} else {
		if (ioctl(dpriv->fd, USB_SET_TX_TIMEOUT, &timeout) < 0)
			return errno_to_libusb(errno);
	}

	if (ioctl(dpriv->fd, USB_DO_REQUEST, &req) < 0)
		return errno_to_libusb(errno);

	itransfer->transferred = req.ucr_actlen;
	return LIBUSB_SUCCESS;
}

static int
cache_config_descriptor(struct libusb_device *dev, uint8_t idx)
{
	struct device_priv *dpriv = usbi_get_device_priv(dev);
	char devnode[32];
	struct usb_gen_descriptor ugd;
	struct usb_config_descriptor cdesc;
	uint16_t len;
	void *buf;
	int fd;

	snprintf(devnode, sizeof(devnode), DEVPATH "%s", dpriv->devname);
	fd = open(devnode, O_RDWR);
	if (fd < 0)
		return errno_to_libusb(errno);

	if (ioctl(fd, USB_GET_CONFIG_DESC, &cdesc) < 0) {
		int err = errno;
		close(fd);
		return errno_to_libusb(err);
	}

	len = UGETW(cdesc.wTotalLength);
	buf = malloc(len);
	if (buf == NULL) {
		close(fd);
		return LIBUSB_ERROR_NO_MEM;
	}

	memset(&ugd, 0, sizeof(ugd));
	ugd.ugd_data = buf;
	ugd.ugd_maxlen = len;
	ugd.ugd_config_index = idx;
	ugd.ugd_offset = 0;

	if (ioctl(fd, USB_GET_FULL_DESC, &ugd) < 0) {
		int err = errno;
		free(buf);
		close(fd);
		return errno_to_libusb(err);
	}
	close(fd);

	free(dpriv->cdesc);
	dpriv->cdesc = buf;
	dpriv->active_config_index = idx;
	return LIBUSB_SUCCESS;
}

static int
errno_to_libusb(int err)
{
	switch (err) {
	case EIO:
		return LIBUSB_ERROR_IO;
	case EACCES:
		return LIBUSB_ERROR_ACCESS;
	case ENOENT:
		return LIBUSB_ERROR_NO_DEVICE;
	case ENOMEM:
		return LIBUSB_ERROR_NO_MEM;
	case ETIMEDOUT:
		return LIBUSB_ERROR_TIMEOUT;
	case EOVERFLOW:
		return LIBUSB_ERROR_OVERFLOW;
	case EPIPE:
		return LIBUSB_ERROR_PIPE;
	case EINTR:
		return LIBUSB_ERROR_INTERRUPTED;
	case ENOSYS:
		return LIBUSB_ERROR_NOT_SUPPORTED;
	default:
		return LIBUSB_ERROR_OTHER;
	}
}
