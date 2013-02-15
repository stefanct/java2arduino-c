#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "libusb-1.0/libusb.h"
#include "j2a_usb.h"

static const uint8_t USB_IF_CLASS = 0xFF;
static const uint8_t USB_IF_SUBCLASS = 0x12;
static const uint8_t USB_IF_PROTOCOL = 0xEF;
static const uint8_t USB_IN_EPNUM = 0x80 | 1;
static const uint8_t USB_OUT_EPNUM = 2;

static bool containsUsbEndpoint(const struct libusb_interface_descriptor *if_desc, uint8_t addr) {
	for (int i = 0; i < if_desc->bNumEndpoints; i++) {
		if (if_desc->endpoint[i].bEndpointAddress == addr)
			return true;
	}
	return false;
}

static bool isArduino(libusb_device *dev) {
	struct libusb_device_descriptor dev_desc;
	int ret = libusb_get_device_descriptor(dev, &dev_desc);
	if (ret != 0)
		return false;

	if (dev_desc.bDeviceClass != 0xff)
		return false;

	struct libusb_config_descriptor *cfg_desc;
	ret = libusb_get_active_config_descriptor(dev, &cfg_desc);
	if (ret != 0)
		return false;

	bool found = false;
	for (int i=0; i < cfg_desc->bNumInterfaces; i++) {
		const struct libusb_interface *intf = cfg_desc->interface;
		for (int j=0; j < intf->num_altsetting; j++) {
			const struct libusb_interface_descriptor *if_desc = intf->altsetting;
			if (if_desc->bInterfaceClass != USB_IF_CLASS
				|| if_desc->bInterfaceSubClass != USB_IF_SUBCLASS
				|| if_desc->bInterfaceProtocol != USB_IF_PROTOCOL)
				continue;
			if (!containsUsbEndpoint(if_desc, USB_IN_EPNUM)
				|| !containsUsbEndpoint(if_desc, USB_OUT_EPNUM))
				continue;
			found = true;
			goto out;
		}
	}
out:
	libusb_free_config_descriptor(cfg_desc);

	return found;
}

uint8_t j2a_usb_init(void) {
	return (libusb_init(NULL) != 0) ? 1 : 0;
}

void *j2a_usb_connect(const char *ignored) {
	int ret;

	libusb_device **list;
	libusb_device *found = NULL;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	if (cnt < 0) {
		ret = 1;
		goto out;
	}

	for (ssize_t i = 0; i < cnt; i++) {
		libusb_device *dev = list[i];
		if (isArduino(dev)) {
			//libusb_get_max_packet_size
			found = dev;
			break;
		}
	}

	if (found == NULL) {
		ret = 1;
		goto out;
	}
	libusb_device_handle *handle;
	ret = libusb_open(found, &handle);
	if (ret != 0)
		goto out;

	ret = libusb_claim_interface(handle, 0);
	if (ret != 0)
		goto out;

out:
	libusb_free_device_list(list, 1);
	if (ret != 0)
		return NULL;
	else
		return handle;
}

void j2a_usb_disconnect(struct j2a_handle *comm) {
	libusb_release_interface(comm->ctx, 0);
	libusb_close(comm->ctx);
}

void j2a_usb_shutdown(void) {
	libusb_exit(NULL);
}

uint8_t j2a_usb_write(struct j2a_handle *comm, uint8_t val) {
	if (comm->idx >= comm->len)
		return 1;
	comm->buf[comm->idx] = val;
	comm->idx++;
	return 0;
}

uint8_t j2a_usb_flush(struct j2a_handle *comm) {
	int transferred;
	int ret = libusb_bulk_transfer(comm->ctx, USB_OUT_EPNUM, comm->buf, comm->idx, &transferred, A2J_TIMEOUT * comm->len);
	if ((ret == 0 || ret == LIBUSB_ERROR_TIMEOUT) && transferred == comm->idx) {
		return 0;
	} else
		return 1;
}

uint8_t j2a_usb_read(struct j2a_handle *comm, uint8_t *val) {
	if (comm->idx >= comm->cnt) {
		int transferred;
		int ret = libusb_bulk_transfer(comm->ctx, USB_IN_EPNUM, comm->buf, comm->len, &transferred, A2J_TIMEOUT * comm->len);
		if ((ret != 0 && ret != LIBUSB_ERROR_TIMEOUT) || transferred == 0)
			return 1;

		comm->cnt = transferred;
		comm->idx = 0;
	}
	*val = comm->buf[comm->idx];
	comm->idx++;
	return 0;
}
