#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "libusb-1.0/libusb.h"
#include "j2a_usb.h"

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

	struct libusb_config_descriptor *cfg_desc;
	ret = libusb_get_active_config_descriptor(dev, &cfg_desc);
	if (ret != 0)
		return false;

	bool found = false;
	for (int i = 0; i < cfg_desc->bNumInterfaces; i++) {
		const struct libusb_interface *intf = cfg_desc->interface;
		for (int j = 0; j < intf->num_altsetting; j++) {
			const struct libusb_interface_descriptor *if_desc = intf->altsetting;
			if (if_desc->bInterfaceClass != A2J_USB_IF_CLASS
				|| if_desc->bInterfaceSubClass != A2J_USB_IF_SUBCLASS
				|| if_desc->bInterfaceProtocol != A2J_USB_IF_PROTOCOL)
				continue;
			if (!containsUsbEndpoint(if_desc, A2J_USB_IN_ADDR)
				|| !containsUsbEndpoint(if_desc, A2J_USB_OUT_ADDR))
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
	int ret = libusb_init(NULL);
	if (ret != 0) {
		fprintf(stderr, "Initializing libusb failed: %s\n", libusb_error_name(ret));
		return 1;
	} else
		return 0;
}

void *j2a_usb_connect(const char *nth_dev) {
	int nth = 0;
	if (nth_dev != NULL) {
		nth = strtol(nth_dev, NULL, 10);
		if (nth < 0)
			return NULL;
	}

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
			if (nth-- == 0) {
				found = dev;
				break;
			}
		}
	}

	if (found == NULL) {
		ret = 1;
		goto out;
	}
	libusb_device_handle *handle;
	ret = libusb_open(found, &handle);
	if (ret != 0) {
		fprintf(stderr, "open failed: %s\n", libusb_error_name(ret));
		goto out;
	}

	ret = libusb_claim_interface(handle, 0);
	if (ret != 0) {
		fprintf(stderr, "claim failed: %s\n", libusb_error_name(ret));
		goto out;
	}

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
	int ret = libusb_bulk_transfer(comm->ctx, A2J_USB_OUT_ADDR, comm->buf, comm->idx, &transferred, A2J_TIMEOUT * comm->len);
	if ((ret == 0 || ret == LIBUSB_ERROR_TIMEOUT) && transferred == (int)comm->idx) {
		return 0;
	} else
		return 1;
}

uint8_t j2a_usb_read(struct j2a_handle *comm, uint8_t *val) {
	if (comm->idx >= comm->cnt) {
		int transferred;
		int ret = libusb_bulk_transfer(comm->ctx, A2J_USB_IN_ADDR, comm->buf, comm->len, &transferred, A2J_TIMEOUT * comm->len);
		if ((ret != 0 && ret != LIBUSB_ERROR_TIMEOUT) || transferred == 0)
			return 1;

		comm->cnt = transferred;
		comm->idx = 0;
	}
	*val = comm->buf[comm->idx];
	comm->idx++;
	return 0;
}
