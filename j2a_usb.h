#ifndef J2A_USB_H
#define J2A_USB_H

#include "j2a.h"

void *j2a_usb_connect(const char *nth_dev);
void j2a_usb_disconnect(struct j2a_handle *comm);
uint8_t j2a_usb_init(void);
void j2a_usb_shutdown(void);
uint8_t j2a_usb_read(struct j2a_handle *comm, uint8_t *val);
uint8_t j2a_usb_write(struct j2a_handle *comm, uint8_t val);
uint8_t j2a_usb_flush(struct j2a_handle *comm);

#endif // J2A_USB_H
