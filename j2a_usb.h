#ifndef J2A_USB_H
#define J2A_USB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "j2a.h"

void *j2a_usb_connect(j2a_handle *comm, const char *nth_dev);
void j2a_usb_disconnect(j2a_handle *comm);
uint8_t j2a_usb_init(void);
void j2a_usb_shutdown(void);
uint8_t j2a_usb_read(j2a_handle *comm, uint8_t *val);
uint8_t j2a_usb_write(j2a_handle *comm, uint8_t val);
uint8_t j2a_usb_flush(j2a_handle *comm);

#ifdef __cplusplus
}
#endif

#endif // J2A_USB_H
