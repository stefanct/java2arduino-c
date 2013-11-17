#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "string.h"
#include <pthread.h>
#include <sched.h>
#include "libusb-1.0/libusb.h"
#include "j2a_usb.h"

typedef struct j2a_usb_priv {
	libusb_device_handle *libusb_handle;
	pthread_t *pthread_rcv;
	pthread_mutex_t *mutex;
	pthread_cond_t *cond;
	enum thread_state state_rcv;
} j2a_usb_priv;

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
		fprintf(stderr, "%s: initializing libusb failed: %s\n", __func__, libusb_error_name(ret));
		return 1;
	} else
		return 0;
}

void j2a_usb_disconnect(j2a_handle *comm) {
	static pthread_mutex_t help = PTHREAD_MUTEX_INITIALIZER;
	if (pthread_mutex_trylock(&help) != 0)
		return;
	j2a_usb_priv *priv = comm->ctx;
	if (priv != NULL) {
		if (priv->mutex != NULL) {
			if (pthread_mutex_lock(priv->mutex) != 0)
				fprintf(stderr, "%s: locking mutex failed.\n", __func__);
			else {
				pthread_t *rcv = NULL;
				if (priv->cond != NULL) {
					if (priv->pthread_rcv != NULL) {
						rcv = priv->pthread_rcv;
						priv->pthread_rcv = NULL;
						priv->state_rcv = SHUTDOWN;
						pthread_cond_broadcast(priv->cond);
					}
				}
				if (pthread_mutex_unlock(priv->mutex) != 0) {
					fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
				}
				// To give potential readers a chance to unlock the mutex. They would handle the vanishing mutex fine anyway, but destroying held mutexes (and condition variables) is undefined.
				sched_yield();
				if (rcv != NULL) {
					if (!pthread_equal(pthread_self(), *rcv))
						pthread_join(*rcv, NULL);
					free(rcv);
				}
				if (priv->libusb_handle != NULL) {
					libusb_release_interface(priv->libusb_handle, 0);
					libusb_close(priv->libusb_handle);
					priv->libusb_handle = NULL;
				}
				if (priv->cond != NULL) {
					pthread_cond_destroy(priv->cond);
					free(priv->cond);
					priv->cond = NULL;
				}
			}
			pthread_mutex_destroy(priv->mutex);
			free(priv->mutex);
			priv->mutex = NULL;
		}

		free(priv);
		comm->ctx = NULL;
	}
	pthread_mutex_unlock(&help);
	pthread_mutex_destroy(&help);
}

static void *j2a_usb_receiver_thread(void *arg) {
	int ret;
	j2a_handle *comm = arg;
	j2a_usb_priv *priv = comm->ctx;

	ret = pthread_mutex_lock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		goto shutdown_threads;
	}

	priv->state_rcv = RUN;
	pthread_cond_broadcast(priv->cond);


	bool run = true;
	while (run) {

		ret = pthread_cond_wait(priv->cond, priv->mutex);
		if (priv->state_rcv == SHUTDOWN) {
			run = false;
		} else if (priv->state_rcv != RUN || ret != 0) {
			goto shutdown_threads_unlock;
		} else if (comm->rcvidx < comm->rcvlen) {
			fprintf(stderr, "%s: buffer not consumed, why did you wake me?\n", __func__);
		} else {
			int transferred;
			ret = libusb_bulk_transfer(priv->libusb_handle, A2J_USB_IN_ADDR, comm->rcvbuf, A2J_BUFFER, &transferred, 500);
			if (ret == 0) {
				comm->rcvidx = 0;
				comm->rcvlen = transferred;
				pthread_cond_broadcast(priv->cond);
			} else if (ret == LIBUSB_ERROR_TIMEOUT) {
				// reevaluate run conditions
			} else {
				goto shutdown_threads_unlock;
			}
		}

		pthread_cond_broadcast(priv->cond);

	}

	ret = pthread_mutex_unlock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		priv->state_rcv = ERROR;
		arg = NULL;
	} else if (arg == NULL) {
shutdown_threads_unlock:
		if (priv->mutex != NULL)
			ret = pthread_mutex_unlock(priv->mutex);
			if (ret != 0) {
				fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
			}
shutdown_threads:
		priv->state_rcv = ERROR;
		arg = NULL;
	}
	j2a_usb_disconnect(comm);
	return arg;
}

void *j2a_usb_connect(j2a_handle *comm, const char *nth_dev) {
	int nth = 0;
	if (nth_dev != NULL) {
		nth = strtol(nth_dev, NULL, 10);
		if (nth < 0)
			return NULL;
	}

	int ret;
	j2a_usb_priv *priv = malloc(sizeof(j2a_usb_priv));
	if (priv == NULL) {
		fprintf(stderr, "%s: mallocing j2a_usb_priv failed.\n", __func__);
		return NULL;
	}
	memset(priv, 0, sizeof(j2a_usb_priv));
	comm->ctx = priv;

	priv->mutex = malloc(sizeof(pthread_mutex_t));
	priv->cond = malloc(sizeof(pthread_cond_t));
	if (priv->mutex == NULL || priv->cond == NULL) {
		fprintf(stderr, "%s: mallocing private variables failed.\n", __func__);
		goto bail_out;
		return NULL;
	}

	ret = pthread_mutex_init(priv->mutex, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: initializing a mutex failed.\n", __func__);
		goto bail_out;
	}

	ret = pthread_cond_init(priv->cond, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: initializing a condition variable failed.\n", __func__);
		goto bail_out;
	}

	libusb_device **list;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	if (cnt < 0)
		goto bail_out;

	libusb_device_handle *handle = NULL;
	for (ssize_t i = 0; i < cnt; i++) {
		libusb_device *dev = list[i];
		if (isArduino(dev) && (nth-- == 0)) {
			ret = libusb_open(dev, &handle);
			if (ret != 0)
				fprintf(stderr, "%s: open failed: %s\n", __func__, libusb_error_name(ret));
			break;
		}
	}

	libusb_free_device_list(list, 1);

	if (handle == NULL)
		goto bail_out;


	priv->libusb_handle = handle;

	ret = libusb_claim_interface(handle, 0);
	if (ret != 0) {
		fprintf(stderr, "%s: claim failed: %s\n", __func__, libusb_error_name(ret));
		goto bail_out;
	}

	ret = pthread_mutex_lock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		goto bail_out;
	}

	priv->pthread_rcv = malloc(sizeof(pthread_t));
	if (priv->pthread_rcv == NULL) {
		fprintf(stderr, "%s: mallocing receiver thread context failed.\n", __func__);
		goto bail_out_unlock;
	}

	priv->state_rcv = STARTUP;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(priv->pthread_rcv, &attr, &j2a_usb_receiver_thread, comm);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "%s: creating receiver thread failed.\n", __func__);
		goto bail_out_unlock;
	}

	while (priv->state_rcv == STARTUP && ret == 0) {
		ret = pthread_cond_wait(priv->cond, priv->mutex);
	}
	if (priv->state_rcv != RUN || ret != 0) {
		fprintf(stderr, "%s: receive thread died on creation.\n", __func__);
		goto bail_out_unlock;
	}

	ret = pthread_mutex_unlock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		goto bail_out;
	}

	return priv;

bail_out_unlock:
	if (priv->mutex != NULL)
		pthread_mutex_unlock(priv->mutex);
bail_out:
		j2a_usb_disconnect(comm);
		return NULL;
}

void j2a_usb_shutdown(void) {
	libusb_exit(NULL);
}

uint8_t j2a_usb_write(j2a_handle *comm, uint8_t val) {
	if (comm->sendidx >= sizeof(comm->sendbuf))
		if (j2a_usb_flush(comm) != 0)
			return 1;
	comm->sendbuf[comm->sendidx] = val;
	comm->sendidx++;
	return 0;
}

uint8_t j2a_usb_flush(j2a_handle *comm) {
	j2a_usb_priv *priv = comm->ctx;
	int transferred;
	int ret = libusb_bulk_transfer(priv->libusb_handle, A2J_USB_OUT_ADDR, comm->sendbuf, comm->sendidx, &transferred, A2J_TIMEOUT * comm->sendlen);
	uint8_t err = !((ret == 0 || ret == LIBUSB_ERROR_TIMEOUT) && transferred == (int)comm->sendidx);
	comm->sendidx = 0;
	comm->sendlen = 0;
	return err;
}

uint8_t j2a_usb_read(j2a_handle *comm, uint8_t *val) {
	uint8_t retval = 0;
	j2a_usb_priv *priv = comm->ctx;
	int ret = pthread_mutex_lock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		return 1;
	}

	while (comm->rcvidx >= comm->rcvlen) {
		if (priv->state_rcv != RUN) {
			if (priv->state_rcv != SHUTDOWN)
				fprintf(stderr, "%s: threads not ready.\n", __func__);
			retval = 1;
			goto out_unlock;
		}
		pthread_cond_broadcast(priv->cond);
		pthread_cond_wait(priv->cond, priv->mutex);
	}
	*val = comm->rcvbuf[comm->rcvidx];
	comm->rcvidx++;
out_unlock:
	if (priv->mutex != NULL) {
		ret = pthread_mutex_unlock(priv->mutex);
		if (ret != 0) {
			fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
			retval = 1;
		}
	}

	return retval;
}
