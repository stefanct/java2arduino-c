#define _POSIX_C_SOURCE 200809L /* for strdup */
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
	pthread_mutex_t *cleanup_mutex;
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
	j2a_usb_priv *priv = comm->ctx;
	if (priv == NULL)
		return;

	if (priv->cleanup_mutex == NULL || pthread_mutex_trylock(priv->cleanup_mutex) != 0)
		return;
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

	pthread_mutex_t *tmp = priv->cleanup_mutex;
	free(priv);
	comm->ctx = NULL;
	pthread_mutex_unlock(tmp);
	free(tmp);
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

static libusb_device_handle *j2a_usb_get_device(int16_t bus_num, int16_t dev_addr) {
	libusb_device **list;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	if (cnt < 0)
		goto cleanup;

	libusb_device_handle *handle = NULL;
	for (ssize_t i = 0; i < cnt; i++) {
		libusb_device *dev = list[i];
		int16_t cur_bus_num = libusb_get_bus_number(dev);
		int16_t cur_dev_addr = libusb_get_device_address(dev);

		if (bus_num >= 0) {
			if (bus_num != cur_bus_num)
				continue;
		}

		if (dev_addr >= 0) {
			if (dev_addr != cur_dev_addr)
				continue;
		}

		if (isArduino(dev)) {
			int ret = libusb_open(dev, &handle);
			if (ret != 0)
				fprintf(stderr, "%s: open failed: %s\n", __func__, libusb_error_name(ret));
			break;
		}
	}

cleanup:
	libusb_free_device_list(list, 1);
	return handle;
}

static int j2a_usb_connect_int(j2a_handle *comm, int16_t bus_num, int16_t dev_addr) {
	j2a_usb_priv *priv = calloc(1, sizeof(j2a_usb_priv));
	if (priv == NULL) {
		fprintf(stderr, "%s: mallocing j2a_usb_priv failed.\n", __func__);
		return -1;
	}

	priv->cleanup_mutex = malloc(sizeof(pthread_mutex_t));
	if (priv->cleanup_mutex == NULL) {
		fprintf(stderr, "%s: mallocing the cleanup mutex failed.\n", __func__);
		free(priv);
		return -1;
	}

	int ret = pthread_mutex_init(priv->cleanup_mutex, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: initializing the cleanup mutex failed.\n", __func__);
		free(priv->cleanup_mutex);
		free(priv);
		return -1;
	}

	comm->ctx = priv;
	priv->mutex = malloc(sizeof(pthread_mutex_t));
	priv->cond = malloc(sizeof(pthread_cond_t));
	if (priv->mutex == NULL || priv->cond == NULL) {
		fprintf(stderr, "%s: mallocing private variables failed.\n", __func__);
		j2a_usb_disconnect(comm);
		return -1;
	}

	ret = pthread_mutex_init(priv->mutex, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: initializing a mutex failed.\n", __func__);
		j2a_usb_disconnect(comm);
		return -1;
	}

	ret = pthread_cond_init(priv->cond, NULL);
	if (ret != 0) {
		fprintf(stderr, "%s: initializing a condition variable failed.\n", __func__);
		j2a_usb_disconnect(comm);
		return -1;
	}

	libusb_device_handle *handle = j2a_usb_get_device(bus_num, dev_addr);
	if (handle == NULL) { // no compatible device found
		j2a_usb_disconnect(comm);
		return 1;
	}

	priv->libusb_handle = handle;

	ret = libusb_claim_interface(handle, 0);
	if (ret != 0) {
		fprintf(stderr, "%s: claim failed: %s\n", __func__, libusb_error_name(ret));
		j2a_usb_disconnect(comm);
		return -1;
	}

	ret = pthread_mutex_lock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		j2a_usb_disconnect(comm);
		return -1;
	}

	priv->pthread_rcv = malloc(sizeof(pthread_t));
	if (priv->pthread_rcv == NULL) {
		fprintf(stderr, "%s: mallocing receiver thread context failed.\n", __func__);
		goto bailout;
	}

	priv->state_rcv = STARTUP;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	ret = pthread_create(priv->pthread_rcv, &attr, &j2a_usb_receiver_thread, comm);
	pthread_attr_destroy(&attr);
	if (ret != 0) {
		fprintf(stderr, "%s: creating receiver thread failed.\n", __func__);
		goto bailout;
	}

	while (priv->state_rcv == STARTUP && ret == 0) {
		ret = pthread_cond_wait(priv->cond, priv->mutex);
	}
	if (priv->state_rcv != RUN || ret != 0) {
		fprintf(stderr, "%s: receive thread died on creation.\n", __func__);
		goto bailout;
	}

	ret = pthread_mutex_unlock(priv->mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		j2a_usb_disconnect(comm);
		return -1;
	}

	return 0;

bailout:
	if (priv->mutex != NULL)
		pthread_mutex_unlock(priv->mutex);
	j2a_usb_disconnect(comm);
	return -1;
}

int j2a_usb_connect_all(j2a_handle ***handlesp, int *lenp) {
	libusb_device **list;
	ssize_t cnt = libusb_get_device_list(NULL, &list);
	if (cnt < 0)
		return -1;

	int tmp = 0;
	int ret = 0;
	j2a_handle *handle = NULL;
	j2a_handle **handles = *handlesp;
	int len = *lenp;
	for (ssize_t d = 0; d < cnt; d++) {
		libusb_device *dev = list[d];
		int16_t bus_num = libusb_get_bus_number(dev);
		int16_t dev_addr = libusb_get_device_address(dev);

		if (handle == NULL) { // reuse if previous device did not match
			handle = malloc(sizeof(j2a_handle));
			if (handle == NULL) {
				ret = -1;
				goto cleanup;
			}
		}
		tmp = j2a_usb_connect_int(handle, bus_num, dev_addr);
		if (tmp > 0)
			continue;
		
		if (tmp < 0) {
			fprintf(stderr, "Initializing handle failed.\n");
			ret = -1;
			goto cleanup;
		}

		// allocate the array (of pointers to structs) if there is none yet
		if (handles == NULL) {
			handles = malloc(sizeof(j2a_handle **));
			if (handles == NULL) {
				ret = -1;
				goto cleanup;
			}
			*handlesp = handles;
		}

		len++;
		// resize the array to hold len pointers
		j2a_handle **tmphandles = realloc(handles, len * sizeof(j2a_handle *));
		if (tmphandles == NULL) {
			ret = -1;
			goto cleanup;
		}
		handles = tmphandles;

		// finally add the new handle to the array
		handles[len - 1] = handle;
		//*(handles + sizeof(j2a_handle *) * (len - 1)) = handle;
		handle = NULL;
		ret++;
	}

	*lenp = len;
	*handlesp = handles;
cleanup:
	free(handle);
	libusb_free_device_list(list, 1);
	return ret;
}

int j2a_usb_connect(j2a_handle *comm, const char *addr) {
	int16_t bus_num = -1;
	int16_t dev_addr = -1;

	{
		int temp;
		char *busptr, *endptr;

		char *tmp_str = strdup(addr);
		if (tmp_str == NULL)
			return -1;

		busptr = strtok(tmp_str, ":");
		if (busptr != NULL)  {
			temp = strtol(busptr, &endptr, 10);
			if (*endptr || temp < 0 || temp > 255) {
				fprintf(stderr, "%s: Illegal bus number: '%s'\n", __func__, busptr);
				free(tmp_str);
				return 1;
			}
			bus_num = (int16_t)temp;

			temp = strtol(tmp_str, &endptr, 10);
			if (*endptr || temp < 0 || temp > 127) {
				fprintf(stderr, "%s: Illegal device number: '%s'\n", __func__, tmp_str);
				free(tmp_str);
				return 1;
			}
			dev_addr = (int16_t)temp;
		}
	}

	return j2a_usb_connect_int(comm, bus_num, dev_addr);
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
