#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include "j2a.h"
#include "j2a_usb.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

j2a_kind kinds[] = {
	{
		.init = j2a_usb_init,
		.connect = j2a_usb_connect,
		.disconnect = j2a_usb_disconnect,
		.shutdown = j2a_usb_shutdown,
		.read = j2a_usb_read,
		.write = j2a_usb_write,
		.flush = j2a_usb_flush,
	}
};

static void free_map(char **map, uint8_t len) {
	if (map != NULL) {
		for (size_t i = 0; i < len; i++)
			free(map[i]);
		free(map);
		map = NULL;
		len = 0;
	}
}

static void free_propmap(j2a_handle *comm) {
	free_map(comm->propmap, comm->propcnt*2);
	comm->propmap = NULL;
	comm->propcnt = 0;
}

static void free_funcmap(j2a_handle *comm) {
	free_map(comm->funcmap, comm->funccnt);
	comm->funcmap = NULL;
	comm->funccnt = 0;
}

uint8_t j2a_init(void) {
	size_t num_comms = ARRAY_SIZE(kinds);
	for (size_t i = 0; i < num_comms; i++) {
		if (kinds[i].init() != 0) {
			for (ssize_t j = i - 1; j >= 0; j--)
				kinds[i].shutdown();
			return 1;
		}
	}
	return 0;
}

void j2a_shutdown(void) {
	size_t num_comms = ARRAY_SIZE(kinds);
	for (size_t i = 0; i < num_comms; i++) {
		kinds[i].shutdown();
	}
}

int j2a_add_sif_handler(j2a_handle *comm, j2a_sif_handler *new) {
	static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
	if (pthread_mutex_lock(&mutex) != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		return 1;
	}

	int ret = 0;
	if (new->handle == NULL)
		ret = 1;
	else {
		j2a_sif_handler *old = comm->sif_handlers;
		new->next = old;
		comm->sif_handlers = new;
	}
	if (pthread_mutex_unlock(&mutex) != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		return ret;
	}
	return ret;
}

static void *handle_sif_packet(void *arg) {
	struct j2a_sif_packet *sif = arg;
	j2a_handle *comm = sif->comm;
	j2a_sif_handler *h = comm->sif_handlers;
	while (h != NULL) {
		if (sif->p.cmd == h->cmd)
			h->handle(sif);
		h = h->next;
	}
	free(sif);
	return NULL;
}

static uint8_t read_byte(j2a_handle *comm, uint8_t *val) {
	if (comm->kind->read(comm, val) != 0)
		return 1;
	if (*val == A2J_ESC) {
		if (comm->kind->read(comm, val) != 0)
			return 1;
		*val += 1;
	} else if (*val == A2J_SOF || *val == A2J_SOS || *val == A2J_ESC) {
		fprintf(stderr, "%s: Unescaped special character inside frame: 0x%02x.\n", __func__, *val);
		return 1;
	}
	return 0;
}

static int read_packet(j2a_handle *comm, j2a_packet *p, uint8_t *seq) {
	if (read_byte(comm, seq) != 0)
		return 10;

	uint8_t cSum = *seq;

	if (read_byte(comm, &p->cmd) != 0)
		return 12;

	if (read_byte(comm, &p->len) != 0)
		return 13;

	cSum ^= A2J_CRC_CMD + p->cmd;
	cSum ^= A2J_CRC_LEN + p->len;

	uint8_t val;
	for (unsigned int i = 0; i < p->len; i++) {
		if (read_byte(comm, &val) != 0)
			return 14;

		p->msg[i] = val;
		cSum ^= val;
	}

	if (read_byte(comm, &val) != 0)
		return 15;

	if (val != cSum)
		return 16; // Checksum of received frame mismatched

	int line = (p->msg[0]<< 8) + p->msg[1];
	switch (p->cmd) {
		case A2J_RET_OOB:
			fprintf(stderr, "%s: oob at line %d.\n", __func__, line);
			return 17; // Function offset was out of bounds
		case A2J_RET_TO: {
			fprintf(stderr, "%s: timeout at line %d.\n", __func__, line);
			return 18; // Timeout while peer was receiving around line \c line
		}
		case A2J_RET_CHKSUM:
			fprintf(stderr, "%s: sent packet did not match checksum.\n", __func__);
			return 19; // Checksum of sent frame mismatched
		case A2J_RET_ESC: {
			fprintf(stderr, "%s: unescaped special byte read at line %d.\n", __func__, line);
			return 19; // Unescaped byte around line \c line
		}
	}
	return 0;
}

static void *j2a_receiver_thread(void *arg) {
	j2a_handle *comm = arg;
	int ret = pthread_mutex_lock(comm->rcvmutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		comm->rcvstate = ERROR;
		return NULL;
	}

	comm->rcvstate = RUN;
	ret = pthread_cond_broadcast(comm->rcvcond);
	if (ret != 0) {
		fprintf(stderr, "%s: broadcasting run state failed.\n", __func__);
		goto shutdown_threads_unlock;
	}

	ret = pthread_mutex_unlock(comm->rcvmutex);
	if (ret != 0) {
		fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		goto shutdown_threads;
	}

	bool run = true;
	while (run) {
		ret = pthread_mutex_lock(comm->rcvmutex);
		if (ret != 0) {
			fprintf(stderr, "%s: locking mutex failed.\n", __func__);
			goto shutdown_threads;
		}
		if (comm->rcvstate == SHUTDOWN) {
			run = false;
			break;
		} else if (comm->rcvstate != RUN) {
			goto shutdown_threads_unlock;
		}
		ret = pthread_mutex_unlock(comm->rcvmutex);
		if (ret != 0) {
			fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
			goto shutdown_threads;
		}
		uint8_t val;

		ret = comm->kind->read(comm, &val);
		if (ret != 0) {
			goto shutdown_threads;
		}
		if (val == A2J_SOF) {
			ret = pthread_mutex_lock(comm->rcvmutex);
			if (ret != 0) {
				fprintf(stderr, "%s: locking mutex failed.\n", __func__);
				goto shutdown_threads;
			}
			comm->rcvpacket = NULL;
			pthread_cond_broadcast(comm->rcvcond);
			while (comm->rcvpacket == NULL) {
				ret = pthread_cond_wait(comm->rcvcond, comm->rcvmutex);
				if (comm->rcvstate == SHUTDOWN) {
					run = false;
					goto out;
				}
				if (comm->rcvstate != RUN || ret != 0)
					goto shutdown_threads_unlock;
			}
			ret = read_packet(comm, comm->rcvpacket, comm->rcvpacket_seq);
			if (ret == 0) {
				comm->rcvpacket_state = 1;
			} else {
				comm->rcvpacket_state = -1;
				fprintf(stderr, "%s: read_packet (SOF) failed: %d.\n", __func__, ret);
			}
			pthread_cond_broadcast(comm->rcvcond);
out:
			ret = pthread_mutex_unlock(comm->rcvmutex);
			if (ret != 0) {
				fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
				comm->rcvstate = ERROR;
			}
		} else if (val == A2J_SOS) {
			j2a_packet p;
			uint8_t seq;
			ret = read_packet(comm, &p, &seq);
			if (ret != 0)
				fprintf(stderr, "%s: read_packet (SOS) failed: %d.\n", __func__, ret);
			else {
				struct j2a_sif_packet *sif = malloc(sizeof(struct j2a_sif_packet));
				if (sif == NULL) {
					fprintf(stderr, "%s: out of memory.\n", __func__);
					continue;
				}
				sif->comm = comm;
				sif->p = p;
				sif->seq = seq;
				
				pthread_attr_t attr;
				pthread_attr_init(&attr);
				pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
				pthread_t sifthread;
				ret = pthread_create(&sifthread, &attr, &handle_sif_packet, sif);
				pthread_attr_destroy(&attr);
			}
		}
	}

	if (run == true) {
shutdown_threads_unlock:
		ret = pthread_mutex_unlock(comm->rcvmutex);
		if (ret != 0) {
			fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		}
shutdown_threads:
		comm->rcvstate = ERROR;
		arg = NULL;
	}

	return arg;
}

j2a_handle *j2a_connect(const char *dev) {
	j2a_handle *comm = malloc(sizeof(j2a_handle));
	if (comm == NULL)
		return NULL;

	unsigned int num_comms = ARRAY_SIZE(kinds);
	for (unsigned int i = 0; i < num_comms; i++) {
		void *ctx = kinds[i].connect(comm, dev);
		if (ctx != NULL) {
			comm->kind = &kinds[i];
			comm->curseq = 0;

			comm->rcvidx = 0;
			comm->rcvlen = 0;
			comm->sendidx = 0;
			comm->sendlen = 0;

			comm->propmap = NULL;
			comm->propcnt = 0;
			comm->funcmap = NULL;
			comm->funccnt = 0;
			comm->sif_handlers = NULL;
			comm->rcvpacket = (j2a_packet *)-1;
			comm->rcvmutex = malloc(sizeof(pthread_mutex_t));
			comm->rcvcond = malloc(sizeof(pthread_cond_t));
			if (comm->rcvmutex == NULL || comm->rcvcond == NULL) {
				fprintf(stderr, "%s: mallocing private variables failed.\n", __func__);
				goto bail_out;
			}
			int ret = pthread_cond_init(comm->rcvcond, NULL) || pthread_mutex_init(comm->rcvmutex, NULL);
			if (ret != 0) {
				fprintf(stderr, "%s: initializing private variables failed.\n", __func__);
				goto bail_out;
			}

			ret = pthread_mutex_lock(comm->rcvmutex);
			if (ret != 0) {
				fprintf(stderr, "%s: locking mutex failed.\n", __func__);
				goto bail_out;
			}

			comm->rcvthread = malloc(sizeof(pthread_t));
			if (comm->rcvthread == NULL) {
				fprintf(stderr, "%s: mallocing receiver thread context failed.\n", __func__);
				goto bail_out_unlock;
			}

			comm->rcvstate = STARTUP;
			pthread_attr_t attr;
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
			ret = pthread_create(comm->rcvthread, &attr, &j2a_receiver_thread, comm);
			pthread_attr_destroy(&attr);
			if (ret != 0) {
				fprintf(stderr, "%s: creating receiver thread failed.\n", __func__);
				goto bail_out_unlock;
			}

			while (comm->rcvstate == STARTUP && ret == 0) {
				ret = pthread_cond_wait(comm->rcvcond, comm->rcvmutex);
			}
			if (comm->rcvstate != RUN || ret != 0) {
				fprintf(stderr, "%s: receive thread did not startup correctly.\n", __func__);
				goto bail_out_unlock;
			}

			if (pthread_mutex_unlock(comm->rcvmutex) != 0) {
				fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
				goto bail_out;
			}

			return comm;
		}
	}
	if (false) {
bail_out_unlock:
		if (pthread_mutex_unlock(comm->rcvmutex) != 0) {
			fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		}
	}
bail_out:
	free(comm);
	return NULL;
}

void j2a_disconnect(j2a_handle *comm) {
	static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
	if (pthread_mutex_trylock(&mutex) != 0) {
		// great, another thread does the work already.
		return;
	}
	if (comm != NULL) {
		if (comm->kind != NULL && comm->kind->disconnect != NULL)
				comm->kind->disconnect(comm);
		if (comm->rcvmutex != NULL) {
			pthread_t *rcv = NULL;
			if (pthread_mutex_lock(comm->rcvmutex) != 0)
				fprintf(stderr, "%s: locking mutex failed.\n", __func__);
			else {
				if (comm->rcvcond != NULL) {
					if (comm->rcvthread != NULL) {
						rcv = comm->rcvthread;
						comm->rcvthread = NULL;
						comm->rcvstate = SHUTDOWN;
						pthread_cond_broadcast(comm->rcvcond);
					}
				}
				if (pthread_mutex_unlock(comm->rcvmutex) != 0) {
					fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
				}
			}

			if (rcv != NULL) {
				if (!pthread_equal(pthread_self(), *rcv))
					pthread_join(*rcv, NULL);
				free(rcv);
			}

			pthread_mutex_destroy(comm->rcvmutex);
			free(comm->rcvmutex);
			comm->rcvmutex = NULL;
			if (comm->rcvcond != NULL) {
				free(comm->rcvcond);
				comm->rcvcond = NULL;
			}

		}
		if (comm->funcmap != NULL)
			free_funcmap(comm);
		if (comm->propmap != NULL)
			free_propmap(comm);

		free(comm);
	}
	pthread_mutex_unlock(&mutex);
	pthread_mutex_destroy(&mutex);
}

static uint8_t writeByte(j2a_handle *comm, uint8_t data) {
	const struct j2a_kind *kind = comm->kind;
	if (data == A2J_SOF || data == A2J_SOS || data == A2J_ESC) {
		if (kind->write(comm, A2J_ESC) != 0)
			return 1;
		if (kind->write(comm, data - 1) != 0)
			return 1;
	} else {
		if (kind->write(comm, data) != 0)
			return 1;
	}
	return 0;
}

void j2a_print_packet(const j2a_packet *p) {
	printf("cmd=%d (0x%02x), ", p->cmd, p->cmd);
	if(p->len > 0) {
		printf("p->len=%d\n", p->len);
		for(unsigned int i = 0; i < p->len; i++) {
			printf("msg[%d]=0x%02x", i, p->msg[i]);
			if (isprint(p->msg[i]))
				printf(" (%c)\n", p->msg[i]);
			else
				printf("\n");
		}
	} else
		printf("msg == null\n");
}

uint8_t j2a_fetch_props(j2a_handle *comm) {
	if (comm->propmap != NULL)
		return 0;

	j2a_packet p;
	p.len = 0;
	if (j2a_send_by_name(comm, &p, "a2jGetProperties") != 0)
		return 1;

	char *ptr = (char *)p.msg;
	size_t todo = p.len;
	char *first = NULL;
	while (ptr < (char *)(p.msg + p.len)) {
		size_t curlen = strnlen(ptr, todo);

		if (curlen > 0) {
			if (comm->propcnt == UINT8_MAX) {
				free(first);
				free_propmap(comm);
				return 1;
			}
			/* This would be valid but let's be forgiving:
			 * the last string does not need to be null-terminated,
			 * we know the end of the payload anyway.
			if (curlen == todo) {
				free(first);
				free_propmap(comm);
				return 1;
			} */
			char *curstr = malloc(curlen + 1);
			if (curstr == NULL) {
				free(first);
				free_propmap(comm);
				return 1;
			}

			strncpy(curstr, ptr, curlen);
			curstr[curlen] = '\0';

			if (first == NULL) {
				first = curstr;
			} else {
				char **tmp = realloc(comm->propmap, sizeof(char *) * (comm->propcnt * 2 + 2));
				if (tmp == NULL) {
					free(first);
					free(curstr);
					free_propmap(comm);
					return 1;
				}
				comm->propmap = tmp;

				comm->propmap[comm->propcnt * 2] = first;
				comm->propmap[comm->propcnt * 2 + 1] = curstr;
				comm->propcnt++;
				first = NULL;
			}
		}
		curlen++;
		ptr += curlen;
		todo -= curlen;
	}

	if (first != NULL) { // key without value
		free(first);
		free_propmap(comm);
		return 1;
	}

	return 0;
}

char *j2a_get_prop(j2a_handle *comm, const char *name) {
	if (j2a_fetch_props(comm) != 0)
		return NULL;
	for (size_t i = 0; i < comm->propcnt; i++) {
		if (strcmp(name, comm->propmap[i * 2]) == 0)
			return comm->propmap[i * 2 + 1];
	}
	return NULL;
}

void j2a_print_propmap(j2a_handle *comm, FILE *stream) {
	fprintf(stream, "Properties map with %d entries:\n", comm->propcnt);
	for (size_t i = 0; i < comm->propcnt; i++) {
		fprintf(stream, "%s→%s\n", comm->propmap[i * 2], comm->propmap[i * 2 + 1]);
	}
}

uint8_t j2a_fetch_funcmap(j2a_handle *comm) {
	if (comm->funcmap != NULL)
		return 0;

	j2a_packet p;
	p.cmd = 0;
	p.len = 0;
	if (j2a_send(comm, &p) != 0)
		return 1;

	char *ptr = (char *)p.msg;
	size_t todo = p.len;
	while (ptr < (char *)(p.msg + p.len)) {
		size_t curlen = strnlen(ptr, todo);
		if (curlen > 0) {
			if (comm->funccnt == UINT8_MAX) {
				free_funcmap(comm);
				return 1;
			}
			/* This would be valid but let's be forgiving:
			 * the last string does not need to be null-terminated,
			 * we know the end of the payload anyway.
			if (curlen == todo) {
				free_funcmap(comm);
				return 1;
			} */
			char *curstr = malloc(curlen + 1);
			if (curstr == NULL) {
				free_funcmap(comm);
				return 1;
			}

			strncpy(curstr, ptr, curlen);
			curstr[curlen] = '\0';

			char **tmp = realloc(comm->funcmap, sizeof(char *) * (comm->funccnt + 1));
			if (tmp == NULL) {
				free(curstr);
				free_funcmap(comm);
				return 1;
			}
			comm->funcmap = tmp;

			comm->funcmap[comm->funccnt] = curstr;
			comm->funccnt++;
		}
		curlen++;
		ptr += curlen;
		todo -= curlen;
	}

	return 0;
}

void j2a_print_funcmap(j2a_handle *comm, FILE *stream) {
	fprintf(stream, "Function name map with %d entries:\n", comm->funccnt);
	for (size_t i = 0; i < comm->funccnt; i++) {
		fprintf(stream, "%s→%zd\n", comm->funcmap[i], i);
	}
}

static uint8_t get_funcidx(j2a_handle *comm, const char *name) {
	if (j2a_fetch_funcmap(comm) != 0 || name == NULL)
		return UINT8_MAX;

	for (size_t i = 0; i < comm->funccnt; i++) {
		if (strcmp(name, comm->funcmap[i]) == 0)
			return i;
	}
	return UINT8_MAX;
}

uint8_t j2a_send_by_name(j2a_handle *comm, j2a_packet *p, const char *func_name) {
	uint8_t idx = get_funcidx(comm, func_name);
	if (idx == UINT8_MAX)
		return 1;

	p->cmd = idx;
	return j2a_send(comm, p);
}

uint8_t j2a_send(j2a_handle *comm, j2a_packet *p) {
	uint8_t ret = 0;

	static pthread_mutex_t send_mutex = PTHREAD_MUTEX_INITIALIZER;
	ret = pthread_mutex_lock(&send_mutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		return 1;
	}
	comm->sendidx = 0;
	comm->sendlen = 0;
	if (comm->kind->write(comm, A2J_SOF) != 0) {
		return 2;
	}
	uint8_t cur_seq = comm->curseq;
	if (writeByte(comm, cur_seq) != 0) {
		return 3;
	}
	if (writeByte(comm, p->cmd) != 0) {
		return 4;
	}
	if (writeByte(comm, p->len) != 0) {
		return 5;
	}

	uint8_t cSum = cur_seq;
	cSum ^= A2J_CRC_CMD + p->cmd;
	cSum ^= A2J_CRC_LEN + p->len;

	for (unsigned int i = 0; i < p->len; i++) {
		uint8_t tmp = p->msg[i];
		if (writeByte(comm, tmp) != 0) {
			return 6;
		}
		cSum ^= tmp;
	}
	if (writeByte(comm, cSum) != 0) {
		return 7;
	}

	if (comm->kind->flush(comm) != 0) {
		return 8;
	}

	// writing done, receiving...
	ret = pthread_mutex_lock(comm->rcvmutex);
	if (ret != 0) {
		fprintf(stderr, "%s: locking mutex failed.\n", __func__);
		return 10;
	}

	/* wait till receive thread is ready */
	while (comm->rcvpacket != NULL) {
		ret = pthread_cond_wait(comm->rcvcond, comm->rcvmutex);
		if (ret != 0) {
			fprintf(stderr, "%s: receive thread did not wake us up correctly.\n", __func__);
			ret = 11;
			goto out_unlock;
		}
	}

	/* initialize rcvpacket variables and notify receive thread to continue */
	comm->rcvpacket = p;
	comm->rcvpacket_state = 0;
	uint8_t received_seq;
	comm->rcvpacket_seq = &received_seq;
	pthread_cond_broadcast(comm->rcvcond);

	/* wait till receiving completes */
	while (comm->rcvpacket == p && comm->rcvpacket_state == 0) {
		ret = pthread_cond_wait(comm->rcvcond, comm->rcvmutex);
		if (ret != 0) {
			fprintf(stderr, "%s: receive thread did not wake us up correctly.\n", __func__);
			ret = 12;
			goto out_unlock;
		}
	}
	if (comm->rcvpacket != p) {
		fprintf(stderr, "%s: current receive packet is not ours!?\n", __func__);
		ret = 13;
		goto out_unlock;
	}

	if (comm->rcvpacket_state < 0) {
		fprintf(stderr, "%s: receiving packet from receive thread failed: %d\n", __func__, comm->rcvpacket_state);
		ret = 14;
		goto out_unlock;
	}

	if (cur_seq != received_seq) {
		fprintf(stderr, "%s: received packet has sequence number %d instead of %d.\n", __func__, received_seq, cur_seq);
		ret = 15;
		goto out_unlock;
	}

	comm->curseq++;
out_unlock:
	if (pthread_mutex_unlock(comm->rcvmutex) != 0 || pthread_mutex_unlock(&send_mutex) != 0) {
		fprintf(stderr, "%s: unlocking mutex failed.\n", __func__);
		ret = 20;
	}

	return ret;
}
