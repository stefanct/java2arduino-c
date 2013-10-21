#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
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

j2a_handle *j2a_connect(const char *dev) {
	unsigned int num_comms = ARRAY_SIZE(kinds);
	for (unsigned int i = 0; i < num_comms; i++) {
		void *ctx = kinds[i].connect(dev);
		if (ctx != NULL) {
			j2a_handle *comm = malloc(sizeof(j2a_handle));
			if (comm == NULL) {
				kinds[i].disconnect(comm);
				return NULL;
			}
			comm->kind = &kinds[i];
			comm->ctx = ctx;
			comm->buf = malloc(A2J_BUFFER);
			if (comm->buf == NULL) {
				free(comm);
				return NULL;
			}
			comm->seq = 0;

			comm->len = A2J_BUFFER;
			comm->idx = 0;
			comm->cnt = 0;

			comm->propmap = NULL;
			comm->propcnt = 0;
			comm->funcmap = NULL;
			comm->funccnt = 0;
			return comm;
		}
	}
	return NULL;
}

void j2a_disconnect(j2a_handle *comm) {
	if (comm != NULL) {
		if (comm->kind != NULL) {
			if (comm->kind->disconnect != NULL)
				comm->kind->disconnect(comm);
		}
		if (comm->funcmap != NULL)
			free_funcmap(comm);
		if (comm->propmap != NULL)
			free_propmap(comm);

		free(comm->buf);
		free(comm);
	}
}

static uint8_t writeByte(j2a_handle *comm, uint8_t data) {
	const struct j2a_kind *kind = comm->kind;
	if (data == A2J_SOF || data == A2J_ESC) {
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

static uint8_t readByte(j2a_handle *comm, uint8_t *val) {
	if (comm->kind->read(comm, val) != 0)
		return 1;
	if (*val == A2J_SOF)
		return 1; // Unescaped delimiter character inside frame;
	if (*val == A2J_ESC) {
		if (comm->kind->read(comm, val) != 0)
			return 1;
		*val += 1;
	}
	return 0;
}

void j2a_print_packet(const j2a_packet *p) {
	printf("p->cmd=%d, p->len=%d\n", p->cmd, p->len);
	printf("cmd=%d (0x%02x), ", p->cmd, p->cmd);
	if(p->len > 0) {
		printf("\n");
		for(unsigned int i = 0; i < p->len; i++)
			printf("msg[%d]=0x%02x\n", i, p->msg[i]);

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
	comm->idx = 0;
	comm->cnt = 0;
	uint8_t cur_seq = comm->seq;
	comm->seq++;
	uint8_t cSum = cur_seq;
	cSum ^= A2J_CRC_CMD + p->cmd;
	cSum ^= A2J_CRC_LEN + p->len;

	if (comm->kind->write(comm, A2J_SOF) != 0)
		return 1;
	if (writeByte(comm, cur_seq) != 0)
		return 2;
	if (writeByte(comm, p->cmd) != 0)
		return 3;
	if (writeByte(comm, p->len) != 0)
		return 4;

	for (unsigned int i = 0; i < p->len; i++) {
		uint8_t tmp = p->msg[i];
		if (writeByte(comm, tmp) != 0)
			return 5;
		cSum ^= tmp;
	}
	if (writeByte(comm, cSum) != 0)
		return 6;

	if (comm->kind->flush(comm) != 0)
		return 7;

	// writing done, receiving...
	comm->idx = 0;
	comm->cnt = 0;

	uint8_t val;
	while (true) {
		uint8_t ret = comm->kind->read(comm, &val);
		if (val == A2J_SOF)
			break;
		if (ret != 0)
			return 10;
	}
	if (readByte(comm, &val) != 0)
		return 11;
	if (cur_seq != val)
		return 12;
	if (readByte(comm, &p->cmd) != 0)
		return 13;
	if (readByte(comm, &p->len) != 0)
		return 14;
	cSum = cur_seq;
	cSum ^= A2J_CRC_CMD + p->cmd;
	cSum ^= A2J_CRC_LEN + p->len;

	for (unsigned int i = 0; i < p->len; i++) {
		if (readByte(comm, &val) != 0)
			return 15;
		p->msg[i] = val;
		cSum ^= val;
	}
	if (readByte(comm, &val) != 0)
		return 16;
	if (val != cSum) {
		return 17; // Checksum of received frame mismatched
	}

	switch (p->cmd) {
		case A2J_RET_OOB:
			return 18; // Function offset was out of bounds
		case A2J_RET_TO:
			//int line = ((msg[0] & 0xff)<<8) + (msg[1] & 0xff);
			return 19; // Timeout while peer was receiving around line \c line
		case A2J_RET_CHKSUM:
			return 20; // Checksum of sent frame mismatched
	}

	return 0;
}
