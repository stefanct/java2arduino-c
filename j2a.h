#ifndef J2A_H
#define J2A_H

#include <stdint.h>
#include "j2a_const.h"

typedef struct j2a_packet {
	uint8_t cmd;
	uint8_t msg[A2J_MAX_PAYLOAD];
	uint8_t len;
} j2a_packet;

typedef struct j2a_handle {
	struct j2a_kind *kind;
	void *ctx;
	uint8_t seq;
	char **funcmap;
	uint8_t funccnt;
	char **propmap;
	uint8_t propcnt;
	uint8_t *buf;
	size_t len;
	size_t idx; /**< index of the first free/unread element of buf */
	size_t cnt; /**< number of valid elements in buf */
} j2a_handle;

typedef struct j2a_kind {
	uint8_t (*init)(void);
	void *(*connect)(const char *dev);
	void (*disconnect)(struct j2a_handle *comm);
	void (*shutdown)(void);
	uint8_t (*read)(struct j2a_handle *comm, uint8_t *val);
	uint8_t (*write)(struct j2a_handle *comm, uint8_t val);
	uint8_t (*flush)(struct j2a_handle *comm);
} j2a_kind;

uint8_t j2a_init(void);
struct j2a_handle *j2a_connect(const char *dev);
void j2a_disconnect(struct j2a_handle *comm);
void j2a_shutdown(void);
uint8_t j2a_fetch_props(struct j2a_handle *comm);
const char * const j2a_get_prop(struct j2a_handle *comm, const char *name);
void j2a_print_propmap(struct j2a_handle *comm, FILE *stream);
uint8_t j2a_fetch_funcmap(struct j2a_handle *comm);
void j2a_print_funcmap(struct j2a_handle *comm, FILE *stream);
uint8_t j2a_send(struct j2a_handle *comm, struct j2a_packet *p);
uint8_t j2a_send_by_name(struct j2a_handle *comm, struct j2a_packet *p, const char *funcName);
void j2a_packet_print(const struct j2a_packet *p);

#endif // J2A_H
