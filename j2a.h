#ifndef J2A_H
#define J2A_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "j2a_const.h"

typedef struct j2a_packet {
	uint8_t cmd;
	uint8_t len;
	uint8_t msg[A2J_BUFFER];
} j2a_packet;

struct j2a_handle;

struct j2a_sif_packet {
	struct j2a_handle *comm;
	j2a_packet p;
	uint8_t seq;
};

typedef struct j2a_sif_handler {
	uint8_t cmd;
	void (*handle)(struct j2a_sif_packet *sif);
	struct j2a_sif_handler *next;
} j2a_sif_handler;

enum thread_state {ERROR, STARTUP, RUN, SHUTDOWN};

typedef struct j2a_handle {
	struct j2a_kind *kind;
	void *ctx;
	uint8_t curseq;
	char **funcmap;
	uint8_t funccnt;
	char **propmap;
	uint8_t propcnt;
	pthread_t *rcvthread;
	enum thread_state rcvstate;
	pthread_mutex_t *rcvmutex;
	pthread_cond_t *rcvcond;
	uint8_t rcvbuf[A2J_BUFFER];
	size_t rcvidx; /**< index of the first unread element of rcvbuf */
	size_t rcvlen; /**< number of valid elements in rcvbuf */
	j2a_packet *rcvpacket;
	int rcvpacket_state; /**< status of rcvpacket. 0 undefined (yet), -1 error, 1 done. */
	uint8_t *rcvpacket_seq;
	j2a_sif_handler *sif_handlers;
	uint8_t sendbuf[A2J_BUFFER];
	size_t sendidx; /**< index of the first free element of sendbuf */
	size_t sendlen; /**< number of valid elements in sendbuf */
} j2a_handle;

typedef struct j2a_kind {
	uint8_t (*init)(void);
	void *(*connect)(j2a_handle *comm, const char *dev);
	void (*disconnect)(j2a_handle *comm);
	void (*shutdown)(void);
	uint8_t (*read)(j2a_handle *comm, uint8_t *val);
	uint8_t (*write)(j2a_handle *comm, uint8_t val);
	uint8_t (*flush)(j2a_handle *comm);
} j2a_kind;

uint8_t j2a_init(void);
j2a_handle *j2a_connect(const char *dev);
void j2a_disconnect(j2a_handle *comm);
void j2a_shutdown(void);
int j2a_add_sif_handler(j2a_handle *comm, j2a_sif_handler *new);
uint8_t j2a_fetch_props(j2a_handle *comm);
char *j2a_get_prop(j2a_handle *comm, const char *name);
void j2a_print_propmap(j2a_handle *comm, FILE *stream);
uint8_t j2a_fetch_funcmap(j2a_handle *comm);
void j2a_print_funcmap(j2a_handle *comm, FILE *stream);
uint8_t j2a_send(j2a_handle *comm, j2a_packet *p);
uint8_t j2a_send_by_name(j2a_handle *comm, j2a_packet *p, const char *funcName);
void j2a_print_packet(const j2a_packet *p);
void j2a_receive(j2a_handle *comm);

#ifndef min
#define min(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef max
#define max(x,y) ((x) > (y) ? (x) : (y))
#endif
#endif // J2A_H
