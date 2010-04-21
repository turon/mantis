//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"

#include "net.h"
#include "deluge.h"
#include "node_id.h"
#include "command_daemon.h"
#include "msched.h"
#include "sem.h"
#include "clock.h"
#include "mutex.h"
#include "printf.h"
#include "simple_fs.h"
#include "led.h"

#define TOPO_PING 1
#define TOPO_REPLY 2

static uint8_t send_stack[192];
static uint8_t commander_stack[MOS_COMMANDER_STACK_SIZE];

typedef struct {
	uint8_t type;
	uint8_t seqNo;
	uint16_t origin;
	uint16_t route;
	int16_t neighbors[28];
} __attribute__ ((aligned (1), packed)) topo_pkt;

uint8_t seqNo;
uint8_t neighbor_count;
comBuf npkt;
topo_pkt *out = (topo_pkt*)npkt.data;
mos_sem_t send_sem;
mos_alarm_t send_alarm;
uint16_t rseed;
mos_mutex_t send_lock;

int16_t reply_cache[10];
uint8_t reply_ndx;

static void reinit()
{
	uint8_t i;
	mos_mutex_lock(&send_lock);
	for (i = 0; i<28; i++) {
		out->neighbors[i] = -1;
	}
	neighbor_count = 0;
	mos_led_display(neighbor_count);
	for (i = 0; i<10; i++) {
		reply_cache[i] = -1;
	}
	reply_ndx = 0;
	mos_mutex_unlock(&send_lock);
}

static void checkNeighbor(uint16_t id)
{
	uint8_t i;
	mos_mutex_lock(&send_lock);
	for (i = 0; i<neighbor_count && i<28; i++) {
		if (id == out->neighbors[i]) {
			mos_mutex_unlock(&send_lock);
			return;
		}
	}
	if (i==neighbor_count && i<28) {
		mos_remove_alarm(&send_alarm);
		out->neighbors[neighbor_count++] = id;
		mos_led_display(neighbor_count);
		rseed = rand();
        send_alarm.msecs = 500 + rseed % 500;
		mos_alarm(&send_alarm);
	}
	mos_mutex_unlock(&send_lock);
}

void sendThread()
{
	while (1)
	{
		mos_sem_wait(&send_sem);
		mos_mutex_lock(&send_lock);
		out->origin = out->route = mos_node_id_get();
		out->type = TOPO_REPLY;
		out->seqNo = seqNo;
		npkt.size = 6 + 2*neighbor_count;
		net_send(&npkt, 77, 0);
		mos_mutex_unlock(&send_lock);
	}
}

void alarmCallback(void* arg)
{
	mos_sem_post(&send_sem);
}

/* Net device interface functions */

int8_t topo_send(comBuf *pkt, va_list args)
{
	return 0;
}

boolean topo_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
	topo_pkt* topo = (topo_pkt*)pkt->data;
	switch (topo->type)
	{
		case TOPO_PING: {
			if (topo->seqNo == seqNo) {
				checkNeighbor(topo->route);
			} else {
				reinit();
				seqNo = topo->seqNo;
				checkNeighbor(topo->route);
				topo->route = mos_node_id_get();
				net_send(pkt, 77, 0);
			}
		} break;
		case TOPO_REPLY: {
			checkNeighbor(topo->route);
			printf("%d: ", topo->origin);
			uint8_t i;
			for (i = 0; i<(pkt->size-6)/2; i++) {
				printf(" %d", topo->neighbors[i]);
			}
			printf("\n");
			
			if (topo->origin == mos_node_id_get())
				break;
			for (i = 0; i<10; i++) {
				if (topo->origin == reply_cache[i])
					break;
			}
			if (i == 10) {
				reply_cache[reply_ndx++] = topo->origin;
				reply_ndx %= 10;
				topo->route = mos_node_id_get();
				net_send(pkt, 77, 0);
			}
		} break;
	}
	return 0;
}

int8_t topo_ioctl(uint8_t request, va_list args)
{
	return -1;
}

/* Command daemon functions for debugging: */

void getTopo()
{
	reinit();
	seqNo++;
	out->origin = out->route = mos_node_id_get();
	out->type = TOPO_PING;
	out->seqNo = seqNo;
	
	npkt.size = 6;
	net_send(&npkt, 77, 0);
}

void start()
{
	mos_sem_init(&send_sem, 0);
	mos_mutex_init(&send_lock);
	
	reinit();
	seqNo = 0;
	rseed = mos_node_id_get();
	send_alarm.func = alarmCallback;
    send_alarm.reset_to = 0;

	simple_fs_init();
	net_init();
	deluge_init();
	net_proto_register(77, topo_send, topo_recv, topo_ioctl);
	
	mos_command_daemon_init();
	mos_register_function("getTopo", getTopo);
	mos_thread_new_havestack(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, commander_stack, PRIORITY_NORMAL);
	mos_thread_new_havestack(sendThread, 192, send_stack, PRIORITY_NORMAL);
}
