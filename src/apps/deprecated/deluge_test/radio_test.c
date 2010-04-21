//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "com.h"
#include "command_daemon.h"
#include "cc1000.h"
#include "mutex.h"
#include "cond.h"
#include "node_id.h"
#include "led.h"

extern uint16_t cc1000_crc_error_count;  //dropped packets due to crc error
extern uint16_t cc1000_mem_error_count;  //dropped packets due to out of memory
extern uint16_t cc1000_sync_error_count; //dropped packets due to sync failure
extern uint16_t cc1000_size_error_count; //dropped packets due to size out of range
#ifdef RADIO_USE_FEC
extern uint16_t cc1000_fec_error_count;  //dropped packets due to fec errors
#endif

static mos_mutex_t sendLock;
static mos_cond_t sendCond;
static uint8_t sending = 0;
static comBuf spkt;

static void sendThread (void)
{
	while(1)
	{
		mos_mutex_lock(&sendLock);
		if (!sending) {
			printf("Suspending send mode.\n");
			while (!sending)
				mos_cond_wait(&sendCond, &sendLock);
			printf("Entering send mode.\n");
		}
		mos_mutex_unlock(&sendLock);
		
		uint8_t i;
		for (i = 0; i<100; i++)
		{
			mos_led_toggle(1);
			spkt.data[0] = 0x43;
			*(int16_t*)&spkt.data[1] = -1;
			*(int16_t*)&spkt.data[3] = mos_node_id_get();
			
			uint8_t j;
			for (j=5; j<20; )
			{
				spkt.data[j] = i;
				j++;
				spkt.data[j] = j;
				j++;
			}
			spkt.size = j;
			com_send(IFACE_RADIO, &spkt);
			mos_thread_sleep(100);
		}
		printf("100\n");
	}
}

static void printErrors(void* arg)
{
	printf("mem: %d, sync: %d, size: %d, crc: %d",
		cc1000_mem_error_count, cc1000_sync_error_count, 
		cc1000_size_error_count, cc1000_crc_error_count);
	#ifdef RADIO_USE_FEC
	printf(", fec: %d", cc1000_fec_error_count);
	#endif
	printf("\n");
	
	cc1000_crc_error_count = 0;
	cc1000_mem_error_count = 0;
	cc1000_sync_error_count = 0;
	cc1000_size_error_count = 0;
	#ifdef RADIO_USE_FEC
	cc1000_fec_error_count = 0;
	#endif
}

static void recvThread()
{
	uint8_t lastpkt = 0;
	uint8_t npkt = 0;
	uint16_t lastnode = -1;
	
	while(1)
	{
		mos_led_toggle(0);
		comBuf* pkt = com_recv_timed(IFACE_RADIO, (uint32_t)12 * (uint32_t)TICKS_PER_SEC);
		if (pkt == NULL) {
			printErrors(NULL);
			continue;
		}
		uint8_t type = pkt->data[0];
		int16_t id = *(int16_t*)&pkt->data[1];
		
		if (id == -1 || id == mos_node_id_get())
		{
			switch (type)
			{
				case 0x43: {
					lastnode = *(int16_t*)&pkt->data[3];
					uint8_t seq = pkt->data[5];
					if (seq < lastpkt)		// rolled over
					{
						printf("%d%% from %d\n", npkt, lastnode);
						printErrors(NULL);
						npkt = 0;
					}
					lastpkt = seq;
					npkt++;
					break;
				}
				case 0x92:
					printf("Setting radio power to %C.\n", pkt->data[3]);
					com_ioctl(IFACE_RADIO, CC1000_TX_POWER, pkt->data[3]);
					break;
				case 0x87:
					printf("Switching to send mode.\n");
					mos_mutex_lock(&sendLock);
					sending = 1;
					mos_cond_broadcast(&sendCond);
					mos_mutex_unlock(&sendLock);
					break;
				case 0x57:
					printf("Switching out of send mode.\n");
					mos_mutex_lock(&sendLock);
					sending = 0;
					mos_mutex_unlock(&sendLock);
					break;
			}
		}
		com_free_buf(pkt);
	}
}

static void send_on()
{
	spkt.data[0] = 0x87;
	*(int16_t*)&spkt.data[1] = prompt_long("node: ");
	spkt.size = 3;
	
	if (*(int16_t*)&spkt.data[1] == mos_node_id_get()) {
		printf("Switching to send mode.\n");
		mos_mutex_lock(&sendLock);
		sending = 1;
		mos_cond_broadcast(&sendCond);
		mos_mutex_unlock(&sendLock);
		return;
	}
	
	// Temporarily switch to max power to make sure the command gets through
	uint8_t oldPower;
	com_ioctl(IFACE_RADIO, CC1000_GET_TX_POWER, &oldPower);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 0xFF);
	com_send(IFACE_RADIO, &spkt);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
}

static void send_off()
{
	spkt.data[0] = 0x57;
	*(int16_t*)&spkt.data[1] = prompt_long("node: ");
	spkt.size = 3;
	
	if (*(int16_t*)&spkt.data[1] == mos_node_id_get()) {
		printf("Switching out of send mode.\n");
		mos_mutex_lock(&sendLock);
		sending = 0;
		mos_mutex_unlock(&sendLock);
		return;
	}
	
	// Temporarily switch to max power to make sure the command gets through
	uint8_t oldPower;
	com_ioctl(IFACE_RADIO, CC1000_GET_TX_POWER, &oldPower);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 0xFF);
	com_send(IFACE_RADIO, &spkt);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
}

static void listen_on()
{
	com_mode(IFACE_RADIO, IF_LISTEN);
}

static void listen_off()
{
	com_mode(IFACE_RADIO, IF_OFF);
}

static void set_power()
{
	spkt.data[0] = 0x92;
	*(int16_t*)&spkt.data[1] = prompt_long("node: ");
	spkt.data[3] = prompt_uint8("power: ");
	spkt.size = 4;
	
	if (*(int16_t*)&spkt.data[1] == mos_node_id_get()) {
		printf("Setting radio power to %C.\n", spkt.data[3]);
		com_ioctl(IFACE_RADIO, CC1000_TX_POWER, spkt.data[3]);
		return;
	}
	
	// Temporarily switch to max power to make sure the command gets through
	uint8_t oldPower;
	com_ioctl(IFACE_RADIO, CC1000_GET_TX_POWER, &oldPower);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 0xFF);
	com_send(IFACE_RADIO, &spkt);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
}

void start(void)
{
	mos_led_display(4);
	mos_mutex_init(&sendLock);
	mos_cond_init(&sendCond);
	
	sending = 0;
	printErrors(NULL);
	listen_on();
	
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 2);
	mos_thread_new(recvThread, 192, PRIORITY_NORMAL);
	mos_thread_new(sendThread, 192, PRIORITY_NORMAL);

	mos_command_daemon_init();
	mos_register_function("on", send_on);
	mos_register_function("off", send_off);
	mos_register_function("listen_on", listen_on);
	mos_register_function("listen_off", listen_off);
	mos_register_function("power", set_power);
	mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
}
