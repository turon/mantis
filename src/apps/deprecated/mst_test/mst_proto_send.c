//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/*   File: mst_proto_send.c                                               */
/* Author: Anmol Sheth   :  anmol.sheth@colorado.edu                      */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* This is a simple program that demonstrates the ability of the net      */
/* Layer.                                                                 */
/**************************************************************************/
//
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "net_mst.h"
#include "uart.h"
#include "node_net_event.h"
#include "com.h"

static comBuf my_packet;

void send();
void recv();

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start() {

	/* start the net thread and mst protocol */
	net_init();
	net_mst_proto_init();

	/* set our network address to 1 */
	net_ioctl(MST_PROTO_ID, NET_MST_SET_ADDR, 1);

	// app threads
	mos_thread_new(recv, 128, PRIORITY_NORMAL);
	mos_thread_new(send, 128, PRIORITY_NORMAL);
}

void recv() {
	
	comBuf* packet;

	while(1) {

		// block on recv
		packet = net_recv(NET_MST_PORT);
		
		// free the packet
		com_free_buf(packet);
	}
}

void send() {

	uint32_t sleep_millis = 10 * 1000;
	uint8_t base_addr = 0;

	while(1) {

		// send packet
		my_packet.size = 0;
		net_send(&my_packet, MST_PROTO_ID, NET_MST_PORT, base_addr, NET_MST_DATA_PACKET, NET_MST_NEW_PACKET);
		
		// nap
		mos_thread_sleep(sleep_millis);
	}
}
