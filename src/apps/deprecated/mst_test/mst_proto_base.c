//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/*   File: mst_proto_base.c                                               */
/* Author: Anmol Sheth, anmol.sheth@colorado.edu                          */
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
#include "com.h"

void sink_thread();

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start() {

	/* start the net thread and mst protocol */
	net_init();
	net_mst_proto_init();

	/* set our network address to 0 */
	net_ioctl(MST_PROTO_ID, NET_MST_SET_ADDR, 0);

	/* set up mst protocol so that we're a base station */
	net_ioctl(MST_PROTO_ID, NET_MST_SET_IS_BASE);

	/* thread for receiving packets from other nodes */
	mos_thread_new(sink_thread, 128, PRIORITY_NORMAL);
}

/** @brief This is the sink thread which runs at the base station. All
    data converges up to the base station
*/
void sink_thread() {

	comBuf* packet;

	while(1) {

		/* block on receive */
		packet = net_recv(NET_MST_PORT);

		// toggle red LED
		mos_led_toggle(3);

		/* free buffer memory */
		com_free_buf(packet);
	}
}
