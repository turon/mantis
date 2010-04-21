//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    relay.c                                                       */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/09/04                                                       */
/* Edited     Charles Gruenwald III : gruenwal@colorado.edu               */
/*   Date: 03/30/04                                                       */
/*                                                                        */
/* Base station app for bionet sensor connected to a host computer over   */
/* a serial link.                                                         */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"
#include "com.h"
#include "mos.h"
#include "command_daemon.h"
#include "uart.h"
#include "led.h"
#include "node_net_event.h"
#include "clock.h"
#include "printf.h"

static comBuf *recv_pkt;
static void quick_blink(uint8_t val);

void relay(void)
{
   IF_SET set;
   net_event_t *event;
   
   com_mode (IFACE_RADIO, IF_LISTEN);
   com_mode (IFACE_SERIAL, IF_LISTEN);
   
   while (1) {
      IF_ZERO (&set);
      IF_SET (IFACE_RADIO, &set);
      IF_SET (IFACE_SERIAL, &set);

      com_select (&set, COM_BLOCK);

      if (IF_ISSET (IFACE_RADIO, &set)) {
	 recv_pkt = com_recv (IFACE_RADIO);
	 event = (net_event_t *)recv_pkt->data;
	 quick_blink(0);
	 if(event->to == 0) { //packet meant for base-station...
	    com_send (IFACE_SERIAL, recv_pkt);
	    quick_blink(1);
	 }
	 com_free_buf (recv_pkt);
      } else if (IF_ISSET (IFACE_SERIAL, &set)) {
	 mos_led_toggle (2);
	 recv_pkt = com_recv (IFACE_SERIAL);
	 com_send (IFACE_RADIO, recv_pkt);
	 com_free_buf (recv_pkt);
      }
   }
   
}

void start(void)
{
   //printf("This is the RELAY application.\n");
   //mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   mos_thread_new(relay, 128, PRIORITY_NORMAL);
}

static void quick_blink(uint8_t val)
{
   val &= 3;
   mos_led_on(val);
   mos_udelay(700);
   mos_led_off(val);
}
