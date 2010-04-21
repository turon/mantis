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
//#include "config.h"
#include "command_daemon.h"
#include "uart.h"
#include "node_net_event.h"
#include "printf.h"
#include "led.h"

static comBuf *recv_pkt;

void relay(void)
{

   com_mode (IFACE_RADIO, IF_LISTEN);
   net_event_t *event;
   
   while (1) {

      mos_led_display (0);
      recv_pkt = com_recv (IFACE_RADIO);
      event = (net_event_t *)recv_pkt->data;

      //printf ("got a packet\n");
      mos_led_display (1);

      if(event->to == 0) { //packet meant for base-station..
	 mos_led_display (2);
	 com_send (IFACE_SERIAL, recv_pkt);
	 mos_led_display (3);
      }
   
      com_free_buf (recv_pkt);
   }
   
}

void start(void)
{
   printf("This is the bionet RELAY application.\n");

   //mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   mos_thread_new(relay, 128, PRIORITY_NORMAL);
}
