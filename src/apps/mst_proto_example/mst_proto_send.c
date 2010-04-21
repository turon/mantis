//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    mst_proto_example.c                                        */
/* Author      Anmol Sheth   :  anmol.sheth@colorado.edu                */
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
#include "mst.h"
#include "uart.h"
#include "node_net_event.h"
#include "com.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#elif defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#elif defined(PLATFORM_MICROBLAZE) 
#include "mb_radio.h"
#else
#warning "NO RADIO DEFINED FOR THIS PLATFORM"
#endif

#include "command_daemon.h"

static comBuf mybuf;

void send();
void recv();

//The app needs to know about the ioctls supported by the protocol
#define SET_ADDR 1
#define SET_DTB 2

#define BASE_STATION_ADDRESS 0
#define BCAST_ADDRESS 0xFF

#define LISTEN_PORT 40

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void)
{
  /* must start the net thread */
   net_init();

   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);

  /* Initialize the MST backend */
   mst_proto_init();

   uint8_t myaddr = mos_node_id_get();
   
   net_ioctl(MST_PROTO_ID, SET_ADDR, (uint8_t)myaddr);
   net_ioctl(MST_PROTO_ID, SET_DTB, 20);

   //This is the basic app thread
   mos_thread_new(recv, 128, PRIORITY_NORMAL);
   mos_thread_new(send, 128, PRIORITY_NORMAL);
}

void recv()
{
   comBuf * buffer;

   while (1)
   {
      buffer = net_recv(LISTEN_PORT);
      mos_led_toggle (1);
      com_free_buf(buffer);
      printf("App:::\n");
   }
}

void send()
{
  uint32_t sleep_time = 1000;
  uint8_t port;
  net_event_t *event;

#ifdef PLATFORM_MICA2
  com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x00);
#endif
#ifdef PLATFORM_TELOSB
   com_ioctl_IFACE_RADIO(CC2420_TX_POWER, 0x00);
#endif

  event = (net_event_t *)mybuf.data;
  event->from =  mos_node_id_get();
  event->to = 0;
  event->event = 9;
  port = 40;
  
  while (1) {
     mybuf.size=6;
     net_send(&mybuf, MST_PROTO_ID, port, true, MST_DATA, 0);
     mos_led_toggle (1);
     mos_thread_sleep (sleep_time);
  }
}
