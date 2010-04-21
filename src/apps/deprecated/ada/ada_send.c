//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


/**************************************************************************/
/* File:    ada_send.c                                                    */
/* Author      Charles Gruenwald III   : charles.gruenwald@colorado.edu   */
/*   Date: 02/14/05                                                       */
/*                                                                        */
/* This application is the node for the ADA demo.                         */
/*                                                                        */
/**************************************************************************/
//
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "mst.h"
#include "uart.h"
#include "com.h"
#include "cc1000.h"
#include "command_daemon.h"

#include "node_net_event.h"

static comBuf mybuf;

void send(void);
void recv(void);

void leds(void *p);

//The app needs to know about the ioctls supported by the protocol
#define SET_ADDR 1
#define SET_DTB 2

#define BASE_STATION_ADDRESS 1
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

   //mos_thread_new(mos_net_daemon, 128, PRIORITY_NORMAL);
   //mos_register_rf_function(LEDS, (void *)leds);

  //This is the basic app thread
  mos_thread_new(recv, 128, PRIORITY_NORMAL);
  mos_thread_new(send, 128, PRIORITY_NORMAL);
}

void recv(void)
{
   comBuf * buffer;

   while (1) {
      buffer = net_recv(LISTEN_PORT);
      //mos_led_toggle (1);
      com_free_buf(buffer);
      printf("App:::\n");
   }
}

void send(void)
{
  uint32_t sleep_time = 1000;
  uint8_t port;

//  com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x04);
  
  mybuf.size=1;
  mybuf.data[0] = 0;
  port = 40;
  
  while (1) {
     mybuf.size=2;
     mybuf.data[0]++;
     mybuf.data[1] = mst_dtb();
     net_send(&mybuf, MST_PROTO_ID, port, true);
     mos_led_toggle (1);
     mos_thread_sleep (sleep_time);
  }
}

void leds(void *p)
{
   uint8_t *params = (uint8_t *)p;
   printf("got the leds command: %C!\n", params[0]);
   mos_led_display(params[0]);
}
