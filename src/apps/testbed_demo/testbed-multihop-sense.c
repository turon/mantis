//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    mst_proto_example.c                                           */
/* Author      Anmol Sheth   :  anmol.sheth@colorado.edu                  */
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

#include "testbed_event.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

#include "command_daemon.h"

static comBuf mybuf;
static uint32_t sense_time = 0;
static uint32_t beacon_time = 5000;
void send();
void recv();

#define LISTEN_PORT 40

void sense_init();

//Gotta synchronize those threads
mos_mutex_t sense_timeout_mux;
mos_mutex_t sending_mux;

void keep_alive();

void sense_timeout();
void output_power();
void parent_f();
void dtb_f();
void sense();
/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void)
{
   mos_mdelay(1000);
   printf("mst Send Started.\n");

  /* Initialize the mst backend */
   mst_proto_init();

   uint8_t myaddr = mos_node_id_get();

   uint16_t id = myaddr;

   send_testbed_event(EVENT_NETWORK, NETWORK_ID, (uint8_t *)&id, 2);

   mst_proto_ioctl(SET_ADDR, (uint8_t)myaddr);
   mst_proto_ioctl(SET_DTB, 20);

   sense_init();

   mos_mutex_init(&sense_timeout_mux);
   mos_mutex_init(&sending_mux);
   
   //This is the basic app thread
   mos_thread_new(recv, 128, PRIORITY_NORMAL);
//   mos_thread_new(keep_alive, 128, PRIORITY_NORMAL);

   mos_thread_new(send, 128, PRIORITY_NORMAL);
   mos_thread_new(mos_command_daemon, 128,
		  PRIORITY_NORMAL);
   
   mos_register_function("sense", sense);
   mos_register_function("sense_timeout", sense_timeout);
   mos_register_function("output_power", output_power);
   mos_register_function("parent", parent_f);
   mos_register_function("dtb", dtb_f);
}

void keep_alive()
{
   while(1)
   {
      send_testbed_event(EVENT_SYSTEM, KEEP_ALIVE, NULL, 0);
      mos_thread_sleep(5000);
   }
}

void sense_timeout()
{
   uint16_t new_timeout = prompt_long("#:");
   mos_mutex_lock(&sense_timeout_mux);
   sense_time = (uint32_t) new_timeout;
   mos_mutex_unlock(&sense_timeout_mux);
   
   printf("New sense timeout: %l\n", new_timeout);
}

void parent_f()
{
   uint16_t parent = mst_parent();
   printf("My parent: %d\n", parent);
   send_testbed_event(EVENT_NETWORK,NEW_PARENT, &parent, 2);
}

void dtb_f()
{
   uint16_t dtb = mst_dtb();
   printf("My dtb: %d\n", dtb);
}

void output_power()
{
   uint8_t new_power = prompt_char("#:");
   com_ioctl_IFACE_RADIO(RADIO_TX_POWER, new_power);
}

void recv()
{
   com_mode(IFACE_RADIO, IF_LISTEN);
   comBuf * buffer;

   while (1)
   {
      buffer = net_recv(LISTEN_PORT);
      mos_led_toggle (1);
      com_free_buf(buffer);
      printf("App:::\n");
   }
}

void sense()
{
   mos_mutex_lock(&sending_mux);
   mybuf.size=6;
   net_send(&mybuf, MST_PROTO_ID, true, MST_DATA, 0);
   mos_mutex_unlock(&sending_mux);
}

void sense_init()
{
  com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x00);
  net_event_t *event = (net_event_t *)mybuf.data;
  event->from =  mos_node_id_get();
  event->to = 0;
  event->event = 9;
}


void send()
{
   uint32_t stimeout;
  while (1) {
     mos_mutex_lock(&sense_timeout_mux);
     stimeout = sense_time;
     mos_mutex_unlock(&sense_timeout_mux);
	
     if(stimeout > 0)
     {
	mos_mutex_lock(&sending_mux);
	mybuf.size=6;
    net_send(&mybuf, MST_PROTO_ID, true, MST_DATA, 0);
	mos_mutex_unlock(&sending_mux);
	mos_thread_sleep (sense_time);
     } else {
	mos_thread_sleep(3000);
     }
     mos_led_toggle (1);
  }
}

