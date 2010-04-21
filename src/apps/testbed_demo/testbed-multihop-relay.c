//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/********************************************************************/
/*   File:  tesbed-multihop-relay.c                                 */
/* Author:  Charles Gruenwald  :  gruenwal@colorado.edu             */
/*   Date:  11/22/05                                                */
/*                                                                  */
/********************************************************************/


#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "mst.h"
#include "node_net_event.h"
#include "com.h"

#include "testbed_event.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif


static comBuf mybuf;

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

  /* Initialize the MST backend */
   mst_proto_init();

   uint8_t myaddr = mos_node_id_get();
   
   net_ioctl(MST_PROTO_ID, SET_ADDR, (uint8_t)myaddr);
   net_ioctl(MST_PROTO_ID, SET_DTB, 20);

   //This is the basic app thread
   mos_thread_new(recv, 128, PRIORITY_NORMAL);
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
