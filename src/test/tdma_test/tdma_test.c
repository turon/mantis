//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    tdma_test.c                                                   */
/* Author      Brian Shucker shucker@cs.colorado.edu                      */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Simple app used for testing cc1000 TDMA MAC                            */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "com.h" //com layer
#include "printf.h"

#ifdef PLATFORM_LINUX

comBuf *pkt;

void recv(void)
{
  uint8_t i;

  com_mode(IFACE_SERIAL, IF_LISTEN);

  while(1)
    {
      pkt = com_recv(IFACE_SERIAL);
      printf("Got packet, size=%d, data: ", pkt->size);
      for(i=0; i<pkt->size; i++)
	printf("%d ", pkt->data[i]);
      putchar('\n');
      com_free_buf(pkt);
    }
}

void start(void)
{
  mos_thread_new(recv, 128, PRIORITY_NORMAL);
}

#else //mica2 or nymph

#define FRAME 1

comBuf pkt;

void send(void)
{
  uint8_t i;
  uint8_t top, bot;

  pkt.size = 32;
  for(i=0; i<64; i++)
    pkt.data[i] = 42;

  i = 0;
  //send packets at full speed
  while(1)
    {
      pkt.data[0] = FRAME;
      pkt.data[1] = i;
      i++;
      mos_led_toggle(0);
      com_send(IFACE_RADIO, &pkt);
    }
}

/* The start function is automatically called by the operating system,
so this is where the application begins execution.  The thread used to
call start has a limited amount of stack space so it is best to create
threads here and then return. */
void start(void)
{
  cc1000_tdma_init(FRAME);
  mos_thread_new(send, 128, PRIORITY_NORMAL);
}

#endif

