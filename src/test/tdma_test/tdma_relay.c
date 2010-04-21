//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    tdma_relay.c                                                  */
/* Author      Brian Shucker shucker@cs.colorado.edu                      */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Simple app used for testing cc1000 TDMA MAC                            */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "com.h" //com layer

comBuf *pkt_in;

void relay_in(void)
{
  com_mode(IFACE_RADIO, IF_LISTEN);

  while(1)
    {
      mos_led_toggle(1);
      pkt_in = com_recv(IFACE_RADIO);
      com_send(IFACE_SERIAL, pkt_in);
      com_free_buf(pkt_in);
    }
}

/* The start function is automatically called by the operating system,
so this is where the application begins execution.  The thread used to
call start has a limited amount of stack space so it is best to create
threads here and then return. */
void start(void)
{
  cc1000_tdma_init(0); //we are the base station
  mos_thread_new(relay_in, 128, PRIORITY_NORMAL);
}
