//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    test.c                                                        */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/08/04                                                       */
/*                                                                        */
/* Testing the mnet network stack                                         */
/**************************************************************************/

#include <inttypes.h>

#include "printf.h"
#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "com.h"
#include "serial.h"

comBuf packet;

void linux_udp_send()
{
   uint8_t i;

   packet.size = 1;
   i = 0;
   
   while(1)
   {
      packet.data[0] = i++;
      com_send(IFACE_UDP, &packet);
      printf("Sent: %d\n", packet.data[0]);

      sleep(1);
   }
}

/* The start function is automatically called by the operating system,
so this is where the application begins execution.  The thread used to
call start has a limited amount of stack space so it is best to create
threads here and then return. */
void start(void)
{
   mos_thread_new(linux_udp_send, 128, PRIORITY_NORMAL);
}
