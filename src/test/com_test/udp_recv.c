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

comBuf *ptr;

void linux_udp_recv()
{
   com_mode(IFACE_UDP, IF_LISTEN);

   while(1)
   {
      ptr = com_recv(IFACE_UDP);
      printf("Size: %d -- Data: %d\n", ptr->size, ptr->data[0]);
      com_free_buf(ptr);
   }
}

/* The start function is automatically called by the operating system,
so this is where the application begins execution.  The thread used to
call start has a limited amount of stack space so it is best to create
threads here and then return. */
void start(void)
{
   mos_thread_new(linux_udp_recv, 128, PRIORITY_NORMAL);
}
