//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    gateway.c                                                     */
/* Author   Jeff Rose      :     rosejn@colorado.edu                      */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Linux app that routes data over both ip and the local serial port.     */
/**************************************************************************/

#include <stdio.h>
#include <inttypes.h>

#include "config.h"
#include "msched.h"  
#include "led.h"
#include "com.h"
#include "serial.h"

comBuf *ptr;
comBuf packet;

void linux_send()
{
   uint8_t byte;

   while(1)
   {
      byte = getchar();
      if(byte == '\n')
	 continue;

      packet.size = 1;
      packet.data[0] = byte;

      com_send(IFACE_SERIAL, &packet);
      printf("sent: %d\n", byte);
   }
}

void linux_recv()
{
   com_mode(IFACE_SERIAL, IF_LISTEN);

   while(1)
   {
      ptr = com_recv(IFACE_SERIAL);
      printf("Size: %d -- Data: %d\n", ptr->size, ptr->data[0]);
      com_free_buf(ptr);
   }
}

void start()
{
   mos_thread_new(linux_send, 128, PRIORITY_NORMAL);
   mos_thread_new(linux_recv, 128, PRIORITY_NORMAL);
}
