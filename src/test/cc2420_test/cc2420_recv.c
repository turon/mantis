//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


/**
 * File:     cc2420_recv.c      
 * Author:   Adam Torgerson
 * Date:     11-22-2004
 *
 * Description: This app is meant to test the functionality of the
 * cc2420 low-level code.
 *
 */

#include "mos.h"
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "clock.h"

#include "cc2420.h"

comBuf send_buf;
comBuf *recv_buf;

void recv_thread(void)
{
   printf("CC2420 Radio Test Receiver\n");

   com_mode(IFACE_RADIO, IF_LISTEN);
   
   while(1) {
 
      recv_buf = com_recv(IFACE_RADIO);
      if (recv_buf)
	 printf("Got a packet: %s!\n", (char *)recv_buf->data);
      
      com_free_buf(recv_buf);
     
   }  
}

void start(void)
{
   mos_thread_new(mos_command_daemon, 192, PRIORITY_NORMAL);
   mos_thread_new(recv_thread, 128, PRIORITY_NORMAL);
}
