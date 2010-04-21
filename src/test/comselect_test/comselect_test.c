//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"

#include "printf.h"

#include "com.h"
#include "msched.h"
#include "clock.h"

/**
 * File:     comselect_test.c      
 * Author:   Charles
 * Date:     07-20-2004
 *
 * Description: This app is for testing the com_select interface
 *
 */

void coms_test()
{
   IF_SET set;
   uint8_t ifaces=0;
   comBuf *recvd;
   com_mode(IFACE_SERIAL, IF_LISTEN);

#ifdef PLATFORM_LINUX
   com_mode(IFACE_TERMINAL, IF_LISTEN);
#else
   com_mode(IFACE_RADIO, IF_LISTEN);
#endif

   
   while(1){
      IF_ZERO(&set);

    
      IF_SET(IFACE_SERIAL, &set);

#ifdef PLATFORM_LINUX
      IF_SET(IFACE_TERMINAL, &set);
#else
      IF_SET(IFACE_RADIO, &set);
#endif
      
      ifaces = com_select(&set,2000);
      
      if(IF_ISSET(IFACE_SERIAL, &set)){
	 printf("serial brought us back\n");
	 recvd = com_recv(IFACE_SERIAL);
	 com_free_buf(recvd);
      }
#ifdef PLATFORM_LINUX
      else if(IF_ISSET(IFACE_TERMINAL, &set)){
	 printf("terminal brought us back\n");
	 recvd = com_recv(IFACE_TERMINAL);
	 com_free_buf(recvd);
      }
#else
      else if(IF_ISSET(IFACE_RADIO, &set)){
	 printf("radio brought us back\n");
	 recvd = com_recv(IFACE_RADIO);

	 printf("[%C-%C]", recvd->data[0], recvd->data[recvd->size-1]);
	 com_free_buf(recvd);
      } 
#endif
      else {
	 printf("timed out, returned %d ifaces\n",ifaces);
      }
     
   }
}

void m_thread()
{
   mos_thread_suspend();
}

void start(void)
{
   mos_thread_new(coms_test, 128, PRIORITY_NORMAL);
}
