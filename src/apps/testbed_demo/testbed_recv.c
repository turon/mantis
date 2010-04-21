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
#include "node_id.h"
#include "testbed_event.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

void recv()
{
   comBuf * buffer;
   com_mode(IFACE_RADIO, IF_LISTEN);

   uint16_t id = mos_node_id_get();
   send_testbed_event(EVENT_NETWORK, NETWORK_ID, (uint8_t *)&id, 2);

   while (1)
   {
      buffer = com_recv(IFACE_RADIO);
      buffer->data[2] = buffer->size;
      send_testbed_event(EVENT_NETWORK, PACKET_RECV,
		 (uint8_t *)&(buffer->data[0]), 3);

      mos_led_toggle (1);
      com_free_buf(buffer);
   }
}


void start(void)
{
   mos_mdelay(1000);
   printf("SYSTEM: NODE: Started.\n");
   
  /* must start the net thread */
   mos_thread_new(recv, 128, PRIORITY_NORMAL);
}

