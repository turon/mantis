//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sense.c  (bionet)                                             */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Sensing application for bionet sensor nodes.                           */
/**************************************************************************/

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "avr-adc.h"
#include "adc.h"
#include "dev.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"

#if defined(PLATFORM_MICA2)
#include "cc1000.h"
#endif


#include "node_net_event.h"
#include "node_id.h"  
#include "mutex.h"
#include "queue.h"
#include "net.h"
#include "clock.h"
#include "simple_proto.h"

#ifdef ARCH_AVR
#include "dev.h"
#endif

#include "bedrest_shared.h"

static uint8_t net_addr;
void send();

void debug_thread()
{
  
  while(1)
    {
      mos_led_off(2);
      mos_thread_sleep(300);
      mos_led_on(2);
      mos_thread_sleep(300);
      
      
    }
}
static comBuf outpacket;
bedrest_t *packet;

void start(void)
{
    uint8_t seq = 0;
     
//   mos_thread_new(debug_thread, 128, PRIORITY_NORMAL);
   net_addr = mos_node_id_get ();
   net_init();
   simple_proto_init();

//   mos_thread_new(send, 196, PRIORITY_NORMAL);
#if defined(PLATFORM_MICA2)
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 255);
#endif

   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while(1) 
   {
       seq++;
       
       net_send(&outpacket, SIMPLE_PROTO_ID, 2, true);
//       if(seq % 32 == 0)
//           mos_led_toggle(0);
   }
}



