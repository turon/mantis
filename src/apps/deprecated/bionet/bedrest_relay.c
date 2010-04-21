//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    relay.c                                                       */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/09/04                                                       */
/* Edited     Charles Gruenwald III : gruenwal@colorado.edu               */
/*   Date: 03/30/04                                                       */
/*                                                                        */
/* Base station app for bionet sensor connected to a host computer over   */
/* a serial link.                                                         */
/**************************************************************************/

#if defined(PLATFORM_MICA2)
#include "cc1000.h"
#endif

#include "msched.h"
#include "com.h"
//#include "config.h"
//#include "command_daemon.h"
#include "uart.h"
#include "node_net_event.h"
#include "printf.h"
#include "net.h"
#include "led.h"
#include "dev.h"
#include "clock.h"
#include "simple_proto.h"

#include "bedrest_config.h"
#include "bedrest_shared.h"


static comBuf *recvd;
net_event_t *event;
bedrest_t *inpacket;
static comBuf send;
net_event_t *sendEvent;



void receiver()
{
   comBuf *recv_pkt;                     //give us a packet pointer
   send.size=4;
   sendEvent = (net_event_t*)&send.data;
   com_mode(IFACE_RADIO, IF_LISTEN);

#if defined(PLATFORM_MICA2)
   //   cc1000_change_freq(FREQ_917_537_MHZ);
#endif

   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      mos_led_toggle(0);
      event = (net_event_t *)recv_pkt->data;
      inpacket = (bedrest_t *)&(recv_pkt->data[6]);

      if(event->to == 0)
      { //packet meant for base-station..
         com_send(IFACE_SERIAL, recv_pkt);

         if(event->event == BEDREST_CONFIG_PACKET)
            mos_led_toggle(2);
#ifdef DTPA_ENABLED
         else
         {  
            mos_led_toggle(1);
            event->to = event->from;
            event->from = 0;
            recv_pkt->size=4;
            com_send (IFACE_RADIO, recv_pkt);
         }
#endif 
      }
            
      com_free_buf(recv_pkt);             //free the recv'd packet to the pool
   }
}



void start(void)
{

   mos_thread_new(receiver,128, PRIORITY_NORMAL);
   
}

