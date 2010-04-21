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

#include "mos.h"
#include "msched.h"
#include "com.h"
//#include "config.h"
//#include "command_daemon.h"
#include "uart.h"
#include "printf.h"
#include "net.h"
#include "led.h"
#include "dev.h"
#include "clock.h"
#include "printf.h"
#include "node_net_event.h"

#include "bedrest_config.h"
#include "bedrest_shared.h"

uint16_t*items;
comBuf *recv_pkt;                     //give us a packet pointer
comBuf send_pkt;

uint8_t digit(uint8_t i)
{
    if(i<10)
        return '0'+i;
    i-=10;
    return 'A'+i;
}

void print16bit(uint16_t i)
{

    send_pkt.data[3] = digit((i>>0)&15);
    send_pkt.data[2] = digit((i>>4)&15);
    send_pkt.data[1] = digit((i>>8)&15);
    send_pkt.data[0] = digit((i>>12)&15);
    com_send(IFACE_SERIAL,&send_pkt);    
}


net_event_t *event;
bedrest_t *inpacket;
static comBuf send;
net_event_t *sendEvent;

void receiver()
{
   send_pkt.size = 6;
   send_pkt.data[4]=32;
   send_pkt.data[5]=0;

   send.size=4;
   sendEvent = (net_event_t*)&send.data;
   sendEvent->from = 0;
    
   com_mode(IFACE_RADIO, IF_LISTEN);
#if defined(PLATFORM_MICA2)
   //   cc1000_change_freq(FREQ_917_537_MHZ);
#endif
   uint8_t a;
   
   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      event = (net_event_t *)recv_pkt->data;
      inpacket = (bedrest_t *)&(recv_pkt->data[6]);
      
      if(event->to == 0)
      { //packet meant for base-station..


         items = (uint16_t*)&recv_pkt->data;
         printf("%d :", recv_pkt->size);
         for(a=0;a<recv_pkt->size>>1;a++)
            print16bit(*items++);
         if((recv_pkt->size&1)==1)
         {
            recv_pkt->data[recv_pkt->size]=0;          
            print16bit(*items++);
         }
      
         printf("\n");

         if(event->event == BEDREST_CONFIG_PACKET)
            mos_led_toggle(2);
#ifdef DTPA_ENABLED
         else
         {  
            mos_led_toggle(1);
            sendEvent->to = event->from;
            com_send (IFACE_RADIO, &send);
         }
#endif 


      }
      com_free_buf(recv_pkt);             //free the recv'd packet to the pool
//      mos_thread_sleep(200);
            
      mos_led_toggle(0);
   }
}



void start(void)
{

   mos_thread_new(receiver,128, PRIORITY_NORMAL);
}

