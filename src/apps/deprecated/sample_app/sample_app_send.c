//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)



#include "mos.h"
#include "net.h"
#include "dev.h"
#include "led.h"
#include "mst.h"
#include "command_daemon.h"
#include "node_net_event.h"
#include "node_id.h"
#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

void send();
void recv();

#define LISTEN_PORT 40

static comBuf pkt;

void start(void)
{
   net_init();
   mst_proto_init();

   uint8_t myaddr = mos_node_id_get();
   net_ioctl(MST_PROTO_ID, SET_ADDR, (uint8_t)myaddr);
   net_ioctl(MST_PROTO_ID, SET_DTB, 20);

   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);

   mos_thread_new(send, 128, PRIORITY_NORMAL);
   mos_thread_new(recv, 128, PRIORITY_NORMAL);
   
}

void send()
{

   uint8_t *lightval;
   net_event_t *event;

   event = (net_event_t *)pkt.data;
   event->from = mos_node_id_get();
   event->to = 0;
   event->event = 9;

   lightval = &(pkt.data[sizeof(net_event_t)]);

   pkt.size= sizeof(net_event_t) + 1;

#ifdef PLATFORM_MICA2
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x00);
#endif

   while(1)
   {
#ifdef PLATFORM_MICA_ANY
      dev_read (DEV_MICA2_LIGHT, lightval, 1);
#elif defined(PLATFORM_TELOSB)
      dev_read (DEV_MSP_TEMPERATURE, lightval, 1);
#endif

      net_send(&pkt, MST_PROTO_ID, LISTEN_PORT, true, MST_DATA, 0);
      mos_led_display(0);
      mos_thread_sleep(100);
      printf("pktdata is %d\n", pkt.data[sizeof(net_event_t)]);
      if(*lightval < 100 && *lightval > 20)
	 mos_led_display(1);
      else if(*lightval < 200)
	 mos_led_display(3);
      else // *lightval > 200
	 mos_led_display(7);
      
      mos_thread_sleep(900);
   }
    
}


void recv()
{
   comBuf *buffer;
   while(1)
   {
      buffer = net_recv(LISTEN_PORT);
      mos_led_toggle(0);
      com_free_buf(buffer);
      printf("recved a packet\n");
   }
}
