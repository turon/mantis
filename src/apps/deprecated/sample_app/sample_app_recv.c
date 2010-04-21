//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "com.h"
#include "dev.h"
#include "led.h"
#include "mst.h"
#include "command_daemon.h"
#include "node_net_event.h"
#include "printf.h"
#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

void recv_thread();
void periodic_beacon();


#define LISTEN_PORT 40

static comBuf beaconBuf, msgBuf;

void start(void)
{
   net_init();
   mst_proto_init();

   mos_thread_new(recv_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(periodic_beacon, 128, PRIORITY_NORMAL);
}

void recv_thread()
{
   comBuf *recvpkt;
   mst_packet_t *mstinfo;
   


   com_mode(IFACE_SERIAL, IF_LISTEN);
   
   while(1)
   {
      recvpkt = net_recv(LISTEN_PORT);

      
      mstinfo = (mst_packet_t *)&(recvpkt->data[recvpkt->size-sizeof(mst_packet_t)]);

      printf("light value: \t%C", recvpkt->data[sizeof(net_event_t)]);
      printf("\tnode: \t%C", mstinfo->src);
      printf("\thops:\t%C", mstinfo->sender_dtb);
      printf("\tsequence:\t%C\n", mstinfo->seqno);
      mos_led_toggle(1);
      com_free_buf(recvpkt);
   }
}

 
void periodic_beacon()
{
   uint16_t sleep_time = 4000; //sleep for 4 second interval
   
   net_ioctl(MST_PROTO_ID, SET_ADDR, 0); //set basestation addr to 0
   net_ioctl(MST_PROTO_ID, SET_DTB, 0);

#ifdef PLATFORM_MICA2
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x00);
#endif
   
   while(1)
   {
      beaconBuf.size=0;
      msgBuf.size = 0;
      
      net_send(&beaconBuf, MST_PROTO_ID, LISTEN_PORT, true, MST_CONTROL, 255);
      mos_led_toggle (2);
      mos_thread_sleep (sleep_time);
   }
}
      
