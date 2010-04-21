//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    ada_base.c                                                    */
/* Author      Charles Gruenwald III   : charles.gruenwald@colorado.edu   */
/*   Date: 02/14/05                                                       */
/*                                                                        */
/* This application is the basestation for the ADA demo.                  */
/*                                                                        */
/**************************************************************************/
//
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "mst.h"
#include "com.h"
#include "cc1000.h"

#include "node_id.h"
#include "node_net_event.h"
#include "command_daemon.h"

static comBuf beconBuf;

void periodic_beacon(void);
void sink_thread(void);

void rset_leds(void);

//The app needs to know about the ioctls supported by the protocol
#define SET_ADDR 1
#define SET_DTB 2

#define BASE_STATION_ADDRESS 1
#define BCAST_ADDRESS 0xFF

#define LISTEN_PORT 40

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/

void start(void)
{
   /* must start the net thread */
   net_init();

   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);

   mos_register_function("rset_leds", rset_leds);
   //This are the required base station threads
   mos_thread_new(sink_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(periodic_beacon, 128, PRIORITY_NORMAL);

}

/** @brief This is the sink thread which runs at the base station. All
    data converges up to the base station
*/
void sink_thread(void)
{
   comBuf *bufPtr;
   mst_proto_init();

   mst_packet_t *mst_pkt;

   com_mode(IFACE_SERIAL, IF_LISTEN);
   
   while(1) {
      bufPtr = net_recv(LISTEN_PORT);

      mst_pkt = (mst_packet_t *)&(bufPtr->data[bufPtr->size -
					       sizeof(mst_packet_t)]);
      
      mos_led_toggle(1);
//      printf("App:Got data Src=%C SeqNo=%C ", mst_pkt->src, mst_pkt->seqno);
//      printf("Data size: %d, Packet size: %d ", bufPtr->size-sizeof(mst_packet_t),
//	     bufPtr->size);
//      printf("dtb: %C ", bufPtr->data[1]);
//      printf("Data: %C\n", bufPtr->data[0]);
      printf("node:\t%C", mst_pkt->src);
      printf("\tdtb:\t%C", bufPtr->data[1]);
      printf("\tseq:\t%C", mst_pkt->seqno);

//      printf("\tdata:\t%C", bufPtr->data[0]);
      printf("\n");
      //App needs to free the buffer
      com_free_buf(bufPtr);
      
   }
}

/** @brief The base station periodically sends out beacons. The
 * current timer is set for 4sec. The format of the control packet is
 * as follows: Src, Dest, Type, SeqNo, DistanceToBase, TTL, ParentID
 */
void periodic_beacon(void)
{
   uint8_t ctr = 0;
   uint32_t sleep_time = 1000;
   uint8_t port;

   mst_packet_t *mst_out_pkt;

   mst_out_pkt = (mst_packet_t *)(beconBuf.data);
   
   mst_proto_init();

   net_ioctl(MST_PROTO_ID, SET_ADDR, BASE_STATION_ADDRESS);
   net_ioctl(MST_PROTO_ID, SET_DTB, 0);
   
   beconBuf.size=0;
   port = 40;

#ifdef PLATFORM_MICA2
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x04);
#endif

   //ParentID - Only used for upstream data packets which can set
   //the parent ID. Can also be used for other doagnostic issues.
   
   beconBuf.size = sizeof(mst_packet_t);

   while (1) {
      mst_out_pkt->src = BASE_STATION_ADDRESS; //source address
      mst_out_pkt->dest = BCAST_ADDRESS; //destination address
      mst_out_pkt->type = MST_CONTROL; //type of packet
      mst_out_pkt->dtb = 0;  //distance to base
      mst_out_pkt->ttl = 5; //DIAMETER OF NETWORK
      mst_out_pkt->parent = 0xFF; //parent...
      mst_out_pkt->seqno = ctr++;

      net_send(&beconBuf, MST_PROTO_ID, port, 0);
      mos_led_toggle(2);
      mos_thread_sleep(sleep_time);
   }
}

void rset_leds(void)
{
   uint8_t leds;
   leds=prompt_uint8("remote change leds to:");
   send_event_arg8(BROADCAST_ADDR, LEDS, leds);
}
