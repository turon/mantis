//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    mst_proto_example.c                                        */
/* Author      Anmol Sheth   :  anmol.sheth@colorado.edu                */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* This is a simple program that demonstrates the ability of the net      */
/* Layer.                                                                 */
/**************************************************************************/
//
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "mst.h"
#include "com.h"

#ifdef PLATFORM_MICA2 || PLATFORM_MICA2DOT
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

static comBuf beaconBuf;
//static comBuf msgBuf;
void periodic_beacon();
void sink_thread();

//The app needs to know about the ioctls supported by the protocol
#define SET_ADDR 1
#define SET_DTB 2

#define BASE_STATION_ADDRESS 0
#define BCAST_ADDRESS 0xFF

#define PORT 40

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void)
{
  /* must start the net thread */
   net_init();

  mos_thread_new(sink_thread, 128, PRIORITY_NORMAL);
  mos_thread_new(periodic_beacon, 128, PRIORITY_NORMAL);

}

/** @brief This is the sink thread which runs at the base station. All
    data converges up to the base station
*/
void sink_thread()
{
   comBuf *bufPtr;
   mst_packet_t *mst_info;

   mst_proto_init();
   
   com_mode(IFACE_SERIAL, IF_LISTEN);
  
   //
   uint8_t node4cnt = 0;
   uint8_t node6cnt = 0;
 
   while(1) {
      bufPtr = net_recv(PORT);

      mst_info = (mst_packet_t *)&(bufPtr->data[bufPtr->size-sizeof(mst_packet_t)]);
      mos_led_toggle(1);

      printf("node:\t%C", mst_info->src);
      printf("\tdtb:\t%C", mst_info->sender_dtb);
      printf("\tseq:\t%C", mst_info->seqno);
      //printf("\tRSSI:\t%C", bufPtr->signal);
      if (mst_info->src == 4){
	printf("\tRecvCnt:%C",node4cnt++);
      }
      if (mst_info->src == 6){
	printf("\tRecvCnt:%C",node6cnt++);
      }

      printf("\n");

      com_send(IFACE_SERIAL, bufPtr);
      //App needs to free the buffer
      com_free_buf(bufPtr);
      
   }
}

/** @brief The base station periodically sends out beacons.  The
 *  current timer is set for 4 seconds.
*/
void periodic_beacon() {

	// time between beacons in milliseconds
	uint32_t sleep_time = 4000;

	// set up mst protocol so that we're base station
	mst_proto_init();
	net_ioctl(MST_PROTO_ID, SET_ADDR, BASE_STATION_ADDRESS);
	net_ioctl(MST_PROTO_ID, SET_DTB, 0);

	// wake up transmitter
	#ifdef PLATFORM_MICA2
		com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x00);
	#endif
	#ifdef PLATFORM_TELOSB
		com_ioctl_IFACE_RADIO(CC2420_TX_POWER, 0x00);
	#endif

	while(1) {
		
		// send beacon to broadcast address
		beaconBuf.size=0;
		net_send(&beaconBuf, MST_PROTO_ID, PORT, true, MST_CONTROL, BCAST_ADDRESS);
		mos_led_toggle(2);
		mos_thread_sleep(sleep_time);
		
		// send message
		//msgBuf.size = 0;
		//net_send(&msgBuf, MST_PROTO_ID, port, true, MST_DATA, 6);
		//mos_thread_sleep(sleep_time);
   }
}
