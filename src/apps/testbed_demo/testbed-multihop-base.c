//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/********************************************************************/
/*   File:  tesbed-multihop-base.c                                  */
/* Author:  Charles Gruenwald  :  gruenwal@colorado.edu             */
/*   Date:  11/22/05                                                */
/*                                                                  */
/********************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "mst.h"
#include "com.h"
#include "command_daemon.h"

#include "testbed_event.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

static comBuf beaconBuf, msgBuf;

//explicit timeout
static uint32_t beacon_time = 0;
//static uint32_t beacon_time = 5000;


void periodic_beacon();
void output_power();
void sink_thread();

#define LISTEN_PORT 40

void beacon();
void test();
void beacon_timeout();

void beacon_init();

void start(void)
{
   mos_mdelay(1000);
   printf("mst Base Started.\n");

   com_mode(IFACE_SERIAL, IF_LISTEN);
   com_mode(IFACE_RADIO, IF_LISTEN);
   
   beacon_init();
//   mos_thread_new(sink_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(periodic_beacon, 192, PRIORITY_NORMAL);
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);

   mos_register_function("test", test);
   mos_register_function("beacon", beacon);
   mos_register_function("beacon_timeout", beacon_timeout);
   mos_register_function("output_power", output_power);
}

void beacon_init()
{
   uint16_t id = 0;
   
   mst_proto_init();
   mst_proto_ioctl(SET_ADDR, BASE_STATION_ADDRESS);
   mst_proto_ioctl(SET_DTB, 0);
   
   send_testbed_event(EVENT_NETWORK, NETWORK_ID, (uint8_t *)&id, 2);
   
   
   com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x00);
}

void test()
{
   printf("this thread still running.\n");
}

void beacon_timeout()
{
   uint16_t new_timeout = prompt_long("#:");
   beacon_time = (uint32_t) new_timeout;
}

void output_power()
{
   uint8_t new_power = prompt_char("#:");
   com_ioctl_IFACE_RADIO(RADIO_TX_POWER, new_power);
   
}


void beacon()
{
   beaconBuf.size=0;
   net_send(&beaconBuf, MST_PROTO_ID, true, MST_CONTROL, 255);
}

/** @brief This is the sink thread which runs at the base station. All
    data converges up to the base station
*/
void sink_thread()
{
   comBuf *bufPtr;
   mst_packet_t *mst_info;
   recv_pkt_event_t recv_event;
   mst_proto_init();
   
   while(1) {
      bufPtr = net_recv(LISTEN_PORT);

      mst_info = (mst_packet_t *)&(bufPtr->data[bufPtr->size-sizeof(mst_packet_t)]);
      mos_led_toggle(1);
      /*
      recv_event.src = mst_info->src;
      recv_event.size = bufPtr->size;
      send_testbed_event(EVENT_NETWORK, PACKET_RECV,
			 (uint8_t *)&recv_event, 3);
			 printf("data packet.\n");
      */
      //App needs to free the buffer
      com_free_buf(bufPtr);
      
   }
}

/** @brief The base station periodically sends out beacons. The
 * current timer is set for 4sec. The format of the control packet is
 * as follows: Src, Dest, Type, SeqNo, DistanceToBase, TTL, ParentID
*/
void periodic_beacon()
{
   while (1) {
      if(beacon_time > 0)
      {
	 beaconBuf.size=0;
	 net_send(&beaconBuf, MST_PROTO_ID, true, MST_CONTROL, 255);
	 mos_thread_sleep(beacon_time);      
      } else {
	 mos_thread_sleep(3000);
      }
      mos_led_toggle (2);
   }
}
