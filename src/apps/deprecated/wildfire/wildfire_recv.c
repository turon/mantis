//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "sem.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"
#include "net.h"
#include "mst.h"
#include "wildfire.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif

void periodic_beacon();
void recv_thread ();

static comBuf beconBuf, msgBuf;
#define BASE_STATION_ADDRESS 0

void start (void)
{

  net_init();
  mst_proto_init();


  mos_thread_new(periodic_beacon, 128, PRIORITY_NORMAL);
  mos_thread_new (recv_thread, 128, PRIORITY_NORMAL);

}

void recv_thread ()
{

  comBuf * recvBuf;
  wildfire_packet_t *packet; 
  mst_packet_t *mst_info;
  uint16_t from;
  uint8_t reset_header = -1;

   com_ioctl_IFACE_RADIO( 0 , 0xff);
 
   while(1) {
      if(((reset_header++) % 15) == 0){
	 printf("from\ttemp\ttemp_c\thmdty\thmdty_c\tw_dir\tw_dir_c\tw_spd\n");
	 reset_header = 1;
      }
      
      mos_led_toggle(0);
      recvBuf = net_recv(20);
      mos_led_toggle(2);

      packet = (wildfire_packet_t *)&(recvBuf->data[0]);
      mst_info = (mst_packet_t *)&(recvBuf->data[recvBuf->size -
						 sizeof(mst_packet_t)]);

      /*
      printf("mst info: %C %C %C %C\n",
	     mst_info -> src,
	     mst_info -> dest,
	     mst_info -> seqno,
	     mst_info -> dtb);
	     
      */
      from = mst_info -> src;

      printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
	     from,
	     packet->temp, 
	     packet->temp_c, 
	     packet->humidity, 
	     packet->humidity_c, 
	     packet->wind_direction, 
	     packet->wind_direction_c, 
	     packet->wind_speed);

      com_free_buf(recvBuf);
   }
}

/** @brief The base station periodically sends out beacons. The
 * current timer is set for 4sec. The format of the control packet is
 * as follows: Src, Dest, Type, SeqNo, DistanceToBase, TTL, ParentID
*/
void periodic_beacon()
{
//uint8_t ctr=0;
   uint32_t sleep_time = 4000;
   uint8_t port;
   
   mst_proto_init();

   net_ioctl(MST_PROTO_ID, SET_ADDR, BASE_STATION_ADDRESS);
   net_ioctl(MST_PROTO_ID, SET_DTB, 0);

   
   port = 40;

#ifdef PLATFORM_MICA2
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x04);
#endif

  
   while (1) {
      beconBuf.size=0;
      msgBuf.size = 0;
      
      net_send(&beconBuf, MST_PROTO_ID, port, true, MST_CONTROL, 255);
      mos_thread_sleep(1000);
      net_send(&msgBuf, MST_PROTO_ID, port, true, MST_DATA, 6);
      
      mos_led_toggle (2);
      mos_thread_sleep (sleep_time);
   }
}

