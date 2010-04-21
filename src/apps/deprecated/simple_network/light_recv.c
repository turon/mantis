//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     light_send.c      
 * Author:   Adam Torgerson
 * Date:     03-18-2004
 *
 * Description: receiver will sit in a tight while loop looking for
 * values received over the radio. Any value it receives it will then
 * show on the leds and send over the serial line using the uart. It is
 * to be used with compatable hardware (nymph/mica2).
 * 
 */

#include <inttypes.h>
#include <stdio.h>

#include "led.h"
#include "msched.h"
#include "command_daemon.h"
#include "com.h"

#ifndef PLATFORM_LINUX
  #include "printf.h"
#endif


void receiver();

uint8_t net_addr = 1;

void start(void){
   //start the commander which gives us 'restart'
   mos_thread_new(mos_command_daemon, 256, PRIORITY_NORMAL);
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
}

void receiver(){
   comBuf send_pkt, *recv_pkt;
   IF_SET set;


   com_mode (IFACE_RADIO, IF_LISTEN); //put radio in listen mode
   
   while (1) {
      IF_ZERO (&set);                         //reset the 'set'
      IF_SET (IFACE_RADIO, &set);             //add radio to 'set'
      com_select (&set, COM_BLOCK);                   //wait on radio
      if (IF_ISSET (IFACE_RADIO, &set)) { 
	 recv_pkt = com_recv (IFACE_RADIO);   //get the packet
	 send_pkt.data[0] = recv_pkt->data[0];//copy the packet
	 send_pkt.data[1] = recv_pkt->data[1];
	 send_pkt.data[2] = recv_pkt->data[2];
	 send_pkt.size = 3;                   //set the size
	 com_free_buf (recv_pkt);             //return buffer to pool
	 com_send (IFACE_SERIAL, &send_pkt);  //forward the packet
      }
   }
}
