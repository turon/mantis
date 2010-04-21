//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     receiver.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: receiver will sit in a tight while loop looking for
 * values received over the radio. Any value it receives it will then
 * show on the leds and send over the serial line using the uart. It is
 * to be used with compatable hardware (nymph/mica2).
 * 
 */

#include "mos.h"
#include "msched.h"
#include "com.h"
#include "net.h"
#include "node_net_event.h"
#include "node_id.h"
#include "simple_proto.h"

#ifndef PLATFORM_LINUX
  #include "printf.h"
#endif

static void receiver();


void start(void){
   net_init();
   simple_proto_init();
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
}

void receiver(){
   net_event_t *event;
   comBuf *recv_pkt;
   uint8_t port = 2;
   uint8_t *payload;
   
   com_mode (IFACE_RADIO, IF_LISTEN);

   while(1){
      recv_pkt = net_recv(port);

      event = (net_event_t *)recv_pkt->data;

      //packet meant for us
      if(event->to == 0)
      {
	 if(event->event == THERMAL)
	 {
	    payload = &(recv_pkt->data[6]);
	    printf("Node: %d, Temp: %C\n", event->from, *payload);
	 } else if(event->event == LIGHT)
	 {
	    payload = &(recv_pkt->data[6]);
	    printf("Node: %d, Light: %C\n", event->from, *payload);
	 } else {
	    printf("No event handler for given event!\n");
	 }
	    
      }
      com_free_buf(recv_pkt);
   }
}

