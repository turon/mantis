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
 * Edited:   Charles Gruenwald III   :   gruenwal@colorado.edu
 * Date:     04-14-2004
 *
 * Description: this will sense the light and send it over the rf.
 */

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "adc.h"
#include "net.h"
#include "dev.h"
#include "com.h"
#include "node_net_event.h"
#include "node_id.h"
#include "simple_proto.h"


static void light_send();
static comBuf outpacket;
static uint8_t last_light;
static uint8_t port = 2;

void start(void){
   net_init(); //initialize the network layer
   simple_proto_init(); //initialize basic routing protocol
   mos_thread_new(light_send, 128, PRIORITY_NORMAL);
}


void light_send(){
   net_event_t *event;
   uint8_t *payload;
   
   event = (net_event_t *)outpacket.data;
   event->from = mos_node_id_get();
   event->to = 0; //to basestation
   event->event = LIGHT; //defined in node_net_event.h
   
   outpacket.size = sizeof(net_event_t) + 1;
   
   payload = &(outpacket.data[6]);

   dev_read (DEV_MICA2_LIGHT, last_light, 1);
   
   while(1){
      dev_read (DEV_MICA2_LIGHT, payload, 1);
      if(*payload != last_light)
      {
	 last_light = *payload;
	 net_send(&outpacket, SIMPLE_PROTO_ID, port, true);
      }
      mos_thread_sleep(1000);
   }
}
