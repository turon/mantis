//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     thermal_send.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: this will sense the temperature and send it over the rf. 
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

static void thermal_send();
static comBuf outpacket;
static uint8_t last_temp;
static uint8_t port = 2;

void start(void){
   net_init(); //initialize the network layer
   simple_proto_init(); //initialize basic routing protocol
   mos_thread_new(thermal_send, 128, PRIORITY_NORMAL);
}


void thermal_send(){
   net_event_t *event;
   uint8_t *payload;
   
   event = (net_event_t *)outpacket.data;
   event->from = mos_node_id_get();
   event->to = 0; //to basestation
   event->event = THERMAL; //defined in node_net_event.h
   
   outpacket.size = sizeof(net_event_t) + 1;
   
   payload = &(outpacket.data[6]);

   dev_read (DEV_MICA2_TEMP, last_temp, 1);
   
   while(1){
      dev_read (DEV_MICA2_TEMP, payload, 1);
//      if(*payload != last_temp)
//      {
	 last_temp = *payload;
	 printf("Sent: %d\n",last_temp);
	 net_send(&outpacket, SIMPLE_PROTO_ID, port, true);
//      }
      mos_thread_sleep(1000);
   }
}
