//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

/**************************************************************************/
/* File:    ch_event.c                                                   */
/* Author      Charles Gruenwald III   :  gruenwal@colorado.edu           */
/*   Date:  04/14/2004                                                    */
/*                                                                        */
/* Description: ch_event.c will listen over the radio and execute a user-*/
/* defined function or a default function based on the net-event.         */
/**************************************************************************/

/** @file ch_event.c
 * @brief Listen over the radio and execute a user-defined function of a default function based on the ch-event.
 *
 * NOTE: This is an alternate net-event source written by Charles.
 * @author Charles Gruenwald III
 * @date 04/14/2004
 */

#ifdef CH_EVENT
#include <inttypes.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#ifndef SCONS
#include <config.h>
#endif

#include "com.h"
#include "led.h"
#include "dev.h"
#include "node_net_event.h"
#include "node_id.h"

#include "net.h"
#include "mst.h"

static comBuf send;
static comBuf *recvd;

#define MAX_RF_EVENTS 30

/** @brief Net-event structure. */
typedef struct {
   /** @brief Event */
   uint16_t event;
   /** @brief Location */
   void (*func_pointer)(void *);
} rf_event_item;

rf_event_item rf_event_list[MAX_RF_EVENTS];

void mos_net_daemon_init (void)
{
   uint16_t i;

   for (i = 0; i < MAX_RF_EVENTS; i++) {
      rf_event_list[i].event = 0;
      rf_event_list[i].func_pointer = 0;
   }
}

static void send_generic_event (uint16_t destination, uint16_t event_to_send)
{
   net_event_t *event = (net_event_t *)send.data;

   uint16_t nodeid = mos_node_id_get();

#if defined(PLATFORM_TELOSB)
   ((uint8_t *)&(event->from))[0] = ((uint8_t *)&nodeid)[0];
   ((uint8_t *)&(event->from))[1] = ((uint8_t *)&nodeid)[1];
   ((uint8_t *)&(event->to))[0] = ((uint8_t *)&destination)[0];
   ((uint8_t *)&(event->to))[1] = ((uint8_t *)&destination)[1];
   ((uint8_t *)&(event->event))[0] = ((uint8_t *)&event_to_send)[0];
   ((uint8_t *)&(event->event))[1] = ((uint8_t *)&event_to_send)[1];
#else
   event->from = mos_node_id_get();
   event->to = destination;
   event->event=event_to_send;
#endif
   net_send(&send, SIMPLE_PROTO_ID, 1, true);
}

void send_event_arg (uint16_t destination, uint16_t event, uint16_t arg)
{
#if defined(PLATFORM_TELOSB)
   send.data[5] = ((uint8_t *)&arg)[0];
   send.data[6] = ((uint8_t *)&arg)[1];
#else
   *(uint16_t *)&send.data[6] = arg;
#endif
   send.size=8;
   send_generic_event(destination, event);
}

void send_event_arg8 (uint16_t destination, uint16_t event, uint8_t arg)
{
   send.data[6] = arg;

   send.size=7;
   send_generic_event(destination, event);
}

void send_event_buf (uint16_t destination, uint16_t event,
		     uint8_t *buffer, uint8_t size)
{
   uint8_t i;
   for(i=0;i<size && i < (COM_DATA_SIZE-NET_EVENT_HDR_SIZE); i++)
      send.data[i+NET_EVENT_HDR_SIZE]=buffer[i];
   
   send.size = NET_EVENT_HDR_SIZE + size;
   send_generic_event(destination, event);
}

void send_event (uint16_t destination, uint16_t event)
{
   send.size = 6;
   send_generic_event(destination, event);
}

/** @brief Get an event source. 
 * @param p Pointer to event
 * @return Event source
 */
uint16_t mos_get_event_source (void *p)
{
   uint8_t *source=(uint8_t *)p;
   source -= NET_EVENT_HDR_SIZE;
   
   return *(uint16_t *)source;
}

bool user_defined_rf_parse (net_event_t *event, uint8_t *data)
{
   uint8_t i;
   //loop through all commands

   for(i=0; i<MAX_RF_EVENTS && rf_event_list[i].event != 0; i++) {
      //we got a match
      if(event->event == rf_event_list[i].event) {
         //execute the command
         rf_event_list[i].func_pointer (data);
         return true;
      }
   }
   return false;
}

int8_t mos_register_rf_function (uint16_t command_to_reg,
				 void (*func_pointer)(void *))
{
   uint8_t i;
   for(i=0;i<MAX_RF_EVENTS && rf_event_list[i].event != 0;i++)
      ;

   if(i+1 >= MAX_RF_EVENTS || command_to_reg == 0)
      return false;
   
   rf_event_list[i].event = command_to_reg;
   rf_event_list[i].func_pointer = func_pointer;
   rf_event_list[++i].event = 0;
   
   return true;
}

bool rf_command_parse (net_event_t *event, uint8_t *data)
{
   if(user_defined_rf_parse(event, data))
      return true;
   
   switch (event->event){
   case RF_LEDS_BLINK:
      mos_led_display(7);
      mos_led_display(0);
      break;
   case CLICK:
      break;
   case TOGGLE_LEDS:
      mos_led_toggle(0);
      mos_led_toggle(1);
      break;
      //TODO: add sounder support.
   default: //TODO: failed command... callback func?
      return false;
   }

   return true;
}

void mos_net_daemon (void)
{
   //rf_event_list[0].event = 0; //init the list
   net_event_t *event;

   while(1) {
      recvd = net_recv(1);
      event = (net_event_t *)recvd->data;//cast the comBuf to an event
      //TODO: add node to a list for each received node.
      //this packet is for us

      if(event->to == BROADCAST_ADDR || event->to == mos_node_id_get()) {
         rf_command_parse(event, &recvd->data[6]);
      }
      else
         ; //TODO: callback if we don't know command?
      com_free_buf(recvd);
   }
}
#endif