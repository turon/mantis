//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    relay.c                                                       */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/09/04                                                       */
/* Edited     Charles Gruenwald III : gruenwal@colorado.edu               */
/*   Date: 03/30/04                                                       */
/*                                                                        */
/* Base station app for bionet sensor connected to a host computer over   */
/* a serial link.                                                         */
/**************************************************************************/


#include "msched.h"
#include "com.h"
//#include "config.h"
#include "command_daemon.h"
#include "uart.h"
#include "node_net_event.h"
#include "printf.h"
#include "net.h"
#include "led.h"
#include "dev.h"
#include "clock.h"

static comBuf out_packet;
static comBuf *recv_pkt_radio;
static comBuf *recv_pkt_serial;
static comBuf *recvd;
static void quick_blink(uint8_t val);


void relay_serial(void)
{
  com_mode (IFACE_SERIAL, IF_LISTEN);
  while (1) {
     
     recv_pkt_serial = com_recv (IFACE_SERIAL);
     mos_led_on(2);
     mos_led_toggle(0);
     if( *(uint16_t *)(&recv_pkt_serial->data[2]) == 0 )
     {
	mos_led_display(recv_pkt_serial->data[6]);
     }
     else
     {
	net_send(recv_pkt_serial, SIMPLE_PROTO_ID, 1, true);
     }
     com_free_buf (recv_pkt_serial);
     mos_led_off(2);
     }
}

void relay_radio(void)
{
   net_event_t *event;
   bedrest_t *inpacket;
   
   com_mode (IFACE_RADIO, IF_LISTEN);
   while(1) {
      mos_led_on(1);
      recv_pkt_radio = net_recv(2);
      mos_led_off(1);
//      mos_led_toggle(2);
//      quick_blink(2);

//      event = (net_event_t *)recv_pkt_radio->data;
//      inpacket = (bedrest_t *)&(recv_pkt_radio->data[6]);
      
//      if(event->to == 0) { //packet meant for base-station..
//#ifdef GET_RSSI
//	 recv_pkt_radio->data[recv_pkt_radio->size++] = recv_pkt_radio->signal;
//#else
//	 recv_pkt_radio->data[recv_pkt_radio->size++] = 255;
//#endif
//	 com_send(IFACE_SERIAL, recv_pkt_radio);
//      }
      
      recv_pkt_radio->size++;
      mos_led_on(0);
      com_send(IFACE_SERIAL, recv_pkt_radio);
      mos_led_off(0);
      com_free_buf(recv_pkt_radio);
   }

}


#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ)
void battery_update(void)
{
   bedrest_t *packet;
   
   net_event_t *event;
   event = (net_event_t *)out_packet.data;
   event->from = 0;
   event->to = 0;
   event->event = BEDREST_PACKET;

   packet = (bedrest_t *)&(out_packet.data[6]);
   packet->temp = 0;
   packet->light = 0;
   packet->avg_accelx = 0;
   packet->avg_accely = 0;
   packet->txpower = 255;


   uint8_t seqno = 0;
   
   while(1) {
      out_packet.size = 17;
      //battery
      dev_open(DEV_MICA2_BATTERY);
      dev_read(DEV_MICA2_BATTERY, &(packet->battery), sizeof(packet->battery));
     dev_close(DEV_MICA2_BATTERY);

      packet->accel_ms = seqno++;
      
      com_send(IFACE_SERIAL, &out_packet);
      mos_thread_sleep(5000);
   }
}

#endif

void start(void)
{
   //initialize the network layer and appropriate protocol
   net_init();
   simple_proto_init();

   //start listening threads
   mos_thread_new(relay_radio, 192, PRIORITY_NORMAL);
   mos_thread_new(relay_serial, 128, PRIORITY_NORMAL);

//   mos_thread_new(battery_update, 128, PRIORITY_NORMAL);
}

static void quick_blink(uint8_t val)
{
   val &= 3;
   mos_led_on(val);
   mos_thread_sleep(100);
   mos_led_off(val);
}
