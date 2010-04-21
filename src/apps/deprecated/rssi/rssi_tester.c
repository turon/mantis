//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sense.c  (bionet)                                             */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Sensing application for bionet sensor nodes.                           */
/**************************************************************************/

#include "mos.h"
//#include "config.h"
#include "msched.h"
#include "led.h"
#include "sem.h"
#include "clock.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"
#include "cc1000.h"
#include "node_net_event.h"
//#include "net_event_list.h"
#include "node_id.h"
#include "avr-adc.h"

#ifdef ARCH_AVR
#include "dev.h"
#endif


static comBuf outpacket;
static uint8_t link_quality; //a number between 0 and 10
static uint8_t miss;

/* local functions */
static void sender();
static void receiver();
static void led_control();

static mos_alarm_t alarm;

static void alarm_callback(void *p)
{
   miss = true;

}

void start(void)
{
   // command daemon, normal user interaction
   mos_thread_new(mos_command_daemon, 192, PRIORITY_NORMAL);

   mos_thread_new(sender, 128, PRIORITY_NORMAL);
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
   mos_thread_new(led_control, 128, PRIORITY_NORMAL);

   alarm.func = alarm_callback;
   miss = false;
   mos_alarm_ticks(&alarm, 2880);

}

static
void sender()
{

   printf("Sender thread initialized.\n");
   
   outpacket.data[0] = 0;
   outpacket.size=30;
   while(1)
   {
      outpacket.data[0] = outpacket.data[0] + 1;
      com_send(IFACE_RADIO, &outpacket);
      
      com_mode(IFACE_RADIO, IF_LISTEN);
      mos_thread_sleep(600);
   }
}

static
void receiver()
{
   comBuf *inpacket;
   uint8_t last_seq;
   uint8_t curr_seq = 0;
   printf("Receiver thread initialized.\n");

   
   com_mode(IFACE_RADIO, IF_LISTEN);
   while(1)
   {
      inpacket = com_recv(IFACE_RADIO);
      curr_seq = inpacket->data[0];
      com_free_buf(inpacket);
      
      if(last_seq == 0xff && curr_seq == 0) {
	 printf(".");
	 if(link_quality < 10)
	    link_quality++;
      } else if(last_seq == curr_seq - 1) {
	 printf(".");
	 if(link_quality < 10)
	    link_quality++;
      } else {
	 printf("m");
	 if(link_quality >= 1)
	    link_quality--;
      }

      last_seq = curr_seq;
   }
}

static
void led_control()
{

   printf("LED control initialized\n");
   while(1)
   {
      if(link_quality <2)
	 mos_led_display(0);
      else if(link_quality >=2 && link_quality <=4)
	 mos_led_display(1);
      else if(link_quality >4 && link_quality < 8)
	 mos_led_display(3);
      else
	 mos_led_display(7);

      if(miss == true)
      {
	 if(link_quality > 0)
	    link_quality--;
	 mos_alarm_ticks(&alarm, 28800);
	 printf("t");
	 miss = false;
      }
      
      
      mos_thread_sleep(300);
   }
}
