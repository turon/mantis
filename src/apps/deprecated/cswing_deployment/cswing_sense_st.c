//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    cswing_sense_st.c                                             */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 04/25/04                                                       */
/*                                                                        */
/* Sensing application for computer science wing deployment.              */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"
#include "node_id.h"
#include "node_net_event.h"
#include "mutex.h"
#include "clock.h"
#ifdef ARCH_AVR
#include "dev.h"
#endif

//shared memory
static comBuf outpacket;

/* functions */
static void comm_init ();
static void quick_blink(uint8_t val);

/* timing */
void sense_and_send_thread();
void sleep_q_thread (void);

void start (void)
{
   comm_init ();

   printf("Welcome to the CS-Wing Deployment App.\n");
   //mos_thread_new (mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new (sense_and_send_thread, 128, PRIORITY_NORMAL);
}

void sense_and_send_thread (){
   uint16_t send_delay=5000; //pause for 5 seconds
   bool last_sampled_temp = false;
   uint8_t new_light;
   int8_t new_temp;

   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while (1) {
      
      if(last_sampled_temp)//because of multiplexing, use last sampled dev
      {
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 last_sampled_temp = false;
      } else {
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 last_sampled_temp = true;
      }
      
      outpacket.data[6] = new_light;    //fill event w/ data
      outpacket.data[7] = new_temp;
      quick_blink(0);      //user feedback
      
      com_send(IFACE_RADIO, &outpacket);//send the packet
      mos_thread_sleep(send_delay);     //sleep
   }
}


static void comm_init ()
{
   outpacket.size = 8; //size of light/temp event
   net_event_t *event = (net_event_t *)outpacket.data; //cast combuf to event packet
   event->from = mos_node_id_get(); //from us
   event->to = 0;          //to the base stattion
   event->event = LIGHT_AND_TEMP; //actual event
}


static void quick_blink(uint8_t val)
{
   val &= 3; //only 3 leds
   mos_led_on(val); //turn led on
   mos_udelay(1000);//wait
   mos_led_off(val);//turn off
}
