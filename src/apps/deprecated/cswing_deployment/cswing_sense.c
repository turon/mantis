//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sense.c (cswing deployment)                                   */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 04/20/04                                                       */
/*                                                                        */
/* Test app for cswing deployment using csma, etc                         */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"
#include "node_net_event.h"
#include "mutex.h"
#include "node_id.h"
#include "clock.h"
#ifdef ARCH_AVR
#include "dev.h"
#endif

static comBuf outpacket;
static uint8_t new_light; 
static uint8_t new_temp;
static uint16_t net_addr;

/* timeout variables */
static uint16_t sense_timeout;
static uint16_t comm_timeout;

/* statistical variables */
static uint16_t light_total;
static uint16_t temp_total;
static uint16_t sample_count;

static mos_mutex_t sample_lock;
boolean last_sampled_temp = true; 

static void blink_led(uint8_t led_to_display);

/* function prototypes */
void store_sample (uint8_t light, uint8_t temp);
void sense_thread ();
void send_thread ();

/* rf commands - getters */
void sense_timeout_get ();
void comm_timeout_get ();
void rf_power_get ();

/* rf commands - setters */
void sense_timeout_set ();
void comm_timeout_set ();
void rf_power_set ();
void rf_power_get ();
void led_disp ();
   
void start (void)
{
   uint8_t status = 0;
   dev_write (DEV_MICA2_SOUNDER, &status, 1);

   net_addr = mos_node_id_get ();
   
   mos_mutex_init (&sample_lock);
   //net_addr = mos_node_id_get ();
   //printf ("just set net addr to %d %d\n", net_addr, mos_node_id_get ());

   com_mode(IFACE_RADIO, IF_OFF); //keep radio off by default
   // mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
   //	  PRIORITY_NORMAL);
   /*
   mos_thread_new(mos_net_daemon, 128, PRIORITY_NORMAL);
   
   mos_register_rf_function (SENSE_DELAY_SET, sense_timeout_set);
   mos_register_rf_function (SENSE_DELAY_GET, sense_timeout_get);
   mos_register_rf_function (COMM_DELAY_SET, comm_timeout_set);
   mos_register_rf_function (COMM_DELAY_GET, comm_timeout_get);
   mos_register_rf_function (RF_POWER_SET, rf_power_set);
   mos_register_rf_function (RF_POWER_GET, rf_power_get);
   mos_register_rf_function (LEDS, led_disp);
   mos_register_rf_function (CLICK, click);
   */
   comm_timeout =  5000;
   sense_timeout = 2000;
   mos_thread_new (send_thread, 256, PRIORITY_NORMAL);
   mos_thread_new (sense_thread, 128, PRIORITY_NORMAL);
}

void sense_thread ()
{
   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while (1) {
      
      if(last_sampled_temp)
      {
	 dev_mode (DEV_MICA2_TEMP, DEV_MODE_ON);
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 dev_mode (DEV_MICA2_LIGHT, DEV_MODE_OFF);
	 //light temp connected
	 last_sampled_temp = false;
      }
      else
      {
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 last_sampled_temp = true;
      }
      store_sample ((new_light * 100) / 256, new_temp);
      //printf("sense thread (every %d seconds)\n", sense_timeout/1000);
      blink_led (0);
      mos_thread_sleep(sense_timeout);
   }
     
}


void send_thread ()
{
   net_event_t *event;
   
   //insert the data into the packet
   outpacket.size = 8;
   printf ("old net_addr is %d\n", net_addr);
   //mos_node_id_get();
   printf ("new net_addr is %d\n", (net_addr = mos_node_id_get()));

   //reset our statistics
   event = (net_event_t *)outpacket.data;
   *((uint16_t *)&outpacket.data[0]) = net_addr; //from us
   event->to = 0;          //to the base stattion
   event->event = LIGHT_AND_TEMP;
   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while(1)
   {
      if(sample_count > 0)
      {
	 mos_mutex_lock (&sample_lock);
	 {
	    outpacket.data[6] = light_total / sample_count;
	    outpacket.data[7] = temp_total / sample_count;
	    sample_count = 0;
	    temp_total = 0;
	    light_total = 0;
	 }
	 mos_mutex_unlock (&sample_lock);

	 com_send(IFACE_RADIO, &outpacket);
	 }
      //printf("send thread (every %d seconds)\n",comm_timeout/1000);
      blink_led(1);
      mos_thread_sleep (comm_timeout);
   }
}


void store_sample(uint8_t light, uint8_t temp)
{
   mos_mutex_lock (&sample_lock);
   {
      light_total += light;
      temp_total += temp;
      sample_count++;
   }
   mos_mutex_unlock (&sample_lock);
}


/* getters */
void sense_timeout_get(void *p)
{
   send_event_arg (mos_get_event_source (p), SENSE_DELAY_VALUE, sense_timeout);
}

void comm_timeout_get(void *p)
{
   send_event_arg (mos_get_event_source (p), COMM_DELAY_VALUE, comm_timeout);
}
void rf_power_get(void *p)
{
   uint8_t rfPower = 0;
#ifdef PLATFORM_MICA2
   rfPower = cc1000_get_power();
#endif
   send_event_arg8 (mos_get_event_source (p), RF_POWER_VALUE, rfPower);
}

/* setters */
void sense_timeout_set(void *p)
{
   sense_timeout = *(uint16_t *)p;
}

void comm_timeout_set(void *p)
{
   comm_timeout = *(uint16_t *)p;
}

void rf_power_set(void *p)
{
#ifdef PLATFORM_MICA2
   cc1000_set_power (*(uint8_t *)p);
#endif
}

void led_disp(void *p)
{
   mos_led_display (*(uint8_t *)p);
}

static void blink_led(uint8_t led)
{
   mos_led_on(led);
   mos_udelay(2000);
   mos_led_off(led);
}
