//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    burleysense.c                                                 */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Sensing application for bionet sensor nodes.                           */
/**************************************************************************/

#include <inttypes.h>

#include "config.h"
#include "msched.h"
#include "led.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"
#include "cc1000.h"
#include "node_id.h"
#include "net_event.h"
#include "net_event_list.h"
#include "mutex.h"
#ifdef ARCH_AVR
#include "dev.h"
#endif
#include "timer.h"

/* shared memory */
static comBuf outpacket;
uint8_t new_light; 
int8_t new_temp;
static uint8_t net_addr;

static bool last_sampled_temp = false;

/* values used for timeout */
static uint16_t sense_delay;
static uint16_t sense_delay_val;
static uint16_t comm_delay;
static uint16_t comm_delay_val;

static net_event_t net_event_storage;

uint16_t light_total;
uint16_t temp_total;
uint16_t sample_count;
static mutex sample_lock;

/* functions */
static void comm_init ();
inline void do_comm ();
inline void do_sense ();
void store_sample (uint8_t light, uint8_t temp);

/* timing */
void sense_and_send_thread();

/* radio functions */
static void click (void *p);
static void led_disp (void *p);
static void comm_delay_set (void *p);
static void comm_delay_get (void *p);
static void sense_delay_set (void *p);
static void sense_delay_get (void *p);
static void rf_power_set (void *p);
static void rf_power_get (void *p);


void start (void)
{
   mos_mutex_init (&sample_lock);
   net_addr = mos_node_id_get ();
   comm_init ();

   mos_thread_new (mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new(sense_and_send_thread, 128, PRIORITY_NORMAL);

   /*
   mos_thread_new (mos_net_daemon, 128, PRIORITY_NORMAL);
   mos_register_rf_function (SENSE_DELAY_SET, sense_delay_set);
   mos_register_rf_function (SENSE_DELAY_GET, sense_delay_get);
   mos_register_rf_function (COMM_DELAY_SET, comm_delay_set);
   mos_register_rf_function (COMM_DELAY_GET, comm_delay_get);
   mos_register_rf_function (LEDS, led_disp);
   mos_register_rf_function (CLICK, click);
   */
   comm_delay = 2500;
   comm_delay_val = 0;
   sense_delay = 500;
   sense_delay_val = 0;
   sample_count = 0;
   temp_total = 0;
   light_total = 0;
}

void sense_and_send_thread (){
   while (1) {
      if(last_sampled_temp)
      {
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 last_sampled_temp = false;
      }
      else
      {
	 dev_read (DEV_MICA2_LIGHT, &new_light, 1);
	 dev_read (DEV_MICA2_TEMP, &new_temp, 1);
	 last_sampled_temp = true;
      }
      outpacket.data[6] = light_total / sample_count;
      outpacket.data[7] = temp_total / sample_count;
      com_send(IFACE_RADIO, &outpacket);
   }
}


static void comm_init ()
{
   net_event_t *event;

   //insert the data into the packet
   outpacket.size = 8;
   //reset our statistics
   event = (net_event_t *)outpacket.data;
   event->from = net_addr; //from us
   event->to = 0;          //to the base stattion
   event->event = LIGHT_AND_TEMP;
}
inline void do_comm ()
{

   }
}

/* setters */
static void led_disp (void *p)
{
   mos_led_display (*(uint8_t *)p);
}

static void rf_power_set (void *p)
{
   cc1000_set_power (*(uint8_t *)p);
}

static void sense_delay_set (void *p)
{
   sense_delay=*(uint16_t *)p;
}

static void comm_delay_set (void *p)
{
   comm_delay=*(uint16_t *)p;
}

/* getters */
static void rf_power_get (void *p)
{
   uint8_t rfPower = cc1000_get_power ();
   send_event_arg8 (mos_get_event_source (p), RF_POWER_VALUE, rfPower);
}

static void sense_delay_get (void *p)
{
   mos_led_display (5);
   send_event_buf (mos_get_event_source (p), SENSE_DELAY_VALUE,
		   (uint8_t *)&sense_delay, sizeof (sense_delay));
}

static void comm_delay_get(void *p){
   send_event_buf(mos_get_event_source(p), COMM_DELAY_VALUE,
		  (uint8_t *)&comm_delay, sizeof(comm_delay));
}

static void click (void *p)
{
   uint8_t j;
   uint8_t status = 1;
   dev_write (DEV_MICA2_SOUNDER, &status, 1);
   for (j = 0; j < 0xff; j++)
      ;
   printf ("chirping.\n");
   mos_led_toggle (2);
   status = 0;
   dev_write (DEV_MICA2_SOUNDER, &status, 1);
}


