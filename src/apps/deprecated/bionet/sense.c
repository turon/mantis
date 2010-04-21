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

#if defined(PLATFORM_MICA2)
#include "cc1000.h"
#endif

#include "mica2-light-temp.h"

#include "node_net_event.h"
#include "node_id.h"
#include "adc.h"

#include "dev.h"


//static comBuf outpacket;

//static uint8_t net_addr;
//static uint16_t delay;
//static boolean verbose;

/* local rf functions */
//static void click(void *p);
//static void led_disp(void *p);


/* functions for controlling verbosity */
//static void set_verbose();
//static void set_quiet();

//static mos_alarm_t backoff_alarm;  //holds alarm for backoff_alarm_func

/* sense and send thread */
void sense_and_send();

/*
static void backoff_alarm_func(void *user_data)
{
}
*/

void start(void)
{
   //delay=32; //pause 1 sec between sensing.

   //net_addr = mos_node_id_get ();

//   mos_net_daemon_init ();
   
   printf("Welcome to the bionet SENSE application.\n");
   // command daemon, normal user interaction
   //mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);

   //mos_register_function("verbose",set_verbose);
   //mos_register_function("quiet",set_quiet);

   // net daemon, handles commands from base station
   //mos_register_rf_function (LEDS, led_disp);
   //mos_register_rf_function (CLICK, click);

//   mos_thread_new(mos_net_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);

   mos_thread_new(sense_and_send, 192, PRIORITY_NORMAL);

   //verbose=FALSE;
}

void sense_and_send()
{
   //net_event_t *event;
   uint8_t new_light; 
   uint8_t new_temp;

   /*
   backoff_alarm.func = backoff_alarm_func;
   backoff_alarm.data = mos_thread_current();

   uint16_t i;
   outpacket.size = 8;
   event = (net_event_t *)outpacket.data;
   event->from = net_addr; //from us
   event->to = 0;          //to the base stattion
   event->event = LIGHT_AND_TEMP;

   printf("turning on\n");
   */

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ)   

   printf("TCCR3: %C/%C\n", TCCR3A, TCCR3B);
#endif
   
   dev_mode(DEV_ADC, DEV_MODE_ON);
   dev_mode(DEV_MICA2_LIGHT, DEV_MODE_ON);
   while(1) {


      //printf("read1\n");
      dev_read(DEV_MICA2_LIGHT, &new_light, sizeof(new_light));
      //outpacket.data[0] = new_light;


      //i = (uint16_t)(new_light);
      //i = (i * 100)/256;
      //outpacket.data[6] = (uint8_t)i;


      //printf("read2\n");
      dev_read (DEV_MICA2_TEMP, &new_temp, sizeof(new_temp));
      //dev_mode (DEV_ADC, DEV_MODE_OFF);

      //outpacket.data[7] = new_temp;


      //if(verbose)
      printf("L/T: %C/%C\n", new_light, new_temp);

      //printf ("about to send\n");
      //mos_led_toggle (0);
      
//      com_send(IFACE_RADIO, &outpacket);
      //printf ("sent a packet\n");
//      mos_alarm(&backoff_alarm, 0, 500);

      //mos_mdelay(600);
      mos_thread_sleep(500);
   }
}

/*
static void click(void *p)
{
   uint16_t j;
   uint8_t status = 1;
   //mos_led_toggle (0);
   
   dev_write (DEV_MICA2_SOUNDER, &status, 1);
   for(j=0;j<0xffff;j++);
   status = 0;
   dev_write (DEV_MICA2_SOUNDER, &status, 1);
}

static void led_disp(void *p)
{
   //mos_led_toggle (2);
   
   mos_led_display((uint8_t)p);
}

static void set_verbose(){ verbose=TRUE; }
static void set_quiet  (){ verbose=FALSE; }

static void averaging () {
//   dev_ioctl(DEV_ADC, AVR_ADC_AVERAGING_ON);
}

static void noaveraging () {
//   dev_ioctl(DEV_ADC, AVR_ADC_AVERAGING_OFF);
}
*/
