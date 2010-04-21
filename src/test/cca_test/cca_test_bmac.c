//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     cca_test.c      
 * 
 */

#include "mos.h"

#include "printf.h"
#include <inttypes.h>
#include <stdlib.h> //gives us random()
#include "node_id.h"
#include "clock.h"
#include "led.h"
#include "uart.h"
#include "msched.h"
#include "command_daemon.h"
#include "com.h"
#include "sem.h"
#include "dev.h"
#include "avr-rssi.h"
#include "avr-adc.h"

//variables that may conflict with real csma
#define CCA_NUM_SAMPLES 20
#define CCA_FLOOR_INIT  0x120
static uint16_t cca_noise_floor = CCA_FLOOR_INIT;
static uint16_t cca_sample_queue[CCA_NUM_SAMPLES]; //for computing cca estimate
static uint8_t  initBackoff = 5; //initial backoff max period in ms

static mos_alarm_t  backoff_alarm;  //holds the alarm for the backoff_alarm_func
static uint16_t rssi_value;

static uint32_t rand_val;

//functions that may conflict with real csma
//alarm funcs
static void backoff_alarm_func(void *user_data);

//static sem timer_sem;
static mos_sem_t timer_sem;

//threads
static void bmac_test();
static void rssi_print();

uint16_t global_rssi_value;
uint8_t sent_flag = false;

void start(void){
   mos_sem_init (&timer_sem, 0);
   
   //give us control over the serial/rf
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);

   mos_thread_new(bmac_test, 128, PRIORITY_NORMAL);
   mos_thread_new(rssi_print, 128, PRIORITY_NORMAL);
}

void bmac_test(){
   uint8_t i;

   com_mode(IFACE_RADIO, IF_LISTEN);

   // Initialize the clear channel assesment queue.
#ifdef PLATFORM_MICA2
   cc1000_rssi_on ();
   for(i = 0; i < CCA_NUM_SAMPLES; i++) {
      rssi_value = adc_read_channel16 (AVR_ADC_CH_0);
      cca_sample_queue[i] = rssi_value;
   }
   cc1000_rssi_off ();
#endif
   
   printf("start rssi val: %d\n", rssi_value);
   
   //start sampling noise floor
   //noise_alarm.func = noise_sample_alarm_func;
   rand_val = random () % initBackoff + 1;
   backoff_alarm.func = backoff_alarm_func;
   backoff_alarm.data = (void *)&timer_sem;
   backoff_alarm.msecs = rand_val;
   backoff_alarm.reset_to = 0;

   mos_alarm(&backoff_alarm);
   //mos_alarm(&backoff_alarm, rand_val);

   mos_sem_wait (&timer_sem);
   
   while(1) {
      rand_val = random () % initBackoff + 1;
      backoff_alarm.msecs = rand_val;
      mos_alarm(&backoff_alarm);
//      mos_alarm(&backoff_alarm, rand_val);
      printf ("waiting on timer sem\n");
      mos_sem_wait (&timer_sem);
      printf ("awoke from timer sem\n");
      sent_flag=true;
   }
}

//this function called when backoff timer expires
//THIS IS IN AN INTERRUPT CONTEXT
static void backoff_alarm_func(void *user_data)
{
   mos_sem_post (&timer_sem);
}

void rssi_print()
{
   uint16_t old_noise_floor = cca_noise_floor;
   while (1) {
      if (cca_noise_floor != old_noise_floor) {
	 printf ("noise floor%d", cca_noise_floor);

	 if (sent_flag) {
	    printf(" sent!!!!");
	    sent_flag = false;
	 }
	 printf ("\n");
	 old_noise_floor = cca_noise_floor;
      }
   }
}
