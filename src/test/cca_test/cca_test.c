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
#include <stdlib.h>
#include "clock.h"
#include "led.h"
#include "uart.h"
#include "msched.h"
#include "command_daemon.h"
#include "com.h"
#include "dev.h"
#include "avr-rssi.h"

#define CCA_NUM_SAMPLES_INIT 3
#define CCA_FLOOR_INIT  250
uint8_t sent_flag = false;
static uint8_t cca_num_samples = CCA_NUM_SAMPLES_INIT;
static uint16_t cca_noise_floor = CCA_FLOOR_INIT; // Init noise floor to sane value
static uint8_t initBackoff = 5; //initial backoff max period in ms
static uint8_t congestBackoff = 10; //congestion backoff max period 
static mos_thread_t *sendThread; //sending thread
static uint8_t i;
static uint32_t rand_val;

static uint16_t rssi_value;
boolean try_send=false;

static mos_alarm_t    backoff_alarm;  //holds the alarm for the backoff_alarm_func


static boolean is_clear();
static void backoff_alarm_func(void *user_data);
static void cca_test();

void start(void){

   backoff_alarm.func = backoff_alarm_func;
   
   com_mode(IFACE_RADIO, IF_LISTEN);
   //give us control over the serial/rf
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   //gives us start
   mos_thread_new(cca_test, 128, PRIORITY_NORMAL);
}

void cca_test(){

   uint32_t rand_val;
   uint16_t send_val=0;
   while(1) {
      // generate a random backoff ammount
      rand_val = random () % initBackoff + 1;
      
      //get the current threads ID
      sendThread = mos_thread_current();

      // start the backoff alarm
      //mos_alarm(&backoff_alarm, rand_val);
      backoff_alarm.msecs = rand_val;
      backoff_alarm.reset_to = 0;
      mos_alarm(&backoff_alarm);
      
      try_send = true;
      //suspend this thread until backoff alarm wake's us
      mos_thread_suspend();

      //while(!sent_flag);
      sent_flag=false;
      printf("send [%d]\n",send_val++);
   }
}

//this function called when backoff timer expires
//THIS IS IN AN INTERRUPT CONTEXT
static void backoff_alarm_func(void *user_data)
{
   if (is_clear ()) {
      //channel clear, fire off send state machine
      sent_flag = true;
      mos_thread_resume(sendThread);
      backoff_alarm.reset_to = 0;
   }
   else {
      //channel busy, wait again
      rand_val = random () % congestBackoff + 1;
      backoff_alarm.reset_to = rand_val * 2;
      
      //mos_alarm(&backoff_alarm, rand_val * 2);  
   }
}
      
/* check to see if the air is clear for transmission */
static boolean is_clear()
{
   //if we're idle, use bmac algorithm to assess channel
   for(i=0;i<cca_num_samples;i++)
   {
#ifdef PLATFORM_MICA2
      rssi_value = rssi_poll();
#else
      rssi_value = 0;
#endif
      if(rssi_value < cca_noise_floor)
	 return false;
   }
   return true;
}
