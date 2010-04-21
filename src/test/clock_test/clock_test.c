//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    blink.c                                                       */
/* Author      Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 12/11/03                                                       */
/* Edited by   Adam Torgerson: adam.torgerson@colorado.edu                */
/*   Date: 12/11/03                                                       */
/* Edited by   Charles Gruenwald  :  gruenwal@colorado.edu                */
/*   Date: 04/14/0/04                                                     */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "com.h"
#include "sem.h"
#include "printf.h"

mos_alarm_t test_timer0, test_timer1, test_timer2, other_timer;
mos_sem_t sem, sem2;
uint32_t test_timer_ms = 250;
uint8_t expired_timer;
void test_timer_callback();
comBuf buf;


void blink_a (void)
{
   uint32_t sleep_time_a = 250;

   test_timer0.func = test_timer_callback;
   test_timer0.data = (void*)0;
   test_timer0.msecs = 250;
   test_timer0.reset_to = 250;

   test_timer1.func = test_timer_callback;
   test_timer1.data = (void*)1;
   test_timer1.msecs = 500;
   test_timer1.reset_to = 500;

   test_timer2.func = test_timer_callback;
   test_timer2.data = (void*)2;
   test_timer2.msecs = 1000;
   test_timer2.reset_to = 1000;


   
   mos_alarm (&test_timer0);
   mos_alarm (&test_timer1);
   mos_alarm (&test_timer2);


   
   while (1) {
      printf("before sem\n");
      
      //mos_thread_sleep (sleep_time_a);
      mos_sem_wait(&sem);
      //sleep toggles yellow
      //printf("%d expired\n", expired_timer);
      
   }


}

void other_timer_callback(void* data)
{
   
   
   //mos_led_toggle(1);
   mos_sem_post(&sem2);
}

void blink_b(void)
{
   /*other_timer.func = other_timer_callback;
   other_timer.msecs = 300;
   other_timer.reset_to = 300;
   mos_alarm(&other_timer);
   */
   while(1) {
      print_clock_list();
      //mos_mdelay(1000);
   }
}

void test_timer_callback(void* data)
{
   expired_timer = (uint8_t)data;
   //  printf("%d expired",expired_timer);
   

   mos_led_toggle(expired_timer);
   mos_sem_post(&sem);
   
   
   
   //  mos_sem_post(&sem);
  //mos_alarm ( &test_timer, test_timer_ms);
}

void start(void)
{
   //mos_led_on(0);
   //mos_led_on(1);
   printf("in start\n");
   
   mos_thread_new (blink_a, 128, PRIORITY_NORMAL);
   //mos_thread_new(blink_b, 128, PRIORITY_NORMAL);
}
