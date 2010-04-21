//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "printf.h"
#include "realtime.h"
#include "clock.h"
#include "led.h"

/**
 * File:     realtime_test.c      
 * Author:   Charles
 * Date:     05-29-2004
 *
 * Description: This app is for testing the real time timer interface
 *
 */
static uint8_t flag=0;
static mos_alarm_t timer_alarm;
static uint32_t time;

void timer_alarm_func(void *data)
{
   mos_led_toggle(1);
//   mos_alarm(&timer_alarm, 10 * 1000);
   time = *real_timer_get_ticks();
   flag=1;
}

void start(void)
{
   printf("Realtime Test app, should display info every 1 minute.\n");
   timer_alarm.func = timer_alarm_func;

   real_timer_init();
   real_timer_clear();
   timer_alarm.reset_to = 10 * 1000;
   timer_alarm.msecs = 10 * 1000;
   //mos_alarm(&timer_alarm, 10 * 1000);
   mos_alarm(&timer_alarm);
   
   for(;;) {
      if(flag) {
	 flag=0;
	 printf("time:\t%l ms\n",time);
      }
   }
   
}
