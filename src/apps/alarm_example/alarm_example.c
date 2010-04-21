//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/** @file alarm_example.c
 *  @author John Ledbetter
 *
 * This file demonstrates the mos alarm system.
 */

#include "mos.h"
#include "clock.h"
#include "printf.h"
#include "sem.h"
#include "led.h"
// our alarm variable. making this global is a good
// idea, since the pointer to it must be valid as
// long as our alarm is active.
mos_alarm_t alarm;
// this semaphore allows us to wake a thread when
// our alarm expires.  
mos_sem_t alarm_sem;

uint8_t i = 1;

// the alarm callback executes inside of an interrupt context,
// so if a large amount of code needs to run, we should post
// a semaphore like so to leave interrupt context.
void alarm_callback(void* data)
{
   mos_led_toggle(2);
   // modifying the reset_to value lets us change
   // the period of the timer on the fly.  what we're
   // doing here is setting the timer to expire in
   // an increasing amount of seconds; ie, first wait
   // 1 second, then 2 seconds, then 3, and so on.
   // setting this value to zero will stop the alarm.
   alarm.reset_to = (++i * 1000);
   
   mos_sem_post(&alarm_sem);
}

// this function will wait for our alarm to expire, then
// use some special functions to determine how much time
// elapsed.
void consumer_thread(void)
{
   while(1)
   {
      // this function sends a command to the mos_shell
      // telling it to reset it's timer.
      mos_clear_ext_timer();
      mos_sem_wait(&alarm_sem);
      // this function sends a command to the mos_shell
      // telling it to display the elapsed time since the
      // timer was cleared.
      mos_show_ext_timer();
      
      printf("alarm expired\n");
   }
   
}


void start(void)
{
   // intialize our semaphore.
   // a value of 0 means that any mos_sem_wait() will block
   // until at least one mos_sem_post() occurs.
   mos_sem_init(&alarm_sem, 0);
   // create our consumer thread, which will wait for our
   // alarm to expire.
   mos_thread_new(consumer_thread, 128, PRIORITY_NORMAL);

   // finally, create our alarm.
   alarm.func = alarm_callback;
   // our initial period is 1 second.
   alarm.msecs = 1000;
   
   // add our alarm to the system.
   mos_alarm(&alarm);
   
}

