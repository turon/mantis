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
#include "command_daemon.h"
#include "msched.h"
/**
 * File:     thread_sleep_test.c      
 * Author:   Charles
 * Date:     05-29-2004
 *
 * Description: This app is for testing the thread_sleep_call().
 *
 */

uint16_t sleep_time=32; //ammount of time to sleep for

void sleepy_thread(); //the thread we'll be putting to sleep
void set_sleep();

void start(void)
{
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new(sleepy_thread, 128, PRIORITY_NORMAL);
   mos_register_function("set_sleep", set_sleep);
}

void sleepy_thread()
{
   uint32_t *ms_ptr;

   printf("Thread Sleep Test App.\n");

   real_timer_init();

   for(;;)
   {
      real_timer_clear();
      mos_thread_sleep(sleep_time);
      ms_ptr=real_timer_get_ticks();
      printf("asked to sleep:\t%d, actual time:\t%l ms\n",
	     sleep_time, *ms_ptr);
   }
}


void set_sleep()
{
   sleep_time=prompt_long("enter the ammount of time to sleep for:");
}
