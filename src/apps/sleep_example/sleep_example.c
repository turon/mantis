//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/** @file sleep_example.c
 *  @author John Ledbetter
 *
 * This file demonstrates the various sleeping methods of the
 * MOS system and explains the differences between them.
 */

#include "mos.h"
#include "msched.h"
#include "printf.h"

void sleepy_thread(void)
{
   uint16_t delay = 1000;
   
   while(1)
   {
      printf("busywaiting for %d seconds...", delay/1000);
      // mos_mdelay() is a busywait loop that delays for a
      // certain number of milliseconds.  The CPU is wide
      // awake for the duration of the delay.
      mos_mdelay(delay);
      printf("done.\n");

      printf("idling for %d seconds...", delay/1000);
      // SUSPEND_STATE_IDLE is the default suspend state
      // for all threads.  Idle mode occurs when all
      // running threads are blocked. The CPU enters a
      // lower power mode, but the system is still awake.
      mos_thread_set_suspend_state(SUSPEND_STATE_IDLE);
      // mos_thread_sleep() suspends the current thread for
      // the specified number of milliseconds.  During this
      // time, the CPU will either enter idle mode or a deep
      // sleep mode, depending on ALL threads' specified
      // suspend states.
      mos_thread_sleep(delay);
      printf("done.\n");

      
      printf("sleeping for %d seconds...", delay/1000);
      // SUSPEND_STATE_SLEEP is a the deep sleep state
      // where the CPU is completely turned off for the
      // duration of the sleep.  Keep in mind that ALL
      // threads must specified SUSPEND_STATE_SLEEP in order
      // for the system to enter deep sleep mode. Otherwise,
      // idle mode is used.
      mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
      mos_thread_sleep(delay);
      printf("done.\n");

      delay += 1000;
   }
   
}




void start(void)
{
   mos_thread_new(sleepy_thread, 128, PRIORITY_NORMAL);
}
