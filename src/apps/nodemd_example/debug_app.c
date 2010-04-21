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

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "com.h"
#include "cc2420.h"
#include "dev.h"
#include "clock.h"

#ifdef MOS_DEBUG
debug_checkpoint_t cp_a;
#endif
mos_alarm_t alarm1;

// simulate temperature reading of 30 degrees
static uint8_t temp = 30;


void alarm1_callback()
{
   // expect that our temp will always be over 40
   // this assertion will fail
#ifdef MOS_DEBUG
   ASSERT(temp > 40);
#endif
}

/*
void thread_a (void)
{
   uint32_t sleep_time_a = 2000;

   // init the checkpoint, estimate thread period based on sleep time
   // + 1 ms for other instructions (negligible)
   mos_debug_register_checkpoint(&cp_a, sleep_time_a + 1);


   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while (1) {

      // timestamp the checkpoint every thread period if this
      // instruction is not run in timeout*2 milliseconds, the thread
      // is deadlocked/livelocked
      mos_debug_checkpoint_reached(&cp_a);

      
      mos_led_toggle (0);

      mos_thread_sleep (sleep_time_a);
   }
}
*/

void thread_test (void)
{
   int i;
  uint32_t sleep_time_a = 1;
  uint32_t start_time, end_time;
  
  mos_led_off(2);

#ifdef MOS_DEBUG
  mos_debug_register_checkpoint(&cp_a, sleep_time_a + 1);
#endif

  mos_thread_set_suspend_state(SUSPEND_STATE_IDLE);

  start_time = mos_get_realtime();  

  // This is tested with 10, 100, 1000 and 10000
  for (i = 0; i < 10000; i++) {
#ifdef MOS_DEBUG
     mos_debug_checkpoint_reached(&cp_a);
#endif
    mos_led_toggle(0);
    //mos_thread_sleep(sleep_time_a);
  }

  end_time = mos_get_realtime();
  
  mos_led_on(2);

  printf("Elapsed time is %l ms!\n", end_time - start_time + 1);
}

void start(void)
{
  
   //set a "buggy" alarm to 10 seconds
   alarm1.msecs = 10000;
   alarm1.reset_to = 0;
   alarm1.func = alarm1_callback;
 
   mos_alarm(&alarm1);
  
  
   // set a "breakpoint" that will show up in the event trace, useful
   // for creating a specific point of reference
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_BREAKPOINT);
#endif

   //mos_thread_new (thread_a, 128, PRIORITY_NORMAL);
   mos_thread_new (thread_test, 128, PRIORITY_NORMAL);
}

