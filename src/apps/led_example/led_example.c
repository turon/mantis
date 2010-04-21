//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/** @file led_example.c
 *  @author John Ledbetter
 *
 * This file demonstrates how to play with the LEDs using
 * MOS.  It should be pretty self explanatory.
 */

#include "mos.h"
#include "led.h"
#include "printf.h"
#include "msched.h"

void start(void)
{

   printf("LED Manipulation Example\n");

   while(1)
   {
      // this first segment of code uses the mos_led_display
      // function to display the value of a number. Since
      // there are 3 LEDs, the maximum number we can display
      // using our binary LEDs is 7 (111b).
      uint8_t i = 0;

      printf("counting from 0 to 7 using mos_led_display().\n");
      
      for(i = 0; i < 8; ++i)
      {
	 mos_led_display(i);
	 mos_thread_sleep(1000);
      }

      
      printf("toggling using mos_led_toggle().\n");
      for(i = 0; i < 10; ++i)
      {
	 // toggle one of the LEDs (0-2)
	 mos_led_toggle(i % 3);
	 mos_thread_sleep(1000);
      }

      
      printf("setting state using mos_led_on() and mos_led_off()\n");
      
      mos_led_on(0);
      mos_thread_sleep(1000);

      mos_led_off(0);
      mos_led_on(1);

      mos_thread_sleep(1000);
   }
   
}
