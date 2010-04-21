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
/*   Date: 04/14/04                                                       */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"

void blink_a (void)
{
   uint32_t sleep_time_a = 2000;
   
   while (1) {
      mos_led_toggle (0);
      mos_thread_sleep (sleep_time_a);
   }
}

void blink_b (void)
{

   uint32_t sleep_time_b = 5000;
   while (1) {
      mos_led_toggle (1);
      mos_thread_sleep (sleep_time_b);
   }  
}

void blink_c (void)
{

   uint32_t sleep_time_c = 200;
   while (1) {
      mos_led_toggle (2);
      mos_thread_sleep (sleep_time_c);
   }  
}
   
/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void)
{
   mos_thread_new (blink_a, 128, PRIORITY_NORMAL);
   mos_thread_new (blink_b, 128, PRIORITY_NORMAL);
   mos_thread_new (blink_c, 128, PRIORITY_NORMAL);
}
