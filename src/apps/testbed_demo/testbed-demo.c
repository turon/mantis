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
#include "printf.h"

void blink (void)
{
  uint32_t sleep_time_a = 2000;
  int8_t a;
   
  mos_led_display(1);
  printf("Changed the led\n");
   
  mos_thread_sleep (sleep_time_a);
  mos_led_display(2);
  printf("Changed the led\n");
  mos_thread_sleep (sleep_time_a);
  mos_led_display(4);
  printf("Changed the led\n");
  mos_thread_sleep (sleep_time_a);
  mos_led_display(2);
  printf("Changed the led\n");
  mos_thread_sleep (sleep_time_a);
  mos_led_display(1);
  printf("Changed the led\n");
  mos_thread_sleep (sleep_time_a);
  mos_led_display(0);
   
  while (1) {
    for(a=0;a<7;a++)
      {
        mos_led_display(a);
        printf("Changed the led\n");
        mos_thread_sleep (sleep_time_a);
      }
    for(a=5;a>=1;a--)
      {
        mos_led_display(a);
        printf("Changed the led\n");
        mos_thread_sleep (sleep_time_a);
      }
  }
}


//This is a standard debugging thread
void debug(void)
{
  mos_thread_sleep(100);//wait for the computer to get ready for us.

  while (1)
    {
//	mos_thread_sleep(1000);
        printf(".");
    }
}


void start(void)
{
  mos_thread_new (blink, 128, PRIORITY_NORMAL);

//put a radio thread here

   mos_thread_new (debug,128, PRIORITY_NORMAL);
   
}
