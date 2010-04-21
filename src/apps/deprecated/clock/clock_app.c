//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*   file:  clock_app.c  
 * edited:  Charles Gruenwald III
 *   date:  02/23/04
 *
 */

#include "mos.h"
#include "clock.h"
#include "clock_app.h"
#include "led.h"
#include "command_daemon.h"
#include "msched.h"
#include "printf.h"

// time registers
ATimer atimes;

mos_alarm_t timer;
bool constant_update = false;

void lprintf(uint8_t in_int);

//print the time in MM/DD/YYYY HH:MM:SS format
void print_time(void)
{
   lprintf(atimes.month);
   printf("/");

   lprintf(atimes.day);
   printf("/%l ",atimes.year);
  
   lprintf(atimes.hours);
   printf (":");

   lprintf(atimes.minutes);
   printf (":");

   lprintf(atimes.seconds);

   printf("\r");
}

//this function prints leading zeroes
void lprintf(uint8_t in_int)
{
   if(in_int <= 9)
      printf("0%d",in_int);
   else
      printf("%d" ,in_int);
}

void set_time(void)
{
   atimes.year = prompt_uint8("year: ");
   atimes.month = prompt_uint8("month: ");
   atimes.day = prompt_uint8("day: ");
   atimes.hours = prompt_uint8("hours: ");
   atimes.minutes = prompt_uint8("minutes: ");
   atimes.seconds = prompt_uint8("seconds: ");
}

void set_constant_update(void)
{
   constant_update = true;
}

void stop_constant_update(void)
{
   constant_update = false;
}

void start(void)
{
   clock_app_init();
   //start the command daemon..
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_register_function("set_time", set_time);
   mos_register_function("show_time", print_time);
   mos_register_function("constant", set_constant_update);
   mos_register_function("stop", stop_constant_update);

}

void clock_app_init(void)
{
   atimes.mtics = 0;
   atimes.seconds = 0;
   atimes.minutes = 5;
   atimes.hours = 9;
   atimes.day = 22;
   atimes.month = 2;
   atimes.year = 2004;
   timer.func = clock_service;
   timer.msecs = 1000;
   timer.reset_to = 1000;
   constant_update = false;
   mos_alarm(&timer); //every second
}

/** Clock service, couting the time from the initialization */
void clock_service(void *p)
{
   atimes.seconds++;	  //increment total secs
   mos_led_display(atimes.seconds);
   
   if(atimes.seconds > 59) {
      atimes.seconds -= 60;
      atimes.minutes++;	
      if(atimes.minutes > 59) {
	 atimes.minutes -= 60;
	 atimes.hours++;	
	 if(atimes.hours > 23) {
	    atimes.hours -= 24;
	    atimes.day++;
#ifdef ARCH_AVR
	    if(atimes.day == pgm_read_byte(&MonthDayTable[atimes.month - 1])) {
#else
	    if(atimes.day == MonthDayTable[atimes.month - 1]) {
#endif
	       atimes.day = 1;
	       atimes.month++;
	       if(atimes.month == 13) {
		  atimes.month = 1;
		  atimes.year++;
	       }//month
	    }//day
	 }//hour
      }//minutes
   }//seconds
}

