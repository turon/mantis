//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#ifndef _CLOCK_APP_H_
#define _CLOCK_APP_H_

// constants/macros/typdefs
typedef struct AbsoluteTimerStructure {	
   uint16_t mtics; 
   uint8_t hours;
   uint8_t minutes;
   uint8_t seconds;
   uint8_t day;
   uint8_t month;
   uint16_t year;
} ATimer;

// month days constants
#ifdef ARCH_AVR
static uint8_t __attribute__ ((progmem)) MonthDayTable[] = {31, 28, 31, 30, 31, 30,
							    31, 31, 30, 31, 30, 31};
#else
static uint8_t MonthDayTable[] = {31, 28, 31, 30, 31, 30,
				  31, 31, 30, 31, 30, 31};
#endif

void clock_app_init(void);
void clock_service(void *p);
void clock_set_second(uint8_t sec);
void clock_set_minute(uint8_t min);
void clock_set_hours(uint8_t h);
void clock_set_day(uint8_t d);
void clock_set_month(uint8_t mon);
void clock_set_years(uint16_t y);
ATimer* clock_get_time(void);

#endif /* _CLOCK_APP_H_ */
