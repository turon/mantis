
//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

/*
  Project Mantis
  File: clock.c
  Author: Jeff Rose
  Date: 4/20/04
  
  A set of typical clock and timer based services for the MOS system.
*/

/** @file clock.h
 * @brief An alarm timer implementation for the avr.
 *
 * @author Original Author: Jeff Rose
 * @author Heavily Modified and debugged: Brian, Cyrus, Charles, Adam
 * @date Created: 04/20/2004 
 * @date Modified: 05/25/2004
*/

#ifndef __CLOCK_H__
#define __CLOCK_H__


#include "mos.h"

#include <inttypes.h>

/** @brief A standard time struct */
typedef struct mos_time_
{
   /** @brief Seconds */
   uint32_t sec;
   /** @brief Microseconds */
   uint32_t usec;
} mos_time_t;

typedef void (*alarm_func)(void *user_data);

/** @brief A structure to hold an alarm
 *
 *   An alarm is attached to a hardware timer, along with
 *   a callback function which is called when the alarm triggers.
 *   Alarms are stored in a linked list, with the relative number
 *   of ticks between each node in the list.
 *
 *   NOTE: The callback function will be executed in an interrrupt
 *   context.
 */

typedef struct mos_alarm_
{
   /** @brief Callback function to call on alarm trigger */
   alarm_func func;
#ifdef PLATFORM_LINUX
   mos_time_t time;
#endif
   /** @brief Data parameter to pass to callback function */
   void *data;
   /** @brief Number of millseconds until alarm triggers */
   uint32_t msecs;
   uint32_t reset_to;
   /** @brief Next alarm */
   struct mos_alarm_ *next;
} mos_alarm_t;


#if !defined(PLATFORM_LINUX)

/** @brief ticks per second */
/** @brief microseconds per tick */
#if defined( CLOCK_SPEED_8_0 )
   #define TICKS_PER_SEC 36600
   #define TICKS_PER_MSEC 45
   #define USECS_PER_TICK 28
#elif defined( CLOCK_SPEED_7_37 )
   #define TICKS_PER_SEC 28800
   #define TICKS_PER_MSEC 29
   #define USECS_PER_TICK 35
#elif defined( CLOCK_SPEED_3_68 )
   #define TICKS_PER_SEC 14400
   #define TICKS_PER_MSEC 14
   #define USECS_PER_TICK 69
#elif defined( CLOCK_SPEED_4_0 )
   #define TICKS_PER_SEC 14400
   #define TICKS_PER_MSEC 14
   #define USECS_PER_TICK 69
#else
   #error "Timeslice not defined for clock. (msched.c)"
#endif

#endif

static uint16_t tslice_ms_count = 0;

#define RESET_TSLICE_COUNTER() tslice_ms_count = 0;

/** @brief Initialize the clock. 
 *
 * Set the interrupt mode, set the prescaler. 
 */
void clock_init();

/** @brief Delay function in microseconds. 
 *
 * @param usec Microseconds to delay
 */
void mos_udelay(uint16_t usec);

/** @brief Delay function in milliseconds.
 *
 * @param msec Milliseconds to delay
 */ 
void mos_mdelay(uint16_t msec);

/** @brief Inserts a new alarm at the given secs and usecs from the current time.
 *
 * Accounts for inserting the new alarm into the alarm list in order and adjusting the relative times of the alarm list as it is called.
 * @param new New alarm to be inserted
 * @param secs Relative time in secs
 * @param usecs Relative time in microseconds
 */
#if defined(PLATFORM_LINUX)
int8_t mos_alarm(mos_alarm_t *alarm);
#else

/** @brief Inserts a new alarm at the given number of ticks from the current time.
 *
 * Accounts for inserting the new alarm into the alarm list in order and adjusting the relative times of the alarm list as it is called.
 * Use this if you want to avoid conversion math.
 * @param new New alarm to be inserted
 * @param ticks Relative time in ticks
 */
void mos_alarm(mos_alarm_t *new);
#endif

/** @brief Convenience function for printing an alarm list. */
void print_clock_list (void);

/** @brief Get the value of the 1msecs timer counter.
 *
 * Each 1msecs, a timer interrupt is generated. This is a count of the number
 * of times that interrupt is serviced; it will overflow in 2^32msecs
 */
uint32_t mos_get_realtime(void);

/** @brief increment the realtime counter.
 * Used upon waking up from deep sleep to increment the timer
 * by the number of milliseconds for which we slept.
 * @param inc the number of milliseconds to increment by.
 */
void mos_inc_realtime(uint16_t inc);


boolean mos_find_alarm(mos_alarm_t *alarm);

boolean mos_remove_alarm(mos_alarm_t *alarm);

/** @brief returns TRUE if there are alarms left in the alarm list;
 * ie, head != NULL.
 */
boolean mos_have_alarms(void);

void resetclock();
void fixme();

#endif
