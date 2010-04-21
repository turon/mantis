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

/*************************************************
 Project Mantis 
 File: clock.c

 Original Author: Jeff Rose
 Date: 4/20/04

 Heavily Modified and debugged: Brian, Cyrus, Charles, Adam
 Date: 5/25/04
  
 An alarm timer implementation for the avr
*************************************************/

#include "mos.h"
#include "mutex.h"
#include "msched.h"
#include "clock.h"
#include "printf.h"
#include "plat_dep.h"
#include "plat_clock.h"
#include "led.h"
#include "command_daemon.h"
#include "sem.h"

#ifdef MOS_DEBUG
#include "mos_debugging.h"
static uint16_t dbiter = 0;
#endif

//#define DEBUG_ALARMS

static mos_alarm_t *head; // head of the list of alarms

static mos_mutex_t clock_mutex;
static uint32_t real_time;
extern uint8_t elapsed_thread_time;

static void set_alarm_timer(mos_alarm_t *alarm);

/** @brief Convert ticks to seconds.
 *
 * @param ticks Original time in ticks 
 */
#define to_secs(ticks) (ticks / (uint32_t)TICKS_PER_SEC)

/** @brief Convert seconds to ticks. 
 *
 * @param ticks Original time in seconds
 */
#define to_usecs(ticks) (ticks * (uint32_t)USECS_PER_TICK)

/** @brief Reset the timer. */
#define clear_timer(void) do { ALARM_CNT = 0;	\
   } while(0);

#define get_ticks() (ALARM_CNT)

uint32_t mos_get_realtime(void)
{
   return real_time;
}

void mos_inc_realtime(uint16_t inc)
{
   real_time += inc;
}


void fire_alarm();

static uint16_t pcl_cnt;
static mos_alarm_t *pcl_p;
void print_clock_list(void)
{
   pcl_cnt = 0;
   pcl_p = head;
   
   printf("Clock list: \n");
   while(pcl_p) {
      printf("alarm%d: %l\t%d\n",
	     pcl_cnt++, pcl_p->msecs, pcl_p->data);

      pcl_p = pcl_p->next;
   }
}

void clock_check(void)
{
   pcl_p = head;
   while(pcl_p)
   {
      if(pcl_p->msecs > 100)
      {
	 mos_led_display(7);
	 while(1);
      }
   }
}

#define clock_add(alarm1, alarm2) do { alarm1->msecs += alarm2->msecs; } while (0)

boolean mos_have_alarms(void)
{
   return (head != NULL);
}

mos_alarm_t* mos_get_next_alarm(void)
{
   return head;
}

boolean mos_find_alarm(mos_alarm_t *alarm)
{

   // we don't want the alarm system to service an alarm
   // while we're trying to remove one.
   handle_t int_handle = mos_disable_ints();
   
   mos_alarm_t *current = head;
   mos_alarm_t *prev = NULL;
   uint8_t iter = 0;
   
   if(alarm == NULL)
     return false;

   while(current) {
      if(current == alarm) { //remove this alarm
	 mos_enable_ints(int_handle);
	 
	 return true;
      }
      current = current->next;
   }

   mos_enable_ints(int_handle);
   return false;
}

boolean mos_remove_alarm(mos_alarm_t *alarm)
{

   // we don't want the alarm system to service an alarm
   // while we're trying to remove one.
   handle_t int_handle = mos_disable_ints();
   
   mos_alarm_t *current = head;
   mos_alarm_t *prev = NULL;
   uint8_t iter = 0;
   
   if(alarm == NULL)
     return false;

   while(current) {
      if(current == alarm) { //remove this alarm
	 if(current == head) //removing head element
	    head = current->next;
	 else if(prev) { 
	    prev->next = current->next;
	 }
	 if(current->next) { //add relative time to next timer
	    clock_add(current->next, current);
	 }
	 mos_enable_ints(int_handle);
	 
	 return true;
      }
      prev = current;
      current = current->next;

      if((++iter) > 10)
	 while(1)
	 {
	    mos_led_display(5);
	    mos_mdelay(300);
	    mos_led_display(0);
	    mos_mdelay(300);
	 }
   }

   mos_enable_ints(int_handle);
   return false;
}

static handle_t ma_int_handle;

void mos_alarm(mos_alarm_t *new)
{
   ma_int_handle = mos_disable_ints();
   // prevent the user from creating a timer with
   // a duration of 0.
   if (new->msecs == 0)
   {
      // printf("why would you want an alarm with a time of 0?\n");
      new->msecs = 1;
   }

   
   set_alarm_timer(new);

#ifdef MOS_DEBUG
	 mos_debug_set_trace(DBCODE_TIMER_SET);
#endif	 
 
   mos_enable_ints(ma_int_handle);
}

/*** Functions private to this file. ***/
/** avoiding local variables... **/
static uint32_t total_msecs;
static mos_alarm_t *sat_curr, *sat_prev;
static uint8_t sat_iter;
static void set_alarm_timer(mos_alarm_t *alarm)
{
   sat_iter = 0;
   total_msecs = 0;
   sat_prev = NULL;
   
   if (!head)
   {
      head = alarm;
      head->next = NULL;
      return;
   }

   // HACK - we should never have a duplicate alarm on the list
   mos_remove_alarm(alarm);

   // init the alarm pointer after we ensure it's not on the list
   alarm->next = NULL;

   for(sat_curr = head; sat_curr; sat_curr = sat_curr->next)
   {
      total_msecs += sat_curr->msecs;
     
      if (alarm->msecs < total_msecs)
      {
	 // middle of the list
	 if (sat_prev)
	 {
	    // insert alarm into the list
	    sat_prev->next = alarm;
	    alarm->next = sat_curr;

	    // recalculate the differences
#ifdef DEBUG_ALARMS
            if(total_msecs - sat_curr->msecs > alarm->msecs) {
               mos_led_display(7);
               while(1);
            }
#endif
	    alarm->msecs -= total_msecs - sat_curr->msecs;
#ifdef DEBUG_ALARMS
            if(alarm->msecs > sat_curr->msecs) {
               mos_led_display(5);
               while(1);
            }
#endif
	    sat_curr->msecs -= alarm->msecs;
	    return;
	 }
	 // head of the list
	 else
	 {
	    alarm->next = head;
	    head = alarm;
	    // recalculate the differences
#ifdef DEBUG_ALARMS
            if(alarm->msecs > alarm->next->msecs) {
               mos_led_display(3);
               while(1);
            }
#endif
            alarm->next->msecs -= alarm->msecs;

	    return;
	 }
	 
      }

      if((++sat_iter) > 10)
	 while(1)
	 {
	    mos_led_display(3);
	    mos_mdelay(300);
	    mos_led_display(0);
	    mos_mdelay(300);
	 }
      

      sat_prev = sat_curr;
   }

   // end of the list
   if(!sat_prev)
      return;
   
   
   sat_prev->next = alarm;
   alarm->next = NULL;
#ifdef DEBUG_ALARMS
   if(total_msecs > alarm->msecs) {
      mos_led_display(1);
      while(1);
   }
#endif
   alarm->msecs -= total_msecs;

   return;
   
}




// we need to call the alarm's function AFTER removing it
// from the list since we may call mos_alarm from inside
// this interrupt handler
/* avoid local variables... */
static mos_alarm_t *fa_curr;
static mos_alarm_t *fa_prev;
static uint8_t fa_iter;
static void fire_alarm(void)
{   
//iter_start:
   //fa_prev = NULL;
   fa_curr = head;
   fa_iter = 0;
   while(fa_curr)
   {
      if (fa_curr->msecs == 0)
      {
	 head = fa_curr->next;
	 /*if (!fa_prev)
            head = fa_curr->next;
         else
	    fa_prev->next = fa_curr->next;*/

#ifdef MOS_DEBUG
	 mos_debug_set_trace(DBCODE_TIMER_FIRED);
#endif	 
	 fa_curr->func(fa_curr->data);

	 if (fa_curr->reset_to != 0)
	 {
	    // reset the msecs to the appropriate value
	    fa_curr->msecs = fa_curr->reset_to;
	    
            set_alarm_timer(fa_curr);

#ifdef MOS_DEBUG
	    mos_debug_set_trace(DBCODE_TIMER_SET);
#endif	 
            
         }
      }
      else {
	 break;
      }
      fa_curr = head;

      if((++fa_iter) > 10)
	 while(1)
	 {
	    mos_led_display(7);
	    mos_mdelay(300);
	    mos_led_display(0);
	    mos_mdelay(300);
	 }
   }
   
   /*for(fa_curr = head; fa_curr; fa_curr = fa_curr->next)
   {
      if (fa_curr->msecs == 0)
      {
	 head = fa_curr->next;
	 if (!fa_prev)
            head = fa_curr->next;
         else
	    fa_prev->next = fa_curr->next;

	 fa_curr->func(fa_curr->data);

	 if (fa_curr->reset_to != 0)
	 {
	    // reset the msecs to the appropriate value
	    fa_curr->msecs = fa_curr->reset_to;
	    
            set_alarm_timer(fa_curr);

	    // here we have re-added a timer into the
	    // list, so we need to start over
            goto iter_start;
            
         }
      }
      else {
	 break;
      }
      
      //fa_prev = fa_curr;
   }*/
   
}

void process_alarm(void)
{
   if(head)
   {
      //processing a timer that has awoken us from sleep
      if(head->msecs == 0)
	 fire_alarm();
   }
}


/* Delay for the given number of milliseconds
 * @param usec Number of milliseconds to be delayed
 */
void mos_mdelay(uint16_t msec)
{
   while(msec > 0) {
      // account for the instructions of processing the
      // 16 bit msec variable
      // TODO: is this fudge factor right?
      mos_udelay (950);
      msec--;
   }
}

/* Delay the current thread for give number of microseconds
 * @param usec number of microseconds to be delayed
 */
#if defined(CLOCK_SPEED_8_0)
void mos_udelay(uint16_t usec)
{
   while(usec > 0) {
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      usec--;
   }
}

#elif defined(CLOCK_SPEED_7_37)
void mos_udelay(uint16_t usec)
{
   while(usec > 0) {
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      usec--;
   }
}
#elif defined(CLOCK_SPEED_3_68)
void mos_udelay(uint16_t usec)
{
   while(usec > 0) {
      asm volatile("nop" ::);
      usec--;
   }
}
#elif defined(CLOCK_SPEED_4_0)
void mos_udelay(uint16_t usec)
{
   while(usec > 0) {
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      usec--;
   }
}
#else
#error "Unimplemented clock speed"
#endif

static void tslice_expired(void)
{
   //static uint16_t sleep_time = 0;

   //sleep_time += (10 * TIMESLICE_20_MS) / (CLOCK_SPEED / 100);
   //if (running)
   {
      
      mos_thread_wakeup(elapsed_thread_time);
      //TODO: what should the proper value here be?
      //sleep_time = 0;
   }
   
}


/* avoid local variables */
ALARM_INT_HEADER()
{
   
   if (ALARM_TIMER_EXPIRED)
   {

#ifdef MOS_DEBUG
      if((++dbiter) >= DEBUG_STATUS_CHECK_INTERVAL &&
	 mos_debug_enabled()) // debug check for deadlock/livelock
      {
	 dbiter = 0;
	 mos_debug_status_check();
      }
      
#endif
      
      
      // increment the 'real time' value
      ++elapsed_thread_time;
      ++real_time;
      ++tslice_ms_count;
      
#ifdef PLATFORM_TELOSB
      // modify the status register stored on the stack
      // to exit low power mode.  
      _BIC_SR_IRQ(LPM0_bits);
#endif
      
#ifdef DEBUG_ALARMS
      // if we have a timer in the list and it's expired
      if(head && (head->msecs) == 0) {
         mos_led_display(4);
         while(1);
      }
#endif
      
      //HACK! the head shouldn't ever have 0 msecs here.
      if(head && (head->msecs) == 0)
      {
	 fire_alarm();
      }
      
      if (head && (--(head->msecs) == 0))
      {
         fire_alarm();
      }
      
      if(check_sleep_time(tslice_ms_count)) {
	 tslice_ms_count = 0;
      }
      else if (tslice_ms_count >= 20 &&
	       context_switch_enabled())
      {
	 tslice_ms_count = 0;
	 tslice_expired();
      }
   }

   
}
