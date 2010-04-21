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
  Modified: Lane Phillips
  Date: 11/9/04
  
  A set of typical clock and timer based services for the MOS system.
*/

#include "clock.h"
#include "msched.h"
#include "mutex.h"

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <semaphore.h>

void alarm_handler(int num);
void alarm_thread();

mos_alarm_t *head;		// Linked list of alarms
sem_t alarm_sem;	// Alarm handler posts here to notify alarm thread
mos_mutex_t alarm_lock;	// Synchronizes access to the alarm list
long sec_diff;		// For simulating mos_set_time()
long usec_diff;

void inline mos_udelay(uint16_t usec)
{
   usleep(usec);
}

void inline mos_mdelay(uint16_t msec)
{
   usleep(msec * 1000);
}

/* Print out a single clock */
void print_clock(mos_alarm_t *alarm){
   printf(" Alarm: ");
   if(alarm->time.sec > 0)
      printf("%d sec ",alarm->time.sec);
   if(alarm->time.usec > 0)
      printf("%d usec ",alarm->time.usec);

   printf("'%s' \n",(char *)alarm->data);
}

/* Print the entire clock list */
void print_clock_list ()
{
   mos_alarm_t *aptr = head;
   printf ("Clock list: \n");
   while (aptr) {
      print_clock(aptr);
      aptr = aptr->next;
   }
   
}

/* remove an alarm from the list */
boolean mos_remove_alarm(mos_alarm_t *alarm)
{
   mos_alarm_t *current = head;
   mos_alarm_t *prev = NULL;
   
   if(alarm == NULL) return false;
   
   mos_mutex_lock(&alarm_lock);
   while(current){
      if(current == alarm){ //remove this alarm
	 if(current == head) //removing head element
	    head=current->next;
	 
	 if(prev){ 
	    prev->next = current->next;
	 }
	 mos_mutex_unlock(&alarm_lock);
	 // Let the alarm thread know something has changed
	 sem_post(&alarm_sem);
	 return true;
      }
      prev = current;
      current=current->next;
   }
   mos_mutex_unlock(&alarm_lock);
   return false;
}

/* Set an alarm to send a SIGALARM signal after a given period. */
int8_t mos_alarm(mos_alarm_t *alarm)
{
   mos_alarm_t *aptr, *prev;
   struct timeval t;
 
   uint32_t msecs = alarm->msecs;
   
   gettimeofday(&t, NULL);
   alarm->time.sec = t.tv_sec + (msecs / 1000);
   alarm->time.usec = t.tv_usec + (msecs %1000)*1000;

   mos_mutex_lock(&alarm_lock);
   // Empty list
   if(head == NULL) {
      alarm->next = NULL;
      head = alarm;

      mos_mutex_unlock(&alarm_lock);
      // Let the alarm thread know something has changed
      sem_post(&alarm_sem);
      return 0;
   }

   // First get the elapsed time and update the head.
   aptr = head;
   prev = NULL;
      
   // Run through the list to find the correct spot
   while(aptr) {
      if((alarm->time.sec < aptr->time.sec) ||
	 (alarm->time.sec == aptr->time.sec &&
	  alarm->time.usec < aptr->time.usec))
      {
	 // We break out if we find the correct spot.  That way if
	 //  the loop hits the end we can add there too
	 break;
      }

      prev = aptr;
      aptr = aptr->next;
   }

   mos_mutex_unlock(&alarm_lock);
   // Let the alarm thread know something has changed
   sem_post(&alarm_sem);
   return 0;
}

void clock_init()
{
   head = NULL;
   sem_init(&alarm_sem, 0, 0);
   mos_mutex_init(&alarm_lock);
   sec_diff = 0;
   usec_diff = 0;
   
   // Make the alarm handling thread high priority so we handle alarms quickly.
   mos_thread_new(alarm_thread, 128, PRIORITY_HIGH);
   
   // Please read the VERY IMPORTANT NOTE in alarm_handler().
   signal(SIGALRM, &alarm_handler);
}

/* Simulate setting the time by storing an offset from the real time. */
int8_t mos_set_time(mos_time_t *tv)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   sec_diff = tv->sec - t.tv_sec;
   usec_diff = tv->usec - t.tv_usec;
   return 0;
}

/* Return simulated time by adding offset to the real time. */
int8_t mos_get_time(mos_time_t *tv)
{
   struct timeval t;
   gettimeofday(&t, NULL);
   tv->sec = t.tv_sec + sec_diff;
   tv->usec = t.tv_usec + usec_diff;
   return 0;
}

/*** Functions private to this file. ***/

void alarm_handler(int num)
{
   /* VERY IMPORTANT NOTE:
    * 
    * The POSIX standard defines a list of "safe" functions.  A safe function
    * may be called from a signal handler even when the signal has interrupted
    * that same function.  The behavior of unsafe functions in this situation
    * is undefined.  (alarm_handler() used to freeze Linux systems before it 
    * was fixed.)  See the man page for signal(2) for a list of safe functions.
    */

   // Notify the alarm handling thread that we got a signal.
   // sem_post is safe and does not block
   sem_post(&alarm_sem);
}

/* This thread does most of the work of safely processing alarms.  See the
 * VERY IMPORTANT NOTE in alarm_handler().  To avoid race conditions that could
 * occur when adding or deleting alarms from the list, this is the ONLY place
 * where the interval timer should be set.  This thread should be run at a 
 * higher priority so it can handle alarms quickly.  This thread spends most of
 * the time waiting on a semaphore, so it should not starve other threads.
 */
void alarm_thread()
{
   mos_alarm_t *aptr = NULL;
   mos_alarm_t *prev = NULL;
   mos_alarm_t *expired = NULL;	// Linked list of expired alarms
   struct timeval t;
   struct itimerval itval;
	
   // Interval timer does not restart
   itval.it_interval.tv_sec = 0;
   itval.it_interval.tv_usec = 0;

   while (1)
   {
      // Wait for something to happen.
      sem_wait(&alarm_sem);
      // Either an alarm signal just came in, or the alarm list was changed
	
      expired = NULL;
      mos_mutex_lock(&alarm_lock);
		
      // What time is it now?
      gettimeofday(&t, NULL);
      // Build a list of expired alarms
      if (head && (head->time.sec < t.tv_sec ||
		   (head->time.sec == t.tv_sec && head->time.usec < t.tv_usec)))
      {
	 // We don't actually "build" a new linked list, we just point to the
	 // original head of the list.
	 expired = head;
	 prev = head;
	 aptr = head->next;
	 while (aptr && (aptr->time.sec < t.tv_sec ||
			 (aptr->time.sec == t.tv_sec && aptr->time.usec < t.tv_usec)))
	 {
	    prev = aptr;
	    aptr = aptr->next;
	 }
	 // Break the expired list off of the list of pending alarms
	 head = aptr;
	 prev->next = NULL;
      }
		
      if (head) {
	 // Set the interval timer to signal the next unexpired alarm.
	 itval.it_value.tv_sec = head->time.sec - t.tv_sec;
	 // Modular math to avoid negative numbers
	 itval.it_value.tv_usec = (head->time.usec + 1000000 - t.tv_usec)%1000000;
	 if (setitimer(ITIMER_REAL, &itval, NULL))
	    perror("alarm_thread");
      } else {
	 // The list is empty, make sure the timer is stopped.
	 itval.it_value.tv_sec = 0;
	 itval.it_value.tv_usec = 0;
	 if (setitimer(ITIMER_REAL, &itval, NULL))
	    perror("alarm_thread");
      }
		
      mos_mutex_unlock(&alarm_lock);
		
      // Now we call the handlers for the expired alarms.
      // We didn't do this before, because the list was locked, and some
      // alarm handlers may want to add or remove alarms.
      for (aptr=expired; aptr; aptr = aptr->next)
      {
	 if(aptr->func != NULL)
	    aptr->func(aptr->data);
      }
   }
}
