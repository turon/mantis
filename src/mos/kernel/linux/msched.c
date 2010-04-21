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
  File:   sched.c
  Author: Simon Wilson
  Edited by: Jeff Rose (1-20-03)
  Edited by: Brian Shucker 2/7/03
  Date: 10-02

  A multi-threaded scheduler for the atmega128 micro-controller.  
*/

#include "mos.h"

#include <semaphore.h>
#include <unistd.h>

#include "sem.h"
#include "msched.h"

mos_thread_t threads[MAX_THREADS];
sem_t launchDelay;

pthread_mutex_t schedMutex;
pthread_mutex_t main_mutex;
pthread_mutex_t fast_mutex;
pthread_cond_t main_cv;

/** Must call this before adding any threads.
 */
void sched_init()
{
   int i;

   mos_sem_init(&launchDelay, 0);
   pthread_mutex_init(&schedMutex, NULL);
   pthread_mutex_init(&fast_mutex, NULL);

   for(i=0; i<MAX_THREADS; i++)
      threads[i].state = EMPTY;

   threads[0].state = READY;
   threads[0].priority = PRIORITY_IDLE;
}


/** Start scheduling threads
 * Must add thread(s) before calling
 * NOTE: This function never returns.
 */
void mos_sched_start()
{
   pthread_mutex_init(&main_mutex, NULL);
   pthread_cond_init (&main_cv, NULL);

   // We just sit on a conditional variable so the main thread
   //   doesn't take up CPU.  Eventually this could be set so we
   //  can exit cleanly
   pthread_mutex_lock(&main_mutex);
   pthread_cond_wait(&main_cv, &main_mutex);

   /*
     while(1)
     sleep(100);
   */
}

void mos_thread_sleep (uint32_t sleep_time)
{
   usleep (sleep_time * 1000);
}

/* Change the time slice size used by the scheduler
 */
void mos_sched_time_slice(uint8_t slice)
{
   return;
}

/** Just a wrapper for starting threads so we can guarantee that they are
 * de-allocated after completion.
 */
void *start_wrapper(void *empty)
{
   mos_thread_t *t;

   // wait for creation to complete
   mos_sem_wait(&launchDelay);

   t = mos_thread_current();
   t->func();
   mos_thread_exit();
   return NULL;
}

/** Create a new thread and put it on the ready queue.
 * NOTE: There is no memory protection so watch your stack sizes.
 * @param function_start Function to call after starting the thread
 * @param stack_size Stack size associated with the thread
 * @param priority Priority of the thread
 * @return THREAD_OK, NO_MORE_THREADS
 */
uint8_t mos_thread_new(void (*function_start)(void),
		       uint16_t stack_size,
		       uint8_t priority)
{	
   int i;
   pthread_attr_t attr;
   struct sched_param param;

   pthread_mutex_lock(&schedMutex);

   //look for empty thread table entry
   for(i=0; i<MAX_THREADS; i++)
      if(threads[i].state == EMPTY)
	 break;
   if(i==MAX_THREADS)
      return NO_MORE_THREADS;

   threads[i].state = READY;
   threads[i].func = function_start;
   threads[i].priority = NUM_PRIORITIES - priority;
   pthread_cond_init(&threads[i].blocker, NULL);
   pthread_mutex_init(&threads[i].blockerMux, NULL);

   //set thread attributes
   pthread_attr_init(&attr);
   param.sched_priority = priority;
   pthread_attr_setschedparam(&attr, &param);

   pthread_create(&threads[i].pt, &attr, start_wrapper, NULL);

   // now allow it to run, after tables are updated
   mos_sem_post(&launchDelay);

   pthread_mutex_unlock(&schedMutex);

   return THREAD_OK;
}

/** Suspend the current thread
 */
void mos_thread_suspend(void)
{
   mos_thread_t *self = mos_thread_current();
   pthread_mutex_lock(&self->blockerMux);
   pthread_cond_wait(&self->blocker, &self->blockerMux);
   pthread_mutex_unlock(&self->blockerMux);
}

void mos_thread_suspend_noints(uint8_t int_handle)
{
   pthread_mutex_unlock(&fast_mutex);
   mos_thread_t *self = mos_thread_current();
   pthread_mutex_lock(&self->blockerMux);
   pthread_cond_wait(&self->blocker, &self->blockerMux);
   pthread_mutex_unlock(&self->blockerMux);
}

/** Resume a thread
 * @param thread thread_t to resume
 */
void mos_thread_resume(mos_thread_t *thread)
{
   pthread_mutex_lock(&thread->blockerMux);
   pthread_cond_signal(&thread->blocker);
   pthread_mutex_unlock(&thread->blockerMux);
}

void mos_thread_resume_noints(mos_thread_t *thread, uint8_t int_handle)
{
   pthread_mutex_unlock(&fast_mutex);
   pthread_mutex_lock(&thread->blockerMux);
   pthread_cond_signal(&thread->blocker);
   pthread_mutex_unlock(&thread->blockerMux);
}

void mos_thread_resume_noints_nodispatch(mos_thread_t *thread, uint8_t int_handle)
{
   pthread_mutex_lock(&thread->blockerMux);
   pthread_cond_signal(&thread->blocker);
   pthread_mutex_unlock(&thread->blockerMux);
}

/** Get the current thread
 * @return Current thread
 */
mos_thread_t *mos_thread_current()
{
   int i;
   pthread_t current = pthread_self();

   for(i=0; i<MAX_THREADS; i++)
      if(pthread_equal(current, threads[i].pt))
	 return &threads[i];

   return NULL;
}

/** Terminate the current thread.
 */
void mos_thread_exit()
{
   mos_thread_t *t = mos_thread_current();
   t->state = EMPTY;
   pthread_exit(NULL);
}

/** Set the minimum sleep for a thread
 */
