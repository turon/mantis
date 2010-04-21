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

/** @file linux/include/msched.h
 * @brief Implementation of MOS scheduler function on top of pthreads
 * @author Jeff Rose
 * @date 10/24/2002
 * This is a scheduler written for the atmega128 mcu, but it is easily
 * portable to a new platform with a little assembly knowledge.
 */

#ifndef SCHED_H_
#define SCHED_H_

#include <inttypes.h>
#include <pthread.h>

#define MAX_THREADS 15

#define NUM_PRIORITIES 4 // Total number of possible priorities
#define PRIORITY_IDLE (NUM_PRIORITIES - 1)
#define PRIORITY_NORMAL (NUM_PRIORITIES - 2)
#define PRIORITY_HIGH (NUM_PRIORITIES - 3)
#define PRIORITY_KERNEL 0

/* Possible thread states. */
#define EMPTY      0
#define RUNNING    1
#define READY      2
#define BLOCKED    3

/* return codes */
#define THREAD_OK       0
#define NO_MORE_THREADS 1

typedef struct thread_s {
   pthread_t pt;
   pthread_cond_t blocker; //used to suspend/resume thread
   pthread_mutex_t blockerMux; //protects blocker
   void (*func)(void);
   uint8_t state;
   uint8_t priority;
   struct thread_s *next;
} mos_thread_t;

void sched_init(void);
void mos_sched_start(void);
void mos_sched_time_slice(uint8_t slice);
void mos_thread_sleep (uint32_t sleep_time);

uint8_t mos_disable_ints(void);
void mos_enable_ints(uint8_t int_handle);

uint8_t mos_thread_new(void (*function_start)(void),
		       uint16_t stack_size,
		       uint8_t priority);
#define mos_thread_new_havestack(func,size,stack,priority) mos_thread_new(func,size,priority)
void mos_thread_suspend(void);
void mos_thread_suspend_noints(uint8_t int_handle);
void mos_thread_resume(mos_thread_t *thread);
void mos_thread_resume_noints(mos_thread_t *thread, uint8_t int_handle);
mos_thread_t *mos_thread_current(void);
void mos_thread_exit(void);

#endif
