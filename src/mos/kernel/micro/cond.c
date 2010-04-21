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

/** @file mos/kernel/avr/cond.c
 * @brief Basic condition variable support.
 *
 * THIS CODE HAS NOT BEEN TESTED
 * 
 * @author Lane Phillips
 * @date Created: 02/18/2005
 */
 
#include "mos.h"

#include "msched.h"
#include "tlist.h"
#include "mutex.h"
#include "cond.h"

void mos_cond_init(mos_cond_t *cond)
{
   mos_tlist_init(&cond->q);
}

void mos_cond_signal(mos_cond_t *cond)
{
   // If we have a waiting thread make it ready
   mos_thread_t *next;
   if ((next = mos_tlist_remove(&cond->q)) != NULL) {
      mos_thread_resume(next);
   }
}

void mos_cond_broadcast(mos_cond_t *cond)
{
   // Wake up all waiting threads
   mos_thread_t *next;
   while ((next = mos_tlist_remove(&cond->q)) != NULL) {
      mos_thread_resume(next);
   }
}

void mos_cond_wait(mos_cond_t *cond, mos_mutex_t *mt)
{
   // Get the current thread ID
   mos_thread_t *id = mos_thread_current();
   // Disable interrupts so we can unlock and wait atomically
   handle_t int_handle = mos_disable_ints();
   // Unlock the mutex
   mos_mutex_unlock(mt);
   // Put us on the list for the condition variable
   mos_tlist_add(&cond->q, id);
   // Suspend us with interrupts already disabled
   mos_thread_suspend_noints(int_handle);
   // Lock the mutex again before returning
   mos_mutex_lock(mt);
}

// Not implemented yet
//int8_t mos_cond_timedwait(mos_cond_t *cond, mutex *mt,
//	const struct timespec *abstime)
//{
//}
