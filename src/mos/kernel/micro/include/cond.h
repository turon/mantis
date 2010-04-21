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

/** @file micro/include/cond.h
 * @brief Basic condition variable support.
 *
 * THIS CODE HAS NOT BEEN TESTED
 * 
 * @author Lane Phillips
 * @date Created: 02/18/2005
 */

#ifndef COND_H_
#define COND_H_

#include "msched.h"
#include "tlist.h"
#include "mutex.h"

/** @brief Condition variable data structure */
typedef struct {
   /** @brief Thead list queue waiting on this condition variable */
   tlist_t q;
} mos_cond_t;

/** @brief Init a condition variable data structure.
 *
 * Must init before use.
 * @param cond Condition variable to init
 */
void mos_cond_init(mos_cond_t *cond);

/** @brief Signal exactly one thread waiting on a condition variable.
 *
 * The calling thread should lock the mutex that the waiting thread used to
 * call mos_cond_wait().  Unlike semaphores, signals are lost when no threads
 * waiting.
 * 
 * @param cond Condition variable to signal
 */
void mos_cond_signal(mos_cond_t *cond);

/** @brief Signal all threads waiting on a condition variable.
 *
 * The calling thread should lock the mutex that the waiting threads used to
 * call mos_cond_wait().  Unlike semaphores, signals are lost when no threads
 * waiting.
 * 
 * @param cond Condition variable to signal
 */
void mos_cond_broadcast(mos_cond_t *cond);

/** @brief Wait on a condition variable.
 * 
 * Atomically releases the mutex and waits on the condition variable.  On return
 * the mutex is locked again.  The caller should use the same mutex that the
 * threads that call mos_cond_broadcast() or mos_cond_signal() use for this
 * condition variable.
 * 
 * @param cond Condition variable to wait on
 * @param mt Mutex to unlock.  Locked on return.
 */
void mos_cond_wait(mos_cond_t *cond, mos_mutex_t *mt);

// Not implemented yet
//int8_t mos_cond_timedwait(mos_cond_t *cond, mutex *mt,
//	const struct timespec *abstime);

#endif
