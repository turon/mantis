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

/** @file micro/include/sem.h
 * @brief Semphore routines for use on sensor nodes.
 * @author Brian Shucker
 * @date 02/05/2003
 */

#ifndef SEMAPHORE_H_
#define SEMAPHORE_H_

#include <inttypes.h>

#include "tlist.h"

/** @addtogroup mos_kernel Kernel API
 * This group contains functions and variables related
 * to kernel activities, such as power management,
 * threading, sychronization, and so on.
 */
//@{

/** @addtogroup mos_kernel_synch Thread Synchronization
 * This group contains information on thread synchronization
 * primitives such as mutexes and semaphores.
 */
//@{


/** @name Semaphore
 * The Semaphore is a standard thread synchronization construct.
 * A thread may post or wait on a semaphore;  posting to a semaphore
 * increments it's internal value.  Waiting on a semaphore waits
 * until the internal value is non-zero, then decrements it.
 */
//@{

/** @brief mos_sem_try_wait() succeeded in decrementing semaphore. */
#define SEM_SUCCESS 0
/** @brief mos_sem_try_wait() failed in decrementing semaphore (ie mos_sem_wait() would have blocked). */
#define SEM_FAIL 1


/** @brief Semaphore data structure */
typedef struct {
   /** @internal Current semaphore count */
   int8_t val;
   /** @internal List of threads waiting on semaphore */
   tlist_t q;
} mos_sem_t;

/** @brief Initialize a semaphore.
 * @param s A pointer to the semaphore to initialize.
 * @param value Initial value to give the semaphore.
 * Usually zero.
 */
void mos_sem_init(mos_sem_t *s, int value);
/** @brief Post to the semaphore.
 * Increments the internal value of the semaphore,
 * @param s A pointer to the semaphore to post.
 */
void mos_sem_post(mos_sem_t *s);


/** @brief Post to a semaphore and wake up the next
 * thread that is blocked on the semaphore.
 * @param s Semaphore to post.
 */
void mos_sem_post_dispatch(mos_sem_t *s);
/** @brief Post to a semaphore and select which thread
 * that is blocked on the semaphore should wake up next.
 * @param s A pointer to the semaphore to post.
 * @thread the thread id of the thread to wake up next.
 */
void mos_sem_select_post(mos_sem_t *s, uint16_t thread);


/** @brief Wait on a semaphore.
 * Decrements the interal value of the semaphore if it
 * is non-zero.  If the value is zero, blocks until it
 * isn't zero.
 * @param s A pointer to the semaphore to wait on.
 */
void mos_sem_wait(mos_sem_t *s);

/** @brief Test the semaphore and decrement if possible, otherwise return.
 * @param s Semaphore
 * @return SEM_SUCCESS or SEM_FAIL
 */
uint8_t mos_sem_try_wait(mos_sem_t *s);
//@}
//@}
//@}
#endif
