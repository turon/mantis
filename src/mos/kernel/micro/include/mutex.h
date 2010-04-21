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

/** @file micro/include/mutex.h
 * @brief Basic mutex support.
 *
 * @author Jeff Rose
 * @date Created: 01/20/2003
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#include "msched.h"
#include "tlist.h"
/** @addtogroup mos_kernel Kernel API */
//@{

/** @addtogroup mos_kernel_synch Thread Synchronization */
//@{


/** @name Mutex
 * The Mutex is a standard thread synchronization construct.
 * A thread may lock or unlock a mutex;  only one thread may
 * have a mutex locked at once, and only that thread is allowed
 * to unlock the mutex.  
 */
//@{

/** @brief mos_mutex_try_lock() failed: Mutex is locked by another thread. */
#define MUTEX_LOCKED 1
/** @brief mos_mutex_try_lock() succeeded: Mutex is now locked by this thread. */
#define MUTEX_OK 0

/** @brief Mutex data structure */
typedef struct mos_mutex_s {
   /** @internal Pointer to thread this mutex belongs to */
   struct thread_s *owner;
   /** @internal Thead list queue waiting on this mutex */
   tlist_t q;
} mos_mutex_t;

/** @brief Initialize a mutex.
 * You must initialize a mutex before using it.
 * @param mt A pointer to the mutex to initialize.
 */
void mos_mutex_init(mos_mutex_t *mt);

/** @brief Lock a mutex.
 * Blocks the thread if the mutex is already locked.
 * @param mt A pointer to the mutex to lock.
 */
void mos_mutex_lock(mos_mutex_t *mt);

/** @brief Unlock a mutex so another thread can enter the protected area of code.
 * @param mt A pointer to the mutex to unlock.
 */
void mos_mutex_unlock(mos_mutex_t *mt);
/** @brief Try to lock a mutex, but do not block if it is already owned.
 * @param mt A pointer to the mutex to attempt to lock. 
 * @return MUTEX_LOCKED if another thread already has a lock, MUTEX_OK otherwise.
 */
int8_t mos_mutex_try_lock(mos_mutex_t *mt);

/** @deprecated Simply disables interrupts.  Returns a handle. */
#define mos_fast_mutex_lock() mos_disable_ints()
/** @deprecated Re-enables interrupts.
 * Pass the handle returned from most_fast_mutex_lock(). */
#define mos_fast_mutex_unlock(handle) mos_enable_ints(handle)

//@}
//@}
//@}

#endif
