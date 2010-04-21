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

/** @file micro/include/rwlock.h
 * @brief Basic read-write locking support.
 *
 * @author Lane Phillips
 * @date Created: 02/25/2005
 */

#ifndef RWLOCK_H_
#define RWLOCK_H_

#include "mos.h"
#include "tlist.h"



/** @addtogroup mos_kernel Kernel API */
//@{

/** @addtogroup mos_kernel_synch Thread Synchronization */
//@{

/** @name Read-Write Lock
 * The Read-Write lock allows cross-thread passing of
 * data, allowing a thread to lock either for reading
 * or writing.
 */
//@{

/** @brief This is returned from the rw_try_ functions if the RWLock is locked. */
#define RWLOCK_LOCKED 1
/** @brief This is returned from the rw_try_ functions if the RWLock was acquired. */
#define RWLOCK_OK 0

/** @brief Rwlock data structure */
typedef struct mos_rwlock_t {
   tlist_t rq;
   tlist_t wq;
   uint8_t rlocked;
   uint8_t wlocked;
} mos_rwlock_t;


/** @brief Initialize a read-write lock.
 *
 * Must init before use.
 * @param lock Read-write lock to init
 */
void rwlock_init(mos_rwlock_t *lock);

/** @brief Lock a read-write lock for reading.
 *
 * This lock may be shared with other threads that have called rwlock_rdlock().
 * Blocks the thread if the read-write lock is already locked for writing.
 * @param lock Read-write lock to lock
 */
void rwlock_rdlock(mos_rwlock_t *lock);

/** @brief Lock a read-write lock for reading if it is free.
 *
 * This lock may be shared with other threads that have called rwlock_rdlock().
 * Does not block.
 * @param lock Read-write lock to lock
 * @return RWLOCK_LOCKED if another thread already has a lock, else RWLOCK_OK
 */
uint8_t rwlock_tryrdlock(mos_rwlock_t *lock);

/** @brief Lock a read-write lock for writing.
 *
 * Only one thread will hold a write lock at a time.
 * Blocks the thread if the read-write lock is already locked for reading or writing.
 * @param lock Read-write lock to lock
 */
void rwlock_wrlock(mos_rwlock_t *lock);

/** @brief Lock a read-write lock for writing if it is free.
 *
 * Only one thread will hold a write lock at a time.
 * Does not block.
 * @param lock Read-write lock to lock
 * @return RWLOCK_LOCKED if another thread already has a lock, else RWLOCK_OK
 */
uint8_t rwlock_trywrlock(mos_rwlock_t *lock);

/** @brief Unlock a read-write lock.
 * 
 * Release both read and write locks.  Threads that are blocking on write locks
 * will wake up before threads blocking on read locks.
 * @param lock Read-write lock to unlock
 */
void rwlock_unlock(mos_rwlock_t *lock);

// Not implemented
/*void rwlock_timedrdlock(mos_rwlock_t *lock,
            const struct timespec *restrict);
void rwlock_timedwrlock(mos_rwlock_t *lock,
            const struct timespec *restrict);*/

//@}
//@}
//@}
#endif
