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

/** @file mos/kernel/linux/include/rwlock.h
 * @brief Basic read-write locking support.
 *
 * @author Lane Phillips
 * @date Created: 02/26/2005
 */

#ifndef RWLOCK_H_
#define RWLOCK_H_

#include "mos.h"
#include <pthread.h>

/** @brief Read-write lock is locked */
#define RWLOCK_LOCKED EBUSY
/** @brief Read-write lock is unlocked */
#define RWLOCK_OK 0

typedef pthread_rwlock_t mos_rwlock_t;

#define rwlock_init(lock)		pthread_rwlock_init(lock,NULL)
#define rwlock_rdlock(lock)		pthread_rwlock_rdlock(lock)
#define rwlock_tryrdlock(lock)	pthread_rwlock_tryrdlock(lock)
#define rwlock_wrlock(lock)		pthread_rwlock_wrlock(lock)
#define rwlock_trywrlock(lock)	pthread_rwlock_trywrlock(lock)
#define rwlock_unlock(lock)		pthread_rwlock_unlock(lock)

// Not implemented
/*void rwlock_timedrdlock(lock,
            const struct timespec *restrict);
void rwlock_timedwrlock(lock,
            const struct timespec *restrict);*/

#endif
