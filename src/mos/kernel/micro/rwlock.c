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
  File: rwlock.c
  Author: Lane Phillips
  Date: 2005-02-25

  Basic read-write locking support.
*/

/* *** THIS CODE HAS NOT BEEN TESTED, but please give it a try. *** */

#include "mos.h"
#include "rwlock.h"

void rwlock_init(mos_rwlock_t *lock)
{
   mos_tlist_init(&lock->rq);
   mos_tlist_init(&lock->wq);
   lock->rlocked = 0;
   lock->wlocked = 0;
}

void rwlock_rdlock(mos_rwlock_t *lock)
{
   // Get the current thread
   mos_thread_t *id = mos_thread_current();
   // Disable interrupts so we can check and lock atomically
   handle_t int_handle = mos_disable_ints();
   // Is it write locked?
   while (lock->wlocked) {
      // Yes, add ourselved to the list of read waiters
      mos_tlist_add(&lock->rq, id);
      // Suspend with interrupts already disabled
      mos_thread_suspend_noints(int_handle);
      // Between being resumed and disabling interrupts here, someone else 
      // could have grabbed the lock, so we loop and check again
      int_handle = mos_disable_ints();
   }
   // Multiple threads may hold a read lock, count us too
   lock->rlocked++;
   mos_enable_ints(int_handle);
}

uint8_t rwlock_tryrdlock(mos_rwlock_t *lock)
{
   handle_t int_handle = mos_disable_ints();
   // Test the lock
   if (lock->wlocked) {
      mos_enable_ints(int_handle);
      // But don't wait for it
      return RWLOCK_LOCKED;
   }
   lock->rlocked++;
   mos_enable_ints(int_handle);
   return RWLOCK_OK;
}

void rwlock_wrlock(mos_rwlock_t *lock)
{
   mos_thread_t *id = mos_thread_current();
   handle_t int_handle = mos_disable_ints();
   // Is it read or write locked?
   while (lock->wlocked || lock->rlocked) {
      // Yes, add ourselved to the list of write waiters
      mos_tlist_add(&lock->wq, id);
      mos_thread_suspend_noints(int_handle);
      // Between being resumed and disabling interrupts, someone else could
      // have grabbed the lock, so we loop and check again
      int_handle = mos_disable_ints();
   }
   // Only one thread at a time can have a write lock
   lock->wlocked = 1;
   mos_enable_ints(int_handle);
}

uint8_t rwlock_trywrlock(mos_rwlock_t *lock)
{
   handle_t int_handle = mos_disable_ints();
   // Test the lock
   if (lock->wlocked || lock->rlocked) {
      mos_enable_ints(int_handle);
      // But don't wait for it
      return RWLOCK_LOCKED;
   }
   lock->wlocked = 1;
   mos_enable_ints(int_handle);
   return RWLOCK_OK;
}

void rwlock_unlock(mos_rwlock_t *lock)
{
   mos_thread_t *next;
   handle_t int_handle = mos_disable_ints();
   if (lock->wlocked) {
      // Write locks are not shared, so our caller must be the guy who has it
      // WARNING: We are obviously not checking who the owner of the lock is,
      // so don't write bad code!
      lock->wlocked = 0;
      // Wake up one write waiter
      if ((next = mos_tlist_remove(&lock->wq)) != NULL) {
	 mos_thread_resume(next);
      } else {
	 // If there are no write waiters, wake up all the read waiters
	 while ((next = mos_tlist_remove(&lock->rq)) != NULL) {
	    mos_thread_resume(next);
	 }
      }
   } else if (lock->rlocked) {
      // WARNING: We are still not checking the owner
      lock->rlocked--;
      if (lock->rlocked==0) {
	 // If no one holds a read lock anymore, wake up one write waiter
	 if ((next = mos_tlist_remove(&lock->wq)) != NULL) {
	    mos_thread_resume(next);
	 }
      }
   }
   mos_enable_ints(int_handle);
}
