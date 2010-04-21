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
  File: mutex.c
  Author: Jeff Rose
  Date: 1-20-03

  Basic mutex support.
*/

#include "mos.h"
#include "mutex.h"

void mos_mutex_init(mos_mutex_t *mt)
{
   mt->owner = NULL;
   mos_tlist_init(&mt->q);
}

void mos_mutex_lock(mos_mutex_t *mt)
{
   mos_thread_t *id;
   handle_t int_handle;

   id = mos_thread_current(); // Get the current thread ID

   int_handle = mos_disable_ints();
   
   // If its locked then add the thread to the queue and block
   if(mt->owner) {
      mos_tlist_add(&mt->q, id);
      mos_thread_suspend_noints(int_handle);
   } else { // If its free then lock it and save the id
      mt->owner = id;
      mos_enable_ints(int_handle);
   }
}

void mos_mutex_unlock(mos_mutex_t *mt)
{
   mos_thread_t *next;
   mos_thread_t *id;
   handle_t int_handle;
   
   id = mos_thread_current(); // Get the current thread ID

   int_handle = mos_disable_ints();
   
   // Make sure the current thread is the owner before unlocking
   if(mt->owner != id) {
     mos_enable_ints(int_handle);
     return;
   }
   // Now if we have a waiting thread make them ready
   if((next = mos_tlist_remove(&mt->q))) {
      mt->owner = next;
      //mos_thread_resume_noints_nodispatch(next, int_handle);
      //mos_enable_ints(int_handle);
      mos_thread_resume_noints(next, int_handle);
   } else { // Just unlock the mutex and clear the owner
      mt->owner = NULL;
      mos_enable_ints(int_handle);
   }
}

int8_t mos_mutex_try_lock(mos_mutex_t *mt)
{
   mos_thread_t *id;
   handle_t int_handle;
   id = mos_thread_current(); // Get the current thread ID

   int_handle = mos_disable_ints();
   
   // If its locked then return a zero
   if(mt->owner) {
      mos_enable_ints(int_handle);
      return MUTEX_LOCKED;
   } else { // If its free then lock it and save the id
      mt->owner = id;
   }

   mos_enable_ints(int_handle);

   return MUTEX_OK;
}
