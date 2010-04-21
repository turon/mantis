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
  File: semaphore.c
  Author: Jeff Rose
  Edited by Hui Dai
  Date: 1-30-03

  Standard counting semaphores...
*/

/** @file semaphore.c
 * @brief Standard counting semaphores.
 * @author Jeff Rose
 * @author Edited by Hui Dai
 * @date Created: 01/30/2003
 */

#include "mos.h"

#include "sem.h"
#include "tlist.h"

void mos_sem_init(mos_sem_t *s, int value)
{
   s->val = value;
   mos_tlist_init(&s->q);
}

void mos_sem_select_post(mos_sem_t *s, uint16_t thread)
{
   handle_t int_handle;
   mos_thread_t *selected;
   int_handle = mos_disable_ints();

   // Post a unit and wake-up the next waiting thread
   s->val++;


   
   if((selected = mos_tlist_get_thread(&s->q, thread)) != NULL) {
      //mos_thread_resume_noints_nodispatch(selected, int_handle);
      //mos_enable_ints(int_handle);
      mos_thread_resume_noints(selected, int_handle);
   } else {
      mos_enable_ints(int_handle);
   }
}

void mos_sem_post_dispatch(mos_sem_t *s)
{
    handle_t int_handle;
    mos_thread_t *thread;
    int_handle = mos_disable_ints();
    
    // Post a unit and wake-up the next waiting thread
    s->val++;


    
    if((thread = mos_tlist_remove(&s->q)) != NULL) {
       mos_thread_resume_noints(thread, int_handle);
   } else {
      mos_enable_ints(int_handle);
   }
}

void mos_sem_post(mos_sem_t *s)
{
   handle_t int_handle;
   mos_thread_t *thread;
   int_handle = mos_disable_ints();

   // Post a unit and wake-up the next waiting thread
   s->val++;
   
      
   if((thread = mos_tlist_remove(&s->q)) != NULL) {
      mos_thread_resume_noints_nodispatch(thread, int_handle);
      mos_enable_ints(int_handle);
   } else {
      mos_enable_ints(int_handle);
   }
}

void mos_sem_wait(mos_sem_t *s)
{
   handle_t int_handle;
   int_handle = mos_disable_ints();

   s->val--;

   // If no resources are available then we wait in the queue
   if(s->val < 0) {
      mos_thread_t *id;
      id = mos_thread_current();
      mos_tlist_add(&s->q, id);
      mos_thread_suspend_noints(int_handle);
   } else
      mos_enable_ints(int_handle);
}


uint8_t mos_sem_try_wait(mos_sem_t *s)
{
   handle_t int_handle;
   int_handle = mos_disable_ints();

   if(s->val > 0) {
      s->val--;
      mos_enable_ints(int_handle);
      return SEM_SUCCESS;
   } else {
      mos_enable_ints(int_handle);
      return SEM_FAIL;
   }
}
