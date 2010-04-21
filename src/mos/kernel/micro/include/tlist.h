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

/** @file tlist.h
 * @brief Implements lists for tracking threads.
 * @author Brian Shucker
 * @author Modified: Shah Bhatti
 * @date Modified: 01/01/2004
 */

#ifndef TLIST_H_
#define TLIST_H_

#include <inttypes.h>

#include "msched.h"

/** @brief Thread list data structure (a queue) */
typedef struct {
   /** @brief Thread at head of queue */
   mos_thread_t *head;
   /** @brief Thread at tail of queue */
   mos_thread_t *tail;
} tlist_t;

/** @brief Initializes a thread list.
 */
#define mos_tlist_init(list) do {		\
      (list)->head = NULL; 			\
      (list)->tail = NULL;			\
   } while(0)

/** @brief Adds a new thread to end of thread list.
 * @param list Thread list to add to
 * @param item Thread to add to list
 */
#define mos_tlist_add(list,item) do {		\
      if((list)->head == NULL) {		\
	 (list)->head = item;			\
	 (list)->tail = item;			\
	 (item)->next = NULL;			\
      } else {					\
	 (list)->tail->next = item;		\
	 (list)->tail = item;			\
	 (item)->next = NULL;			\
      }						\
   } while(0)

/** @brief Removes thread from front of thread list.
 * @param list Thread list to remove from
 * @return Thread removed from list (NULL if empty)
 */
mos_thread_t *mos_tlist_remove(tlist_t *list);

/** @brief Removes specific thread from thread list.
 * @param list Thread list to remove from
 * @param id id of thread to remove
 * @return Thread removed (NULL if empty)
 */
mos_thread_t *mos_tlist_get_thread(tlist_t *list, uint16_t id);

/** @brief Adds a new thread to ordered thread list.
 */
void mos_tlist_ordadd(tlist_t *list, mos_thread_t *item);

/** @brief Adjusts the sleepQ with elapsed time. 
 */
#define mos_tlist_adjustst(list,atime) do {	\
      mos_thread_t *adjust_front = (list)->head;	\
      while(adjust_front) {			\
	 if(atime > adjust_front->st) {		\
	    atime -= adjust_front->st;		\
	    adjust_front->st = 0;		\
	    adjust_front = adjust_front->next;	\
	 } else {				\
	    adjust_front->st -= atime;		\
	    break;				\
	 }					\
      }						\
   } while(0)

/** @brief Pointer to thread at the front of thread list.
 * @param list Thread list to get thread from
 * @return Thread removed from list (NULL if empty)
 */
#define mos_tlist_head(list) (list)->head

#endif
