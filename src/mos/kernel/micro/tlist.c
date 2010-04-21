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
  File: tlist.c
  Author: Brian Shucker
  Modified: Shah Bhatti, 1/1/04

  Implements lists for tracking threads.
*/

#include "mos.h"

#include "tlist.h"
#include "msched.h"

/*
void mos_tlist_init(tlist_t *list)
{
   list->head = NULL;
   list->tail = NULL;
}

void mos_tlist_add(tlist_t *list, mos_thread_t *item)
{
   //empty list case first
   if(list->head == NULL) {
      list->head = item;
      list->tail = item;
      item->next = NULL;
   } else { //list has at least one element
      list->tail->next = item;
      list->tail = item;
      item->next = NULL;
   }
}
*/

mos_thread_t *mos_tlist_remove(tlist_t *list)
{
   mos_thread_t *remove_front;
   remove_front = list->head;
   if(remove_front != NULL) {
      list->head = remove_front->next;
      remove_front->next = NULL;
   }
   return remove_front;
}

/*
mos_thread_t *mos_tlist_ptrtothread(tlist_t *list)
{
   return list->head;
}
*/

mos_thread_t *mos_tlist_get_thread(tlist_t *list, uint16_t id)
{
   if(list == NULL) return NULL;

   mos_thread_t *front = list->head;
   mos_thread_t *parent = front;

   //if there is only 1 element
   if(front == list->tail){
      if((uint16_t)front == id){
	 list->head = NULL;
	 list->tail = NULL;
	 return front;
      }
      return NULL;
   }
  
   //else, front != tail, has 2 or more elements
   while((front != NULL) && ((uint16_t)front != id)) {
      parent = front;
      front  = front->next;  
   }

   if(front == list->head) {
      list->head = front->next;
      return front;
   } else if(front != NULL) {
      if(front == list->tail)
	 list->tail = parent;
      parent->next = front->next;
   }
   return front;
}

void mos_tlist_ordadd(tlist_t *list, mos_thread_t *item)
{
   mos_thread_t *before;
   mos_thread_t *after;

   //empty list case first
   if(list->head == NULL) {
      list->head = item;
      list->tail = item;
      item->next = NULL;
      return; //and we're done
   }
   
   //insert in order
   before = NULL;
   after = list->head;
   while(after) {
      // item will be inserted before "after"
      if(item->st <= after->st) {
	 after->st -= item->st;
	 item->next = after;
	 // need to set new head
	 if (before == NULL) {
	    list->head = item;
	 } else { //normal insert
	    before->next = item;
	 }
	 break;
      } else {
	 // pass over this one, adjusting relative time
	 item->st -= after->st;
	 before = after;
	 after = after->next;
      }
   }

   if(after == NULL) {
      // insert at tail
      before->next = item;
      item->next = NULL;
      list->tail = item;
   }
}

/*
void mos_tlist_adjustst(tlist_t *list, uint32_t atime)
{
   mos_thread_t *adjust_front = list->head;
  
   while(adjust_front) {
      if(atime > adjust_front->st) {
	 atime -= adjust_front->st;
	 adjust_front->st = 0;
	 adjust_front = adjust_front->next;
      } else {
	 adjust_front->st -= atime;
	 break;
      }
   }

   //if(front) {
   //  if(atime > front->st)
	 //atime = front->st;
	 //  front->st -= atime;
	 //}
}
*/
