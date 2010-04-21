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

/**
   Project Mantis
   File:   com.c
   Authors: Jeff Rose & Brian Shucker
   Date:   01-18-04
  
**/

#include "mos.h"

#ifdef PLATFORM_LINUX
#include <signal.h>
// So com.c will compile for Linux
// TODO maybe Linux needs its own plat_dep.h file
typedef uint8_t handle_t;
#endif

#include "clock.h"
#include "mem.h"
#include "led.h"
#include "com.h"
#include "sem.h"
#include "msched.h"
#include "mutex.h"

#ifdef RADIO_USE_FEC
#include "fec.h"
// pull these in from cc1000.c
extern uint8_t data_fec[FEC_DATA_PARITY_COUNT];
extern uint16_t cc1000_fec_error_count;
extern uint16_t cc1000_crc_error_count;
extern uint16_t cc1000_success_count;
#endif

typedef struct {
   mos_thread_t *thread_ptr;
   boolean timed_out;
} timed_thread_t;

static comBuf *get_free_buf(void);
static void save_full_buf(uint8_t iface, comBuf *buf);
static void timeout_callback(void *data); //callback from timer

/** @brief Mutexes for sending on each iface. */
mos_mutex_t if_send_mutexes[MAX_IFS];	

/** @brief Table of thread ids per iface. */
static mos_thread_t *if_threads[MAX_IFS];

/** @brief The free buffer list. */
static comBuf *free_bufs;     
/** @brief Separate full lists for each iface. */   
static comBuf *if_bufs[MAX_IFS];

#ifdef MOS_NO_USE_DYNAMIC_MEMORY
static comBuf com_bufs[NUM_BUFS];
#endif



/************ Interfaces for upper layers. *************/

uint8_t com_init(void)
{
   uint8_t i;
   handle_t int_handle;
   comBuf *buf;

   int_handle = mos_fast_mutex_lock();
   
   free_bufs = NULL; // Init the empty list
   
   // Clear the function table, init the recv semaphores
   // & init the free buf list
   for(i = 0; i < MAX_IFS; i++) {
      if_threads[i] = NULL;
      mos_mutex_init(&if_send_mutexes[i]);
      if_bufs[i] = NULL;
   }
   
   // Allocate the buffers and create the free buffer list
   for(i = 0; i < NUM_BUFS; i++) {
#ifdef MOS_NO_USE_DYNAMIC_MEMORY
      buf = &com_bufs[i];
#else
      buf = (comBuf*)mos_mem_alloc(sizeof(comBuf));
#endif
      if(buf) {
	 buf->next = free_bufs;
	 free_bufs = buf;
      } else {
#ifndef PLATFORM_LINUX
	 mos_led_display (5);
	 mos_mdelay (500);
	 mos_led_display (7);
	 mos_mdelay (500);
	 mos_led_display (5);
	 mos_mdelay (500);
	 mos_led_display (7);
	 mos_mdelay (500);
#endif
      }
   }
   mos_fast_mutex_unlock(int_handle);
   
   return 0;
}

#ifndef PLATFORM_LINUX
static mos_alarm_t timer;
comBuf *com_recv_timed(uint8_t iface, uint32_t msecs)
{
   comBuf *buf;
   timed_thread_t current;
   boolean timer_enabled = false;
   current.timed_out = false;
   current.thread_ptr = mos_thread_current();
   handle_t int_handle = mos_fast_mutex_lock();

   // Make sure another thread isn't already waiting on this iface
   if(if_threads[iface] != NULL) {
      mos_fast_mutex_unlock(int_handle);
      return NULL;
   }

   // If we don't have a waiting buf then save the threadID in the table
   // and suspend
   if(if_bufs[iface] == NULL) {
      timer.func = timeout_callback;
      timer.data = &current;
      
      timer.msecs = msecs;
      timer.reset_to = 0;
      mos_alarm(&timer);

      timer_enabled = true;
      if_threads[iface] = mos_thread_current();

      //mos_thread_new(mytest, 128, PRIORITY_HIGH);
      mos_thread_suspend_noints(int_handle);
      int_handle = mos_fast_mutex_lock();
   }

   if(current.timed_out || !if_bufs[iface]) {
      if_threads[iface] = NULL;
      mos_fast_mutex_unlock(int_handle);
      return NULL;
   }
   
#ifdef RADIO_USE_FEC
   uint8_t ret = FEC_UNCORRECTABLE_ERRORS;
   
   if(iface == IFACE_RADIO) {
      while(ret == FEC_UNCORRECTABLE_ERRORS) {  // is this while loop suitable for timed com_recv?
	 fec_init(FEC_DATA_PARITY_COUNT);
	 if((ret = fec_decode(if_bufs[iface]->data,
			      if_bufs[iface]->size,
			      data_fec)) != FEC_NO_ERRORS) {
	    if(ret == FEC_UNCORRECTABLE_ERRORS) {
	       cc1000_crc_error_count++;
	       
         //com_free_buf(if_bufs[iface]);    if_bufs[iface] = NULL;
         buf = if_bufs[iface]; 
         if_bufs[iface] = buf->next; 
         com_free_buf(buf); 
	       
         while(if_bufs[iface] == NULL) {
		       if_threads[iface] = mos_thread_current();
		       mos_thread_suspend_noints(int_handle);
		       int_handle = mos_fast_mutex_lock();
	       }
	    } else {
	       cc1000_fec_error_count++;
	    }
	 }
      }
      cc1000_success_count++;
   }
#endif
   
   // Grab the first buffer on the list and shift the head pointer
   buf = if_bufs[iface];
   if_bufs[iface] = buf->next;

   // Take this thread out of the table and unlock the mutex
   if_threads[iface] = NULL;
   if(timer_enabled)
      mos_remove_alarm(&timer);
   mos_fast_mutex_unlock(int_handle);
   
   return buf;
}
#endif

comBuf *com_recv_noblock(uint8_t iface)
{
   comBuf *buf;
   handle_t int_handle = mos_fast_mutex_lock();

   // Make sure another thread isn't already waiting on this iface
   if(if_threads[iface] != NULL) {
      mos_fast_mutex_unlock(int_handle);
      return NULL;
   }

   // If we don't have a waiting buf then return NULL
   if(if_bufs[iface] == NULL) {
      mos_fast_mutex_unlock(int_handle);
      return NULL;
   }

   buf = if_bufs[iface];
   if_bufs[iface] = buf->next;
   mos_fast_mutex_unlock(int_handle);
   return buf;
}

comBuf *com_recv(uint8_t iface)
{
   comBuf *buf;
   handle_t int_handle = mos_fast_mutex_lock();

   // Make sure another thread isn't already waiting on this iface
   if(if_threads[iface] != NULL) {
      mos_fast_mutex_unlock(int_handle);
      return NULL;
   }

   // If we don't have a waiting buf then save the threadID in the table
   // and suspend
try_recv_again:
   if(if_bufs[iface] == NULL) {
      if_threads[iface] = mos_thread_current();
      mos_thread_suspend_noints(int_handle);
      int_handle = mos_fast_mutex_lock();
   }

#ifdef RADIO_USE_FEC
   uint8_t ret = FEC_UNCORRECTABLE_ERRORS;
   
   if(iface == IFACE_RADIO) {
      while(ret == FEC_UNCORRECTABLE_ERRORS) {
	 fec_init(FEC_DATA_PARITY_COUNT);
	 if((ret = fec_decode(if_bufs[iface]->data,
			      if_bufs[iface]->size,
			      data_fec)) != FEC_NO_ERRORS) {
	    if(ret == FEC_UNCORRECTABLE_ERRORS) {
	       cc1000_crc_error_count++;
	       
         //com_free_buf(if_bufs[iface]); if_bufs[iface] = NULL;               
         buf = if_bufs[iface]; 
         if_bufs[iface] = buf->next; 
         com_free_buf(buf); 
         buf = NULL;
             
	       goto try_recv_again;
	    } else {
	       cc1000_fec_error_count++;
	    }
	 }
      }
#ifdef RADIO_EXTRA_CRC
      if_bufs[iface]->size -= 2;
      uint16_t crc = *(uint16_t *)&if_bufs[iface]->data[if_bufs[iface]->size];
      if (crc_compute(if_bufs[iface]->data, if_bufs[iface]->size) != crc) {
	 cc1000_crc_error_count++;

	 //com_free_buf(if_bufs[iface]);   if_bufs[iface] = NULL;
         buf = if_bufs[iface]; 
         if_bufs[iface] = buf->next; 
         com_free_buf(buf); 
         buf=NULL;

	 goto try_recv_again;
      } else
#endif
	 cc1000_success_count++;
   }
#endif
   
   // Grab the first buffer on the list and shift the head pointer
   buf = if_bufs[iface];
   if_bufs[iface] = buf->next;
   
   // Take this thread out of the table and unlock the mutex
   if_threads[iface] = NULL;
   mos_fast_mutex_unlock(int_handle);
   
   return buf;
}

//This will flush all of the packets on a particular iface.
void com_flush(uint8_t iface)
{
   comBuf *recv;
   IF_SET set;
   while(1) { //break out of the loop on null recv packet
      IF_ZERO(&set);
      IF_SET(iface, &set);
      com_select(&set, false);
      if(IF_ISSET(iface, &set)) {
	 recv = com_recv(iface);
	 com_free_buf(recv);
      } else
	 return;
   }   
}

uint8_t com_select(IF_SET *iset, uint32_t msec)
{
   uint8_t i, bit, ifcount;
   handle_t int_handle;
   uint8_t retval = 0;
   IF_SET oset;
   IF_ZERO(&oset); // Start with an empty set
   ifcount = 0; // So we can return the number of if's with bufs ready

   int_handle = mos_fast_mutex_lock();
   
   // Make sure no other threads are holding any of these ifaces or unregistered
   for(i = 0; i < MAX_IFS; i++) {
      bit = 1 << i;
      if(*iset & bit) { // Check if a given bit is set
	 // If one of the ifaces is already taken we return an error
	 if (if_threads[i] != NULL)
	    retval = SELECT_IFACE_BUSY;
	    
	 // We check for bufs here too just to save having to do another loop
	 if (if_bufs[i] != NULL){
	    oset |= bit;
	    ifcount++;
	 }
      }
   }

   // Just return the error if an iface is already taken
   if(retval == SELECT_IFACE_BUSY || retval == IFACE_NOT_REGISTERED) {
      mos_fast_mutex_unlock(int_handle);
      return retval;
   } else if(oset || msec == 0) {
      // Otherwise return if their are bufs to be had
      *iset = oset;
      mos_fast_mutex_unlock(int_handle);
      return ifcount;
   }

   mos_alarm_t com_timer;
   timed_thread_t current;
   current.timed_out = false;
   // If we get here we need to grab all the relevant ifaces and suspend until
   // something comes
   current.thread_ptr = mos_thread_current();
   for(i = 0; i < 8; i++) {
      bit = 1 << i;
      if(*iset & bit) // Check if a given bit is set
	 if_threads[i] = current.thread_ptr;
   }

   if(msec != 0xffffffff) {
      com_timer.func = timeout_callback;
      com_timer.data = &current;
      com_timer.msecs = msec;
      com_timer.reset_to = 0;
      mos_alarm(&com_timer);
   }

   mos_thread_suspend_noints(int_handle);
   int_handle = mos_fast_mutex_lock();
   
   //check to see if com_select timed out
   if(msec != 0xffffffff) {
      if(current.timed_out) {
	 for(i = 0; i < 8; i++) {
	    bit = 1 << i;
	    if (*iset & bit){ // Check if a given bit is set
	       if_threads[i] = NULL;
	    }
	 }
	 *iset = oset;
	 mos_fast_mutex_unlock(int_handle);
	 return ifcount;
      } else { //didn't time out, need to stop alarm
	 mos_remove_alarm(&com_timer);
      }
   }
   // We have been woken up so a buf should be available somewhere.  Create the
   // IF_SET to send back and get out of here
   for(i = 0; i < 8; i++) {
      bit = 1 << i;
      if(*iset & bit) { // Check if a given bit is set
	 if(if_bufs[i] != NULL) {
	    oset |= bit;
	    ifcount++;
	 }

	 // Now let go of each iface we had taken
	 if_threads[i] = NULL;
      }
   }

   *iset = oset;
   mos_fast_mutex_unlock(int_handle);
   return ifcount;
}

void com_free_buf(comBuf *buf)
{
   if (buf == NULL)
      return;
   
   handle_t int_handle = mos_fast_mutex_lock();
   buf->next = free_bufs;
   free_bufs = buf;
   buf = NULL;
   mos_fast_mutex_unlock(int_handle);
}

char is_elf_packet(comBuf *buf) {
   if (buf->size < 4) {
   	return 0;
   }

   if (buf->data[0] == ELFPKT_BYTE0 && 
	   buf->data[1] == ELFPKT_BYTE1 && 
	   buf->data[2] == ELFPKT_BYTE2 && 
	   buf->data[3] == ELFPKT_BYTE3) {
	return 1;
   }
   
  return 0;
}

/** Interfaces for lower driver layers. **/

void com_swap_bufs(uint8_t iface, comBuf *buf, comBuf **ret)
{
   handle_t int_handle = mos_fast_mutex_lock();
   
   if(buf == NULL) { // Just allocate a new buf
      *ret = get_free_buf();
   } else {
      mos_thread_t *p;   

#ifdef USING_ELF

   if (is_elf_packet(buf)) {
      elfbuf = buf;
      mos_thread_resume(elfthread);
   } else {
#endif
      // save the buffer to the ifaces list and then wake a
      // waiting thread if need be Finally, return a new buffer
      save_full_buf(iface, buf);

      // If a thread is waiting then wake it up
      if(if_threads[iface] != NULL) {
	 p = if_threads[iface];
	 if_threads[iface] = NULL;
	 mos_thread_resume_noints_nodispatch(p, int_handle);
	 //mos_thread_resume_noints(p, int_handle);
      }
	 
      *ret = get_free_buf();
      
#ifdef USING_ELF
   }
#endif
      
   }

   mos_fast_mutex_unlock(int_handle);
   return;
}

/*** Some functions private to this file. ***/

/** @brief Save a full buffer.
 * @param iface Interface to use
 * @param buf Buffer to save
 */
static void save_full_buf(uint8_t iface, comBuf *buf)
{
   comBuf *buf_iter;
   buf->next = NULL;

   //If its an empty list than save the buf to the head
   if(if_bufs[iface] == NULL) {
      if_bufs[iface] = buf;
      return;
   }

   // Traverse the list until we hit the end and then save the buf
   for(buf_iter = if_bufs[iface]; buf_iter->next != NULL; buf_iter = buf_iter->next)
      ;
   buf_iter->next = buf;
   return;
}


/** @brief Find a free buffer. 
 * @return The next free buffer or NULL if they are all full
 */
static comBuf *get_free_buf(void)
{
   comBuf *buf;
   
   if(free_bufs != NULL) {
      buf = free_bufs;
      free_bufs = buf->next;
      return buf;
   } else {
      return NULL;
   }
}

/** @brief A timed select callback has expired */
void timeout_callback(void *data)
{
   timed_thread_t *ptr = (timed_thread_t *)data;
   ptr->timed_out = true;
   mos_thread_resume_noints_nodispatch(ptr->thread_ptr);   
}




void buf_insert16(uint8_t* p, uint16_t pos, uint16_t word)
{
   p[pos++] = (word >> 0) & 0xFF;
   p[pos]   = (word >> 8) & 0xFF;
}


void buf_insert32(uint8_t* p, uint16_t pos, uint32_t dword)
{
   p[pos++] = (dword >>  0) & 0xFF;
   p[pos++] = (dword >>  8) & 0xFF;
   p[pos++] = (dword >> 16) & 0xFF;
   p[pos]   = (dword >> 24) & 0xFF;
}

uint16_t buf_extract16(uint8_t* p, uint16_t pos)
{
   uint16_t ret = 0;

   
   ret |= (p[pos++] << 0);
   ret |= (p[pos]   << 8);

   return ret;
   
}

uint32_t buf_extract32(uint8_t* p, uint16_t pos)
{
   uint32_t ret = 0;
   
     
   ret |= ((uint32_t)(p[pos++]) <<  0);
   ret |= ((uint32_t)(p[pos++]) <<  8);
   ret |= ((uint32_t)(p[pos++]) << 16);
   ret |= ((uint32_t)(p[pos])   << 24);
   
   return ret; 
}
