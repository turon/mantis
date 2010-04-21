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
  Implements a circular queue for keeping track of bytes.
  
*/

#include "mos.h"
#include "queue.h"

void mos_queue_init(queue_t *q, uint8_t *buffer, uint8_t size)
{
   // the physical memory address where the buffer is stored
   q->buf = buffer;

   // size of the queue
   q->size = size;

   // init the header and tail pointer
   q->head = 0;
   q->length = 0;
}

uint8_t mos_queue_add(queue_t *q, uint8_t byte)
{
   // make sure the buffer has room
   if(q->length < q->size) {
      // save data byte at end of buffer
      q->buf[(q->head + q->length) % q->size] = byte;

      // increment the length
      q->length++;
      return Q_OK;
   }
   else return Q_FULL;
}

uint8_t mos_queue_remove(queue_t *q, uint8_t *byte)
{
   if(q->length) {
      // get the first character from buffer
      *byte = q->buf[q->head];
    
      q->head++;
    
      // once the head reach the end of the buffer
      // then move it to the front
      if(q->head >= q->size)
         q->head %= q->size;
    
      // decrease the count of the data
      q->length--;
      return Q_OK;
   }
   else
      return Q_UNDERFLOW;
}

uint8_t mos_queue_dump(queue_t* q, uint8_t dumpNum)
{
   uint8_t num;
	
   // check if data are more than the requested dumbed number
   if(dumpNum < q->length) {
      // move the head to new position
      q->head += dumpNum;
      // check to see if execeeding the limit
      if(q->head >= q->size)
	 q->head %= q->size;

      q->length -= dumpNum;
      num = dumpNum;
   }
   else { // clean the whole queue
      num = q->length;
      q->length = 0;
      // not necessary to set the head
   }

   return num;
}

uint8_t mos_queue_peek(queue_t *q, uint8_t index, uint8_t *byte)
{
   if(q->length) {
      *byte = q->buf[(q->head + index) % (q->size)];
      return Q_OK;
   }
   return Q_EMPTY;
}

uint8_t mos_queue_length(queue_t *q)
{
   return q->length;
}

void mos_queue_cleanup(queue_t *q)
{
   q->length = 0;
}
