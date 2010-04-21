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

#ifndef QUEUE_H_
#define QUEUE_H_

#include <inttypes.h>

/*error codes*/
#define Q_OK 0
#define Q_FULL 1
#define Q_UNDERFLOW 2
#define Q_EMPTY 3

/** @file queue.h
 * @brief Small circular queue library
 */

/** @brief Queue data structure */
typedef struct
{
  /** @brief Data pointer to the buffer */
  uint8_t *buf;    
  /** @brief Maximum size of the buffer */
  uint8_t size;    
  //  uint8_t head, tail;
  /** @brief Length of the actual data */
  uint8_t length;  
  /** @brief Head of the buffer */
  uint8_t head;    
  //  uint8_t tail = (head + length - 1) % size
} queue_t;

/** @brief Init the queue.
 *
 * Inits a new byte queue. The elements of the queue will be stored
 * in buffer.
 * @param q Queue to init
 * @param buffer A byte array
 * @param size Size of buffer
 */
void mos_queue_init(queue_t *q, uint8_t *buffer, uint8_t size);

/** @brief Add a new byte to the tail of the queue.
 * @param q Queue to add a byte to
 * @param byte Byte to add
 * @return Q_OK or Q_UNDERFLOW
 */
uint8_t mos_queue_add(queue_t *q, uint8_t byte);

/** @brief Return the length of a given queue.
 * @param q Queue to check length of
 * @return Length of the queue
 */
uint8_t mos_queue_length(queue_t *q);

/** @brief Remove the first byte from the head of the queue.
 * @param q Queue to remove a byte from
 * @param byte Byte that was removed
 * @return Q_OK or Q_UNDERFLOW
 */
uint8_t mos_queue_remove(queue_t *q, uint8_t *byte);

/** @brief Dump specific number of bytes out from the buffer.
 *
 * These bytes are removed from the buffer
 * @param q Queue to dump bytes from
 * @param dumpNum Number of bytes to dump
 * @return Number of dumped bytes
 */
uint8_t mos_queue_dump(queue_t *q, uint8_t dumpNum);

/** @brief "peek" one byte from the queue.
 *
 * Doesn't delete the byte, just looks
 * @param q Queue to peek
 * @param index Index to peek at
 * @param byte Peeked byte
 * @return Q_OK, Q_EMPTY
 */
uint8_t mos_queue_peek(queue_t *q, uint8_t index, uint8_t *byte);

/** @brief Empty the queue.
 * @param q Queue to empty
 */
void mos_queue_cleanup(queue_t *q);

#endif
