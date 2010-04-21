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

/** @file kernel/include/mem.h
 * @brief Memory management system. 
 * 
 * Memory management system for allocating thread stack space in memory. We keep a linked list of free memory blocks and use a best fit match to allocate new blocks.
 * @author Brian Shucker
 * @author Edited: Jeff Rose (Converted generic heap to our specific memory 
 * manager.)
 * @date Created: 11/02/1999
 */

/** @brief A node of managed memory */
typedef struct node_s {
   /** @brief Size of blocks */
   uint16_t size;
   /** @brief Linked list of pointers */
   struct node_s *prev;
   /** @brief Pointer to ned node */
   struct node_s *next;
} node_t;

#include <inttypes.h>
#include "msched.h"

/** @brief Initialize the heap with a single free region of memory.
 * @param region Memory region to initialize
 * @param size Size of the memory region
 */
void mem_init(void *region, uint16_t size);
/** @brief Try to allocate a region of memory.
 * @param size Size of the region to allocate.
 * @return NULL if not enough memory is available, else the allocated memory.
 */
void *mos_mem_alloc(uint16_t size);
/** @brief Free a block of memory.
 * @param block Block of memory to free
 */
void mos_mem_free(void *block);

/** @brief Check the number of bytes used for a given thread's stack
 * 
 * @param t The thread who's stack shall be checked.
 * @return The number of bytes used
 */
uint16_t mos_check_stack(mos_thread_t *t);

#ifdef DEBUG_MEMORY
void print_memory(uint8_t verbose);
#endif
