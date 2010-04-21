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
  File: mem.c
  Author: Brian Shucker
  Edited: Jeff Rose (Converted generic heap to our specific memory manager.)
  Date: 11-2-99
  
  Memory management system for allocating thread stack space in memory.  We
  keep a linked list of free memory blocks and use a best fit match to allocate
  new blocks.
*/

#include "mos.h"
#include "mem.h"
#include "msched.h"
#include "plat_dep.h"

#ifdef DEBUG_MEMORY		// from msched.h
#ifdef ARCH_AVR
#include <avr/pgmspace.h>
#endif
#include "printf.h"

const char sMemBlock[] ARCH_PROGMEM = "[%C]: node_t: %04x, data: %04x, size %d\n";
const char sMemTotal[] ARCH_PROGMEM = "%C memory blocks, %d allocated, %d free, %d total\n";
#endif

inline void flag_block(node_t *n);
inline void combine(node_t *n1, node_t *n2);

/* We keep a list of the free memory blocks that is ordered by physical location
   so we can easily aggregate adjacent blocks. */
static node_t *freelist; // List head  

void mem_init(void *region, uint16_t size)
{
   /*set up the initial free list with one region*/
   freelist = (node_t *)region;
   freelist->size = size - sizeof(node_t);
   freelist->prev = freelist;
   freelist->next = freelist;
}

void *mos_mem_alloc(uint16_t size)
{
   node_t *current;
   node_t *best;
   
   if(freelist == NULL) // No free memory available
      return NULL; 

   current = freelist;
   best = NULL;

   if (size % 2 != 0) //need to increase size for word alignment
     size++;
   
   // Run through free list and find best fitting block. 
   do
   {
      if (current->size >= size)
      {
	 if (best == NULL)
	    best = current;
	 else if (current->size < best->size)
	    best = current;
      }
      current = current->next;
   } while (current != freelist);

   // if we could not find a block, return NULL
   if (best == NULL)
      return NULL;
   
   
   
   // Determine if we should cut a piece off of this free region or return
   //   the whole thing if it isn't large enough to do so. 
   if (best->size <= size+sizeof(node_t)) // Just return this region
   {
     // Pull the node out of the list. 
      if (best->prev == best)
	 freelist = NULL;     // Last free node...
      else
      {
	 best->prev->next = best->next;
	 best->next->prev = best->prev;
      }
      
      // If we just removed the front node, then we have to set freelist
      //   to the next node 
      if (best == freelist)
	 freelist = best->next;

      flag_block(best); // Fill the block for later analysis
      
      return (uint8_t *)best + sizeof(node_t);
   }
   else // Cut a piece off the end of this block to create a new node.
   {
     // Pull the new node off the best fitting one. 
      current = (node_t *)((uint8_t *)best + best->size - size);
      best->size -= size + sizeof(node_t);
      current->size = size;

      flag_block(current); // Fill the block for later analysis
      return (uint8_t *)current + sizeof(node_t);
   }
}

void mos_mem_free(void* block)
{
   node_t *current;
   
   // Get the region's header. 
   node_t *region = (node_t *)((uint8_t *)block - sizeof(node_t));

   if (freelist == NULL) { // This is now the only free memory
      freelist = region;
      region->next = region;
      region->prev = region;
   } else {
      /* Put the free'd node onto the freelist.  The list is organized
	 by each node's physical location in memory. */ 
      current = freelist;
      while (current < region) {
	 current = current->next;
	 if (current == freelist)
	    break;
      }

      /* Now current is the node immediately after the one to insert. */
      region->next = current;
      region->prev = current->prev;
      current->prev->next = region;
      current->prev = region;

      /* If this is now the first block, we need to update freelist pointer. */
      if (region < freelist)
	 freelist = region;
   }

   /* Check to see if we can combine blocks of memory since the new node
      could be physically adjacent to it's neighbors. */
   if ((uint8_t *)region->next ==
       (uint8_t *)(region) + region->size + sizeof(node_t))       // Next is adjacent
      combine(region, region->next);

   if ((uint8_t *)region->prev ==
       (uint8_t *)(region) - region->prev->size - sizeof(node_t)) // Previous adjacent
      combine(region->prev, region);
}

/** @brief Flag a block of memory with 0xEF
 *
 * Fill a block of memory with 0xEF so it's easier to do analysis later on the usage.
 * @param n Block to fill
 */
inline void flag_block(node_t *n)
{
   uint16_t index;
   uint8_t *ptr;

   ptr = (uint8_t *)n + sizeof(node_t);

   for(index = 0; index < (n->size); index++)
      ptr[index] = 0xEF;
}

/** @brief Combine two adjacent blocks(node_ts)
 * 
 * NOTE: n1 must precede n2 both physically and in the freelist.
 * @param n1 First block
 * @param n2 Second block
 */
inline void combine(node_t *n1, node_t *n2)
{
   /*remove n2 from the free list*/
   n1->next = n2->next;
   n2->next->prev = n1;

   /*adjust size of n1*/
   n1->size = n1->size + n2->size + sizeof(node_t);
}

//returns free bytes
uint16_t mos_check_stack(mos_thread_t *t) {
   uint16_t i;
#ifdef PLATFORM_TELOSB
#define DEBUG_STACK_VAL 0xEFEF
#else
#define DEBUG_STACK_VAL 0xEF
#endif
   
    for(i = 0; i < t->stackSize; ++i) {
	if(t->stack[i] != DEBUG_STACK_VAL) {
	    break;
	}
    }
    
    return i;
}

#ifdef DEBUG_MEMORY
void print_memory(uint8_t verbose)
{
#if defined(PLATFORM_MICROBLAZE)
   extern stackval_t _end;
#else
   extern uint8_t _end; //linker-provided symbol for end of memory
#endif
   uint16_t total = MEMORY_BASE + MEMORY_SIZE - IDLE_STACK_SIZE - (uint16_t)&_end;
   
   uint8_t nb = 0;
   uint16_t free = 0;
   node_t* n = freelist;
   if (n) do {
      if (verbose)
	 printf_P(sMemBlock, nb, n, ((uint16_t)n) + sizeof(node_t), n->size);
      nb++;
      free += n->size;
      n = n->next;
   } while (n != freelist);
   
   printf_P(sMemTotal, nb, total-free, free, total);
}
#endif
