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

/** @file msp430/include/context_switch.h
 * @brief Assembly routines required to preform a context switch on the MSP430 architecture.
 */
#ifndef __context_switch_h__
#define __context_switch_h__

/** @brief save the necessary registers for a context switch
 * Save current stack pointer
 * Change stack pointer to process stack
 * Push address of start_thread function onto stack
 * Push zeroes for initial register vals and sreg
 * Save adjusted stack ptr to thread struct
 * Restore kernel stack ptr
 */
#define CONTEXT_SWITCH_PREAMBLE()                  \
{                                                  \
   asm volatile("mov.w r1, %0\n\t"                 \
		:"=r"(_current_thread->sp) : );    \
                                                   \
   asm volatile("mov.w %0, r1\n\t"                 \
		:: "r"(stack_addr) );              \
                                                   \
   asm volatile("push %0\n\t"                      \
                ::"r"(start_wrapper));             \
                                                   \
   for(i = 0; i < 13; i++)                         \
      asm volatile("push r3\n\t");                 \
                                                   \
   asm volatile("mov.w r1, %0\n\t"                 \
		:"=r"(threads[id].sp) : );         \
                                                   \
   asm volatile("mov.w %0, r1\n\t"                 \
		::"r"(_current_thread->sp) );      \
                                                   \
}


/** @brief save the current thread's context to the stack
 * Push status register
 * Push general purpose registers
 * Save the stack pointer to the thread struct
 */
#define PUSH_THREAD_STACK()			\
   {						\
      asm volatile(				\
	 "push r2\n\t"				\
	 "dint\n\t"				\
	 );					\
      asm volatile(				\
	 "push r15\n\t"				\
	 "push r14\n\t"				\
	 "push r13\n\t"				\
	 "push r12\n\t"				\
	 "push r11\n\t"				\
	 "push r10\n\t"				\
	 "push r9\n\t"				\
	 "push r8\n\t"				\
	 "push r7\n\t"				\
	 "push r6\n\t"				\
	 "push r5\n\t"				\
	 "push r4\n\t"				\
	 );					\
      asm volatile(				\
         "mov.w r1, %0\n\t"                     \
         :"=r"(_current_thread->sp) : );	\
   }

/** @brief retrieve a thread and context from the stack
 * Restore the stack pointer from the thread struct
 * Pop the status register
 * Pop the general purpose registers
 */
#define POP_THREAD_STACK()                      \
{                                               \
   asm volatile("mov.w %0, r1\n\t"              \
		::"r"(_current_thread->sp));    \
                                                \
   asm volatile(                                \
      "pop r4\n\t"                              \
      "pop r5\n\t"                              \
      "pop r6\n\t"                              \
      "pop r7\n\t"                              \
      "pop r8\n\t"                              \
      "pop r9\n\t"                              \
      "pop r10\n\t"                             \
      "pop r11\n\t"                             \
      "pop r12\n\t"                             \
      "pop r13\n\t"                             \
      "pop r14\n\t"                             \
      "pop r15\n\t"                             \
      );                                        \
                                                \
   asm volatile("pop r2\n\t");                  \
   asm volatile("eint\n\t");                    \
}

#endif
