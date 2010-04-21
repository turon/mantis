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

/** @file context_switch.h
 * @brief Assembly routines required to preform a context switch.
 */
#ifndef __context_switch_h__
#define __context_switch_h__


/** @file avr/include/context_switch.h
 *  @brief save the necessary registers for a context switch
 * on the AVR architecture
 *
 * Save current stack pointer
 * Change stack pointer to process stack
 * Push address of start_thread function onto stack
 * Push zeroes for initial register vals and sreg
 * Save adjusted stack ptr to thread struct
 * Restore kernel stack ptr
 */
#define CONTEXT_SWITCH_PREAMBLE()			\
   {							\
      asm volatile(					\
	 "in %A0, __SP_L__\n\t"				\
	 "in %B0, __SP_H__\n\t"				\
	 : "=r" (_current_thread->sp) : );		\
							\
      asm volatile(					\
	 "out __SP_H__, %B0\n\t"			\
	 "out __SP_L__, %A0\n\t"			\
	 :: "r" (stack_addr) );				\
							\
      asm volatile(					\
	 "push %A0\n\t"					\
	 "push %B0\n\t"					\
	 :: "r" (start_wrapper) );			\
							\
      for(i = 0; i < 31; i++)				\
	asm volatile("push __zero_reg__\n\t" ::);	\
							\
      asm volatile(					\
	 "in %A0, __SP_L__\n\t"				\
	 "in %B0, __SP_H__\n\t"				\
	 : "=r" (threads[id].sp) : );			\
							\
      asm volatile(					\
	 "out __SP_H__, %B0\n\t"			\
	 "out __SP_L__, %A0\n\t"			\
	 :: "r" (_current_thread->sp) );		\
							\
   }

/** @brief save the current thread's context to the stack
 * Save all the registers
 * Save SREG
 * Push the stack pointer
 */
#define PUSH_THREAD_STACK()			\
   {						\
      asm volatile(				\
	 "push r24\n\t"				\
	 "in r24, __SREG__\n\t"			\
	 "cli\n\t"				\
	 "push r24\n\t"				\
	 );					\
      asm volatile(				\
	 "push r31\n\t"				\
	 "push r30\n\t"				\
	 "push r29\n\t"				\
	 "push r28\n\t"				\
	 "push r27\n\t"				\
	 "push r26\n\t"				\
	 "push r25\n\t"				\
	 "push r23\n\t"				\
	 "push r22\n\t"				\
	 "push r21\n\t"				\
	 "push r20\n\t"				\
	 "push r19\n\t"				\
	 "push r18\n\t"				\
	 "push r17\n\t"				\
	 "push r16\n\t"				\
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
	 "push r3\n\t"				\
	 "push r2\n\t"				\
	 /*"push r1\n\t"*/			\
	 /*"push r0\n\t"*/			\
	 );					\
      asm volatile(				\
	 "in %A0, __SP_L__\n\t"			\
	 "in %B0, __SP_H__\n\t"			\
	 : "=r" (_current_thread->sp) : );	\
   }

/** @brief retrieve a thread and context from the stack
 * Restore the stack pointer
 * Restore SREG
 * Restore the other registers
 */
#define POP_THREAD_STACK()			\
   {						\
      asm volatile(				\
	 "out __SP_H__, %B0\n\t"		\
	 "out __SP_L__, %A0\n\t"		\
	 :: "r" (_current_thread->sp));		\
      asm volatile(				\
	 /*"pop r0\n\t"*/			\
	 /*"pop r1\n\t"*/			\
	 "pop r2\n\t"				\
	 "pop r3\n\t"				\
	 "pop r4\n\t"				\
	 "pop r5\n\t"				\
	 "pop r6\n\t"				\
	 "pop r7\n\t"				\
	 "pop r8\n\t"				\
	 "pop r9\n\t"				\
	 "pop r10\n\t"				\
	 "pop r11\n\t"				\
	 "pop r12\n\t"				\
	 "pop r13\n\t"				\
	 "pop r14\n\t"				\
	 "pop r15\n\t"				\
	 "pop r16\n\t"				\
	 "pop r17\n\t"				\
	 "pop r18\n\t"				\
	 "pop r19\n\t"				\
	 "pop r20\n\t"				\
	 "pop r21\n\t"				\
	 "pop r22\n\t"				\
	 "pop r23\n\t"				\
	 "pop r25\n\t"				\
	 "pop r26\n\t"				\
	 "pop r27\n\t"				\
	 "pop r28\n\t"				\
	 "pop r29\n\t"				\
	 "pop r30\n\t"				\
	 "pop r31\n\t"				\
	 "pop r24\n\t"				\
	 "out __SREG__, r24\n\t"		\
	 "pop r24\n\t"				\
	 "sei\n\t"				\
	 );					\
   }


#endif

