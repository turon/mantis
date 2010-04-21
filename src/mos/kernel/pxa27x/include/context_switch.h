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

/** @file pxa27x/include/context_switch.h
 * @author John Ledbetter
 * @brief Assembly routines required to perform a context switch on the PXA27X architecture.
 */
#ifndef __context_switch_h__
#define __context_switch_h__
// pc = r15
// lr = r14 (link register)
// sp = r13
// push: stmfd sp!,{register list}
// pop:  ldmfd sp!,{register list}
// return: mov pc,lr


/** @brief save the necessary registers for a context switch
 * Save current stack pointer
 * Change stack pointer to process stack
 * Push address of start_thread function onto stack
 * Push zeroes for initial register vals and sreg
 * Save adjusted stack ptr to thread struct
 * Restore kernel stack ptr
 */
#define CONTEXT_SWITCH_PREAMBLE()                     \
{                                                     \
   asm volatile("mov %0, sp\n\t"                      \
		:"=r"(_current_thread->sp) : );       \
   asm volatile("mov sp, %0\n\t"                      \
		::"r"(stack_addr) );                  \
   asm volatile("stmfd sp!,{%0}\n\t"                  \
		::"r"(start_wrapper));                \
   asm volatile("mov %0,#0\n\t"                       \
		:"=r"(helper_var) : );                \
   for(i = 0; i < 14; ++i){                           \
      asm volatile("stmfd sp!,{%0}"                   \
                ::"r"(helper_var) );}                 \
   asm volatile("mov %0,sp\n\t"                       \
		:"=r"(threads[id].sp) : );            \
   asm volatile("mov sp, %0\n\t"                      \
		::"r"(_current_thread->sp));          \
}


/*                  \ */
/* {                                                  \ */
/*    asm volatile("mov.w r1, %0\n\t"                 \ */
/* 		:"=r"(_current_thread->sp) : );       \ */
/*                                                    \ */
/*    asm volatile("mov.w %0, r1\n\t"                 \ */
/* 		:: "r"(stack_addr) );                 \ */
/*                                                    \ */
/*    asm volatile("push %0\n\t"                      \ */
/*                 ::"r"(start_wrapper));             \ */
/*                                                    \ */
/*    for(i = 0; i < 13; i++)                         \ */
/*       asm volatile("push r3\n\t");                 \ */
/*                                                    \ */
/*    asm volatile("mov.w r1, %0\n\t"                 \ */
/* 		:"=r"(threads[id].sp) : );            \ */
/*                                                    \ */
/*    asm volatile("mov.w %0, r1\n\t"                 \ */
/* 		::"r"(_current_thread->sp) );         \ */
/*                                                    \ */
/* }                                                    */


/** @brief save the current thread's context to the stack
 * Copy SR to helper
 * Push helper
 * Push general purpose registers
 * Save the stack pointer to the thread struct
 */
#define PUSH_THREAD_STACK()                     \
{                                               \
   asm volatile("mrs %0,cpsr\n\t "               \
                :"=r"(last_sr) : );          \
  /*  asm volatile("stmfd sp!, {%0}\n\t"   */        \
	/* 	::"r"(helper_var)); */             \
   asm volatile("stmfd sp!,{r0-r12,r14}\n\t"); \
   asm volatile("mov %0, sp\n\t"               \
		:"=r"(_current_thread->sp) : ); \
}

/** @brief retrieve a thread and context from the stack
 * Restore the stack pointer from the thread struct
 * Pop the general purpose registers
 * Pop the status register
 */
#define POP_THREAD_STACK()                       \
{                                                \
   asm volatile("mov sp,%0\n\t"                 \
		::"r"(_current_thread->sp));     \
   asm volatile("ldmfd sp!, {r0-r12,r14}\n\t"); \
  /*  asm volatile("ldmfd sp!, {%0}\n\t"  */          \
	/* 	:"=r"(helper_var) : ); */           \
   asm volatile("msr cpsr,%0\n\t"               \
                ::"r"(last_sr));             \
}


#endif
