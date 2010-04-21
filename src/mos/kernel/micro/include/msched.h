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

#ifndef SCHED_H_
#define SCHED_H_
#include "mos.h"
#include "plat_dep.h"

#ifdef SCONS
#include "optsconfig.h"
#endif
/** @file micro/include/msched.h
 * @brief Generic scheduler functions common across
 * all platforms.
 */


/** @addtogroup mos_kernel Kernel API */
//@{

/** @addtogroup mos_threading Threading
 * This group contains functions, structures and values
 * related to creating and managing threads.
 */
//@{

// Uncomment to enable some memory and stack debugging functions
//#define DEBUG_MEMORY


/** @brief Thread states. */
enum {
   /** @brief Thread is empty. */
   EMPTY = 0,
   /** @brief Thread is running. */
   RUNNING,
   /** @brief Thread will run at next opportunity. */
   READY,
   /** @brief Thread is blocked. */
   BLOCKED,
   /** @brief Thread is sleeping. */
   SLEEPING
};
/** @brief Maximum number of allowed threads */
#define MAX_THREADS 10

/** @brief Size of stack for initial/idle thread */
#define IDLE_STACK_SIZE  256 //128
/** @brief Size of stack for start thread */
#define START_STACK_SIZE 256

/** @brief The msec period of a single prescaled sleep clock tick */
#define MSECS_PER_SLEEP_TICK 31
/** @brief MSECS_PER_SLEEP_TICK should really be 31.25
    Divide your number by SLEEP_TICK_OVERFLOW and add it to the result
    for more precise math */
#define SLEEP_TICK_OVERFLOW  4
/** @brief The msec period of the maximum sleep time (256 * 32) */
#define MSECS_MAX_SLEEP      8000

/** @brief Minimum stack size */
#define STACK_MIN    MIN_STACK_SIZE
/** @brief Small stack size */
#define STACK_SMALL  STACK_MIN * 2
/** @brief Medium stack size */
#define STACK_MEDIUM STACK_MIN * 3
/** @brief Large stack size */
#define STACK_LARGE  STACK_MIN * 4


/** @brief Address just past top of heap */
#define HEAP_TOP MEMORY_BASE + (MEMORY_SIZE - IDLE_STACK_SIZE)

/** @brief Thread priorities */
enum {
   /** @brief Kernel level priority */
   PRIORITY_KERNEL = 0,
   /** @brief Sleep level priority */
   PRIORITY_SLEEP,
   /** @brief High level priority, pre-empts everything */
   PRIORITY_HIGH,
   /** @brief Normal level priority */
   PRIORITY_NORMAL,
   /** @brief Idle priority */
   PRIORITY_IDLE,
   /** @brief Total number of priorities */
   NUM_PRIORITIES
};

/** @brief Suspend states */
enum {
   /** @brief Thread is not waiting on interrupts, ok to sleep */
   SUSPEND_STATE_IDLE = 0,
   /** @brief Thread needs an interrupt, do not sleep */
   SUSPEND_STATE_SLEEP,
   /** @brief Total number of suspend states */
   SUSPEND_STATE_MAX
};

/** @brief Thread returns ok */
#define THREAD_OK       0
/** @brief Thread manager has no more threads left */
#define NO_MORE_THREADS 1
/** @brief Thread manager has run out of memory */
#define NO_MORE_MEMORY  2
/** @brief Bad thread priority specified */
#define BAD_THREAD_PRIORITY 3


/** @brief State that disables context switching */
#define CONTEXT_SWITCH_DISABLED 0
/** @brief State that enables context switching */
#define CONTEXT_SWITCH_ENABLED  1

/** @brief Thread structure definition */
typedef struct thread_s {
   /** @brief Stack pointer */
   stackval_t *sp;
   /** @brief Pointer to stack memory for de-allocating */
   stackval_t *stack;
   /** @brief Size of stack (for debugging) */
   uint16_t stackSize;
   /** @brief Function pointer to thread's start func */
   void (*func)(void);
   /** @brief Thread sleep time */
   uint32_t st;
   /** @brief Current thread state */
   uint8_t state;
   /** @brief Current thread suspend state */
   uint8_t suspend_state;
   /** @brief Thread priority */
   uint8_t priority;
   /** @brief Port number -- Only used for net recv */
   uint8_t port;
   /** @brief Next thread on list */
   struct thread_s *next;
   struct thread_s *waiting_for;
   uint8_t thread_id;
} mos_thread_t;

/** @internal Init the scheduler.
 *
 * Must call this before adding any threads.
 */
void sched_init (void);

/** @internal Start scheduling threads.
 *
 * Must add thread(s) before calling start. 
 * (NOTE: This function never returns.) 
 */

void mos_sched_start (void);

/** @brief Create a new thread and put it on the ready queue.
 *
 * Note: There is no memory protection so watch your stack sizes. 
 * @param function_start Pointer to function to call on thread start
 * @param stack_size Stack space to give the thread.  If the specified
 * stack size is smaller than MIN_STACK_SIZE, MIN_STACK_SIZE will be used
 * instead.
 * @param priority Priority of the thread
 * @return THREAD_OK, NO_MORE_THREADS, NO_MORE_MEMORY, BAD_THREAD_PRIORITY
 */
uint8_t mos_thread_new (void (*function_start)(void),
			memtype_t stack_size,
			uint8_t priority);

/** @brief Create a new thread with the given stack and put it on the ready queue.
 *
 * Note: There is no memory protection so watch your stack sizes.  Also, this
 * thread should not exit, or the memory allocator will try to free the stack.
 * @param function_start Pointer to function to call on thread start
 * @param stack_size Size of the thread's stack
 * @param stack_addr Stack space that you have already allocated
 * @param priority Priority of the thread
 * @return THREAD_OK, NO_MORE_THREADS, NO_MORE_MEMORY, BAD_THREAD_PRIORITY
 */
uint8_t mos_thread_new_havestack (void (*function_start)(void),
				  memtype_t stack_size,
				  stackval_t* stack_addr,
				  uint8_t priority);

/** @brief Terminate the calling thread.
 */
void mos_thread_exit (void);

extern mos_thread_t *_current_thread;


/** @brief Set the suspend state of the current running thread
 * this is the minimum sleep state allowed by the active thread
 */
#define mos_thread_set_suspend_state(state) _current_thread->suspend_state = state

/** @brief Get the suspend state of the current running thread
 *
 */
#define mos_thread_get_suspend_state() _current_thread->suspend_state

/** @brief Get the port the current thread will block on
 */
#define mos_thread_get_port() _current_thread->port

/** @brief Set the port the current thread is blocked on
 */
#define mos_thread_set_port(port) _current_thread->port = port

/** @brief Suspend to a specific state
 * @param state the state to suspend to
 */
void mos_thread_suspend_state(uint8_t state);

/** @brief Block the currently running thread.
 */
#define mos_thread_suspend() mos_thread_suspend_state (SUSPEND_STATE_IDLE);

/*#ifdef MOS_DEBUG
void mos_thread_suspend_noints(handle_t int_handle, mos_thread_t* waiting_for);
#else*/
void mos_thread_suspend_noints(handle_t int_handle);   
//#endif

/** @brief Resume a blocked thread
 * @param thread Thread to resume
 */
void mos_thread_resume(mos_thread_t *thread);
void mos_thread_resume_noints(mos_thread_t *thread, handle_t int_handle);

/** @brief Put the currently running thread to sleep.
 *  Units are in ms
 *  Minimum sleeptime is 128 ms (will be promoted)
 */
void mos_thread_sleep(uint32_t sleeptime);

/** @brief Get a pointer to the currently executing thread.
 * @return Currently running thread
 */
#define mos_thread_current() _current_thread

/** @brief Put the kernel in single-threaded operation mode
 */
void mos_single_threaded (void);

/** @brief Put the kernel in multi-threaded operation mode  */
void mos_multi_threaded (void);

/** @brief Return whether context switching is enabled or not */
int context_switch_enabled(void);

/** @brief Enable interrupts if we are not nested in another disable_ints call.
 * @param handle Interrupt handle to enable
 */
//inline void mos_enable_ints(uint8_t handle);
//#define mos_enable_ints(int_handle) (SREG = int_handle)

/** @brief Disable interrupts.
 * @return Interrupt handle (use with mos_enable_ints)
 */
//static inline handle_t mos_disable_ints (void);    
//static inline void mos_enable_ints(handle_t int_handle);

int mos_check_sleep_time(uint8_t sleep_time);

void mos_check_stack_ptr();

void start (void);

#ifdef DEBUG_MEMORY
void print_threads(uint8_t verbose);
#endif

#endif

//@}
//@}
