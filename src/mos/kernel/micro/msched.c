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

#include "plat_dep.h"
#include "mos.h"

#include "msched.h"
#include "tlist.h"
#include "mem.h"
#include "led.h" //TODO: remove when done debugging
#include "clock.h"
#include "context_switch.h" //provides asm for saving/switching/restoring stack
#include "plat_clock.h"
#include "mutex.h"
#include "sem.h"
#include "printf.h"

#ifdef MOS_DEBUG
#include "mos_debugging.h"
#endif

#ifdef DEBUG_MEMORY		// from msched.h

#ifdef ARCH_AVR
#include <avr/pgmspace.h>
#endif

#include "printf.h"

//#define SLEEP_ALARMS

const char sThreadRow[] ARCH_PROGMEM = "thread %d: sp %x, stack %x, size %d, unused %d, state %C, priority %C\n";
const char sThreadTot[] ARCH_PROGMEM = "%C threads, %d stack allocated, %d stack used\n";
const char sStackOver[] ARCH_PROGMEM = "stack %C has overflowed\n";
const char sStackFull[] ARCH_PROGMEM = "stack %C looks full\n";
#endif

static uint8_t running;    // flag for the state of the scheduler

uint16_t last_sleep_time; // time the processor was sleeping (msecs)
uint8_t elapsed_thread_time = 0;


static mos_thread_t *wakeup_front;
static mos_thread_t threads[MAX_THREADS]; // Our array of system thread data structs
mos_thread_t *_current_thread;      // The currently running thread
static mos_thread_t *sleep_thread;      // The currently sleeping thread

static tlist_t readyQ[NUM_PRIORITIES]; // Ready Queue with priorities
static tlist_t sleepQ;                 // Sleep Queue

#if defined(PLATFORM_MICROBLAZE)
extern stackval_t _end; //linker-provided symbol for end of memory
#else
extern uint8_t _end;
#endif

/* WARNING: You can not use local variables in the dispatcher because they mess
   with the calling functions stack.  So we use these */ 
static uint8_t d_index;
#ifdef PLATFORM_IMOTE2
static uint32_t helper_var;
#endif

static boolean context_switch_state;

/** @brief Heart of the scheduler.
 *
 * Saves the current thread's context and then restores a new
 * thread. This function implements both context switching and
 * the actual scheduling algorithm.
 * NOTE: the naked attribute tells the compiler not to add any
 * prologue or epilogue code.
 */
//microblaze architecture ignores the naked attribute
#if defined(PLATFORM_MICROBLAZE)
void dispatcher(void);
#else
void __attribute__((naked)) dispatcher(void);
#endif
void idle_loop(void);
void start_wrapper(void);
//static void mos_thread_wakeup_sleep (void);
void mos_thread_wakeup(uint16_t sleep_time);
void mos_thread_wakeup_noints(uint16_t sleep_time,
        handle_t int_handle);

void mos_single_threaded(void)
{
    handle_t int_handle = mos_disable_ints();
    context_switch_state = CONTEXT_SWITCH_DISABLED;
    mos_enable_ints(int_handle);
}

void mos_multi_threaded(void)
{
    handle_t int_handle = mos_disable_ints();
    context_switch_state = CONTEXT_SWITCH_ENABLED;
    mos_enable_ints(int_handle);
}

int context_switch_enabled(void)
{
    return context_switch_state;
}

void sched_init(void)
{
    uint8_t i;

    DISABLE_INTS();

// Init the memory system for our stacks. 
#if defined(PLATFORM_TELOSB) || defined(ARCH_AVR)
    mem_init(&_end, HEAP_TOP - ((int)&_end));
#elif defined (PLATFORM_MICROBLAZE)
    extern uint32_t *__thread_stack_start;
    extern uint32_t *__thread_stack_size;
    stackval_t *mem_start = &__thread_stack_start;
    stackval_t mem_size = &__thread_stack_size;
    mem_init(mem_start, mem_size);
#else
#warning "memory regions not defined for this platform"
#endif


    // init the thread table
    for(i = 0; i < MAX_THREADS; i++) {
        threads[i].sp = 0;
        threads[i].state = EMPTY;
        threads[i].suspend_state = SUSPEND_STATE_SLEEP;
        threads[i].priority = 0;
        threads[i].next = NULL;
        threads[i].waiting_for = NULL;
    }

    // Allocate space for the kernel thread, which is currently running. 
    _current_thread = &threads[0];
    _current_thread->state = RUNNING;  
    _current_thread->priority = PRIORITY_IDLE;

    context_switch_state = CONTEXT_SWITCH_ENABLED;

#if defined(DEBUG_MEMORY) 
    // For debugging, fill these fields in for the kernel thread, too.
    _current_thread->stack = (uint8_t *)HEAP_TOP;
    _current_thread->stackSize = IDLE_STACK_SIZE;

    // Flag the block for debugging, normally memory manager does this.
    // Be careful not to write over our current stack.
    uint8_t *j = HEAP_TOP;
    stackval_t *sp;

    GET_SP(sp);

    for(; j < sp; j++)
        *j = 0xEF;
#endif

    //Init the ready queue
    for(i = 0; i < NUM_PRIORITIES; i++)
        mos_tlist_init(&readyQ[i]);

}

void mos_sched_start(void)
{
    running = TRUE;

    //initialize the hardware timers which control
    //sleeping and timeslicing
    kernel_timer_init();
    sleep_timer_init();
 
    //mos_sem_init(&dco_sem, 0);
    //mos_thread_new(mos_calibrate_dco_thread, 64, PRIORITY_IDLE);
    // Once interrupts are enabled here, the sheduler will start to schedule
    // threads and do time slicing.
    ENABLE_INTS();
    
    //Start executing the first thread
    dispatcher();
    
    // On to the idle loop
    idle_loop();
}

void mos_thread_suspend_state(uint8_t state)
{
    if(state >= SUSPEND_STATE_MAX)
        state = SUSPEND_STATE_IDLE;
    handle_t int_handle = mos_disable_ints();

    _current_thread->next = NULL;
    _current_thread->state = BLOCKED; // Update the thread state
    // store the old suspend state in the sleep time,
    // which we aren't using
    _current_thread->st = (uint32_t)_current_thread->suspend_state;
    _current_thread->suspend_state = state;

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_BLOCK);
#endif

    mos_thread_wakeup_noints(elapsed_thread_time,
            int_handle);
}

/*#ifdef MOS_DEBUG
  void mos_thread_suspend_noints(handle_t int_handle, mos_thread_t* waiting_for)
#else*/
void mos_thread_suspend_noints(handle_t int_handle)
    //#endif
{
    _current_thread->next = NULL;
    _current_thread->state = BLOCKED; // Update the thread state
    // store the old suspend state in the sleep time,
    // which we aren't using
    _current_thread->st = (uint32_t)_current_thread->suspend_state;
    _current_thread->suspend_state = SUSPEND_STATE_IDLE;

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_BLOCK);
#endif

    mos_thread_wakeup_noints(elapsed_thread_time,
            int_handle);
}

void mos_thread_resume(mos_thread_t *thread)
{
    handle_t int_handle;
    int_handle = mos_disable_ints();

    // put given thread onto the ready Queue if it was blocked.
    if(thread->state == BLOCKED) {
        mos_tlist_add(&readyQ[thread->priority], thread);
        thread->state = READY;
        thread->suspend_state = (uint8_t)thread->st;

#ifdef MOS_DEBUG
        mos_debug_set_trace(DBCODE_THREAD_UNBLOCK);
#endif
    }

    mos_thread_wakeup_noints(elapsed_thread_time,
            int_handle);
}

void mos_thread_resume_noints(mos_thread_t *thread, handle_t int_handle)
{
    // put given thread onto the ready Queue if it was blocked.
    if(thread->state == BLOCKED) {
        mos_tlist_add(&readyQ[thread->priority], thread);
        //thread->waiting_for = NULL;
        thread->state = READY;
        thread->suspend_state = (uint8_t)thread->st;

#ifdef MOS_DEBUG
        mos_debug_set_trace(DBCODE_THREAD_UNBLOCK);
#endif 
    }

    mos_thread_wakeup_noints(elapsed_thread_time,
            int_handle);
}


void mos_thread_resume_noints_nodispatch(mos_thread_t *thread, handle_t int_handle)
{
    // put given thread onto the ready Queue if it was blocked.
    if(thread->state == BLOCKED) {
        mos_tlist_add(&readyQ[thread->priority], thread);
        thread->waiting_for = NULL;
        thread->state = READY;
        thread->suspend_state = (uint8_t)thread->st;

#ifdef MOS_DEBUG
        mos_debug_set_trace(DBCODE_THREAD_UNBLOCK);
#endif
    }   
}


void mos_thread_sleep(uint32_t sleeptime)
{
    handle_t int_handle;

    // what is our minimum? the sleep time is accurate to +/- 1 ms
    /*if(sleeptime < 5)
      sleeptime = 5;*/

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_SLEEP);
#endif

    int_handle = mos_disable_ints();

    _current_thread->state = SLEEPING; // Update the thread state
    _current_thread->st = sleeptime + elapsed_thread_time; // Update the sleeptime
    mos_tlist_ordadd(&sleepQ, _current_thread);

    mos_thread_wakeup_noints(elapsed_thread_time, int_handle);
}

/** @brief wake up a thread along with decrementing the sleep time
 * this function wakes sleeping threads if necessary, otherwise just
 * decrements the sleep time of the head of the list.
 * @param down_time ammount of time passed since last sleeping
 */
void mos_thread_wakeup(uint16_t sleep_time)
{
    //Adjust the sleepQ with the times that has already expired.
    mos_tlist_adjustst(&sleepQ, sleep_time);

    //get the first thread in the sleep queue
    wakeup_front = mos_tlist_head(&sleepQ);

    //loop through the sleep queue
    while((wakeup_front != NULL) && (wakeup_front->st == 0)) {

        //remove the thread if sleep time is 0
        sleep_thread = mos_tlist_remove(&sleepQ);
        sleep_thread->state = READY; // Update the thread state
        //add thread to ready queue
        mos_tlist_add(&readyQ[sleep_thread->priority], sleep_thread);
        wakeup_front = mos_tlist_head(&sleepQ);

#ifdef MOS_DEBUG
        mos_debug_set_trace(DBCODE_THREAD_WAKEUP);
#endif

    }

    dispatcher();   // do the context switch.
}

void mos_thread_wakeup_noints(uint16_t sleep_time,
        handle_t int_handle)
{
    //Adjust the sleepQ with the times that has already expired.
    mos_tlist_adjustst(&sleepQ, sleep_time);

    //get the first thread in the sleep queue
    wakeup_front = mos_tlist_head(&sleepQ);

    //loop through the sleep queue
    while((wakeup_front != NULL) && (wakeup_front->st == 0)) {

        //remove the thread if sleep time is 0
        sleep_thread = mos_tlist_remove(&sleepQ);
        sleep_thread->state = READY; // Update the thread state
        //add thread to ready queue
        mos_tlist_add(&readyQ[sleep_thread->priority], sleep_thread);
        wakeup_front = mos_tlist_head(&sleepQ);

#ifdef MOS_DEBUG
        mos_debug_set_trace(DBCODE_THREAD_WAKEUP);
#endif

    }

    mos_enable_ints(int_handle);

    dispatcher();   // do the context switch.
}

#ifdef MOS_DEBUG
static mos_thread_t* oldthread;
#endif

/** @brief Real dispatcher function
 *
 * Doxygen doesn't seem to like gcc attributes
 */

handle_t dispatcher_int_handle;

#if defined(PLATFORM_MICROBLAZE)
void dispatcher(void)
#else
void __attribute__((naked)) dispatcher(void)
#endif
{
   // Prologue: Saves the current state.
   // All registers go onto current stack and then the stack is saved

   dispatcher_int_handle = mos_disable_ints();
   
   running = FALSE;
   elapsed_thread_time = 0;
   
   PUSH_THREAD_STACK();

#ifdef MOS_DEBUG
   oldthread = _current_thread;
#endif
   
   // Only change the thread's state if it was running, not blocked
   if(_current_thread->state == RUNNING) {
      _current_thread->state = READY;
      mos_tlist_add(&readyQ[_current_thread->priority], _current_thread);
   }
   
   // Now bring up the new thread
   // idle thread's existence guarantees we'll get something
   for(d_index = 0; d_index < NUM_PRIORITIES; d_index++) {
      _current_thread = mos_tlist_remove(&readyQ[d_index]);
      if(_current_thread != NULL) {
#ifdef MOS_DEBUG
            //if we didn't just switch the same thread
            if(oldthread != _current_thread)
                mos_debug_set_trace(DBCODE_CONTEXT_SWITCH);

#endif
            break;
        }

    }
    _current_thread->state = RUNNING;
   
   POP_THREAD_STACK();
   running = TRUE;
   
// return must be explicit because naked functions don't have one
#ifdef PLATFORM_IMOTE2
    asm volatile("ldmfd r13!,{pc}");

#elif PLATFORM_MICROBLAZE
#else
    asm volatile("ret\n");
#endif

    mos_enable_ints(dispatcher_int_handle);
}

boolean check_sleep_time(uint8_t sleep_time)
{
    mos_thread_t* mythread = mos_tlist_head(&sleepQ);

    if(mythread
            && mythread->st == sleep_time 
            && _current_thread->priority >  mythread->priority)
    {
        mos_thread_wakeup(sleep_time);
        return true;
    }
    return false;
}

/** @brief set timer0 and go to sleep for x ammount of ms
 * @param time ammount of time to sleep the kernel for in ms
 */
#ifdef PLATFORM_TELOSB
static inline void kernel_sleep(uint16_t time)
#else
static inline void kernel_sleep(uint8_t time)
#endif
{
    SET_SLEEP_TIMER_VALUE(0);
    SET_SLEEP_OCR_VALUE(time);

    // must wait for these bits to be updated on the async clock
    WAIT_FOR_ASYNC_UPDATE();

    handle_t int_handle = mos_disable_ints();
    ENABLE_SLEEP_TIMER();

    PRE_KERNEL_SLEEP();

    ENABLE_INTS();
    SLEEP();

    mos_enable_ints(int_handle);

    POST_KERNEL_SLEEP();


}

static void idle_mode(void)
{
    handle_t int_handle;
    int_handle = mos_disable_ints();
    ENABLE_IDLE();
    ENABLE_INTS();

    IDLE();

    mos_enable_ints(int_handle);
}

//this function decides whether we can enter a sleep
//mode or whether we have to idle to wait for an interrupt
inline static uint8_t decide_power_save_mode(void)
{
    handle_t i;
    uint8_t suspend_state;
    mos_thread_t* sleep_thread;

    //first thread on the sleep queue defines 
    //the minimum ammount we can sleep
    sleep_thread = mos_tlist_head(&sleepQ);

    //minimum time to accurately deep sleep is 64 ms
    if(sleep_thread != NULL && sleep_thread->st < 46)
    {
        suspend_state = SUSPEND_STATE_IDLE;
    }
    else if (mos_have_alarms()) 
    {
        //can't enter sleep mode if there are oustanding
        //alarms < minimum sleep time
#ifdef SLEEP_ALARMS
        mos_alarm_t* next_alarm =  mos_get_next_alarm();
        if(next_alarm->msecs < 46)
#endif
            suspend_state = SUSPEND_STATE_IDLE;
    }
    else
    {//no alarms, sleep time greater than minimum
        suspend_state = threads[0].suspend_state;
        // pass over idle thread (index 0)
        for(i = 1; i < MAX_THREADS; i++) {
            if(threads[i].state == EMPTY)
                continue;
            if(threads[i].suspend_state < suspend_state) {
                suspend_state = threads[i].suspend_state;
                if(suspend_state == SUSPEND_STATE_IDLE) {
                    break;
                }
            }
        }
    }

    // just re-using the index var here
    i = mos_disable_ints();
    //suspend_state = SUSPEND_STATE_IDLE;
    switch(suspend_state) {
        case SUSPEND_STATE_SLEEP:
            //mos_led_toggle(1);
            ENABLE_SLEEP();
            DISABLE_ALARM_TIMER();
            // first thread to wake up should have a full 20ms timeslice
            RESET_TSLICE_COUNTER();  
            break;
        case SUSPEND_STATE_IDLE:
        default:
            ENABLE_IDLE();
    }
    mos_enable_ints(i);

    return suspend_state;
}



#ifdef PLATFORM_TELOSB
inline static uint16_t set_last_sleep_time(mos_thread_t *sleep_thread)
{
    uint32_t st = sleep_thread->st;
    uint16_t ocr_val;

    mos_alarm_t* next_alarm;

    st = sleep_thread->st;

    if(mos_have_alarms())
    {
        next_alarm = mos_get_next_alarm();
        if(next_alarm->msecs < sleep_thread->st)
            st = next_alarm->msecs;
        next_alarm->msecs -= st;
    }


    // 15984 * 4.1 ~= 0xfffe
    if (st > 15984)
    {
       ocr_val = 0xfffe;
       last_sleep_time = 15984;
    } else {
       
       // 32768 ticks per second
       // => 32.768 ticks per millisecond
       // scaled by 8 => 4.096 ticks per millisecond
       // the following is equivalent to st * 4.1
       ocr_val = st * 4 + st / 10;

        if(ocr_val == 0)
            ocr_val = 1;
        last_sleep_time = st;
    }
    return ocr_val;

}

#else
#define INIT_TIME 15
inline static uint8_t set_last_sleep_time(mos_thread_t *sleep_thread)
{
    uint32_t st;
    uint8_t ocr_val;

    st = sleep_thread->st;


#ifdef SLEEP_ALARMS
    mos_alarm_t* next_alarm;
    if(mos_have_alarms())
    {
        next_alarm = mos_get_next_alarm();
        if(next_alarm->msecs < sleep_thread->st)
            st = next_alarm->msecs;
        next_alarm->msecs -= st;
    }
#endif

    if(st > MSECS_MAX_SLEEP) {
        ocr_val = 255;
        last_sleep_time = MSECS_MAX_SLEEP;
    }
    else
    {
        ocr_val = (uint8_t)(((st-INIT_TIME)<<2)/125);    //125 = 31.25*4    //Scaled math, 4 = 1<<2
        last_sleep_time = (uint32_t)(((ocr_val*125)>>2) + INIT_TIME);       //Scaled math 
    }
    return ocr_val;
}
#endif



/** @brief The idle loop.
 *
 * The lowest priority thread (idle thread) runs this loop.
 * It contains the sleep logic
 */
void idle_loop(void)
{
    mos_thread_t *sleep_thread;
    // Do power management, deadlock detection, etc. stuff here...
    while(1) {
        sleep_thread = mos_tlist_head(&sleepQ);
        if(sleep_thread) {

            if(decide_power_save_mode() == SUSPEND_STATE_IDLE)
            {
                idle_mode();
                continue;
            }

#ifdef PLATFORM_TELOSB
            uint16_t ocr_val;
#else
            uint8_t ocr_val;
#endif

            ocr_val = set_last_sleep_time(sleep_thread);//get ocr val

#ifdef MOS_DEBUG
            MOS_DEBUG_PRE_SLEEP();
#endif

            kernel_sleep(ocr_val);//sleep for the ocr value
        } else { //no sleeping threads
            idle_mode();
        }
    }
}

uint8_t mos_thread_new(void (*function_start) (void),
                       memtype_t stack_size,
                       uint8_t priority)
{
   if(stack_size < MIN_STACK_SIZE)
      stack_size = MIN_STACK_SIZE;

   if (priority >= NUM_PRIORITIES)
      return BAD_THREAD_PRIORITY;
   
   
   stackval_t *stack_addr;
   handle_t int_handle;
   uint8_t id, i;

    int_handle = mos_disable_ints();

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_NEW);
#endif

    for(id = 0; id < MAX_THREADS; id++) {
        if(threads[id].state == EMPTY)
            break;
    }
    if(id == MAX_THREADS) {
        mos_enable_ints(int_handle);
        return NO_MORE_THREADS;
    }

   //Allocate memory for the stack space.
   stack_addr = (stackval_t *)mos_mem_alloc(stack_size);
   if(stack_addr == NULL) {
      mos_enable_ints(int_handle);
      return NO_MORE_MEMORY; // Can't continue if we don't have memory
   }
   
   // Now save the beginning of the stack to the thread's stack variable
   threads[id].stack = stack_addr;
   
   // Next point the stack pointer to the top
   // of the stack
#if defined(PLATFORM_MICROBLAZE)
   stack_addr = (stackval_t *)((stackval_t)stack_addr + (stackval_t)stack_size);
#else
   stack_addr = ((uint8_t*)stack_addr) + stack_size - 1;
#endif

   // make sure the stack is uint16_t aligned for PLATFORM_TELOSB
#ifdef PLATFORM_TELOSB
   //((uint16_t)stack_addr) &= ~(0x01);
   // the above line will round down, past where the stack should start.
   // the new line below will round up, into the stack.
   stack_addr = (((uint16_t)stack_addr) + 1) & ~(0x01);
// make sure the stack is uint32_t aligned for PLATFORM_IMOTE2
#elif defined(PLATFORM_MICROBLAZE)
   //round down, back into the stack since stack_addr is at the top
   stack_addr = (stackval_t)stack_addr & (stackval_t)(~0x03);
#elif PLATFORM_IMOTE2
   //((uint32_t)stack_addr) &= ~(0x03);
   // ditto here; round up, not down.
   stack_addr = (((uint32_t)stack_addr) + 3) & ~(0x03);
#endif


   threads[id].stackSize = stack_size;
   threads[id].func = function_start;
   threads[id].state = READY; // Init thread state
   threads[id].suspend_state = SUSPEND_STATE_IDLE;
   threads[id].priority = priority;
   threads[id].thread_id = id;

   CONTEXT_SWITCH_PREAMBLE();

   mos_tlist_add(&readyQ[priority], &threads[id]);	
  
   mos_enable_ints(int_handle);
   if(running)
      dispatcher();
   
   return THREAD_OK;
}

uint8_t mos_thread_new_havestack(void (*function_start) (void),
				 memtype_t stack_size,
				 stackval_t* stack_addr,
				 uint8_t priority)
{
   
   if (priority >= NUM_PRIORITIES)
      return BAD_THREAD_PRIORITY;

   
   handle_t int_handle;
   uint8_t id, i;
   
   int_handle = mos_disable_ints();

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_NEW);
#endif

    for(id = 0; id < MAX_THREADS; id++) {
        if(threads[id].state == EMPTY)
            break;
    }
    if(id == MAX_THREADS) {
        mos_enable_ints(int_handle);
        return NO_MORE_THREADS;
    }

    // Stack space is already allocated, but check pointer anyway
    if(stack_addr == NULL) {
        mos_enable_ints(int_handle);
        return NO_MORE_MEMORY; // Can't continue if we don't have memory
    }

    // Make it look like the stack was allocated from the memory manager.
    // This will allow us to free it in case the thread exits.
    // (I originally wrote this function to avoid the overhead of the memory
    // manager.  But this "overhead" is only six bytes per block.  The real
    // usefulness of this function is that we can see the stack during static
    // analysis.)
    node_t *block = (node_t *)stack_addr;
    stack_size -= sizeof(node_t);
    stack_addr += sizeof(node_t);
    block->size = stack_size;

    // Flag the block for debugging, normally memory manager does this
    uint16_t j;
    for(j = 0; j < stack_size; j++)
        stack_addr[j] = 0xEFEF;

    // Now save the memory addr and point to the top of the stack.
    threads[id].stack = stack_addr;
    stack_addr += stack_size - 1;

    threads[id].stackSize = stack_size;
    threads[id].func = function_start;
    threads[id].state = READY; // Init thread state
    threads[id].suspend_state = SUSPEND_STATE_IDLE;
    threads[id].priority = priority;

    CONTEXT_SWITCH_PREAMBLE();

    mos_tlist_add(&readyQ[priority], &threads[id]);	

    mos_enable_ints(int_handle);
    if(running)
        dispatcher();

    return THREAD_OK;
}

/** @brief A wrapper for starting threads so we can guarantee that they are
 *  de-allocated after completion. 
 */
void start_wrapper(void)
{
    _current_thread->func();

    mos_thread_exit();
}

void mos_thread_exit(void)
{
    uint8_t id;
    handle_t int_handle;

    int_handle = mos_disable_ints();

#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_THREAD_EXIT);
#endif

    // Find the id of the currently running thread
    for(id = 0; id < MAX_THREADS; id++) {
        if(&threads[id] == _current_thread)
            break;
    }

    // De-allocate the stack memory and free the thread space
    mos_mem_free(threads[id].stack);
    threads[id].state = EMPTY;
    threads[id].suspend_state = SUSPEND_STATE_SLEEP;
 
    mos_enable_ints(int_handle);
    dispatcher();
}


#ifdef DEBUG_MEMORY
// Print list of allocated threads and their stack usage
void print_threads(uint8_t verbose)
{
    uint8_t nt = 0;
    uint16_t totstack = 0;
    uint16_t usestack = 0;
    uint8_t ok = 1;

    uint8_t i;
    for(i = 0; i < MAX_THREADS; i++) {
        if(threads[i].state != EMPTY) {
            // check for flag bytes that haven't been clobbered
            uint16_t unused = mos_check_stack(&threads[i]);
            if(verbose)
                printf_P(sThreadRow, i, threads[i].sp, threads[i].stack, 
                        threads[i].stackSize, unused, threads[i].state,
                        threads[i].priority);
            nt++;
            totstack += threads[i].stackSize;
            usestack += (uint16_t)threads[i].sp - (uint16_t)threads[i].stack;
            if(threads[i].sp < threads[i].stack) {
                printf_P(sStackOver, i);
                ok = 0;
            }
            if(unused == 0) {
                printf_P(sStackFull, i);
                ok = 0;
            }
        }
    }
    printf_P(sThreadTot, nt, totstack, usestack);
}
#endif

SLEEP_INT_HEADER()
{
   
    if(SLEEP_TIMER_EXPIRED)
    {
       
#ifdef MOS_DEBUG
        MOS_DEBUG_POST_SLEEP();
#endif

#ifdef SLEEP_ALARMS
        process_alarm();
#endif

    
        SET_ALARM_TIMER_VALUE(0);
        DISABLE_SLEEP_TIMER();

        //if (last_sleep_time > 1000)
        //{
        clock_init();
        kernel_timer_init();
        //}

        ENABLE_ALARM_TIMER();

#ifdef PLATFORM_TELOSB
        _BIC_SR_IRQ(LPM4_bits);
#endif
        //we have woken up from sleep, idle thread will call dispatcher

        if(running)
        {
            // increment the real_time value by last_sleep_time
            mos_inc_realtime(last_sleep_time);
            mos_thread_wakeup(last_sleep_time);
        }

    }
}
