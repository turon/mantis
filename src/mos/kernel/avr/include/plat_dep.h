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


/** @file avr/include/plat_dep.h
 * @brief AVR Platform specific functions used in the kernel
 *
 */
 
// ATMEGA/AVR platform dependent "stuff"

#ifndef __PLAT_DEP_H__
#define __PLAT_DEP_H__

#include "mos.h"
#include <avr/wdt.h>

// 4k + 256 memory mapped IO at beginning
/** @brief Total bytes of memory */
#define MEMORY_SIZE  (4096)
#define MEMORY_BASE (256)
#define ARCH_PROGMEM PROGMEM

#define ALARM_CNTRLA TCCR1A
#define ALARM_CNTRLB TCCR1B
#define ALARM_FLAG   OCF1A
#define ALARM_CNT    TCNT1
#define ALARM_INT    OCIE1A
#define ALARM_OCR    OCR1A
#define ALARM_SIG    SIG_OUTPUT_COMPARE1A

#define SLEEP_CNTRL TCCR0
#define SLEEP_INT   OCIE0
#define SLEEP_OCR   OCR0
#define SLEEP_CNT   TCNT0
#define SLEEP_FLAG  OCF0
#define SLEEP_SIG   SIG_OUTPUT_COMPARE0

/*#define TIMER_OCR           OCR3A
#define TIMER_INT           OCIE3A
#define TIMER_CNT           TCNT3
#define TIMER_CNTRLA        TCCR3A
#define TIMER_CNTRLB        TCCR3B
#define TIMER_FLAG          OCF3A
#define TIMER_SIG           SIG_OUTPUT_COMPARE3A
#define OCR_MAX             0xFFFF
*/

#define SOFT_INT_SIG SIG_INTERRUPT7

#define TRIGGER_INT_BIT 7 //which user interrupt to use to trigger dispatcher
                          //NOTE: if you change this you must change signal handler
                          //ex: SIGNAL(INTx)

#define SETUP_SOFT_INT()                          \
   EIMSK |= (1 << TRIGGER_INT_BIT);               \
   EICRB &= ~(0x3 << (TRIGGER_INT_BIT - 4) * 2);  \
   DDRE |= (1 << TRIGGER_INT_BIT)
#define DISABLE_SOFT_INT() PORTE |= (1 << TRIGGER_INT_BIT)
#define DO_SOFT_INT() PORTE &= ~(1 << TRIGGER_INT_BIT)

// in order to wake from sleep mode the timer must be async
#define WAIT_FOR_ASYNC_UPDATE() \
      while ((ASSR & (1 << OCR0UB)) || (ASSR & (1 << TCN0UB)));

// stack pointer type
typedef uint8_t stackval_t;

//16bit address space
typedef uint16_t memtype_t;

// size of the SR -- used for mos_(enable|disable)_ints();
typedef uint8_t handle_t;

// minimum stack size
#define MIN_STACK_SIZE 64

#define ENABLE_INTS() sei()
#define DISABLE_INTS() cli()

#define PLAT_INIT() do {                                              \
    WDTCR = (1 << WDCE) | (1 << WDE); /*disable watchdog timer*/      \
    WDTCR = 0x00;                                                     \
    MCUCSR |= (1 << JTD); /*disable on-chip debugging*/               \
} while(0);


inline void plat_init();

#define ENABLE_WD_TIMER() WDTCR |= (1 << WDE) //enable watchdog timer
#define DISABLE_WD_TIMER() do {						\
      WDTCR = ((1 << WDCE) | (1 << WDE)); /*disable watchdog timer*/	\
      WDTCR = 0x00;							\
   } while(0)
#define SET_WD_TIMER_VALUE(val) do{              \
      WDTCR |= ((1 << WDCE) | (1 << WDE));         \
      WDTCR &= ~(1 << WDCE);                     \
      WDTCR |= val;                              \
   } while(0)  
#define RESET_WD_TIMER() asm volatile("wdr\n")

// timeslicing macros
#define DISABLE_ALARM_TIMER() TIMSK &= ~(1 << ALARM_INT)
#define ENABLE_ALARM_TIMER() TIMSK |= (1 << ALARM_INT)
#define ALARM_TIMER_VALUE ALARM_CNT
// avr doesn't need to double check why an int. occurred.
#define ALARM_TIMER_EXPIRED (1)

#define SET_ALARM_TIMER_VALUE(value) ALARM_CNT = value
#define SET_ALARM_OCR_VALUE(value) ALARM_OCR = value

#define PRE_KERNEL_SLEEP() SFIOR |= (1 << PUD); // disable all pull-ups
#define POST_KERNEL_SLEEP() SFIOR &= ~(1 << PUD); // re-enable the pull-ups

// sleep timer macros
#define DISABLE_SLEEP_TIMER() TIMSK &= ~(1 << SLEEP_INT)
#define ENABLE_SLEEP_TIMER() TIMSK |= (1 << SLEEP_INT)
#define SET_SLEEP_TIMER_VALUE(value) SLEEP_CNT = value
#define SET_SLEEP_OCR_VALUE(value) SLEEP_OCR = value;
// avr doesn't need to double check why an int. occurred.
#define SLEEP_TIMER_EXPIRED (1)

// set sleep enabled, modes for sleep mode
#define ENABLE_SLEEP()                                  \
   MCUCR |= (1 << SE) | (1 << SM1) | (1 << SM0);        \
   MCUCR &= ~(1 << SM2);

// set sleep enable, modes for idle mode
#define ENABLE_IDLE()                                   \
   MCUCR |= (1 << SE);					\
   MCUCR &= ~((1 << SM2) | (1 << SM1) | (1 << SM0));                

// perform sleep/idle
#define SLEEP() asm volatile("sleep\n")
#define IDLE() asm volatile("sleep\n")

#define SLEEP_INT_HEADER() SIGNAL(SLEEP_SIG)
#define ALARM_INT_HEADER() SIGNAL(ALARM_SIG)
#define SOFT_INT_HEADER() SIGNAL(SOFT_INT_SIG)
//extern uint16_t last_sleep_time;

/** @brief Initialize the hardware timer
 * This timer is used for the kernel time slice.
 */
void kernel_timer_init(void);

/** @brief initialize the timer used for sleeping
 *
 */
void sleep_timer_init(void);

/** @brief enables interrupts
 *
 */
#define mos_enable_ints(int_handle) (SREG = int_handle)

/** @brief disables interrupts
 *
 */
handle_t mos_disable_ints(void);


#define GET_SP(sp)              \
    asm volatile(               \
   	"in %A0, __SP_L__\n\t"  \
   	"in %B0, __SP_H__\n\t"  \
   	: "=r" (sp) : )         \






#endif
