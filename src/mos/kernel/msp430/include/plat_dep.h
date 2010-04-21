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


/** @file msp430/include/plat_dep.h
 * @brief MSP430 Platform specific functions used in the kernel
 *
 */

#ifndef __PLAT_DEP_H__
#define __PLAT_DEP_H__

#include <sys/types.h>
#include <signal.h>
#include <io.h>

#include "mos.h"

// 10k of RAM
/** @brief Total bytes of memory */
#define MEMORY_SIZE 10240
#define MEMORY_BASE 0x1100
#define ARCH_PROGMEM


#define RAMFUNC __attribute__ ((section (".data")))

#define SETUP_SOFT_INT() do{     \
   P2DIR |=  (1 << 5);           \
   P2SEL |=  (1 << 5);           \
   P2IES &= ~(1 << 5);           \
   P2IE  |=  (1 << 5);           \
}while(0)

#define DISABLE_SOFT_INT() P2IFG &= ~(1 << 5)
#define DO_SOFT_INT()      P2IFG |=  (1 << 5)

// not necessary for msp430
#define WAIT_FOR_ASYNC_UPDATE()

// stack pointer type
typedef uint16_t stackval_t;

//16bit address space
typedef stackval_t memtype_t;

// size of the SR -- used for mos_(enable|disable)_ints();
typedef uint16_t handle_t;

// minimum stack size
#define MIN_STACK_SIZE 128

#define ENABLE_INTS() eint()
#define DISABLE_INTS() dint()


#define PRE_KERNEL_SLEEP()
#define POST_KERNEL_SLEEP()

// turn off TimerA
#define DISABLE_ALARM_TIMER() TACTL &= ~(MC_3)
// start TimerA
#define ENABLE_ALARM_TIMER() TACTL |= MC_UPTO_CCR0

#define ALARM_TIMER_VALUE (TAR)
// make sure the interrupt was triggered
// because of an overflow
#define ALARM_TIMER_EXPIRED (TAIV == 10)

#define SET_ALARM_TIMER_VALUE(value) TAR = value
#define SET_ALARM_OCR_VALUE(value) TACCR0  = value

// turn off TimerB
#define DISABLE_SLEEP_TIMER() TBCTL &= ~(MC_3)
// start TimerB
#define ENABLE_SLEEP_TIMER() TBCTL |= MC_UPTO_CCR0
#define SET_SLEEP_TIMER_VALUE(time) TBR = time
#define SET_SLEEP_OCR_VALUE(value) TBCCR0  = value
// make sure the interrupt was triggered
// because of a capture
#define SLEEP_TIMER_EXPIRED (TBIV == 14)

// msp430 doesn't need to do anything special
// to enable low power modes
#define ENABLE_SLEEP()
#define ENABLE_IDLE()

// set low power mode bits in the SR to sleep
#define SLEEP() _BIS_SR(LPM3_bits)
#define IDLE() _BIS_SR(LPM0_bits)

#define SOFT_INT_HEADER()   interrupt (PORT2_VECTOR) soft_trig_int()
#define SLEEP_INT_HEADER()  interrupt (TIMERB1_VECTOR) sleep_wake_int()
#define ALARM_INT_HEADER() interrupt (TIMERA1_VECTOR) tslice_sig_int()

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
#define mos_enable_ints(int_handle) WRITE_SR(int_handle)

/** @brief disables interrupts
 *
 */
handle_t mos_disable_ints(void);


#define GET_SP(sp) {		\
      asm volatile(		\
	 "mov.w r1, %0\n\t"	\
	 :"=r" (sp) : );	\
	 }


#define srandom(seed) srand(seed)


/** @brief recalibrates the DCO to CLOCK_SPEED from plat_clock.h.
 * this function sources Timer A off of the DCO, then calculates an adjustment to the DCO based on how long it takes Timer A to overflow. */
void mos_recalibrate_dco(void);

/** @brief using the DCO to calculate 1 MS intervals is more accurate than using the 32 Khz timer; unfortunantly the DCO can vary in speed due to change in temperature, humidity, etc.  use mos_recalibrate_dco() to readjust the DCO speed. */
//#define TIMESLICE_USE_DCO



// this value should be 32, but the clock divisor is set to 2 by clock_init().
#define TIMESLICE_20_MS (32)

#endif
