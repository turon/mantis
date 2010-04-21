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
#ifndef __PLAT_DEP_H__
#define __PLAT_DEP_H__
#include <inttypes.h>
#include "plat_clock.h"

typedef uint32_t stackval_t;

//32bit address space
typedef stackval_t memtype_t;
typedef uint32_t handle_t;

//32bit address space
typedef stackval_t memtype_t;

#define ALARM_TIMER_EXPIRED (OSSR & OIER_E5)
#define ALARM_INT_HEADER void ostimer_fired


// place holders ////////////////////
#define SLEEP_INT_HEADER void placeholder__
#define SLEEP_TIMER_EXPIRED TRUE

#define DISABLE_ALARM_TIMER() (OMCR5 = OMCR_C | OMCR_P | OMCR_R | OMCR_CRES(0x0))


#define ENABLE_ALARM_TIMER() do{ \
    OMCR5 = OMCR_C | OMCR_P | OMCR_R | OMCR_CRES(0x4); \
    OSCR5 = 0; \
}while(0)

#define DISABLE_INTS()
#define ENABLE_IDLE()
#define IDLE()
#define SET_SLEEP_TIMER_VALUE(value) 
#define SET_ALARM_TIMER_VALUE(value) (OSCR5 = value)
#define SET_SLEEP_OCR_VALUE(value)
#define ENABLE_SLEEP_TIMER()
#define DISABLE_SLEEP_TIMER()
#define PRE_KERNEL_SLEEP()
#define SLEEP()
#define POST_KERNEL_SLEEP()
#define ENABLE_SLEEP()
#define GET_SP(sp) asm volatile("mov %0,sp\n\t" :: "r"(sp))

///////////////////////////////////

#define ARCH_PROGMEM
#define WAIT_FOR_ASYNC_UPDATE()

#define sleep_timer_init()

static inline void uart_init() 
{
}


#define clock_init()

#define com_send_IFACE_SERIAL(x)


#define MEMORY_BASE 0x5C000000
#define MEMORY_SIZE 262144


#define SR_IF_MASK 0x000000C0
/** @brief enables interrupts
 *
 */
static inline void mos_enable_ints(handle_t int_handle)
{
   int_handle &= SR_IF_MASK;
   uint32_t sreg = 0;
   
   asm volatile ("mrs %0, CPSR\n"
		 "bic %0, %1, %2\n"
		 "orr %0, %1, %3\n"
		 "msr CPSR_c, %1\n"
		 : "=r"(sreg)
		 : "0"(sreg), "i"(SR_IF_MASK), "r"(int_handle));
   
   
}

#define ENABLE_INTS() do { \
  uint32_t statusReg = 0;  \
  asm volatile (                      \
	       "mrs %0,CPSR\n\t"      \
	       "bic %0,%1,#0xc0\n\t"  \
	       "msr CPSR_c, %1"       \
	       : "=r" (statusReg)     \
	       : "0" (statusReg)      \
     );                               \
}while(0)
  
/** @brief disables interrupts
 *
 */
static inline handle_t mos_disable_ints()
{
   handle_t result = 0;
   uint32_t tmp = 0;
   
   asm volatile("mrs %0, CPSR\n"
		"orr %1, %2, %4\n"
		"msr CPSR_cf, %3\n"
		: "=r"(result), "=r" (tmp)
		: "0" (result), "1" (tmp), "i" (SR_IF_MASK));
   
   return result;
}



#define PIN_SET(regbit)  (_GPSR(regbit) |= _GPIO_bit(regbit))
#define PIN_CLR(regbit)  (_GPCR(regbit) |= _GPIO_bit(regbit))
#define PIN_READ(regbit) ((_GPLR(regbit) & _GPIO_bit(regbit)) != 0)

#define PIN_OUT(regbit) do{          \
 _GPIO_setaltfn(regbit,0);           \
 _GPDR(regbit) |= _GPIO_bit(regbit); \
}while(0)


#define PIN_INP(regbit) do{             \
 _GPIO_setaltfn(regbit,0);              \
 _GPDR(regbit) &= ~(_GPIO_bit(regbit)); \
}while(0)

#endif
