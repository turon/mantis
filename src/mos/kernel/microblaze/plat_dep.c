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
#include "plat_clock.h"

// last_sleep_time declared in msched.c
extern uint16_t last_sleep_time;
extern void ostimer_vired(void);

//for hacking
#define FIT_INTERRUPT_ID XPAR_OPB_INTC_0_FIT_TIMER_0_INTERRUPT_INTR

static uint16_t nested_disables = 0;
XInterruptHandler fit_interrupt_handler(void *CallbackRef)
{
    ostimer_fired();
//    ALARM_INT_HEADER();
    return NULL;
}

handle_t mos_disable_ints(void)     
{
   //obtain the current status register
   handle_t sreg = mfmsr();

   microblaze_disable_interrupts();
   nested_disables++;

   return sreg;
}

void mos_enable_ints(handle_t int_handle)
{
    --nested_disables;
//    if(--nested_disables == 0)
//        microblaze_enable_interrupts();

    if(int_handle & 0x02)
    {
        handle_t sreg = GET_MSR();
        sreg |= 0x02;
        SET_MSR(sreg);
    }
//    SET_MSR(int_handle);
}

//don't need to do anything special here
void clock_init ()
{
    mb_register_and_enable_interrupt(FIT_INTERRUPT_ID,
            (XInterruptHandler)fit_interrupt_handler,
            NULL);
}

void kernel_timer_init(void)
{
}


void sleep_timer_init()
{
   last_sleep_time = 0;
}

