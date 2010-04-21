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

#if defined(ARCH_AVR)

#include "dev.h"

// last_sleep_time declared in msched.c
extern uint16_t last_sleep_time;


handle_t mos_disable_ints(void)     
{
    handle_t sreg;
    sreg = SREG;
    cli();

    return sreg;
}

//don't need to do anything special here
void clock_init (void)
{
}

void kernel_timer_init(void)
{
    //set to CTC mode
    ALARM_CNTRLA &= ~((1 << WGM11) | (1 << WGM10));
    ALARM_CNTRLB &= ~((1 << WGM13));
    ALARM_CNTRLB |= (1 << WGM12);

    //set ocr to 20 ms
    ALARM_OCR = TIMESLICE_20_MS;

    //clear interrupt flag mask
    TIFR |= (1 << ALARM_FLAG);
    TIMSK |= (1 << ALARM_INT);

    //set the prescaler
    ALARM_CNTRLB |= (1 << CS10);
    ALARM_CNTRLB &= ~((1 << CS11) | (1 << CS12));

}


void sleep_timer_init(void)
{
    SLEEP_CNTRL &= ~(0 << SLEEP_INT);
    ASSR = 1 << AS0;

    //set prescaler and CTC mode
    SLEEP_CNTRL = (1 << CS02) | (1 << CS01) | (1 << CS00) | (1 << WGM01);
    SLEEP_CNTRL &= ~(1 << WGM00);

    SLEEP_CNT = 0;

    //set OCR to ??? ms
    SLEEP_OCR = 1;
    TIMSK &= ~(1 << SLEEP_INT);
    TIFR |= (1 << SLEEP_FLAG);
    last_sleep_time = 0;
}

#endif
