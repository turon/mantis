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

// last_sleep_time is declared in msched.c
extern uint16_t last_sleep_time;


void mos_recalibrate_dco(void)
{
   handle_t h = mos_disable_ints();

   clock_init();
   kernel_timer_init();
   
   mos_enable_ints(h);
}



handle_t mos_disable_ints(void)
{
   handle_t sreg;
   asm volatile ("mov.w r2, %0\n\t" : "=r"(sreg));
   dint();
   return sreg;
}

void kernel_timer_init(void)
{
   // begin reset/init
   TACTL = TACLR;
   // source: 32768Hz ACLK, interrupt enabled.
   TACTL = TASSEL_ACLK | TAIE;

   // clear all other registers out
   TAR = 0;
   TACCTL0 = TACCTL1 = TACCTL2 = 0;
   TACCR0 = TACCR1 = 0;

   // 32 ticks at 32768ticks/second is ~ 1 millisecond
   TACCR0  = TIMESLICE_20_MS;
   

   // count up to ccr0 continuously.
   TACTL |= MC_UPTO_CCR0;
}

void sleep_timer_init(void)
{
   TBCTL = TBCLR;
   // src = ACLK, DIV = 8, INT = enabled, 16bit
   TBCTL = TBSSEL_ACLK | ID_DIV8 | TBIE | CNTL_0;

   last_sleep_time  = 0;
}

void clock_init(void)
{
    uint16_t old_ccr2_val;
    int16_t new_ccr2_val;
    uint8_t rsel_val;
    uint8_t dco_val;
   
    BCSCTL1 = XT2OFF | DIVA1 | RSEL2 | RSEL0;
    BCSCTL2 = 0;
    TACTL = TASSEL_2 | TACLR; // clear clock, source is SMCLK
    TACTL |= MC1; // start timer in continuous mode
    CCTL2 = CM0 | CCIS0 | CAP; // rising edge capture

    for(;;) {
	while(!(CCTL2 & CCIFG))
	    ; // wait for capture flag

	old_ccr2_val = CCR2;
	CCTL2 &= ~CCIFG;

	while(!(CCTL2 & CCIFG))
	    ; // wait for capture flag
	CCTL2 &= ~CCIFG;

	rsel_val = BCSCTL1;
	rsel_val &= (RSEL0 | RSEL1 | RSEL2); // mask out rsel bits
	dco_val = DCOCTL;

	if(dco_val == 0xff) {
	    if(rsel_val < 7) {
		BCSCTL1++;
	    } else {
		goto err;
	    }
	} else if(dco_val == 0) {
	    if(rsel_val != 0) {
		BCSCTL1--;
	    } else {
		goto err;
	    }
	} else {
	    new_ccr2_val = CCR2;
	    new_ccr2_val -= old_ccr2_val;
	    old_ccr2_val = CCR2;
	    new_ccr2_val -= (CLOCK_SPEED / (32768 / 4));
	    if(new_ccr2_val < 0) {
		DCOCTL++;
		continue;
	    } else if(new_ccr2_val == 0) {
		goto err;
	    } else {
		DCOCTL--;
		continue;
	    }
	}

	DCOCTL = 0x60; // center dco
    }
err:
    CCTL2 = 0;
    BCSCTL1 &= ~(DIVA1);
    
    return;
   
}
