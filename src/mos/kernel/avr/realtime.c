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


#include "mos.h"

#include "msched.h"
#include "led.h"

#ifndef SCONS
#include "config.h"
#endif

/** @brief Local Variables for measuring time elapsed
 *
 */
static uint32_t realtime_count;
static uint16_t skew_adjust;

void real_timer_clear()
{
   realtime_count=0;
#if defined ARCH_AVR
   TCNT2 = 0;
#endif
}

void real_timer_init(void)
{
   skew_adjust=1;
   uint8_t int_handle;
   int_handle = mos_disable_ints();
#if defined ARCH_AVR
   TCCR2 &= ~((1 << WGM20));
   TCCR2 |= (1 << WGM21);//Clear timer after interrupt
   TCCR2 |= (1 << CS21) | (1 << CS20);
   OCR2   = 114;          //fire an interrupt every ms
   TIFR  |= (1 << OCF2);  //clear flag
   TIMSK |= (1 << OCIE2); //turn on interrupt
#endif
   real_timer_clear();
   mos_enable_ints(int_handle);
}

inline uint32_t* real_timer_get_ticks()
{
   return &realtime_count;
}

#if defined ARCH_AVR
SIGNAL(SIG_OUTPUT_COMPARE2)
{
/*
   realtime_count++;
   if((skew_adjust++ % 160) == 0){
      realtime_count++;
      skew_adjust = 1;
   }
*/
}
#endif

