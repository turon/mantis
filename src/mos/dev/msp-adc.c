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

/*
  Project Mantis
  File:   msp-adc.c
  Author: Charles Gruenwald III
  Date:   05-20-05

  This file implements the lowest level control of the analog to digital
  conversion system on the msp430x16x device.
*/

#include "mos.h"

#ifdef PLATFORM_TELOSB

#include "msp-adc.h"
#include "dev.h"
#include "mutex.h"
#include "sem.h"

uint8_t adc_channel;

void adc_set_channel(uint8_t ch)
{
   uint8_t addr = 0;
   //set the addr where conversion result will reside
   ADC12CTL1 |= ((addr & 0x0F) << 11);

   //set the addr to read from the corresponding channel
   ADC12MCTL0 |= (ch & 0x0F);

}

uint16_t adc_get_conversion16(uint8_t ch)
{

   ADC12CTL0 &= ~ENC;

   adc_on();

   adc_set_channel(ch);
   
   //set source of sampling signal (SAMPCON) to be the output of
   //the sampling timer
   ADC12CTL1 |= SHP;

   //start the conversion
   ADC12CTL0 |= ENC | ADC12SC;

   //polling wait for conversion to complete
   while(ADC12CTL1 & ADC12BUSY);

   uint16_t sample = ADC12MEM0;
   return sample;
}


#endif
