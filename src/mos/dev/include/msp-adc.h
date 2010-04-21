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

/** @file msp-adc.h
 * @brief Implements the lowest level control of the analog to digital 
 * conversion system on the msp430x16x device.
 * @author Charles Gruenwald III
 * @date Created: 05/20/2005
 */

#ifndef _MSP_ADC_H_
#define _MSP_ADC_H_

#ifdef ARCH_AVR

#define adc_on() ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADIF)
#define adc_off() ADCSRA &= ~(1 << ADEN)

#elif defined(PLATFORM_TELOSB)

#define adc_on()    /* turn on reference generator */	\
   ADC12CTL0 |= REF2_5V | REFON;			\
   /* turn on ADC12 */					\
   ADC12CTL0 |= ADC12ON;				\
   /* enable conversion to take place */		\
   /* ADC12CTL0 |= ENC; */

#define adc_off()    /* turn off reference voltage */	\
   /* ADC12CTL0 &= ~REFON; */				\
   /* turn off adc core */	 			\
   /* ADC12CTL0 &= ~ADC12ON; */

#endif

/** @brief Init the adc device.
 */
void adc_init();

/** @brief Allow out-of-band interface with 16-bit read support.
 * @param ch Channel to read
 */
uint16_t adc_read_channel16(uint8_t ch);
uint16_t adc_get_conversion16(uint8_t ch);

#endif
