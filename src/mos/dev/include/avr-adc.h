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

/**
  Project Mantis
  File:   avr-adc.h
  Author: Brian Shucker
  Date:   04-14-04

  Analog-to-digital converter device driver for AtMEGA128
*/

/** @file avr-adc.h
 * @brief Implements the lowest level control of the analog to digital 
 * conversion system on the atmega128.
 * @author Jeff Rose
 * @author Modified: Brian Shucker
 * @date Created: 01/21/2003
 * @date Modified: 04/14/2004
 */

#ifndef _AVR_ADC_H_
#define _AVR_ADC_H_


#define AVR_ADC_AVERAGING_ON 1
#define AVR_ADC_AVERAGING_OFF 2
//function prototype


uint16_t rssi_poll (void);

/* ADC clock prescaler values:
   (Slower conversion is more accurate and uses less power, but the
   conversion rate (clk. freq. / prescaler) should be at least 50 khz.)
*/
#define AVR_ADC_PRESCALE_2 0x00	// Fastest
#define AVR_ADC_PRESCALE_4 0x02	
#define AVR_ADC_PRESCALE_8 0x03	
#define AVR_ADC_PRESCALE_16 0x04	
#define AVR_ADC_PRESCALE_32 0x05	
#define AVR_ADC_PRESCALE_64 0x06 // Recommended level for 4mhz clock
#define AVR_ADC_PRESCALE_128 0x07 // Slowest

#define AVR_ADC_PRESCALE_MASK 0x07 // Register mask for setting prescaler

/* ADC voltage reference:
   (Either an internal voltage reference of 2.56v or the AVCC pin)
*/
#define AVR_ADC_VREF_AVCC 0x01 // AVCC 
#define AVR_ADC_VREF_IREF 0x03 // Internal 2.56v reference

#define AVR_ADC_VREF_MASK 0xC0 // Register mask for setting vref


/* ADC channel selection:
   (Selects which input pin to use for the conversion.)
*/
#define AVR_ADC_CH_0            0x00
#define AVR_ADC_CH_1          	0x01
#define AVR_ADC_CH_2	       	0x02
#define AVR_ADC_CH_3           	0x03
#define AVR_ADC_CH_4           	0x04
#define AVR_ADC_CH_5           	0x05
#define AVR_ADC_CH_6           	0x06
#define AVR_ADC_CH_7           	0x07

#define AVR_ADC_CH_MASK         0x07

#endif
