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
  File:   adc.h
  Author: Jeff Rose
  Date:   01-21-03
  
  This file holds all the defines for the various ADC settings.
*/

#ifndef ADC_H_
#define ADC_H_

/* ADC clock prescaler values:
   (Slower conversion is more accurate and uses less power, but the
   conversion rate (clk. freq. / prescaler) should be at least 50 khz.)
*/
#define ADC_PRESCALE_2		0x00	// Fastest
#define ADC_PRESCALE_4		0x02	
#define ADC_PRESCALE_8		0x03	
#define ADC_PRESCALE_16		0x04	
#define ADC_PRESCALE_32		0x05	
#define ADC_PRESCALE_64		0x06	// Recommended level for 4mhz clock
#define ADC_PRESCALE_128	0x07	// Slowest

#define ADC_PRESCALE_MASK       0x07    // Register mask for setting prescaler

/* ADC voltage reference:
   (Either an internal voltage reference of 2.56v or the AVCC pin)
*/
#define ADC_VREF_AVCC           0x01    // AVCC 
#define ADC_VREF_IREF           0x03    // Internal 2.56v reference
#define ADC_VREF_MASK           0xC0    // Register mask for setting vref

/* ADC channel selection:
   (Selects which input pin to use for the conversion.)
*/
#define ADC_CH_0                0x00
#define ADC_CH_1       		0x01
#define ADC_CH_2	       	0x02
#define ADC_CH_3       		0x03

#define ADC_CH_MASK             0x07

/** Initialize the adc system variables.
 * NOTE: Must do this before using any other adc functions. 
 */
void mos_adc_init();
/** Take the adc lock so no other threads take the device. 
 */
void mos_adc_open(void);
/** Turn off ADC module on the chip.
 * If left on the converter will continue to drain power so be
 * sure to call this function when you are finished.
 */
void mos_adc_close(void);
/** Set the ADC prescaler value.
 * This effects the speed, accuracy and power consumption of the
 * conversion so read the data sheet if you want to use
 * something besides the default. 
 * @param prescale Prescaler value to set
 */
inline void mos_adc_prescale(uint8_t prescale);
/** Do a 10 bit conversion using the given analog input channel.
 * NOTE: You must set the pin as an input and set it to 0x00 so the
 * internal pullup resistors are disabled.)
 * @param ch Channel to convert
 */
uint16_t mos_adc_ten(uint8_t ch);
/** Do an 8 bit conversion using the given analog input channel.
 * NOTE: You must set the pin as an input and set it to 0x00 so the
 * internal pullup resistors are disabled.)
 * @param ch Channel to convert
 */
uint8_t mos_adc_eight(uint8_t ch);
/** Do an 8 bit conversion of the given analog input channel by polling.
 * NOTE: You must set the pin as an input and set it to 0x00 so the
 * internal pullup resistors are disabled.)
 * @param ch Channel to convert
 */
uint8_t mos_adc_eight_polling(uint8_t ch);

#endif
