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

  Modifications:
    12-07-07, jwm:  Fixed adc_set_channel to work
                    Added adc_init, including selection of internal Vref range 1.5,2.5
					and forced pulse-sampling.
					Moved adc_on outside of get_conversion for user code to call
					NOTE the extended_sampling is not yet tested, as is incorporation
					of full features for alternate sample clocks, timing, etc.
					bare-bones for the moment.
*/

#include "mos.h"

#ifdef PLATFORM_TELOSB

#include "msp-adc.h"
#include "dev.h"
#include "mutex.h"
#include "sem.h"


//----------------------------------------
// Telos ADC Initialization
//----------------------------------------
// CHOSE ONE OF THESE SOMEWHERE!!!
//
//#define VREF_Vcc
//#define VREF_2_5      // But this is the default in any event
//#define VREF_1_5

// CHOSE ONE OF THESE SOMEWHERE!!!
//
#define PULSE_SAMPLING_MODE     // But this is the default in any event
//#define EXTENDED_SAMPLING_MODE


void adc_init()
{
//  printf("\n\rADC_INIT:");

    ADC12CTL0 &= ~ENC;      // Clear the ENC bit to allow setting register bits

    ADC12IE = 0;            // In this MOS application, don't use Interrupt Enable/Vectors

    P6SEL = 0xFF;           // Disable Port6 Pin buffers for ADC in to elim. parasitics
                            // Note P6 Pins 6,7 have multiple use so these may need to
                            // be set/reset elsewhere.
                            // Should really just set the ones we're going to use

    // In this MOS application use the Internal Reference Voltage
    // The actual turn-on of the Vref is done in 'adc_on();' via REFON of CTL0 register.
    // Here we select set whether it will be Vcc (ie battery/supply) or the internal
    // reference of 2.5 or 1.5 volts
    //
#ifndef VREF_Vcc
#ifdef VREF_1_5
    ADC12CTL0 &= ~REF2_5V;  // Use 1.5V Vref, bit=0
//  printf("  Vref=1.5");
#else
    ADC12CTL0 |= REF2_5V;   // Use 2.5V Vref, bit=1: default setting
//  printf("  Vref=2.5");
#endif
    // Not only do we need to enable the internal vref via the VREF_2
    // but also the conversion memory control registers
    //
    ADC12MCTL0 |= SREF_1;   // This is for Vref+ to AVss on TelosB
                            // Setting the '2.5' or '1.5' is done
#else                       // If VREF_Vcc is defined all we need is to set it in memctl
    ADC12MCTL0 |= SREF_0;   // This is for Vcc to Gnd on TelosB
//  printf("  Vref=Vcc");
#endif  // VREF_Vcc

    // Sampling Mode: 1 of 2 options available
#ifdef EXTENDED_SAMPLING_MODE
   ADC12CTL1 &= ~SHP;
#else       // PULSE_SAMPLING_MODE is default
   ADC12CTL1 |= SHP;
#endif

//  printf("  P6SEL = %x ",P6SEL);
//  printf("  P6DIR = %x ",P6DIR);
//  printf("  MCTL0 = %x ",ADC12MCTL0);
//  printf("  CTL1 = %x ",ADC12CTL1);

}



//----------------------------------------
// Telos ADC channel set
//----------------------------------------
uint8_t adc_channel;

void adc_set_channel(uint8_t ch)
{
   uint8_t addr = 0;

   //set the addr where conversion result will reside
   ADC12CTL1 |= ((addr & 0x0F) << 11);

   // set the channel addr to read from
   // NOTE this does not set the SREF bits of the ADC12MCTLx
   // to select the reference source:....do that in the init routine
   // or elsewhere.  This method retains those bits set somewhere else
   ADC12MCTL0 &= 0xF0;      // Clear out the previous channel setting
   ADC12MCTL0 |= (ch &0xF); // and add in the new setting

}

//----------------------------------------
// Telos ADC Conversion routine that sets the channel for you
//----------------------------------------

uint16_t adc_get_conversion16(uint8_t ch)
{

   ADC12CTL0 &= ~ENC;	// Clear the ENC bit to allow setting register bits

//   adc_on();      // do this elsewhere!
                    // in your code, to facilitate multiple sampling
                    // noticing that the stabilization period of the Vref is quoted 17mS
                    // for each time you do this!

   adc_set_channel(ch);

   //set source of sampling signal (SAMPCON) to be the output of
   //the sampling timer
//   ADC12CTL1 |= SHP;      // This sets Pulse Sampling Mode....do that in adc_init not here

   //start the conversion
   ADC12CTL0 |= ENC | ADC12SC;

   //polling wait for conversion to complete
   while(ADC12CTL1 & ADC12BUSY);

   uint16_t sample = ADC12MEM0;
   return sample;
}

#endif	// PLATFORM_TELOSB
