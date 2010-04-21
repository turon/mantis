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

#ifndef PLATFORM_LINUX
#include "dev.h"
#include "mutex.h"
#include "sem.h"
#include "adc.h"

static mos_sem_t adc_sem;
static uint8_t mode;
mos_mutex_t adc_mutex;
static uint8_t adc_channel;
static uint16_t adc_val;

#ifndef ARCH_AVR

void adc_init(void)
{
   
}


#else

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

void adc_init(void)
{
   //init adc state
   mos_sem_init(&adc_sem, 0);
   mos_mutex_init(&adc_mutex);
   mode = DEV_MODE_OFF;
   adc_off();
   adc_channel = 1;
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1);
}

static void adc_set_channel(uint8_t ch)
{
   adc_off();
   ADMUX = ch /*| (1 << REFS0) | (1 << REFS1)*/; //set the channel
   adc_channel = ch;
   adc_on();
}

uint16_t adc_poll(uint8_t ch)
{
   uint16_t ret_val;

   if(mode == DEV_MODE_OFF)
      adc_on();

   if(ch != adc_channel)
      adc_set_channel(ch);
   
   ADCSRA &= ~(1 << ADIE); //disable the AD Interrupt
   ADCSRA |= (1 << ADIF); //clear any old conversions...
   ADCSRA |= (1 << ADEN) | (1 << ADSC); //turn on on and start conversion

   //poll ADSC (clears once conversion is complete)
   while (ADCSRA & (1 << ADSC))
      ;

   ret_val = ADCL;  
   ret_val |= (ADCH << 8);

   if(ch != adc_channel)
      adc_set_channel(adc_channel);
   
   if(mode == DEV_MODE_OFF)
      adc_off();
   
   return ret_val;
}

uint16_t adc_read_channel16(uint8_t ch)
{
   uint16_t retval;
   
   //turn on if necessary
   if(mode == DEV_MODE_OFF)
      adc_on();
   
   //set to appropriate channel
   if(ch != adc_channel)
      adc_set_channel(ch);
   
   //start the conversion, enable interrupt 
   ADCSRA |= (1 << ADIF);
   ADCSRA |= (1 << ADSC) | (1 << ADIE) | (1 << ADEN);
   
   // Wait for the conversion interrupt handler to post the semaphore.
   mos_sem_wait(&adc_sem);
   retval = adc_val;
   if(ch != adc_channel)
      adc_set_channel(adc_channel);
   
   if(mode == DEV_MODE_OFF)
      adc_off();
   
   return retval;
}

uint8_t adc_read_channel8(uint8_t ch)
{
   return adc_read_channel16(ch) >> 2;
}

//TODO: add support for nonblocking, fast read for use by CSMA driver

/*implement the dev layer interface*/

/*reading the adc gets the current value of the current channel (use ioctl
  to change channels).  A 1-byte read returns an 8-bit conversion.  Any larger
  read returns a 10-bit conversion, with the MSB first.*/
uint16_t dev_read_DEV_ADC(void *buf, uint16_t count)
{
   //copy into the provided buffer
   if(count == 1) {
      //write as a byte
      ((uint8_t *)buf)[0] = adc_read_channel8(adc_channel);
      count = 1;
   } else {
      //write as a 16-bit word
      ((uint16_t *)buf)[0] = adc_read_channel16(adc_channel);
      count = 2;
   }

   return count;
}

uint16_t dev_write_DEV_ADC(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

/*the adc turns on automatically and stays on.  Use the mode function
  to turn it back off to save power.*/
//TODO: come up with a better way to handle ADC modes
uint8_t dev_mode_DEV_ADC(uint8_t new_mode)
{
   if(mode == new_mode)
      return new_mode;
   
   mode = new_mode;
   
   switch(new_mode) {
   case DEV_MODE_OFF:
      adc_off();
      break;
   case DEV_MODE_IDLE:
      break;
   case DEV_MODE_ON:
      adc_on();
      break;
   }

   return new_mode;
}

/*this implements specific commands to the adc device*/
uint8_t dev_ioctl_DEV_ADC(int8_t request, ...)
{
   int arg;
   va_list ap;
   
   va_start(ap, request);
   
   switch(request) {
   case ADC_SET_CHANNEL:
      // extract from varargs (must be at least 16 bits)
      arg = va_arg(ap, int);
      adc_channel = arg; //record channel setting
      adc_set_channel(adc_channel);
      break;

   default:
      return DEV_BAD_IOCTL;
   }

   va_end(ap);
   
   return DEV_OK;
}

SIGNAL(SIG_ADC)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif

// grab the conversion value, low MUST be first
   adc_val = ADCL;
   adc_val |= (ADCH << 8);
   mos_sem_post_dispatch(&adc_sem);
}

#endif
#endif
