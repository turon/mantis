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

/* File mica2-battery.c
 * Author: Charles Gruenwald III
 * Date: 04/23/2004
 */

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "adc.h"
#include "avr-adc.h"
#include "mica2-battery.h"
#include "dev.h"
#include "clock.h"
#include "mutex.h"

#include "printf.h"

#if defined(MICA2_BATTERY) || !defined(SCONS)

static uint8_t mode;
mos_mutex_t battery_mutex;

#if defined (PLATFORM_MICA2) || defined (PLATFORM_MICAZ)

#define MICA2_BATTERY_PORT PORTA
#define MICA2_BATTERY_DIRE DDRA
#define MICA2_BATTERY_MASK (1 << 5)

#if defined (PLATFORM_MICA2)
#define mica2_battery_on() MICA2_BATTERY_DIRE |= MICA2_BATTERY_MASK;	\
   MICA2_BATTERY_PORT &= ~MICA2_BATTERY_MASK;				\
   MICA2_BATTERY_PORT |= MICA2_BATTERY_MASK;				\
   mos_thread_sleep(5);
   //mos_udelay(100); //pause for accuracy

#define mica2_battery_off() MICA2_BATTERY_PORT &= ~MICA2_BATTERY_MASK; \
   MICA2_BATTERY_DIRE &= ~MICA2_BATTERY_MASK;

#elif defined(PLATFORM_MICAZ)
#define mica2_battery_on() mos_thread_sleep(5);
#define mica2_battery_off();
#endif

#define BATTERY_CHANNEL AVR_ADC_CH_7

#elif defined (PLATFORM_MICA2DOT)

#define MICA2_BATTERY_PORT PORTC
#define MICA2_BATTERY_DIRE DDRC
#define BATT_HI (1 << 5)
#define BATT_LO (1 << 7)

#define mica2_battery_on() MICA2_BATTERY_DIRE |= BATT_HI | BATT_LO;	\
   MICA2_BATTERY_PORT |= BATT_HI;					\
   MICA2_BATTERY_PORT &= ~BATT_LO;

#define mica2_battery_off() MICA2_BATTERY_DIRE &= ~(BATT_HI | BATT_LO); 

#define BATTERY_CHANNEL AVR_ADC_CH_1

#endif

uint8_t dev_mode_DEV_MICA2_BATTERY(uint8_t new_mode)
{
   mode = new_mode;
   switch(new_mode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      mica2_battery_on();
      break;

   case DEV_MODE_OFF:
      mica2_battery_off();
      break;

   default:
      return DEV_UNSUPPORTED;
   }

   return DEV_OK;
}

/*here are functions to implement dev interface*/
uint16_t dev_read_DEV_MICA2_BATTERY(void *buf, uint16_t count)
{
   uint32_t calculation;
   uint16_t sample_value;
   
   if(mode == DEV_MODE_OFF)
      mica2_battery_on();

   dev_open(DEV_ADC);
   //sample the value
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8(BATTERY_CHANNEL);
   if(count == 2) {
      //sample_value = adc_read_channel16(BATTERY_CHANNEL);
      sample_value = adc_poll(BATTERY_CHANNEL);
      
#if defined (PLATFORM_MICA2) || defined (PLATFORM_MICAZ)
      calculation = (1252352L) / (uint32_t)sample_value;//adc_max / value
#elif defined (PLATFORM_MICA2DOT)
      calculation = (uint32_t)(614400) / (uint32_t)sample_value;
      //printf("bs: %d", sample_value);
#endif
      //((uint16_t *)buf)[0] = (uint16_t)sample_value;
      ((uint16_t *)buf)[0] = (uint16_t)calculation;
   }
   
   dev_close(DEV_ADC);
   
   if(mode == DEV_MODE_OFF)
      mica2_battery_off();
   
   return count;
}

uint16_t dev_write_DEV_MICA2_BATTERY(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_BATTERY(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_battery_init(void)
{
   mica2_battery_off();
   mos_mutex_init(&battery_mutex);
   mode = DEV_MODE_OFF;
}

#endif
#endif
