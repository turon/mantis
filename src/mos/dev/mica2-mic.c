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

/* File mic.c
 * Author: Charles Gruenwald III
 * Date: Mar 10, 2004
 */

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "adc.h"
#include "avr-adc.h"
#include "mica2-mic.h"
#include "dev.h"
#include "mutex.h"

#if defined(MICA2_MIC) || !defined(SCONS)

//port definitions
/** @brief Mic directional port. */
#define MIC_PORT_DIRE DDRC
/** @brief Mic primary port. */
#define MIC_PORT PORTC
/** @brief Mic pin mask port. */
#define MIC_PIN_MASK 0x08

static uint8_t mode;
mos_mutex_t mic_mutex;

/*some utility functions*/
static void mic_power_on(void)
{
   MIC_PORT_DIRE |= MIC_PIN_MASK;
   MIC_PORT &= ~MIC_PIN_MASK;
   MIC_PORT |= MIC_PIN_MASK;

   mos_thread_sleep(5);
}

static void mic_power_off(void)
{
   MIC_PORT &= ~MIC_PIN_MASK;
   MIC_PORT_DIRE &= ~MIC_PIN_MASK;
}

/*here are functions to implement dev interface*/

uint8_t dev_mode_DEV_MICA2_MIC(uint8_t new_mode)
{
   switch(new_mode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      mic_power_on(); //power up
      break;

   case DEV_MODE_OFF:
      mic_power_off(); //power down
      break;

   default:
      return DEV_UNSUPPORTED;
      break;
   }
   mode = new_mode;
   
   return DEV_OK;
}

uint16_t dev_read_DEV_MICA2_MIC(void *buf, uint16_t count)
{
   //might have to turn the mic on
   if(mode == DEV_MODE_OFF)
      mic_power_on();

   //sample the value
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8(AVR_ADC_CH_2);
   else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(AVR_ADC_CH_2);
   else {
      return DEV_UNSUPPORTED;
   }

   //if we turned it on, turn it back off
   if(mode == DEV_MODE_OFF)
      mic_power_off();

   return count;
}

uint16_t dev_write_DEV_MICA2_MIC(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_MIC(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_mic_init(void)
{
   //init state vars
   mic_power_off();
   mos_mutex_init(&mic_mutex);
}

#endif
#endif
