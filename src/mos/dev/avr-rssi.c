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

/* File avr-rssi.c
 * Author: Charles Gruenwald III , Robert Havlik
 * Date: 04/23/2004
 */

#include "mos.h"

#ifdef ARCH_AVR

#include "adc.h"
#include "avr-adc.h"
#include "avr-rssi.h"
#include "dev.h"

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
#include "cc1000.h"
#include "clock.h" //gives us mos_udelay

#if defined(AVR_RSSI) || !defined(SCONS)

static uint8_t mode;
mos_mutex_t rssi_mutex;
extern uint8_t adc_channel; //hopefully this grabs it from the dev layer...

/*here are functions to implement dev interface*/

uint8_t dev_mode_DEV_AVR_RSSI(uint8_t new_mode)
{
   mode = new_mode;
   switch(new_mode) {
   case DEV_MODE_ON:
      avr_rssi_on();
      break;
   case DEV_MODE_OFF:
      avr_rssi_off();
      break;
   default:
      return DEV_UNSUPPORTED;
      break;
   }

   return DEV_OK;
}

uint8_t dev_ioctl_DEV_AVR_RSSI(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

uint16_t rssi_poll(void)
{
   uint16_t ret_val;

   if(mode == DEV_MODE_OFF) {
      avr_rssi_on();
      //mos_udelay (250);
   }

   //dev_open(DEV_ADC);

   ret_val = adc_poll(0xC0);

   //dev_close(DEV_ADC);
   
   if (mode == DEV_MODE_OFF)
      avr_rssi_off();

   return ret_val;
}


uint16_t dev_read_DEV_AVR_RSSI(void *buf, uint16_t count)
{
   if(mode == DEV_MODE_OFF) {
      avr_rssi_on();
      //mos_udelay (250);
   }

   dev_open(DEV_ADC);
   
   //sample the value
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8(AVR_ADC_CH_0);
   else if (count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(AVR_ADC_CH_0);
   else
      ;

   dev_close(DEV_ADC);
   
   if(mode == DEV_MODE_OFF)
      avr_rssi_off();

   return count;
}

uint16_t dev_write_DEV_AVR_RSSI(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

void avr_rssi_init(void)
{
   //init state vars
   mos_mutex_init(&rssi_mutex);
   mode = DEV_MODE_OFF;
}

#endif
#endif
#endif
