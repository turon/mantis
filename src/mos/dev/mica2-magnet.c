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
  Charles Gruenwald 4-22-04

  Integrated magnetometer sensor for the mica2.  This driver
  registers with the dev layer as two different devs, but they are
  intertwined in hardware so we implement it in one file.
*/

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "mica2-magnet.h"
#include "adc.h"
#include "avr-adc.h"
#include "dev.h"
#include "mutex.h"

#if defined(MICA2_MAGNET) || !defined(SCONS)

mos_mutex_t magnet_x_mutex;
mos_mutex_t magnet_y_mutex;
static uint8_t mode;

/*the following functions implement the dev interface */

/*magnetometer device functions*/
static void magnet_on(void) 
{
   MAGNET_PORT_DIRE |= MAGNET_PIN_MASK;
   MAGNET_PORT &= ~MAGNET_PIN_MASK;
   MAGNET_PORT |= MAGNET_PIN_MASK;
}

static void magnet_off(void)
{
   MAGNET_PORT_DIRE &= ~(MAGNET_PIN_MASK);
   MAGNET_PORT &= ~(MAGNET_PIN_MASK);
}

uint16_t dev_read_DEV_MICA2_MAGNET_X(void *buf, uint16_t count)
{
   if(mode == DEV_MODE_OFF)
      magnet_on();
   
   dev_open(DEV_ADC);
   
   if (count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8 (AVR_ADC_CH_5);
   else if (count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16 (AVR_ADC_CH_5);
   else
      ;

   dev_close(DEV_ADC);
   
   if(mode == DEV_MODE_OFF)
      magnet_off();
   
   return count;
}

uint16_t dev_write_DEV_MICA2_MAGNET_X(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_MAGNET_X(uint8_t md)
{   
   switch(md) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      magnet_on();
      break;

   case DEV_MODE_OFF:
      magnet_off();
      break;

   default:
      return DEV_UNSUPPORTED;
   }
   mode = md;

   return DEV_OK;
}

uint8_t dev_ioctl_DEV_MICA2_MAGNET_X(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

uint16_t dev_read_DEV_MICA2_MAGNET_Y (void *buf, uint16_t count)
{
   if(mode == DEV_MODE_OFF)
      magnet_on();

   dev_open(DEV_ADC);
   
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8 (AVR_ADC_CH_6); 
   else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16 (AVR_ADC_CH_6);
   else
      ;

   dev_close(DEV_ADC);
   
   if(mode == DEV_MODE_OFF)
      magnet_off();
  
   return count;
}

uint16_t dev_write_DEV_MICA2_MAGNET_Y(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_MAGNET_Y(uint8_t md)
{
   switch(md) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      magnet_on();
      break;
      
   case DEV_MODE_OFF:
      magnet_off();
      break;
      
   default:
      return DEV_UNSUPPORTED;
   }
   mode = md;
   
   return DEV_OK;
}

uint8_t dev_ioctl_DEV_MICA2_MAGNET_Y(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

/*last, we have our init functions*/
void mica2_magnet_init(void)
{
   mos_mutex_init(&magnet_x_mutex);
   mos_mutex_init(&magnet_y_mutex);
   magnet_off();
}

#endif
#endif
