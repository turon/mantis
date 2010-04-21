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

  Integrated accelerometer sensor for the mica2.  This driver
  registers with the dev layer as two different devs, but they are
  intertwined in hardware so we implement it in one file.
*/

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "mica2-accel.h"
#include "adc.h"
#include "avr-adc.h"
#include "dev.h"
#include "mutex.h"
#include "clock.h"

#if defined(MICA2_ACCEL) || !defined(SCONS)

mos_mutex_t accel_mutex;
static uint8_t mode;

/*the following functions implement the dev interface */

/*accelerometer device functions*/

/** @brief Turn on the accel sensor.
 */
static void accel_on(void) 
{
   ACCEL_PORT_DIRE |= ACCEL_PIN_MASK;
   ACCEL_PORT &= ~ACCEL_PIN_MASK;
   ACCEL_PORT |= ACCEL_PIN_MASK;
}

/** @brief Turn off the accel sensor.
 */
static void accel_off(void)
{
   ACCEL_PORT_DIRE &= ~(ACCEL_PIN_MASK);
   ACCEL_PORT &= ~(ACCEL_PIN_MASK);
}

/** @brief Read the sensor's x-acceleration.
 * @return x-acceleration
 */
uint16_t dev_read_DEV_MICA2_ACCEL_X(void *buf, uint16_t count)
{
   if(mode == DEV_MODE_OFF)
      accel_on();
   
   dev_open(DEV_ADC);
   
   //sample the value
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8(AVR_ADC_CH_3);
   else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(AVR_ADC_CH_3);
   else {
      dev_close(DEV_ADC);
      return DEV_UNSUPPORTED;
   }

   dev_close(DEV_ADC);
   
   if(mode == DEV_MODE_OFF)
      accel_off();
   
   return count;
}

uint16_t dev_write_DEV_MICA2_ACCEL_X(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_ACCEL_X(uint8_t md)
{
   switch(md) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      accel_on();
      break;

   case DEV_MODE_OFF:
      accel_off();
      break;

   default:
      return DEV_UNSUPPORTED;
   }
   mode = md;

   return DEV_OK;
}

uint8_t dev_ioctl_DEV_MICA2_ACCEL_X(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

/** @brief Read the sensor's y-acceleration.
 * @return y-acceleration
 */
uint16_t dev_read_DEV_MICA2_ACCEL_Y(void *buf, uint16_t count)
{
   if(mode == DEV_MODE_OFF)
      accel_on();

   dev_open(DEV_ADC);
   
   //sample the value
   if(count == 1)
      ((uint8_t *)buf)[0] = adc_read_channel8(AVR_ADC_CH_4);
   else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(AVR_ADC_CH_4);
   else {
      dev_close(DEV_ADC);
      return DEV_UNSUPPORTED;
   }

   dev_close(DEV_ADC);

   if(mode == DEV_MODE_OFF)
      accel_off();
  
   return DEV_OK;
}

uint16_t dev_write_DEV_MICA2_ACCEL_Y(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_ACCEL_Y(uint8_t md)
{
   switch(md) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      accel_on();
      break;

   case DEV_MODE_OFF:
      accel_off();
      break;

   default:
      return DEV_UNSUPPORTED;
   }
   mode = md;

   return DEV_OK;
}

uint8_t dev_ioctl_DEV_MICA2_ACCEL_Y(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_accel_init(void)
{
   mos_mutex_init (&accel_mutex);
   accel_off();
}

#endif
#endif
