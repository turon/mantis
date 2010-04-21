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
  Brian Shucker 4-14-04
  Based on light.c and temp.c by Charles Gruenwald

  Integrated light and temp sensor driver for the mica2.  This driver
  registers with the dev layer as two different devs, but they are
  intertwined in hardware so we implement it in one file.
*/

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "mica2-light-temp.h"
#include "adc.h"
#include "avr-adc.h"
#include "dev.h"
#include "mutex.h"
#include "clock.h" //provides mos_msleep

#if !defined(PLATFORM_LINUX) && !defined(PLATFORM_AVRDEV)

#define LIGHT 1
#define TEMP 2
#define LT_OFF 3

mos_mutex_t lt_mutex;

static uint8_t mode;
static uint8_t selected; //which device is selected

/*some utitily functions*/
static void select(uint8_t new_dev)
{
   if(selected == new_dev)
      return;
   
   switch (new_dev) {
   case TEMP:
      select_temp();
      break;
   case LIGHT:
      select_light();
      break;
   case LT_OFF:
      select_off();
      selected = new_dev;
      return;
   }

   mos_thread_sleep(5);
   //mos_mdelay(20);
   
   selected = new_dev;
}

/*the following functions implement the dev interface*/

/*light device functions*/

uint16_t dev_read_DEV_MICA2_LIGHT(void *buf, uint16_t count)
{
   uint8_t old_sel = selected;

   if(selected != LIGHT)
      select(LIGHT);

   dev_open(DEV_ADC);
   
   if(count == 1) { 
      //((uint8_t *)buf)[0] = adc_read_channel8(LIGHT_ADC_CHANNEL);
      ((uint8_t *)buf)[0] = adc_read_channel16(LIGHT_ADC_CHANNEL) >> 2;
      //((uint8_t *)buf)[0] = adc_poll(LIGHT_ADC_CHANNEL) >> 2;
   } else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(LIGHT_ADC_CHANNEL);
   else {
      return DEV_UNSUPPORTED;
   }

   dev_close(DEV_ADC);

   select(LT_OFF);
   
   if(old_sel != LIGHT)
      select(old_sel);
   
   return count;
}

/*temp device*/

uint16_t dev_read_DEV_MICA2_TEMP(void *buf, uint16_t count)
{
   uint8_t index;
   uint8_t old_sel = selected;

   if(selected != TEMP)
      select(TEMP);

   dev_open(DEV_ADC);
   
   if(count == 1) {
      //index = adc_read_channel8(TEMP_ADC_CHANNEL);
      index = adc_read_channel16(TEMP_ADC_CHANNEL) >> 2;
      //index = adc_poll(TEMP_ADC_CHANNEL) >> 2;
      ((int8_t *)buf)[0] = pgm_read_byte(&temp_vals[index/* - 3*/]);
   } else if(count == 2)
      ((uint16_t *)buf)[0] = adc_read_channel16(TEMP_ADC_CHANNEL);
   else {
      return DEV_UNSUPPORTED;
   }

   dev_close(DEV_ADC);

   select(LT_OFF);
   
   if(old_sel != TEMP)
      select(old_sel);
   
   return count;
}

uint16_t dev_write_DEV_MICA2_TEMP(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_TEMP(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

uint8_t dev_mode_DEV_MICA2_LIGHT(uint8_t newMode)
{
   mode = newMode;
   switch(newMode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      select(LIGHT);
      break;

   case DEV_MODE_OFF:
      select(LT_OFF);
      break;

   default:
      return DEV_UNSUPPORTED;
      break;
   }

   return DEV_OK;
}

uint16_t dev_write_DEV_MICA2_LIGHT(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_LIGHT(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

uint8_t dev_mode_DEV_MICA2_TEMP(uint8_t new_mode)
{
   switch(new_mode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      select(TEMP);
      break;

   case DEV_MODE_OFF:
      select(LT_OFF);
      break;

   default:
      return DEV_UNSUPPORTED;
      break;
   }
   mode = new_mode;

   return DEV_OK;
}

/*last, we have our init functions*/

/** @brief Init the light & temp sensors.
 */
void mica2_light_temp_init(void)
{
   mos_mutex_init(&lt_mutex);
   dev_mode(DEV_MICA2_LIGHT, DEV_MODE_OFF);
   select(LT_OFF);
}

#endif
#endif
