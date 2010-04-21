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

#if defined(PLATFORM_MICA_ANY)

#include "mica2-humidity.h"
#include "avr-i2c.h"
#include "dev.h"
#include "mutex.h"

#if defined(MICA2_HUMIDITY) || !defined(SCONS)

mos_mutex_t humidity_mutex;
uint8_t powered;
static uint8_t mode;

static void mica2_humidity_on(void)
{
   uint8_t i;
   i = HUMIDITY_PIN_MASK;

   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, HUMIDITY_CLOCK);
   
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, HUMIDITY_POWER_ADDR);
   dev_write(DEV_AVR_I2C, &i, sizeof(i));   
}

static void mica2_humidity_off(void)
{
   uint8_t i;
   i = ~(HUMIDITY_PIN_MASK);
   
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, HUMIDITY_POWER_ADDR);
   dev_write(DEV_AVR_I2C, &i, sizeof(i));
}

uint8_t dev_read_DEV_MICA2_HUMIDITY(void *buf, uint16_t count)
{
   avr_i2c_init();
   
   if(mode == DEV_MODE_OFF)
      mica2_humidity_on();

   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, HUMIDITY_DATA_ADDR);
   dev_read(DEV_AVR_I2C, (uint8_t *)buf, count);
   
   if(mode == DEV_MODE_OFF)
      mica2_humidity_off();
   
   return count;
}

uint8_t dev_write_DEV_MICA2_HUMIDITY(void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_HUMIDITY(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_humidity_init(void)
{
   mica2_humidity_off();
   powered = DEV_MODE_OFF;
   mos_mutex_init(&humidity_mutex);
}

#endif
#endif
