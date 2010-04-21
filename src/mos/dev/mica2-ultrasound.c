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

#define ULTRASOUND_PORT PORTC
#define ULTRASOUND_MASK_ENABLE 0x04
#define ULTRASOUND_MASK_ON 0x01
#define ULTRASOUND_DIRE DDRC

#if defined(MICA2_ULTRASOUND) || !defined(SCONS)

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "printf.h"
#include "dev.h"
#include "mica2-ultrasound.h"
#include "mutex.h"

/*the basic on and off functions are available to external users,
  but the recommended use is through the dev_write call*/

static mos_mutex_t ultrasound_mutex;

void mica2_ultrasound_enable(void)
{
   printf("Enabling Ultrasound!\n");
   ULTRASOUND_DIRE |= (ULTRASOUND_MASK_ENABLE);  
   ULTRASOUND_PORT |= ULTRASOUND_MASK_ENABLE; //power on
}

void mica2_ultrasound_disable(void)
{
   printf("Disabling Ultrasound!\n");
   ULTRASOUND_PORT &= ~(ULTRASOUND_MASK_ENABLE);//power off
} 

void mica2_ultrasound_on(void)
{
   printf("turning Ultrasound ON!\n");
   ULTRASOUND_DIRE |= (ULTRASOUND_MASK_ON);  
   ULTRASOUND_PORT |= ULTRASOUND_MASK_ON; //power on
}

void mica2_ultrasound_off(void)
{
   printf("turning Ultrasound OFF!\n");
   ULTRASOUND_PORT &= ~(ULTRASOUND_MASK_ON);//power off
} 

/*write 0 to ultrasound to turn off, other numbers to turn on*/
//TODO: send more meaningful instructions to ultrasound, like beep duration
uint16_t dev_write_DEV_MICA2_ULTRASOUND(const void *buf, uint16_t count)
{
   mos_mutex_lock(&ultrasound_mutex);
   
   if(((uint8_t *)buf)[0] == 0)
      mica2_ultrasound_off();
   else
      mica2_ultrasound_on();

   mos_mutex_unlock(&ultrasound_mutex);
   
   return count;
}

uint16_t dev_read_DEV_MICA2_ULTRASOUND(void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_ULTRASOUND(uint8_t md)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_ULTRASOUND(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_ultrasound_init(void)
{
   mos_mutex_init(&ultrasound_mutex);
   mica2_ultrasound_off();
}

#endif
#endif
