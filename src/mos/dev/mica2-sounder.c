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

#define SOUNDER_PORT PORTC
#define SOUNDER_MASK (1 << 2)
#define SOUNDER_DIRE DDRC

#include "mos.h"

#if defined(PLATFORM_MICA_ANY)

#include "dev.h"
#include "mica2-sounder.h"
#include "mutex.h"

#if defined(MICA2_SOUNDER) || !defined(SCONS)

mos_mutex_t sounder_mutex;

static void mica2_sounder_on(void)
{
   SOUNDER_DIRE |= SOUNDER_MASK;
   SOUNDER_PORT &= ~SOUNDER_MASK;
   SOUNDER_PORT |= SOUNDER_MASK;
}

static void mica2_sounder_off(void)
{
   SOUNDER_PORT &= ~(SOUNDER_MASK);
   SOUNDER_DIRE &= ~(SOUNDER_MASK);
} 

/*write 0 to sounder to turn off, other numbers to turn on*/
uint16_t dev_write_DEV_MICA2_SOUNDER(const void *buf, uint16_t count)
{
   if(((uint8_t *)buf)[0] == 0)
      mica2_sounder_off();
   else
      mica2_sounder_on();

   return DEV_OK;
}

uint16_t dev_read_DEV_MICA2_SOUNDER(void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_MICA2_SOUNDER(uint8_t md)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_MICA2_SOUNDER(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

void mica2_sounder_init(void)
{
   mos_mutex_init(&sounder_mutex);
   mica2_sounder_off();
}

#endif
#endif
