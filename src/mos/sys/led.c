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
  Project Mantis
  File: leds.c
  Author: Jeff Rose
  Date: 1-24-03

  Control of the onboard leds.
*/

/** @file led.c
 * @brief Control of the onboard leds.
 * @author Jeff Rose
 * @date 01/24/2003
 */

#include <inttypes.h>

#ifndef SCONS
#include <config.h>
#endif

#include "led.h"
#include "plat_dep.h"
//#include "clock.h"

#define STATE_ON(s,led)  s |= (1 << led)
#define STATE_OFF(s,led) s &= ~( 1<< led)
#define STATE_IS_ON(s, led) (s & (1 << led))
#define SET_STATE(s, val) s = val

static uint8_t state;

void mos_led_init(void)
{
   uint8_t i;

   LED_INIT_HARDWARE();

   for(i = 0; i < NUM_LEDS; i++) {
      STATE_OFF(state,i);
   }
   LED_DISP(state);

}

uint8_t mos_led_get_num_leds(void)
{
   return NUM_LEDS;
}

void mos_led_on(uint8_t led)
{
    STATE_ON(state,led);
    LED_DISP(state);
}

void mos_led_off(uint8_t led)
{
    STATE_OFF(state,led);
    LED_DISP(state);
}

void mos_led_blink(uint8_t led)
{
#ifndef PLATFORM_LINUX
   mos_led_off(led);
   mos_udelay(0xffff);
   mos_led_on(led);
   mos_udelay(0xffff);
   mos_led_off(led);
   mos_udelay(0xffff);
   mos_led_on(led);
   mos_udelay(0xffff);
   mos_led_off(led);
#endif
}

void mos_led_toggle(uint8_t led)
{
   if(STATE_IS_ON(state,led)) {
       mos_led_off(led);
   } else {
       mos_led_on(led);
   }
}

void mos_led_display(uint8_t display_value)
{
  SET_STATE(state,display_value);
  LED_DISP(display_value);
}
