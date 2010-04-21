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

/** @file sys/include/led.h
 * @brief Control of the onboard leds for a sensor node.
 * @author Jeff Rose
 * @date 01/24/2003
 */

#ifndef LED_H_
#define LED_H_

#define NUM_LEDS        3

#ifndef PLATFORM_LINUX

#include "mos.h"

#ifdef ARCH_AVR
#define LED_YELLOW 0
#define LED_GREEN 1
#define LED_RED 2
#endif

/* Specify the port, data direction register and pin mask. */
#ifdef ARCH_AVR
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ) || defined (PLATFORM_MICA2DOT)
#define LED_PORT PORTA
#define LED_DIR DDRA
#define LED_MASK 0x07

#define LED_DISP(val) LED_PORT = ~val
#define LED_INIT_HARDWARE() LED_DIR |= LED_MASK;

#else

#define LED_DISP(led) asm volatile("nop" ::);
#define LED_INIT_HARDWARE() asm volatile("nop" ::);

#endif

#elif PLATFORM_TELOSB
#define LED_PORT P5OUT
#define LED_DIR P5DIR
#define LED_MASK ((1 << 4) | (1 << 5) | (1 << 6))

#define LED_DISP(val) do{                \
   LED_PORT |= LED_MASK;                 \
   LED_PORT &= ~((val << 4) & LED_MASK); \
  }while(0)

#define LED_INIT_HARDWARE() LED_DIR |= LED_MASK;

#elif PLATFORM_IMOTE2

#define LED_DISP(val) do{             \
  (val & 1) ? LED_ON(0) : LED_OFF(0); \
  (val & 2) ? LED_ON(1) : LED_OFF(1); \
  (val & 4) ? LED_ON(2) : LED_OFF(2); \
}while(0);

#define LED_INIT_HARDWARE() do{ \
   /* enable GPIO buffers */	\
   PSSR = (PSSR_RDH | PSSR_PH); \
   /* set LEDs as output */     \
   PIN_OUT(103); \
   PIN_OUT(104); \
   PIN_OUT(105); \
}while(0)

#elif PLATFORM_MICROBLAZE
#include "mb_led.h"

#define LED_DISP(val) do{             \
 mb_led_disp(val); \
}while(0);

#define LED_INIT_HARDWARE() do{ \
   mb_led_init(); \
; \
}while(0)


#else


#error "Undfined LEDs for this platform"
#endif


#else

#include "gevent.h"
#include "node_id.h"

/* Provide the same interface in XMOS.  
 * Changing LEDs sends an event to the simulator.
 */

#define LED_DISP(val)	do {					\
      gevent_send(LED_EVENT, "ii", "%i LED_DISP %x\n",		\
		  mos_node_id_get(), val);			\
   } while(0)
#endif

/** @brief Init the leds. */
void mos_led_init(void);

/** @briefGet the number of leds.
 * @return Number of leds
 */
uint8_t mos_led_get_num_leds(void);

/** @brief Turn on a given led.
 * @param led Led to turn on
 */
void mos_led_on(uint8_t led);

/** @brief Turn off a given led.
 * @param led Led to turn off
 */
void mos_led_off(uint8_t led);

/** @brief Toggle a given led.
 * @param led Led to toggle
 */
void mos_led_toggle(uint8_t led);

/** @brief Blink the LED twice
 */
void mos_led_blink(uint8_t led);

/** @brief Display a given led.
 * @param display_value Display value
 */
void mos_led_display(uint8_t display_value);

#endif
