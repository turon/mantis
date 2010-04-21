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
  File: mantis.c
  Author: Jeff Rose
  Date: 1-25-03

  This is where it all starts, and ends.
*/

/** @file main.c
 * @brief This is where it all starts, and ends.
 * @author Jeff Rose
 * @date 01/25/2003
 */

#include "mos.h"
#include <stdlib.h>

#ifndef PLATFORM_LINUX

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
#include "cc1000_raw.h"
#include "cc1000_tdma.h"
#include "cc1000_csma.h"
#endif

#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

#if defined(PLATFORM_TELOSB)
#include "hardware-id.h"
#include "telos-flash.h"
#endif


#if defined(ARCH_AVR)
#include "avr-eeprom.h"
#include "avr-adc.h"
#include "atmel-flash.h"
#include "avr-i2c.h"
#include "avr-rssi.h"
#include <avr/wdt.h>
#endif

#if defined(PLATFORM_MICA_ANY)
#include "mica2-ultrasound.h"
#include "mica2-light-temp.h"
#include "mica2-sounder.h"
#include "mica2-battery.h"
#include "mica2-magnet.h"
#include "mica2-accel.h"
#include "mica2-mic.h"
#include "hardware-id.h"
#include "mica2-gps.h"
#endif

#if defined(ARCH_MICRO)
#include "adc.h"
#endif

#include "node_id.h"
#include "uart.h"
#include "dev.h"
#include "printf.h"
#include "loopback.h"
#include "dev-loopback.h"
#endif

#ifdef PLATFORM_LINUX
#include "arg.h"
#include "gevent.h"
#include <stdio.h>
#define START_STACK_SIZE 0

#include "serial.h"
#include "terminal.h"
//#include "udp.h"
#include "xmos_radio.h"
#include "xmos-flash.h"

#endif

#include "led.h"
#include "msched.h"
#include "com.h"
#include "clock.h"

#ifdef MOS_DEBUG
#include "mos_debugging.h"
#endif

/** @brief Application's start function. */
extern void start(void);

/** @brief preStart() is the function for the startup thread, which is the
 * one spawned by main().  
 *
 * It does all the initialization that needs to happen after the 
 * kernel is up and running, and then it calls the application start function.
 */
void pre_start(void)
{
   //TODO: this should somehow be auto-generated
#ifdef PLATFORM_LINUX
   mos_node_id_init();	// must be called before gevent_init ()
   gevent_init();

   serial_init();
   terminal_init();
   //udp_init();
   xmos_radio_init();	// gevent_init() must be called first
   xmos_flash_init();	// gevent_init() must be called first
   mos_thread_suspend(); //fixes threading issue with freebsd

#elif defined(ARCH_MICRO)
   uart_init();
   printf_init();
   plat_init();
   clock_init();
#if defined(PLATFORM_TELOSB)
   // clock_init() clobbers TimerA
   // re-initialize TimerA here as a kludge.
   kernel_timer_init();
#endif
   mos_node_id_init();

   //dev_loopback_init();
   //com_loopback_init();


   // seed the rng
   //srandom(mos_node_id_get());
#endif //if defined(ARCH_AVR)

#ifdef MOS_DEBUG
   mos_debug_post_init();
#endif
   
   start();
}

/** @brief main function, inits the kernel and spawns the start thread. */
#ifdef PLATFORM_LINUX
int main(int argc, char *argv[])
{
   arg_init(argc,argv);
#else
int main(void)
{
#endif
   sched_init(); //init scheduler--THIS MUST BE FIRST
   mos_led_init(); //init leds early, to allow led debugging elsewhere
   com_init(); //init com system

#ifdef MOS_DEBUG
   mos_debug_init();
#endif
   mos_thread_new(pre_start, START_STACK_SIZE, PRIORITY_NORMAL);
   
   //start the scheduler (never returns)
   mos_sched_start();
   
   return 0;
}
