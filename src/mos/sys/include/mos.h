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

/** @file mos.h
 * @author Jeff Rose
 * @date 05/12/2004
 * @brief Some system wide stuff that many files will want to include.
 */

#ifndef _MOS_H_
#define _MOS_H_

#include <inttypes.h>

#ifndef SCONS
#include "config.h"
#endif

#ifdef ARCH_AVR
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#endif

#ifdef PLATFORM_TELOSB
#include <io.h>
#endif

#ifdef PLATFORM_IMOTE2
#include <pxa27x_registers.h>
#include "plat_clock.h"
#endif

#ifdef PLATFORM_MICROBLAZE
#include "xparameters.h"
#include "xgpio.h"
#include "xilinx_help.h"
#include "xutil.h"
#include "mb_interface.h"
#endif

#ifdef ARCH_MICRO
#include "plat_dep.h"

#ifdef _STDIO_H_
#error "Do not include stdio.h, use printf.h for printf()"
#endif
// we really don't want to include stdio.h
#define _STDIO_H_

#define boolean int
//typedef uint8_t boolean;

/* Define some functions for easier I/O. */
#else
#include <stdbool.h>
typedef uint8_t boolean;
#endif

#ifndef TRUE
#define TRUE  1
#endif
#ifndef true
#define true  1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef false
#define false 0
#endif

#ifndef NULL
#define NULL 0
#endif


/* Uncomment the following line and recompile MOS if you do not want MOS to use
 * dynamically-allocated memory.
 */
//#define MOS_NO_USE_DYNAMIC_MEMORY

/* Uncomment to enable functions for debugging locks in multithreaded programs.
 */
//#define DEBUG_LOCKING

//initialization procedures for various devices
//this functions gets called before start()
inline void plat_init();

#endif
