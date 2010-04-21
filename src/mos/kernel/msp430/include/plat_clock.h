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

/** @file msp430/include/plat_clock.h
 * @brief clock speed macros, as well as precalculated data for
 * uart baud rate timing
 * @author John Ledbetter
 */

#ifndef __PLAT_CLOCK_H__
#define __PLAT_CLOCK_H__

#if   defined(CLOCK_SPEED_2_0)
#define CLOCK_SPEED      (uint32_t)2000000

#define UBR0_INIT  0x22
#define UBR1_INIT  0x00
#define UMCTL_INIT 0xDD

#elif defined(CLOCK_SPEED_3_68)
#define CLOCK_SPEED      (uint32_t)3686400

#define UBR0_INIT  0x40
#define UBR1_INIT  0x00
#define UMCTL_INIT 0x00

#elif defined(CLOCK_SPEED_4_0)
#define CLOCK_SPEED      (uint32_t)4000000

#define UBR0_INIT  0x45
#define UBR1_INIT  0x00
#define UMCTL_INIT 0xAA

#elif defined(CLOCK_SPEED_4_37)
#define CLOCK_SPEED     (uint32_t)4377600

#define UBR0_INIT  0x4C
#define UBR1_INIT  0x00
#define UMCTL_INIT 0x00

#else
#error Clock Speed Not Defined
#endif



#define CLOCK_SPEED_0    CLOCK_SPEED //no prescaling
#define CLOCK_SPEED_8    (CLOCK_SPEED /    (uint32_t)8)
#define CLOCK_SPEED_32   (CLOCK_SPEED /   (uint32_t)32)
#define CLOCK_SPEED_64   (CLOCK_SPEED /   (uint32_t)64)
#define CLOCK_SPEED_128  (CLOCK_SPEED /  (uint32_t)128)
#define CLOCK_SPEED_256  (CLOCK_SPEED /  (uint32_t)256)
#define CLOCK_SPEED_1024 (CLOCK_SPEED /  (uint32_t)1024)

#define EXT_CLOCK      32768
#define EXT_CLOCK_0    EXT_CLOCK //no prescaling
#define EXT_CLOCK_8    (EXT_CLOCK /    8)
#define EXT_CLOCK_32   (EXT_CLOCK /   32)
#define EXT_CLOCK_64   (EXT_CLOCK /   64)
#define EXT_CLOCK_128  (EXT_CLOCK /  128)
#define EXT_CLOCK_256  (EXT_CLOCK /  256)
#define EXT_CLOCK_1024 (EXT_CLOCK / 1024)

#endif
