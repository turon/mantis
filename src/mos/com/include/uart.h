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

/** @file com/include/uart.h
 * @brief Uart layer built for the com stack.
 *
 * @author Jeff Rose
 * @date 03/01/2004
 */

#ifndef _UART_H_
#define _UART_H_

#ifndef SCONS
#include "mos.h"
#endif

#ifndef PLATFORM_LINUX
 
/** @brief Define the ioctl request flags. */
#define PARITY    1

#define UART_IOCTL_BAUD_RATE 0
#define UART_IOCTL_COM_MODE0 1
#define UART_IOCTL_RAW_MODE0 2
#define UART_IOCTL_COM_MODE1 3
#define UART_IOCTL_RAW_MODE1 4

#define UART0 0
#define UART1 1

/** @brief These are the possible baud rate arguments. */
#ifdef CLOCK_SPEED_1_0
#define B2400    25
#define B4800    12
#define B9600    6
#define B14400   3
#define B19200   2
#define B28800   1
#define B38400   1
#define B57600   0
#define B76800   0
#define B115200  0
#endif

#ifdef CLOCK_SPEED_1_84
#define B2400    47
#define B4800    23
#define B9600    11
#define B14400   7
#define B19200   5
#define B28800   3
#define B38400   2
#define B57600   1
#define B76800   1
#define B115200  0
#endif

#ifdef CLOCK_SPEED_2_0
#define B2400    51
#define B4800    25
#define B9600    12
#define B14400   8
#define B19200   6
#define B28800   3
#define B38400   2
#define B57600   1
#define B76800   1
#define B115200  0
#endif

#ifdef CLOCK_SPEED_3_68
#define B2400    95
#define B4800    47
#define B9600    23
#define B14400   15
#define B19200   11
#define B28800   7
#define B38400   5
#define B57600   3
#define B76800   2
#define B115200  1
#endif

// U2X = 1 for these values
#ifdef CLOCK_SPEED_4_0
#define B2400    207
#define B4800    103
#define B9600    51
#define B14400   34
#define B19200   25
#define B28800   16
#define B38400   12
#define B57600   8
#define B76800   6
#define B115200  3
#endif

#ifdef CLOCK_SPEED_7_37
#define B2400    191
#define B4800    95
#define B9600    47
#define B14400   31
#define B19200   23
#define B28800   15
#define B38400   11
#define B57600   7
#define B76800   5
#define B115200  3
#define B230400  1
#endif

#ifdef CLOCK_SPEED_8_0
#define B2400    207
#define B4800    103
#define B9600    51
#define B14400   34
#define B19200   25
#define B28800   16
#define B38400   12
#define B57600   8
#define B76800   6
#define B115200  3
#define B230400  1
#define B250000  1
#endif

#ifdef CLOCK_SPEED_11_05
#define B2400    287
#define B4800    143
#define B9600    71
#define B14400   47
#define B19200   35
#define B28800   23
#define B38400   17
#define B57600   11
#define B76800   8
#define B115200  5
#define B230400  2
#define B250000  2
#endif

#ifdef CLOCK_SPEED_14_74
#define B2400    383
#define B4800    191
#define B9600    95
#define B14400   63
#define B19200   47
#define B28800   31
#define B38400   23
#define B57600   15
#define B76800   11
#define B115200  7
#define B230400  3
#define B250000  3
#define B500000  1
#endif

#ifdef CLOCK_SPEED_16_0
#define B2400    416
#define B4800    207
#define B9600    103
#define B14400   68
#define B19200   51
#define B28800   34
#define B38400   25
#define B57600   16
#define B76800   12
#define B115200  8
#define B230400  3
#define B250000  3
#define B500000  1
#endif

#ifdef CLOCK_SPEED_18_43
#define B2400    479
#define B4800    239
#define B9600    119
#define B14400   79
#define B19200   59
#define B28800   39
#define B38400   29
#define B57600   19
#define B76800   14
#define B115200  9
#define B230400  4
#define B250000  4
#endif

#ifdef CLOCK_SPEED_20_0
#define B2400    520
#define B4800    259
#define B9600    129
#define B14400   86
#define B19200   64
#define B28800   42
#define B38400   32
#define B57600   21
#define B76800   15
#define B115200  10
#define B230400  4
#define B250000  4
#endif

#else
  #error "Clock Speed Undefined"
#endif

//#define NEED_RAW_MODE

/** @brief Init the Uart. */
void uart_init(void);

#ifdef NEED_RAW_MODE

#define uart_read(uart_num, buf, count) uart_read##uart_num(buf, count)
#define uart_write(uart_num, buf, count) uart_write##uart_num(buf, count)
uint8_t uart_read0(uint8_t *buf, uint8_t count);
uint8_t uart_write0(uint8_t *buf, uint8_t count);
uint8_t uart_read1(uint8_t *buf, uint8_t count);
uint8_t uart_write1(uint8_t *buf, uint8_t count);

#endif

#if defined(USB_SERIAL)
#define DEFAULT_BAUD_RATE B115200
#elif defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ) || defined(PLATFORM_MICROBLAZE)
#define DEFAULT_BAUD_RATE B57600
#elif defined(PLATFORM_TELOSB)
#define DEFAULT_BAUD_RATE 57600
#elif defined(PLATFORM_MICA2DOT) || defined(PLATFORM_AVRDEV)
#define DEFAULT_BAUD_RATE B38400
#else
#error "Undefined baud rate for this platform"
#endif


#endif
