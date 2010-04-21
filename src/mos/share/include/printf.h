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

/** @file printf.h
 * @author Adam Torgerson
 * @brief printf (subset) implementation for the node.
 * @date 12/12/2003
 */


/** @addtogroup mos_printf printf
 * This module contains functions that allow a node
 * attached to a PC to easily output data to the
 * mos_shell.
 * #include "printf.h"
 */
//@{

#ifndef PLATFORM_LINUX

#include "plat_dep.h"

void printf_init(void);

#ifdef printf
#undef printf
#endif

// This masks the printf_P function provided by libc, just like our printf
// masks the libc printf.  Change the name to something else if you want the
// old printf_P.
#ifdef printf_P
#undef printf_P
#endif

#ifdef puts
#undef puts
#endif

#ifdef putchar
#undef putchar
#endif

/** @name printf
 * The printf function is similiar to the printf() function used in regular
 * C programming, except that in this case the output is sent over the UART,
 * meaning it is readable on a PC or other connected device.
 *
 * This implementation does not support all formatting and has a few
 * differences from the standard C function:
 *    -# use \%C for 8-bit numbers.  Example output: 255
 *    -# use \%d for 16-bit numbers.  Example output: 65535
 *    -# use \%l for 32-bit numbers.  Example output: 4294967295
 *    -# use \%c for characters.  Example output: a
 *    -# use \%s for strings.  Example output: hello world!
 *    -# use \%b for binary.  Example output: 00110001
 *    -# use \%o for octal.  Example output:  31
 *    -# use \%h for hexadecimal. Example output: 3f
 *    -# use \%x for hexadecimal. Example output: 0x3f
 *    -# padding and minumum width modifiers are supported. Example output: 00012
 *
 * See src/apps/printf/test_printf.c for more examples.
 *
 * Note:  All numeric types are unsigned.  printf does not support
 * negative signed numbers.
 */
//@{

/** @brief print a non-formatted string.
 * @param msg the string to print.
 * @return 0
 */
int puts(const char *msg);
/** @brief print a single character.
 * @param character the character to print.
 * @return 0
 */
int putchar(int character);
/** @brief print a formatted string.
 * @param format the format string specifying what arguments
 * are included and how to display them.
 * @param ... any arguments referenced in the format string.
 * @return 0 on success, -1 if the format string is NULL.
 */
int printf(const char *format, ...);
//@}
//@}
#ifdef ARCH_AVR
int16_t printf_P(const prog_char *format, ...);
#else
int16_t printf_P(const char *format, ...);
#define printf_P(format,args...) printf(format, ## args)
#endif


#else
#include <stdio.h>
int16_t printf_P(const char *format, ...);
#define printf_P(format,args...) printf(format, ## args)
#endif

#define COLOR_NONE "\e[0m"
#define COLOR_BLACK "\e[30m"
#define COLOR_RED "\e[31m"
#define COLOR_GREEN "\e[32m"
#define COLOR_YELLOW "\e[33m"
#define COLOR_BLUE "\e[34m"
#define COLOR_MAGENTA "\e[35m"
#define COLOR_CYAN "\e[36m"
#define COLOR_WHITE "\e[37m"




