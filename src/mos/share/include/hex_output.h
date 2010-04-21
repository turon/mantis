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

#ifndef __HEX_OUTPUT_H__
#define __HEX_OUTPUT_H__
#include "mos.h"
#include "printf.h"

/** @addtogroup mos_printf printf
//@{

/** @name hex printing
* These functions allow you to print data that might contain non-printable
* characters.  The output is in the standard hex output format, with
* hex representation on the left, and characters to the right.
* #include "hex_output.h"
*/
//@{
/** @brief print a hexdump of buffer.
 * This function prints 16 characters per line.
 * @param buffer a pointer to an array of bytes to print.
 * @param size the number of bytes to print.
 */
#define print_hexdump(buffer, size) print_hexdump_ex(buffer, size, 16)

/** @brief print a hexdump of a buffer.
 * @param buffer a pointer to an array of bytes to print.
 * @param size the number of bytes to print.
 * @param line_length the number of characters to print on each
 * line.
 */
void print_hexdump_ex(const char* buffer, uint16_t size, uint16_t line_length);
//@}
//@}

#endif
