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
#include <ctype.h>
#include "mos.h"
#include "printf.h"
#include "hex_output.h"

/** @file hex_output.c
 * @author John Ledbetter
 *
 * This file implements a function to print
 * a standard hex display of an array of data.
 */

void print_hexdump_ex(const char* buffer, uint16_t length, uint16_t line_length)
{
   
   int i, j;
   int lines = length / line_length;
   int leftover = length % line_length;
   const char* p_cur = buffer;
   const char* p_ln = buffer;
   
   for(i = 0; i < lines; ++i)
   {
      for(j = 0; j < line_length; ++j)
      {
	 printf("%02h ", (int)(*p_cur) & 0xFF);
	 ++p_cur;
      }
      p_cur = p_ln;

      printf("\t");
      for(j = 0; j < line_length; ++j)
      {
	 printf("%c", isprint(*p_cur) && !isspace(*p_cur) ? *p_cur : '.');
	 
	 ++p_cur;
      }
      printf("\n");
      p_ln += line_length;
      
   }

   for(i = 0; i < leftover; ++i)
   {
      printf("%02h ", (int)*p_cur & 0xFF);
      ++p_cur;
   }
   
   for(i = 0; i < line_length - leftover; ++i)
      printf("   ");

   printf("\t");
   
   for(i = 0; i < leftover; ++i)
   {
      printf("%c", isprint(*p_ln) && !isspace(*p_ln) ? *p_ln : '.');
    
      ++p_ln;
   }

   printf("\n");
   
}
