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
  A module for 16 bit CRC-CCITT 
*/

#include "crc.h"

/** @brief crc-ccitt truncated polynomial. */
#define POLY 0x1021          
/** @brief crc-ccitt initial value. */
#define INITIAL_VALUE 0xFFFF

uint16_t crc_compute(uint8_t *buf, uint16_t size)
{
   uint16_t index, crc;
   uint8_t v, xor_flag, byte, bit;
   
   crc = INITIAL_VALUE;

   for(index = 0; index < size; index++) {
      byte = buf[index];
      /*
	Align test bit with leftmost bit of the message byte.
      */
      v = 0x80;

      for(bit = 0; bit < 8; bit++) {
	 if(crc & 0x8000)
            xor_flag= 1;
	 else
            xor_flag= 0;

	 crc = crc << 1;
            
	 /*  Append next bit of message to end of CRC if it is not zero.
	     The zero bit placed there by the shift above need not be
	     changed if the next bit of the message is zero. */
	 if(byte & v)
	    crc= crc + 1;

	 if(xor_flag)
	    crc = crc ^ POLY;

	 /* Align test bit with next bit of the message byte. */
	 v = v >> 1;
      }
   }

   /* We have to augment the crc in order to comply with the ccitt spec. */
    for(bit = 0; bit < 16; bit++) {
        if(crc & 0x8000)
            xor_flag= 1;
        else
            xor_flag= 0;

	crc = crc << 1;

        if(xor_flag)
            crc = crc ^ POLY;
    }

    return crc;
}

