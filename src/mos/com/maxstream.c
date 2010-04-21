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


#include "mos.h"
#include "com.h"

#ifdef PLATFORM_AVRDEV

#include "maxstream.h"
#include "uart.h"


#if defined(MAXSTREAM) | !defined(SCONS)

/** @brief Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Returns 0 on successful transmission
 */
uint8_t com_send_IFACE_RADIO(comBuf* buf)
{
   return com_send(IFACE_SERIAL2, buf);
}

/** @brief Sets mode for this driver.
 * @param md Mode
 * @return New mode set
*/
void com_mode_IFACE_RADIO (uint8_t md)
{
   return;
}



 
void maxstream_init()
{
   com_mode(IFACE_SERIAL2, IF_LISTEN);
   com_ioctl(IFACE_SERIAL2, UART_IOCTL_BAUD_RATE, B9600);
}

/** @brief Generic io control for this driver.
 * @param request i/o request
 * @param ap Arguements
 * @return Always returns 0
*/
void com_ioctl_IFACE_RADIO (uint8_t request, ...)
{
   int arg;
   va_list ap;
   va_start (ap, request);

   switch (request) {
   default:
      arg = va_arg (ap, int);
      break;
   }

   va_end (ap);
}

#endif
#endif
