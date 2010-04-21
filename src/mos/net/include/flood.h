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

/**************************************************************************/
/* File:       flood.h                                                    */
/* Author:     Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 04/15/04                                                       */
/*                                                                        */
/* Broadcast flooding implemented for the mos net layer.                  */
/**************************************************************************/

/** @file flood.h
 * @brief A broadcast flooding implementation for the mos net layer.
 * @author Jeff Rose
 * @date 04/15/2004
 */

#ifndef _FLOOD_H_
#define _FLOOD_H_

#include "mos.h"

#include "net.h"
#include "com.h"
#include "printf.h"
#include <stdarg.h>

#define FLOOD_PROTO_ID 1

typedef struct flood_pkt_t
{
   uint16_t src;
   uint16_t dest;
   uint8_t seq_num;
} flood_pkt;

/** @brief Setup the flooding protocol and register with the net layer. */
void flood_init();

/** @brief Send a packet using the flooding protocol. 
 * @param pkt Packet to send
 * @param args Arguements
*/
int8_t flood_send(comBuf * pkt, va_list args);

/** @brief Receive a packet that came using the flooding protocol. 
 *
 * NOTE: Must subtract the sizeof route header from pkt->size so
 *       the net layer can determine whether any more data is available.
 * @param pkt Packet received
 * @param footer Location
 * @return FALSE if failure, else return TRUE
 */
boolean flood_recv(comBuf *pkt, uint8_t **footer, uint8_t port);

/* Set some protocol specific parameters. */
/** @brief Set some protocol specific parameters. 
 * @param request IO request
 * @param args Arguements
 */
int8_t flood_ioctl(uint8_t request, va_list args);

#endif
