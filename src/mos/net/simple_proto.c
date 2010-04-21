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
/* File:       simple_proto.c                                             */
/* Author:     Carl Hartunge  :  carl.hartung@colorado.edu                */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* Simple protocol example                                                */
/**************************************************************************/

/** @file simple_proto.c
 * @brief Simple protocol example
 * @author Carl Hartung
 * @date Created: 11/12/2004
 */

#ifdef SIMPLE_PROTO
#include <stdlib.h>
#include <string.h>

#include "led.h"
#include "node_id.h"
#include "mutex.h"
#include "simple_proto.h"

void simple_proto_init()
{
  /* must be called by all protocols */
  net_proto_register(SIMPLE_PROTO_ID, simple_proto_send, simple_proto_recv, simple_proto_ioctl);
}

int8_t simple_proto_send(comBuf *pkt, va_list args)
{
   return 0;
}


boolean simple_proto_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
  /* must be called if the protocol wants to try to send the packet to the app */
  return is_app_waiting_on(port);
}

int8_t simple_proto_ioctl(uint8_t request, va_list args)
{
  return 0;
}
#endif 
