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
/* File:       mst_proto.h                                                */
/* Author:     Carl Hartunge  :  carl.hartung@colorado.edu                */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* MST protocol example                                                   */
/**************************************************************************/

/** @file mst.h
 * @brief MST protocol example
 * @author Carl Hartung
 * @date Created: 11/12/2004
 */

#include "mos.h"

#include <stdarg.h>
#include "printf.h"

#include "net.h"
#include "com.h"

#ifndef _MST_H_
#define _MST_H_

//Ioctl macros

#define SET_ADDR 1
#define SET_DTB 2
#define SET_DEST 3

#define MST_CONTROL 1
#define MST_DATA 2

#define BASE_STATION_ADDRESS 0
#define BCAST_ADDRESS 0xFF


//#define TESTBED_DIAGNOSTICS

#define MST_USE_CRC

typedef struct {
   uint8_t sender_dtb;
   uint8_t last_hop;
   uint8_t src;
   uint8_t dest;
   uint8_t type;
   uint8_t seqno;
   uint8_t pkt_dtb;
   uint8_t ttl;
   uint8_t next_hop;
#if defined(MST_USE_CRC)
   uint8_t crc_l;
   uint8_t crc_h;
#endif
} mst_packet_t;

uint8_t mst_dtb();
uint8_t mst_parent();

void mst_proto_init();
int8_t mst_proto_send(comBuf *pkt, va_list args);
boolean mst_proto_recv(comBuf *pkt, uint8_t **footer, uint8_t port);
int8_t mst_proto_ioctl(uint8_t request, va_list args);


#endif
