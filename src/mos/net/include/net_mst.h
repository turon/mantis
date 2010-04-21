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
/*   File: net_mst.h                                                      */
/* Author: Rafael Salomon                                                 */
/*   Date: 2007-04-06                                                     */
/*                                                                        */
/* net layer plugin: MST protocol                                         */
/**************************************************************************/

#include "mos.h"
#include <stdarg.h>
#include "printf.h"
#include "net.h"
#include "com.h"

#ifndef _NET_MST_H_
#define _NET_MST_H_

/* mst_test_proto_ioctl() options */
#define NET_MST_SET_IS_BASE 0
#define NET_MST_SET_ADDR 1

/* packet types */
#define NET_MST_BEACON_PACKET 1
#define NET_MST_DATA_PACKET 2

/* for making send function call more readable */
#define NET_MST_NEW_PACKET true
#define NET_MST_FORWARD_PACKET false

/* broadcast address */
#define NET_MST_BROADCAST_ADDR 0xFF

/* default port */
#define NET_MST_PORT 40

/* network size */
#define NET_MST_MAX_ADDR 254 // 0xFF (255) is used for broadcast
#define NET_MST_MAX_NETWORK_DIAMETER 20

/* beacon parameters */
#define NET_MST_BEACON_FREQUENCY_MILLIS 8000

/* error codes */
#define NET_MST_ERROR_PACKET_TOO_BIG -10
#define NET_MST_ERROR_INVALID_PACKET_TYPE -11
#define NET_MST_ERROR_INVALID_IOCTL_OPTION -12
#define NET_MST_ERROR_INVALID_ADDR -13

/* meta info for packets */
typedef struct {
	uint8_t packet_type; // types are defined in net_mst.h
	uint8_t packet_seq_no; // packet sequence number.  unique in combination with src_addr.
	uint8_t src_addr; // node that sent packet originally
	uint8_t prev_addr; // last node that forwarded packet
	uint8_t next_addr; // next node supposed to forward packet
	uint8_t dest_addr; // node supposed to receive packet
	uint8_t hop_count; // the number of nodes a packet has travelled through
} net_mst_packet_t;

boolean net_mst_is_base();
uint8_t net_mst_addr();
uint8_t net_mst_hop_count(uint8_t dest_addr);
uint8_t net_mst_next_addr(uint8_t dest_addr);

void net_mst_proto_init();
int8_t net_mst_proto_send(comBuf *pkt, va_list args);
boolean net_mst_proto_recv(comBuf *pkt, uint8_t **footer, uint8_t port);
int8_t net_mst_proto_ioctl(uint8_t request, va_list args);

#endif
