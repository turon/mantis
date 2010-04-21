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

#ifndef __TESTBED_EVENT_H__
#define __TESTBED_EVENT_H__

#include "mos.h"

#define EVENT_SYSTEM 1
#define EVENT_NETWORK 2
#define EVENT_SENSE 3
#define EVENT_APP 4

//system events
#define KEEP_ALIVE 1

// network events
#define NETWORK_ID 1 
#define PACKET_SENT 2 
#define PACKET_RECV 3 
#define NEW_PARENT 4


// sense events
#define TEMPERATURE 1
#define LIGHT 2
#define HUMIDITY 3


void testbed_init();

void send_testbed_event(uint8_t event_type,
			uint8_t event_id,
			uint8_t *event_data,
			uint8_t event_size);

void send_net_id(uint16_t id);

typedef struct {
   uint16_t src;
   uint8_t size;
} recv_pkt_event_t;

typedef struct {
   uint16_t dest;
   uint8_t size;
} send_pkt_event_t;

#endif
