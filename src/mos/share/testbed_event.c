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

#include "testbed_event.h"
#include "com.h"

#ifdef PLATFORM_TELOSB
#define com_send_stdout(packet_ptr) com_send(IFACE_SERIAL2, packet_ptr)
#else
#define com_send_stdout(packet_ptr) com_send(IFACE_SERIAL, packet_ptr)
#endif


//static comBuf event_buf;
comBuf event_buf;
static mos_mutex_t tb_mux;

void testbed_init()
{
   mos_mutex_init(&tb_mux);
}

void send_testbed_event(uint8_t event_type,
		uint8_t event_id,
		uint8_t *event_data,
		uint8_t event_size)
{
   mos_mutex_lock(&tb_mux);
   event_buf.data[0] = 0;
   event_buf.data[1] = 0;

   event_buf.data[2] = event_type;
   event_buf.data[3] = event_id;

   uint8_t *s = event_data;
   uint8_t *d = &(event_buf.data[4]);
   uint8_t count;

   
   for(count = 0; count < event_size; count++)
      *(d++)=*(s++);

   event_buf.size = event_size + 4;
   com_send_stdout(&event_buf);
   mos_mutex_unlock(&tb_mux);
}


void send_net_id(uint16_t id)
{
   mos_mutex_lock(&tb_mux);
   event_buf.data[0] = 0;
   event_buf.data[1] = 0;
   event_buf.data[2] = EVENT_NETWORK;
   event_buf.data[3] = NETWORK_ID;
   event_buf.data[4] = (uint8_t)(0xFF & id);
   event_buf.data[5] = (uint8_t)(id >> 8);
   event_buf.size = 6;
   com_send_stdout(&event_buf);
   mos_mutex_unlock(&tb_mux);
}
