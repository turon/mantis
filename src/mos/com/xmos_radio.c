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
  Project Mantis
  File:   xmos_radio.c
  Authors: Jeff Rose, Lane Phillips
  Date:   2004-08-04
  
  Simulate the radio by sending packets to a simulation server.
*/

#include "mos.h"

#ifdef PLATFORM_LINUX

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "node_id.h"
#include "msched.h"
#include "xmos_radio.h"
#include "gevent.h"

#if defined(XMOS_RADIO) || !defined(SCONS)


//#define HEXDUMP(BUF,SIZE)	{int hdi; for (hdi=0; hdi<SIZE; hdi++) printf("%02hhx ", BUF[hdi]); printf("\n");}

static uint8_t current_mode;

static void listenTo (void);

/* Initialize the XMOS radio.  Called by preStart() in src/mos/sys/main.c */
void xmos_radio_init (void)
{
   current_mode = IF_OFF;
   
   // Don't even bother launching the listen thread if we are not
   // connected to the simulation server.
   if (!gevent_connected) return;
   
   // Create a thread to constantly listen on the socket and
   // save packets.
   mos_thread_new(listenTo, 128, PRIORITY_NORMAL);
   
   return;
}

/* Send the packet to the server as a simulator event. */
uint8_t com_send_IFACE_RADIO (comBuf *buf)
{
   return (uint8_t)gevent_send(RADIO_PACKET_EVENT, "ix",
			       "Send packet: node %i, %i bytes.\n",
			       mos_node_id_get(), buf->size, buf->data);
}

void com_mode_IFACE_RADIO (uint8_t mode)
{
   current_mode = mode;
}


void com_ioctl_IFACE_RADIO (uint8_t request, ...)
{
}

/** @brief See complete description. 
 *  
 * This function is meant to be run as a separate thread that is
 * constantly reading bytes off a udp socket.  In order to make
 * the behavior the same in xmos and amos I am dropping packets
 * unless the user has explicitly turned on the radio device. */
static void listenTo (void)
{
   static char* pkt[COM_DATA_SIZE];
   comBuf *buf = NULL;
   int size;
   uint32_t id;

   while(gevent_connected) {
      /* Receive a radio packet as a simulation event. */
      size = gevent_recv(RADIO_PACKET_EVENT, "ix", &id, pkt);
      if (size < 0) {
	 fprintf(stderr, "gevent_recv returned %i.\n", size);
	 return;
      }

      /* Drop packets if the radio is not listening. */
      if (current_mode != IF_LISTEN) continue;
      
      if (!buf) {
	 com_swap_bufs(IFACE_RADIO, NULL, &buf);
	 /* Drop packets if we can't get a buffer. */
	 if (!buf) continue;
      }
   	
      buf->size = size - 4;
      memcpy(buf->data, pkt, buf->size);
		
      /* Make the buffer available for com_recv(). */
      com_swap_bufs(IFACE_RADIO, buf, &buf);
   }
}

#endif
#endif
