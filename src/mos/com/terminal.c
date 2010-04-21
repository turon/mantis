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
  File:   terminal.c
  Authors: Charles Gruenwald III
  Date:   03-12-04
  
  terminal driver for stdin and stdout - xmos processes
*/

#include "mos.h"

#ifdef PLATFORM_LINUX

#include <string.h>

#include "stdio.h"
#include "com.h"
#include "msched.h"
#include "terminal.h"
#include "unistd.h"
#include "mutex.h"
#include "mcs.h"

#if defined(TERMINAL) || !defined(SCONS)

static comBuf *packetBuf;

//extern mutex serial_mutex;

void term_listen(void);

uint8_t terminal_init(void)
{
   // get initial buffer
   com_swap_bufs(IFACE_TERMINAL, (comBuf *)NULL, &packetBuf);
   packetBuf->size = 0;

   /* Create a thread to constantly listen on the serial port and
      save packets if we are in listen mode. */
   mos_thread_new(term_listen, 128, PRIORITY_NORMAL);
   
   return 0;
}

uint8_t com_send_IFACE_TERMINAL (comBuf *buf)
{
   //   puts(buf->data);
   switch (buf->data[0]) {
   case WORD_PRINT: {
      printf ("special number packet: %d\n", *((uint16_t *)&buf->data[1]));
      fflush (stdout);
      return 0;
   }
   case LOADER_PRESENT: {
      comBuf pkt;
      printf ("Sending shell present from terminal\n");
      pkt.size = 1;
      pkt.data[0] = START;
      com_send (IFACE_SERIAL, &pkt);
   }
   }

   //mos_mutex_lock (&serial_mutex);
   printf("%s",buf->data);
   fflush (stdout);
   //mos_mutex_unlock (&serial_mutex);
   return 0;
}

void com_mode_IFACE_TERMINAL (uint8_t mode)
{
}

void com_ioctl_IFACE_TERMINAL (uint8_t request, ...)
{
}

/** @brief See full description.
 *
 *  This function is meant to be run as a separate thread that is
 * constantly reading bytes off the serial port.  In order to make
 * the behavior the same in xmos and amos I am dropping packets
 * unless the user has explicitly turned on the serial device. 
 */
void term_listen(void)
{
   char buf[COM_DATA_SIZE];
   
   while(1) {
      if(packetBuf != NULL) {
	 fgets(buf, sizeof (buf) - 1, stdin);
	 int size = strlen (buf) + 1;

	 memcpy (packetBuf->data, buf, size);
	 packetBuf->size = strlen(packetBuf->data);
	 
	 com_swap_bufs(IFACE_TERMINAL, packetBuf, &packetBuf);
	 
	 while(packetBuf == NULL) {
	    sleep(1);
	    printf("com layer ran out of packets in terminal!\n");
	    com_swap_bufs(IFACE_TERMINAL, packetBuf, &packetBuf);
	 }
      }
      else
	 com_swap_bufs(IFACE_TERMINAL, NULL, &packetBuf);
   }
}

#endif
#endif
