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
   File:   serial.c
   Authors: Jeff Rose
   Date:   02-01-04
  
   Mac layer built on top of udp datagram sockets.  Just broadcasts messages
   to a well known port where the receiver would be.  This only supports a
   single receiver right now...
*/

#include "mos.h"

#ifdef PLATFORM_LINUX

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "msched.h"
#include "node_id.h"
#include "udp.h"

#if defined(UDP) || !defined(SCONS)

#define SERVER_PORT 1521
#define RADIO_PACKET_EVENT 3

static comBuf *packetBuf;

uint8_t current_mode;
int send_fd;

void listenTo (void);

uint8_t udp_init (void)
{
   if((send_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      perror("socket");
      exit(1);
   }

   // Have to set broadcast permissions or we get permission denied
   //setsockopt(send_fd, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
   
   current_mode = IF_OFF;
   
   // get initial buffer
   com_swap_bufs(IFACE_RADIO, (comBuf *)NULL, &packetBuf);
   packetBuf->size = 0;

   /* Create a thread to constantly listen on the socket and
      save packets. */
   mos_thread_new(listenTo, 128, PRIORITY_NORMAL);
   
   return 0;
}

uint8_t com_send_IFACE_UDP (comBuf *buf)
{
   char *pkt;
   int32_t event_id, event_size, event_src;
   int numbytes;
   struct sockaddr_in their_addr; // connector's address information
   
   their_addr.sin_family = AF_INET;     // host byte order
   their_addr.sin_port = htons(SERVER_PORT); // short, network byte order
   inet_aton("127.0.0.1", &(their_addr.sin_addr));
   memset(&(their_addr.sin_zero), '\0', 8); // zero the rest of the struct
   
   // Allocate a full packet for the event
   pkt = malloc(buf->size + 12);

   event_id = (int32_t)RADIO_PACKET_EVENT;
   event_size = (int32_t)buf->size+4;
   event_src = (int32_t)mos_node_id_get();
   
   // Copy our event header and the comBuf into the allocated packet
   memcpy(pkt, &event_size, 4);
   memcpy(pkt+4, &event_id, 4);
   memcpy(pkt+8, &event_src, 4);   
   memcpy(pkt+12, &buf->data, buf->size);

   if ((numbytes = sendto(send_fd, pkt, (buf->size+12), 0,
			  (struct sockaddr *)&their_addr,
			  sizeof(struct sockaddr))) == -1) {
      perror("sendto");
      exit(1);
   }

   free(pkt);
   return 0;
}

void com_mode_IFACE_UDP (uint8_t mode)
{
   current_mode = mode;
}


void com_ioctl_IFACE_UDP (uint8_t request, ...)
{
}

/** @brief See complete description. 
 *  
 * This function is meant to be run as a separate thread that is
 * constantly reading bytes off a udp socket.  In order to make
 * the behavior the same in xmos and amos I am dropping packets
 * unless the user has explicitly turned on the udp device. */
void listenTo (void)
{
   int sockfd;
   struct sockaddr_in my_addr;    // my address information
   struct sockaddr_in their_addr; // connector's address information
   int addr_len;
   char *pkt;
   
   comBuf *buf = NULL;

   pkt = malloc(COM_DATA_SIZE+12);
   
   if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      perror("socket");
      exit(1);
   }

   my_addr.sin_family = AF_INET;         // host byte order
   my_addr.sin_port = htons(mos_node_id_get());     // short, network byte order
   my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
   memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct

   addr_len = sizeof(struct sockaddr);  // Need this to recv

   if(bind(sockfd, (struct sockaddr *)&my_addr,
	   sizeof(struct sockaddr)) == -1) {
      perror("bind");
      exit(1);
   }

   while(1) {
      // We can't do anything without a comBuf so just hang out until
      // we get one
      if(current_mode == IF_LISTEN) {
	 while(1) {
	    com_swap_bufs(IFACE_RADIO, buf, &packetBuf);
	    if(packetBuf == NULL)
	       usleep(250);
	    else
	       break;
	 }
      } else {
	 usleep(100);
	 continue;
      }
      
      // TODO: This might not be a safe way to do things if the packets
      // don't come in a single recvfrom call...  Look into it.
      int16_t ret;
      if((ret = recvfrom(sockfd, pkt, COM_DATA_SIZE+12,
			       0,(struct sockaddr *)&their_addr,
			       &addr_len)) == -1) {
	 perror("recvfrom");
	 exit(1);
      }

      buf->size = ret - 12;
      memcpy(&buf->data, pkt+12, buf->size);
   }
   close(sockfd);
}

#endif
#endif
