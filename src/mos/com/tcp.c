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
  
   Mac layer built on top of tcp sockets.  This driver is specifically
   created for use with the mos_net addressing scheme.
*/

/** @file tcp.c
 * @brief Mac layer built on top of tcp sockets.
 *
 * This driver is specifically created for use with the mos)net addressing scheme.
 * @author Jeff Rose
 * @date 02/01/2004
 */

#ifdef PLATFORM_LINUX

#include "mos.h"
#include "com.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <glib.h>

#include "msched.h"
#include "udp.h"

#if defined(TCP) || !defined(SCONS)

#define BACKLOG 4096

typedef struct neighbor_t
{
   uint16_t maddr;          // neighbor's mos address
   struct sockaddr_in addr; // neighbor's tcp address information
   int sockfd;
}neighbor;

static comBuf *packetBuf;

uint8_t current_mode;
int send_fd;
GHashTable *neighbor_table;

void listenTo();

uint8_t tcp_init()
{
   /* Setup the hash table of neighbors. */
   neighbor_table = g_hash_table_new(g_int_hash, g_int_equal);
   
   // register with com layer and set the default iface_mode to off
   com_register_iface(IFACE_TCP, udp_send,
		      udp_mode, udp_ioctl);
   current_mode = IF_OFF;
   
   // get initial buffer
   packetBuf = com_swap_bufs(IFACE_TCP, (comBuf *)NULL);
   packetBuf->size = 0;
 
   return 0;
}

uint8_t tcp_send(comBuf *buf)
{

   return 0;
}

uint8_t tcp_mode(uint8_t mode)
{
   current_mode = mode;

   switch(mode) {
   case IF_OFF:
   case IF_STANDBY:
   case IF_IDLE:
      break;
   case IF_LISTEN:
      /* Create a thread to constantly listen on the socket and
	 save packets. */
      mos_thread_new(listenTo, 128, PRIORITY_NORMAL);
      break;
   default:
      break;
   }
   
   return 0;
}

/* Generic control function for doing things like changing the
   properties of the connection etc... */
uint8_t tcp_ioctl(uint8_t request, va_list ap)
{
   switch(request)
   {
   }

   return 0;
}

/* This function is meant to be run as a separate thread that is
   constantly reading bytes off a udp socket.  In order to make
   the behavior the same in xmos and amos I am dropping packets
   unless the user has explicitly turned on the udp device. */
void listenTo()
{
   int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
   struct sockaddr_in my_addr;    // my address information
   struct sockaddr_in their_addr; // connector's address information
   int sin_size;
   int yes=1;
   
   if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("socket");
      exit(1);
   }
   
   if(setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(int)) == -1) {
      perror("setsockopt");
      exit(1);
   }
        
   my_addr.sin_family = AF_INET;         // host byte order
   my_addr.sin_port = htons(MANTIS_PORT);     // short, network byte order
   my_addr.sin_addr.s_addr = INADDR_ANY; // automatically fill with my IP
   memset(&(my_addr.sin_zero), '\0', 8); // zero the rest of the struct
   
   if(bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr))
      == -1) {
      perror("bind");
      exit(1);
   }
   
   if(listen(sockfd, BACKLOG) == -1) {
      perror("listen");
      exit(1);
   }
   
   while(1) {
      struct timeval tv;
      fd_set readfds, exceptfds;

      tv.tv_sec = 2;
      tv.tv_usec = 500000;

      FD_ZERO(&readfds);
      FD_ZERO(&exceptfds);

      FD_SET(STDIN_FILENO, &readfds);
      
      /* Run select on all current sockets and the listening socket. */
      select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
      
      if (FD_ISSET(STDIN_FILENO, &readfds))
	 printf("A key was pressed!\n");
      else
	 printf("Timed out.\n");
      

      


      
      sin_size = sizeof(struct sockaddr_in);
      if ((new_fd = accept(sockfd, (struct sockaddr *)&their_addr,
			   &sin_size)) == -1) {
	 perror("accept");
	 continue;
      }
      printf("Server: got connection from %s\n", inet_ntoa(their_addr.sin_addr));
      
      if (send(new_fd, "Hello, world!\n", 14, 0) == -1)
	 perror("send");  
   }
}


/* Functions private to this file. */
void fill_read_set(gpointer set)
{
   //fd_set *rset = (fd_set *)set;

   
}

#endif
#endif
