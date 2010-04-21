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

/// Event 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include "event.h"

#define PORT 1629
#define MAXDATASIZE 256

int sockfd;  

int event_init() {

   struct hostent *he;
   struct sockaddr_in server_addr; // connector's address information 

   if ((he=gethostbyname("127.0.0.1")) == NULL) {  // get the host info 
      perror("event_init: gethostbyname");
      exit(1);
   }

   if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("event_init: socket");
      exit(1);
   }

   server_addr.sin_family = AF_INET;    // host byte order 
   server_addr.sin_port = htons(PORT);  // short, network byte order 
   server_addr.sin_addr = *((struct in_addr *)he->h_addr);
   memset(&(server_addr.sin_zero), '\0', 8);  // zero the rest of the struct 

   if (connect(sockfd, (struct sockaddr *)&server_addr,
	       sizeof(struct sockaddr)) == -1) {
      perror("event_init: connect");
      exit(1);
   }

   return sockfd;
}

void SendEvent(int sock, Event *ev)
{
   send(sock, &ev->eid, sizeof(uint32_t), 0);
}

uint8_t mos_event_send(uint8_t *data, uint8_t size) {
   printf("Byte[0]: %x\nByte[1]: %x\n", data[0], data[1]);
   printf("Size: %d\n", size);
   if(send(sockfd, data, size, 0) == -1)
      perror("event_send: send");

   return 0;
}
