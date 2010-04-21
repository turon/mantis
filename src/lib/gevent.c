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

/** @file gevent.c
 * @author Lane Phillips	Lane.Phillips@colorado.ed
 * @date 2004/09/29
 * @brief Generic events for XMOS simulator.
 */
 
#include "mos.h"

/* This stuff only works in XMOS. */
#ifdef PLATFORM_LINUX

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include "sem.h"
#include "mutex.h"
#include "arg.h"
#include "node_id.h"
#include "msched.h"
#include "gevent.h"
#include <errno.h>
#include <unistd.h>

typedef struct gevent
{
	struct gevent* next;	// next event in queue
	uint32_t type;
	uint32_t size;	// size of buffer (inlcluding first byte)
	uint8_t buf[1];	// first byte of buffer
} gevent;

/* Global flag to indicate that we are connected to the simulation server. */
int gevent_connected = 0;

/* Semaphors to indicate an incoming event is available. */
static mos_sem_t sems[MAX_EVENT_TYPE+1];
/* Mutexes for modifying the event queues. */
static mos_mutex_t locks[MAX_EVENT_TYPE+1];

/* Queue of incoming events, one for each type. */
// TODO: Some of the event types are not used by XMOS.  Maybe we should
// separate them into contiguous blocks, then we can make queues just for the 
// events in one block and save space.
static gevent* heads[MAX_EVENT_TYPE+1];
static gevent* tails[MAX_EVENT_TYPE+1];

/* Datagram sockets for communicating with the server. */
static int ssock;
static int rsock;

static void recvThread();
static void dumpEvent(FILE* stream, gevent* event, const char* format);

/* Set up connection to the simulation server and start the receive thread. */
int gevent_init()
{
	gevent_connected = 0;
	if(!mos_arg_check("-viz", NULL)) {
		return 0;
	}

	/* Check command line options or use defaults. */
	uint8_t* address = GEVENT_ADDRESS;
	mos_arg_check("-vizaddress", &address);

	uint8_t* port;
	int sport = GEVENT_SEND_PORT;
	if(mos_arg_check("-vizsendport", &port)) {
		sport = atoi(port);
	}
	
	/* Default receive port is the node's ID. */
	int rport = mos_node_id_get();
	if(mos_arg_check("-vizrecvport", &port)) {
		rport = atoi(port);
	}
	
	/* Create a datagram socket for sending events to the server. */
	if((ssock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("gevent_init: socket");
		return -1;
	}

	/* Get the simulation server's address. */
	struct sockaddr_in addr;				// server's address information
	addr.sin_family = AF_INET;				// host byte order
	addr.sin_port = htons(sport);			// short, network byte order
	struct hostent* entry = gethostbyname(address);
	if (!entry) {
		perror("gevent_init: gethostbyname");
		return -1;
	}
	memcpy(&(addr.sin_addr), entry->h_addr_list[0], entry->h_length);
	memset(&(addr.sin_zero), '\0', 8); 		// zero the rest of the struct

	/* We will always be sending to the same place, so connect the socket. */
	if (connect(ssock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in))) {
		perror("gevent_init: connect");
		return -1;
	}
	
	/* Create a datagram socket for receiving events from the server. */
	if((rsock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("gevent_init: socket (2nd call)");
		return -1;
	}
	
	/* Bind the socket to a port that is specific to this XMOS node. */
	addr.sin_family = AF_INET;				// host byte order
	addr.sin_port = htons(rport);			// short, network byte order
	addr.sin_addr.s_addr = INADDR_ANY;
	memset(&(addr.sin_zero), '\0', 8); 		// zero the rest of the struct
	
	if (bind(rsock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in))) {
		perror("gevent_init: bind");
		return -1;
	}
	
	/* Initialize event queues. */
	int i;
	for (i=0; i<MAX_EVENT_TYPE; i++)
	{
		mos_sem_init(&sems[i], 0);
		mos_mutex_init(&locks[i]);
		heads[i] = NULL;
		tails[i] = NULL;
	}

	/* Set this flag once everything else is successful. */
	gevent_connected = 1;
	printf("Viz on!\n");
	
	int32_t x = 50;
	int32_t y = 50;
	char* arg;
	if (mos_arg_check("-x", &arg)) {
		x = atol(arg);
	}
	if (mos_arg_check("-y", &arg)) {
		y = atol(arg);
	}
	arg = (mos_arg_argv())[0];	// file name of node executable
	
	/* Notify server that there is a new node running. */
	gevent_send(NEW_NODE_EVENT, "iiix", "%i NEW NODE x: %i, y: %i\n",
		mos_node_id_get(), x, y, strlen(arg), arg);
	
	/* Start listening for incoming events. */
	mos_thread_new(recvThread, 128, PRIORITY_NORMAL);
	
	return 0;
}

/* Format and send an event to the simulation server. */
int gevent_send(uint32_t type, const char* format, const char* message, ...)
{
	static uint8_t buf[MAX_EVENT_SIZE];
	va_list args;
	if (gevent_connected)
	{
		const char* f = format;
		uint32_t size = 8;	/* Leave room for 8-byte header. */
		va_start(args, message);
		while (*f)
		{
			/* Parse repeat count. */
			int repeat = 0;
			while (isdigit(*f))
				repeat = 10*repeat + *f++ - '0';
			if (repeat==0) repeat = 1;
			
			if (*f == 'i') {
				for (; repeat>0; repeat--) {
					/* For portability ints are sent in network byte order. */
					*(int32_t*)&buf[size] = htonl(va_arg(args, int32_t));
					size += 4;
				}
			} else if (*f == 's') {
				memcpy(&buf[size], va_arg(args, char*), repeat);
				size += repeat;
			} else if (*f == 'x') {
				repeat = va_arg(args, int);
				memcpy(&buf[size], va_arg(args, char*), repeat);
				size += repeat;
			}
			f++;
		}
		va_end(args);
		
		/* Put the event size (not including header) and type in the header. */
		/* Use network byte order for all ints. */
		*(uint32_t*)&buf[0] = htonl(size-8);
		*(uint32_t*)&buf[4] = htonl(type);

		if (send(ssock, buf, size, 0) == -1) {
			perror("gevent_send: send");
			return -1;
		}
	} else {
		va_start(args, message);
		/* We are not connected to the server, print a message to the console. */
		vprintf(message, args);
		va_end(args);
	}
	return 0;
}

/* Receive and parse an event from the simulation server.
 * This function blocks until an event is available. */
int gevent_recv(uint32_t type, const char* format, ...)
{
	if (!gevent_connected) return 0;
	if (type>MAX_EVENT_TYPE) return -1;
	va_list args;
	
	/* Wait for the next event. */
	mos_sem_wait(&sems[type]);
	/* Remove it from the queue. */
	mos_mutex_lock(&locks[type]);
	gevent* event = tails[type];
	if (!event) {
		/* I'm pretty sure it should not get here, because of the semaphor. */
		mos_mutex_unlock(&locks[type]);
		fprintf(stderr, "Impossible error: %s, line %i\n", __FILE__, __LINE__);
		return -1;
	}
		
	tails[type] = event->next;
	if (tails[type] == NULL)
		/* We just removed the last event in the queue. */
		heads[type] = NULL;
	mos_mutex_unlock(&locks[type]);

	const char* f = format;
	int size = 0;
	va_start(args, format);
	while (*f && size < event->size)
	{
		int repeat = 0;
		while (isdigit(*f))
			repeat = 10*repeat + *f++ - '0';
		if (repeat==0) repeat = 1;
		
		if (*f == 'i') {
			for (; repeat>0; repeat--) {
				/* For portability ints are sent in network byte order. */
				*va_arg(args, int32_t*) = ntohl(*(int32_t*)&event->buf[size]);
				size += 4;
			}
		} else if (*f == 's') {
			memcpy(va_arg(args, char*), &event->buf[size], repeat);
			size += repeat;
		} else if (*f == 'x') {
			memcpy(va_arg(args, char*), &event->buf[size], event->size - size);
			size = event->size;
		}
		f++;
	}
	va_end(args);
	
	if (size != event->size) {
		fprintf(stderr, "gevent_recv: Format size %i does not equal event size %i.\n", size, event->size);
		dumpEvent(stderr, event, format);
		free(event);
		return -1;
	}
	free(event);
	return size;
}

static void cleanUp()
{
	int type;
	gevent* event;
	for (type=0; type<=MAX_EVENT_TYPE; type++) {
		mos_mutex_lock(&locks[type]);
		while (tails[type]) {
			event = tails[type];
			tails[type] = event->next;
			free(event);
		}
		heads[type] = NULL;
		mos_mutex_unlock(&locks[type]);
	}
	if (close(ssock)) perror("recvThread: close");
	if (close(rsock)) perror("recvThread: close");
}

static void recvThread()
{
	uint8_t buf[MAX_EVENT_SIZE];
	int len;
	while(gevent_connected)
	{
		/* Wait for a packet. */
		len = recv(rsock, buf, MAX_EVENT_SIZE, 0);
		if (len<0) {
			perror("recvThread: recv");
			continue;
		}
		if (len<8) continue;
		
		/* Get event size and type. */
		/* For portability ints are sent in network byte order. */
		uint32_t size = ntohl(*(uint32_t*)&buf[0]);
		uint32_t type = ntohl(*(uint32_t*)&buf[4]);
		if (type>MAX_EVENT_TYPE) continue;
		
		if (type == DISCONNECT_NODE_EVENT) {
			gevent_connected = 0;
			cleanUp();
			//printf("Received DISCONNECT event from the server.\n");
			break;
		}
		else if (type == KILL_NODE_EVENT) {
			gevent_connected = 0;
			cleanUp();
			//printf("Received KILL event from the server.\n");
			exit(0);
		}
		
		/* Allocate the new event (first data byte is already counted in sizeof(gevent)). */
		gevent* event = (gevent*)malloc(sizeof(gevent)+(size-1));
		event->next = NULL;
		event->type = type;
		event->size = size;
		memcpy(event->buf, &buf[8], size);
		
		/* Put it on the queue. */
		mos_mutex_lock(&locks[type]);
		if (heads[type])
			heads[type]->next = event;
		else
			/* Queue is empty. */
			tails[type] = event;
		heads[type] = event;
		mos_mutex_unlock(&locks[type]);
		
		/* Notify any waiters that a new event is available. */
		mos_sem_post(&sems[type]);
	}
}

/* Dump an event to the stream.  For debugging. */
static void dumpEvent(FILE* stream, gevent* event, const char* format)
{
	static uint8_t buf[MAX_EVENT_SIZE+1];

	fprintf(stream, "event->size: %i\n", event->size);
	fprintf(stream, "event->type: %i\n", event->type);
	fprintf(stream, "fields:\n");

	const char* f = format;
	uint32_t size = 0;
	while (*f && size < event->size)
	{
		int repeat = 0;
		while (isdigit(*f))
			repeat = 10*repeat + *f++ - '0';
		if (repeat==0) repeat = 1;
		
		if (*f == 'i') {
			for (; repeat>0; repeat--) {
				/* network byte order */
				int32_t i = ntohl(*(int32_t*)&event->buf[size]);
				fprintf(stream, "\toffset: %i, type: i, value: %i\n", size, i);
				size += 4;
			}
		} else if (*f == 's') {
			memcpy(buf, &event->buf[size], repeat);
			int i;
			for (i=0; i<repeat; i++)
				if (buf[i]<32 || buf[i]>126)
					buf[i] = '.';
			buf[i] = 0;
			fprintf(stream, "\toffset: %i, type: s, value: %s\n", size, buf);
			size += repeat;
		} else if (*f == 'x') {
			fprintf(stream, "\toffset: %i, type: x, value:\n\t\t", size);
			int i;
			for (i=0; i<repeat; i++)
				fprintf(stream, "%2x ", event->buf[size+i]);
			fprintf(stream, "\n");
			size = event->size;
		}
		f++;
	}
}

#endif	/* PLATFORM_LINUX */
