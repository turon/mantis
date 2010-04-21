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


/** @file gevent.h
 * @author Lane Phillips	Lane.Phillips@colorado.ed
 * @date 2004/09/29
 * @brief Generic events for XMOS simulator.
 */

#ifndef GEVENT_H_
#define GEVENT_H_

#include "mos.h"

/* The default address for the simulation server is localhost.
 * Use -vizaddress on the command line to change it.
 */
#define GEVENT_ADDRESS		"127.0.0.1"
/* The default port for the sending to the simulation server is 1521.
 * Use -vizsendport on the command line to change it.
 */
#define GEVENT_SEND_PORT	1521
/* The default port for receiving events from the simulation server is the 
 * value of the node's ID.  Use -vizrecvport on the command line to change it.
 */

/* These event types should match those in src/tools/tenodera/modules/events.py
 * Some of these are not used in XMOS, but are included so you know they are
 * already reserved.
 */
#define NEW_NODE_EVENT			1
#define REMOVE_NODE_EVENT		2
#define RADIO_PACKET_EVENT		3
#define NET_SUBSCRIBE_EVENT		4
#define NET_UNSUBSCRIBE_EVENT	5
#define LED_EVENT				6
#define EXIT_EVENT            	7
#define MOVE_NODE_EVENT       	8
#define RADIO_MODEL_LINK      	9
#define REMOVE_ALL_EVENT		10
#define RADIO_MODEL_FORWARD		11
#define LAUNCH_NODE_EVENT       12
#define DISCONNECT_NODE_EVENT   13
#define KILL_NODE_EVENT         14

#define MAX_EVENT_TYPE			14

#define MAX_EVENT_SIZE			256

/* This stuff only works in XMOS. */
#ifdef PLATFORM_LINUX

#include <stdint.h>

/* Set to non-zero if we have successfully connected to the simulation server.
 * Unset this if the connection has had an unrecoverable error.
 */
extern int gevent_connected;

/* Set up connection to the simulation server and start the receive thread.
 * You must start the node with the -viz argument if you want to connect
 * to the simulation server.
 * Returns 0 on success, -1 on error.
 */
int gevent_init();

/* Format the arguments into a packet.  Format string tells how to
 * pack the data:
 * 
 * Ni	- next N arguments are 32-bit ints
 * Ns	- next argument points to an array of char, copy N chars from it
 * x	- get N from next argument, then copy N chars from subsequent argument
 * 
 * If the server is not up, output the arguments with printf() using 'message' 
 * as the format string.
 * Returns 0 on success, -1 on error.
 */
int gevent_send(uint32_t type, const char* format, const char* message, ...);

/* Receive and parse an event from the simulation server.
 * This function blocks until an event is available.
 * Format string tells how to fill in arguments:
 * 
 * Ni	- next N arguments are pointers to 32-bit ints
 * Ns	- copy N chars to the next argument, which points to a char array
 * x	- copy the rest of the packet into a char array
 * 
 * Returns size of event on success, -1 on error.
 */
int gevent_recv(uint32_t type, const char* format, ...);

#else /* We are compiling for the sensor node; the gevent functions do nothing. */

#define gevent_connected								0
#define gevent_init()									0
#define gevent_send(type,format,msg,args...)			0
#define gevent_recv(type,format,args...)				0

#endif	/* PLATFORM_LINUX */

#endif /* GEVENT_H_ */
