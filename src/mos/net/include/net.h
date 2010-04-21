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
/* File:       net.h                                                      */
/* Author:     Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 04/15/04                                                       */
/*                                                                        */
/* networking layer.                                                      */
/**************************************************************************/

/** @file net.h
 * @brief Top level event based networking layer.
 * @author Jeff Rose, Charles Gruenwald III
 * @author Modified: Cyrus Hall
 * @author Modified: Carl Hartung
 * @date Created: 04/15/2004
 * @date Modified: 07/22/2004
 * @date Modified: 11/12/2004
 */

#ifndef _NET_H_
#define _NET_H_

#include "mos.h"

#include "com.h"
#include "printf.h"
#include <stdarg.h>

#if defined(PLATFORM_MICROBLAZE)
#include "mb_radio.h"
#endif


/** @addtogroup mos_net Net Layer
 * The network layer allows the application
 * to abstract away routing to a lower level.
 * Different routing protocols can be implemented
 * using the net layer, and then used from an
 * application.
 */
//@{


/** @brief No space left in the comBuf. */
#define NET_BUF_FULL -20 
/** @brief Not a valid protocol id. */
#define NET_PROTO_INVALID -1

/** @brief Maximum number of protocols. */
#define NET_PROTO_MAX 3

/** PROTOCOL LIST **/
#define SIMPLE_PROTO_ID 17
#define MST_PROTO_ID    18
#define RTS_PROTO_ID    19
#define DELUGE_PROTO_ID 20
#define CTP_PROTO_ID    0xC4

/** END PROTOCOL LIST **/

typedef int8_t (*net_proto_send)(comBuf *pkt, va_list args);
typedef boolean (*net_proto_recv)(comBuf *pkt, uint8_t **footer, uint8_t port);
typedef int8_t (*net_proto_ioctl)(uint8_t request, va_list args);

/** @brief Net protocol data structure. */
typedef struct
{
  /** @brief Protocol ID */
   uint8_t proto_id;
  /** @brief Protocol send function */
   net_proto_send sfunc;
  /** @brief Protocol receive function */
   net_proto_recv rfunc;
  /** @brief Protocol io control function */
   net_proto_ioctl ifunc;
}net_proto;

/** Setup the net layer. */
void net_init();

/** @brief Send an event with the currently selected protocol. 
 * @param event_id Event ID
 * @param pkt Packet sent
 * @return NET_PROTO_INVALID if protocol invalid, else return ret
 */
int8_t net_send(comBuf *pkt, uint8_t proto_id, uint8_t port, ...);

/** @brief  Register a new protocol to be used with the net layer. 
 * @param proto New protocol
 * @param sfunc Protocol send function
 * @param rfunc Protocol receive function
 * @param ifunc Protocol io control function
 * @return NET_PROTO_INVALID if protocol invalid, -1 if no space left, else return
 */
int8_t net_proto_register(uint8_t proto, net_proto_send sfunc,
			  net_proto_recv rfunc, net_proto_ioctl ifunc);

/** @brief Set some protocol specific options. 
 * @param proto Protocol
 * @param request IO Request
 * @return NET_PROTO_INVALID if protocol invalid, else return retval
 */
int8_t net_ioctl(uint8_t proto, uint8_t request, ...);

/** @internal is_app_waiting_on() method called by protocols
 * @param port check all waiting threads for the specified port
 * @return true if it sent the packet to an app, false if it did not.
 */
boolean is_app_waiting_on(uint8_t port);
 
/** @brief net_recv method called by applications
 * @param port listening on specified port
 * @return returns comBuf to application
 */
comBuf * net_recv(uint8_t port);

void net_recv_packet();

/** @internal A background thread that listens on everything for event traffic.*/
void net_thread();

//@}

#endif
