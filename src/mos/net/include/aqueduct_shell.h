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
/* File:       aqueduct_shell.h                                           */
/* Author:     Lane Phillips  :  lane.phillips@colorado.edu               */
/*   Date:                                                                */
/**************************************************************************/

/** @file aqueduct_shell.h
 * @brief Aqueduct shell implementation
 * 
 * This header contains declarations that are used by the code that implements
 * a special shell for running Aqueduct experiments.  This file is included by
 * both src/mos/net/aqueduct_shell.c, which is MOS code, and src/apps/deluge_test/aqshell.c,
 * which is XMOS code.  The constants defined in this header are duplicated in
 * src/apps/deluge_test/ExperServer.java.
 * 
 * @author Lane Phillips
 * @date Created:          
 */

#ifndef _AQUEDUCT_SHELL_H_
#define _AQUEDUCT_SHELL_H_

#define AQUEDUCT_SHELL

#include "com.h"

// Packet types
#define AQSHELL_MESSAGE '#'
#define AQSHELL_START '0'
#define AQSHELL_NEWVERSION '1'
#define AQSHELL_COMPLETEPAGE '2'
#define AQSHELL_COMPLETEPROG '3'
#define AQSHELL_CLEARSTATS '4'
#define AQSHELL_STARTSTATS '5'
#define AQSHELL_STOPSTATS '6'
#define AQSHELL_SAVESTATS '7'
#define AQSHELL_GETSTATS '8'
#define AQSHELL_GETSTATS_REPLY '9'
#define AQSHELL_SETVERSION 'A'
#define AQSHELL_SETVERSION_REPLY 'B'
#define AQSHELL_SUMMARY 'C'
#define AQSHELL_PROFILE 'D'
#define AQSHELL_REQUEST 'E'
#define AQSHELL_DATA 'F'
#define AQSHELL_GETID 'G'
#define AQSHELL_GETID_REPLY 'H'
#define AQSHELL_SUMMARY_SEND 'I'
#define AQSHELL_PROFILE_SEND 'J'
#define AQSHELL_REQUEST_SEND 'K'
#define AQSHELL_DATA_SEND 'L'
#define AQSHELL_CLOSE 'M'
#define AQSHELL_SETLOG 'N'
#define AQSHELL_NOOP 'O'
#define AQSHELL_GETVERSION 'P'
#define AQSHELL_SETIMAGESIZE 'Q'
#define AQSHELL_SETCACHESIZE 'R'
#define AQSHELL_ALLQUIET 'S'
#define AQSHELL_NOTQUIET 'T'
#define AQSHELL_HEARTBEAT 'U'
#define AQSHELL_DEADNODE 'V'
#define AQSHELL_STOPUPDATE 'W'
#define AQSHELL_CACHEHIT 'X'
#define AQSHELL_CACHEMISS 'Y'
#define AQSHELL_CACHEHITFORWARD 'Z'
#define AQSHELL_CACHEHITOLDFORWARD 'a'

// Flags to tell how this packet should be handled
#define AQSHELL_F_ACK 0x01				// Acknowledging a command
#define AQSHELL_F_PLEASEACK 0x02		// Receiver of command should ACK
#define AQSHELL_F_RESEND 0x04			// Unused
#define AQSHELL_F_FORWARDREPLY 0x08		// XMOS shell should forward reply to Java server
#define AQSHELL_F_SHELLCTL 0x10			// Command intended for XMOS shell
#define AQSHELL_F_NODECTL 0x20			// Command intended for node, XMOS shell will forward

struct aqshell_header {
	uint8_t command;
	uint8_t flags;
	uint16_t id;
	uint8_t seq;
	uint8_t length;
} __attribute__ ((packed));

#define AQSHELL_DATA_SIZE (COM_DATA_SIZE - sizeof(struct aqshell_header))

struct aqshell_pkt {
	struct aqshell_header head;
	uint8_t data[AQSHELL_DATA_SIZE];
} __attribute__ ((packed));


#ifndef PLATFORM_LINUX

#ifdef AQUEDUCT_SHELL

void aqshell_init();
void aqshell_send(uint8_t type, uint8_t* data, uint8_t len);

#else

// compile to nothing
static inline void aqshell_init() {}
static inline void aqshell_send(uint8_t type, uint8_t* data, uint8_t len) {}

#endif /*AQUEDUCT_SHELL*/
#endif /*PLATFORM_LINUX*/
#endif /*_AQUEDUCT_SHELL_H_*/
