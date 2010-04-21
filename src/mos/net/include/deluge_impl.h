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
/* File:       deluge_impl.h                                              */
/* Author:     Lane Phillips  :  lane.phillips@colorado.edu               */
/*   Date: 02/24/05                                                       */
/*                                                                        */
/* Deluge protocol implementation                                         */
/**************************************************************************/

/** @file deluge_impl.h
 * @brief Deluge protocol implementation
 * 
 * This header contains declarations that are used by the code that implements
 * Deluge.  If you want to use Deluge in your application, you do not need to
 * include this file.  Include deluge.h instead.
 * 
 * @author Lane Phillips
 * @date Created: 02/24/05
 */

#ifndef _DELUGE_IMPL_H_
#define _DELUGE_IMPL_H_

#include "deluge.h"

#include "com.h"
#include "simple_fs.h"
#include "clock.h"
#include "sem.h"
#include "avr-eeprom.h"
#include "dev.h"

#ifndef PLATFORM_LINUX
#ifdef ARCH_AVR
#include "boot.h"
#endif
#include "command_daemon.h"
#endif

/* Compilation switches for various Deluge policies and debugging. */

// Nodes only listen neighbors with their id + or - 1
//#define DELUGE_FORCE_MULTIHOP

//#define DELUGE_PRINT_PACKETS
//#define DELUGE_PRINT_EVENT

//#define DELUGE_COMMANDER

// Store packet counts
//#define DELUGE_KEEP_STATS

// Transfer data, but don't load and execute it.  Useful for experiments.
#define DELUGE_NO_REPROGRAM

// Member and forwarding nodes behave the same, forwarding nodes don't reprogram
//#define DELUGE_NO_FORWARD

#ifndef DELUGE_NO_FORWARD

// Check for link symmetry before using DTB
#define DELUGE_SYMMETRIC_LINKS

// Member nodes don't use DTB
#define MEMBER_NO_DTB

// Forward data packets as soon as they arrive, don't wait for a full page
// This one is stupid, don't bother trying it.
//#define DELUGE_FORWARD_DATA_IMMEDIATELY

// Handle new requests immediately, don't wait for the next timeout
//#define DELUGE_FORWARD_REQUESTS_IMMEDIATELY

// Drop outgoing request if we timeout too many times
#define DELUGE_CLEAR_OUTGOING

// Scale request timeout by DTB
#define DELUGE_DTB_REQUEST_TIMEOUT
//#define DELUGE_DTB_EXP_REQUEST_TIMEOUT

// Size of cache in pages
#define DELUGE_CACHE_PAGES 10
#endif

/* Deluge parameter constants: */
#define DELUGE_K         1
#define DELUGE_FORWARD_K	4
#define DELUGE_LAMBDA   2
#define DELUGE_T_L	(2*(uint32_t)TICKS_PER_SEC)	// 2 seconds
#define DELUGE_T_H	(256*(uint32_t)TICKS_PER_SEC)	// 256 seconds
#define DELUGE_T_R	((uint32_t)TICKS_PER_SEC/(uint32_t)2)	// half a second
#define DELUGE_OMEGA    ((uint32_t)8)
// Estimate the time it takes to transmit 1 data packet plus 20 bytes overhead at 9600 bps plus a 10 ms backoff
#define DELUGE_T_TX	((uint32_t)TICKS_PER_SEC*(uint32_t)84*(uint32_t)8/(uint32_t)9600 + (uint32_t)TICKS_PER_SEC/(uint32_t)100)
#define DELUGE_TX_DELAY_INC	1
#define DELUGE_TX_DELAY_MAX	500
#define DELUGE_TX_DELAY_MIN	0

//#ifdef DELUGE_SYMMETRIC_LINKS
#define DELUGE_NEIGHBOR_COUNT	5
//#endif

#ifdef DELUGE_COMMANDER
typedef struct {
	uint8_t seq;
	uint8_t command;
	uint16_t to;
	uint16_t id;
	uint8_t type;		// DELUGE_PACKET_COMMAND
} deluge_foot_command;

#define DELUGE_COMMAND_HELLO 1
#define DELUGE_COMMAND_STOPALL 3
#define DELUGE_COMMAND_CLEARSTATS 2
#define DELUGE_COMMAND_STARTRECORD 4
#define DELUGE_COMMAND_ENDRECORD 5
#define DELUGE_COMMAND_SENDSTATS 6
#define DELUGE_COMMAND_SENDSTATS_REPLY 7
#define DELUGE_COMMAND_FINISHED 8
#define DELUGE_COMMAND_VERSION 9
#define DELUGE_COMMAND_VERSION_REPLY 10
#define DELUGE_COMMAND_NEIGHBORS 11
#define DELUGE_COMMAND_NEIGHBORS_REPLY 12
#define DELUGE_COMMAND_WIPE 13
#define DELUGE_COMMAND_BOOT 14
#define DELUGE_COMMAND_CACHESIZE 15
#define DELUGE_COMMAND_POWER 16
#endif

/* Protocol footers: */

/* All the footers end with a type field which can be checked like so:
 * 	uint8_t type = packet->data[packet->size-1];
 * 
 * Footers use the 'packed' attribute for XMOS compatibility.
 */
#define DELUGE_PACKET_SUMMARY	1
#define DELUGE_PACKET_PROFILE	2
#define DELUGE_PACKET_REQUEST	3
#define DELUGE_PACKET_DATA		4
#define DELUGE_PACKET_COMMAND	5
 
typedef struct {
	//#ifdef DELUGE_SYMMETRIC_LINKS
	uint16_t neighbors[DELUGE_NEIGHBOR_COUNT];
	//#endif
	// The current image version number
	uint8_t version;
	// MEMBER:	Highest page available + 1
	// FORWARD:	Highest page available AT THE ROOT OF THE SPT + 1
	int8_t highPage;
	// MEMBER:	0
	// FORWARD:	Distance to root of the SPT
	uint8_t dtb;
	// ID of sender
	uint16_t id;
	uint8_t type;	// DELUGE_PACKET_SUMMARY
} __attribute__ ((packed)) deluge_foot_summary;

typedef struct {
	// Size in bytes of the current image
	uint32_t codeSize;
	// CRC of the whole image
	uint16_t crc;
	// Version number of the image
	uint8_t version;
	// Highest page in the image + 1
	int8_t goalPage;
	// ID of sender
	uint16_t id;
	uint8_t type;	// DELUGE_PACKET_PROFILE
} __attribute__ ((packed)) deluge_foot_profile;

typedef struct {
	// Bit vector indicating which packets of the page are needed
	uint16_t packets[3];
	// ID of the node this request is sent to
	uint16_t to;
	// Requested version
	uint8_t version;
	// Requested page
	int8_t page;
	// Amount to adjust transmit delay (see txDelay below)
	int8_t rateChange;
	// ID of sender
	uint16_t id;
	uint8_t type;	// DELUGE_PACKET_REQUEST
} __attribute__ ((packed)) deluge_foot_request;

// This footer follows DELUGE_PACKET_SIZE bytes of data
typedef struct {
	uint8_t version;	// Version number
	int8_t page;		// Page number
	uint8_t packet;		// Packet number
	// ID of sender
	uint16_t id;
	uint8_t type;		// DELUGE_PACKET_DATA
} __attribute__ ((packed)) deluge_foot_data;

// -2 for net layer footer
#define DELUGE_PACKET_SIZE (COM_DATA_SIZE-2-sizeof(deluge_foot_data))
#define DELUGE_PACKETS_PER_PAGE		(48)
// -2 for page CRC at the end of the last packet
#define DELUGE_PAGE_SIZE (DELUGE_PACKETS_PER_PAGE * DELUGE_PACKET_SIZE - 2)
#define DELUGE_ADDR(page, packet)	((uint32_t)(page)*DELUGE_PAGE_SIZE + (packet)*DELUGE_PACKET_SIZE)

/* Some of the Deluge state variables have different meanings depending on
 * which mode Deluge is running in.
 */

#define DELUGE_STATE_MAINTAIN	1
#define DELUGE_STATE_MAINTAIN2	2
#define DELUGE_STATE_RX			3
#define DELUGE_STATE_TX			4
#define DELUGE_STATE_FORWARD	5
// Don't change state if we see this
#define DELUGE_STATE_IGNORE		6

typedef struct
{
	uint32_t codeSize;
	// CRC of the entire program
	uint16_t programcrc;
	// Bit vector of pages we need
	// There are enough bits here to cover 120 KB of flash
	// Only bits 0 through goalPage-1 are use, all others are ignored
	uint8_t pagesNeeded[6];
	// Version of the code image
	uint8_t version;
	// MEMBER:	Page we are currently receiving; equivalent to highPage
	// FORWARD:	Page we are currently caching
	int8_t incomingPage;
	// MEMBER:	Highest page available + 1; equivalent to incomingPage
	// FORWARD:	Highest page on the MEMBER node at the root of this SPT + 1
	int8_t highPage;
	// Highest page in the image + 1
	int8_t goalPage;
} deluge_control_block;

struct deluge_entry;

typedef struct {
	// Persistent state; this is enough information to restart Deluge mid-update
	deluge_control_block dcb;
	
	// Protocol state; see the Deluge paper
	volatile uint8_t state;
	// Number of summaries overheard in this round
	volatile uint8_t nSummaries;
	// Number of profiles overheard in this round
	volatile uint8_t nProfiles;
	// TODO merge flags into a bit vector
	// Flag to start the next round with the minimum duration
	volatile uint8_t detectedInconsistency;
	// Flag to send a profile during the next round
	volatile uint8_t heardObsolete;
	// Someone else is requesting an update
	volatile uint8_t heardRequest;
	// Someone else is receiving an update
	volatile uint8_t heardData;
	// MEMBER:	Node to send request to in RX state
	// FORWARD:	Parent node in SPT
	volatile uint16_t updaterNode;
	// bit vector for packets we want
	uint16_t incomingPackets[3];
	// how many packets are we requesting
	uint8_t nPacketsRequested;
	// packets in our forwarding cache
	uint16_t cachedPackets[3];
	// CRC of the page we are currently receiving
	uint16_t incomingcrc;
	// Page that was requested from us
	volatile int8_t outgoingPage;
	// Bit vector of packets that were requested from us
	volatile uint16_t outgoingPackets[3];
	// Number of times our request has timed out
	uint8_t nRequests;
	// Listening time during the current MAINTAIN round before sending
	// a summary or profile
	uint32_t sumT;
	// Distance to root of the SPT via updaterNode
	// Summaries from farther nodes are ignored, unless requests to updaterNode
	// time out too many times, in which case dtb is set to its maximum.
	uint8_t dtb;
	
	// Number of milliseconds to sleep between packets in TX state
	volatile int32_t txDelay;
	// MEMBER:	The file containing the code image
	// FORWARD: The file caching the last page
	mos_file* image_file;
	// Why did we wake up?  Timeout or packet received?
	// TODO do we really need this?
	//volatile uint8_t wakeup_reason;
	// Prevents concurrent modification of Deluge state
	mos_mutex_t delugeLock;
	int8_t index;

	#ifdef DELUGE_SYMMETRIC_LINKS
	// Nodes we have heard from recently
	uint16_t neighbors[DELUGE_NEIGHBOR_COUNT];
	// DTBs we have found through symmetric links
	//uint8_t symdtbs[DELUGE_NEIGHBOR_COUNT];
	// Index into the array for storing the next neighbor we hear from
	uint8_t nextNeighbor;
	#endif
	
	#if DELUGE_CACHE_PAGES > 1
	// Page numbers of pages we have cached
	int8_t cache_map[DELUGE_CACHE_PAGES];
	// Cache page corresponding to outgoingPage
	int8_t outgoingCachePage;
	// Cache page corresponding to incomingPage
	int8_t incomingCachePage;
	// hack for testing cache sizes
	int8_t cacheSize;
	#endif
} deluge_app;

typedef struct deluge_entry
{
	// Pointer to this instance's state
	deluge_app* app;
	// Tells dispatcher (see deluge_proto.c) that an event occurred in this
	// Deluge instance
	//sem app_sem;
	uint8_t copyState;
	// State to switch to when dispatchThread wakes up
	uint8_t nextState;
	// Implements timeouts
	mos_alarm_t alarm;
	// Duration of the current round of MAINTAIN state
	uint32_t roundT;
} deluge_entry;

// Implemented in deluge_proto.c
void deluge_wakeup(deluge_entry* entry);
void deluge_saveStats();

// Implemented in deluge_app.c
void stateTransition(deluge_app* app, uint8_t newState);
void deluge_saveState(deluge_app* app);
void deluge_dispatch(deluge_app* app);
void runReprogram(deluge_app* app);
void reboot();
void handleSummary(deluge_app* app, deluge_foot_summary* summary);
void handleProfile(deluge_app* app, deluge_foot_profile* profile);
void handleRequest(deluge_app* app, deluge_foot_request* request);
void handleData(deluge_app* app, deluge_foot_data* data, uint8_t* packet);
int8_t deluge_app_staticinit();
int8_t deluge_app_init(deluge_app* app, int8_t index);
//void deluge_print(deluge_app* app);
//void deluge_set_version(deluge_app* app);

#ifdef DELUGE_COMMANDER
// Implemented in deluge_admin.c
void sendFinished(deluge_app* app, comBuf* pkt, uint8_t port);
void handleCommand(deluge_app* app, comBuf* pkt, uint8_t port);
#endif

#endif /*_DELUGE_IMPL_H_*/
