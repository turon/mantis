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

#include "mos.h"

#ifdef ARCH_AVR

#include "msched.h"
#include "com.h"
#include "node_id.h"
#include "clock.h"
#include "deluge_impl.h"
#include "printf.h"
#include "led.h"
#include "boot.h"
#include "net.h"
#include "sem.h"
#include "crc.h"
#include "atmel-flash.h"
#include "simple_fs.h"
#include <stdlib.h>
#include "deluge_msgs.h"
#include "aqueduct_shell.h"

/* Protocol state variables: */

static comBuf spkt;					// comBuf for sending

extern uint8_t deluge_portmap[DELUGE_INSTANCE_COUNT];
extern deluge_entry table[DELUGE_INSTANCE_COUNT];
#ifdef DELUGE_KEEP_STATS
extern uint16_t packet_counts[DELUGE_STATS_COUNT][4];
extern uint8_t deluge_recordstats;
#endif


/* Private functions: */

#ifdef DELUGE_SYMMETRIC_LINKS
static void checkNeighbor(deluge_app* app, uint16_t neighbor)
{
	int8_t i;
	//uint8_t mask = 1;
	for (i=0; i<DELUGE_NEIGHBOR_COUNT; i++)
	{
		if (neighbor==app->neighbors[i]) {
			return;
		}
		//mask <<= 1;
	}

	i = app->nextNeighbor;
	app->neighbors[i] = neighbor;
	//mask = 1 << i;
	app->nextNeighbor = (i + 1) % DELUGE_NEIGHBOR_COUNT;
	// Our neighbor list changed, this could affect routing
	//app->detectedInconsistency = 1;

	//app->symdtbs[i] = 255;
}
	
static int8_t checkSymmetry(deluge_app* app, deluge_foot_summary* summary)
{
	uint16_t id = mos_node_id_get();
	uint8_t j;
	for (j=0; j<DELUGE_NEIGHBOR_COUNT; j++)
	{
		if (id==summary->neighbors[j]) {
			/*if (summary->version == app->dcb.version &&
				(summary->highPage > app->dcb.highPage ||
				(app->index != 0 && summary->highPage == app->dcb.highPage)))
			{
				app->symdtbs[i] = summary->dtb;
			}*/
			//app->symmetric |= mask;
			return 1;
		}
	}
	//app->symmetric &= ~mask;
	// We're not in their list, send a summary
	//app->detectedInconsistency = 1;
	return 0;
}

static void clearSymmetry(deluge_app* app, uint16_t node)
{
	uint8_t i;
	//uint8_t mask = 1;
	for (i=0; i<DELUGE_NEIGHBOR_COUNT; i++)
	{
		if (node==app->neighbors[i]) {
			app->neighbors[i] = 0xFFFF;
			//app->symdtbs[i] = 255;
			//app->symmetry &= ~mask;
			// Our neighbor list changed, this could affect routing
			//app->detectedInconsistency = 1;
		}
		//mask <<= 1;
	}
}
#else
static inline void checkNeighbor(deluge_app* app, uint16_t neighbor) {}
#endif

#if DELUGE_CACHE_PAGES > 1
static int8_t cachedPageIndex(deluge_app* app, int8_t page)
{
	int8_t i;
	for (i=0; i</*DELUGE_CACHE_PAGES*/app->cacheSize; i++)
		if (app->cache_map[i] == page)
			return i;
	return -1;
}

static void saveIncomingCachePage(deluge_app* app, int8_t page)
{
	app->cache_map[app->incomingCachePage] = page;
	app->incomingCachePage = (app->incomingCachePage + 1) % /*DELUGE_CACHE_PAGES*/app->cacheSize;
}
#endif

static void setTimeout(deluge_app* app, uint32_t ticks, uint8_t reason)
{
	table[app->index].nextState = reason;
    //deprecated call, replaced with?
    //TODO: fixme
//	mos_alarm_ticks(&table[app->index].alarm, ticks);
}

static inline void setRequestTimeout(deluge_app* app, uint8_t timeoutState)
{
	#ifdef DELUGE_DTB_REQUEST_TIMEOUT
	uint32_t reqT = random() % DELUGE_T_R;
	if (app->dtb > 1) {
		//printf("#nPacketsRequested: %C\n", app->nPacketsRequested);
		reqT += app->dtb * (app->nPacketsRequested + DELUGE_OMEGA) * DELUGE_T_TX;
	} else
		reqT += DELUGE_OMEGA * DELUGE_T_TX;
	#else
	uint32_t reqT = DELUGE_OMEGA * DELUGE_T_TX + random() % DELUGE_T_R;
	#endif
	// this timeout will be reset as long as we keep getting data packets
	setTimeout(app, reqT, timeoutState);
}

static void clearTimeout(deluge_app* app)
{
	mos_remove_alarm(&table[app->index].alarm);
}

void stateTransition(deluge_app* app, uint8_t newState)
{
	mos_remove_alarm(&table[app->index].alarm);
	table[app->index].nextState = newState;
	table[app->index].copyState = 1;
	deluge_wakeup(&table[app->index]);
}

void deluge_saveState(deluge_app* app)
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_CONTROL_BLOCK_ADDR + app->index*DELUGE_CONTROL_BLOCK_SIZE);
	dev_write (DEV_AVR_EEPROM, (uint8_t *)&app->dcb, DELUGE_CONTROL_BLOCK_SIZE);
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
}

static void loadState(deluge_app* app)
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_CONTROL_BLOCK_ADDR + app->index*DELUGE_CONTROL_BLOCK_SIZE);
	dev_read (DEV_AVR_EEPROM, (uint8_t *)&app->dcb, DELUGE_CONTROL_BLOCK_SIZE);
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
}

static void saveBootCB(deluge_app* app, boot_control_block_t* cb)
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
	dev_write (DEV_AVR_EEPROM, (uint8_t *)cb, sizeof (boot_control_block_t));
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
}

static void loadBootCB(deluge_app* app, boot_control_block_t* cb)
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
	dev_read (DEV_AVR_EEPROM, (uint8_t *)cb, sizeof (boot_control_block_t));
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
}

static void moveToNextPage(deluge_app* app)
{
	uint8_t index = app->dcb.incomingPage / 8;
	uint8_t mask = 1 << (app->dcb.incomingPage % 8);
	
	// clear bit for page we just finished
	app->dcb.pagesNeeded[index] &= ~mask;
	
	// find the next needed page
	do {
		app->dcb.incomingPage++;
		mask = mask << 1;
		if (mask == 0) {
			mask = 1;
			index++;
		}
		if (app->dcb.pagesNeeded[index] & mask) break;
	}
	while (app->dcb.incomingPage < app->dcb.goalPage);
	app->dcb.highPage = app->dcb.incomingPage;
}

static void findBadPages(deluge_app* app)
{
	uint8_t page = 0;
	uint8_t index = 0;
	uint8_t mask = 1;
	uint16_t crc;
	
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_PAGE_CRC_ADDR);
	for (; page<app->dcb.goalPage; page++)
	{
		dev_read (DEV_AVR_EEPROM, (uint8_t *)&crc, 2);
		if (crc != mos_file_crc(app->image_file, DELUGE_ADDR(page, 0), (uint32_t)DELUGE_PAGE_SIZE)) {
			app->dcb.pagesNeeded[index] |= mask;
			if (app->dcb.incomingPage == app->dcb.goalPage)
				app->dcb.incomingPage = page;
		}
		mask = mask << 1;
		if (mask == 0) {
			mask = 1;
			index++;
		}
	}
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
	
	if (app->dcb.incomingPage == app->dcb.goalPage)
		// The program CRC failed, but we didn't find a page that failed.
		// This is extremely improbable, but just in case, we start all over:
		app->dcb.incomingPage = 0;
	app->dcb.highPage = app->dcb.incomingPage;
}

typedef void (*reboot_func)(void);

void reboot()
{
	// Jump to bootloader
	#ifdef DELUGE_PRINT_EVENT
	printf_P(sReboot);
	printf_P(sDisplayTime);		// display time
	#endif
	mos_disable_ints ();
	SPCR &= (1 << SPE);
	reboot_func rfunc = (reboot_func)0x1E000;
	rfunc();
}

void runReprogram(deluge_app* app)
{
	boot_control_block_t bcb;
	loadBootCB(app, &bcb);
	bcb.start_addr = app->image_file->start;
	bcb.byte_count = app->image_file->length;
	bcb.reprogram = 1;
	saveBootCB(app, &bcb);
	
	/*#ifdef DELUGE_KEEP_STATS
	deluge_saveStats();
	#endif*/
	
	reboot();
}

static void doMaintain1(deluge_app* app)
{
	//clearTimeout(app);
	// beginning of round
	if (app->detectedInconsistency) {
		// Rule M.2
		table[app->index].roundT = DELUGE_T_L;
	} else {
		// Rule M.3
		if (table[app->index].roundT < DELUGE_T_H)
			table[app->index].roundT *= 2;
		if (table[app->index].roundT > DELUGE_T_H)
			table[app->index].roundT = DELUGE_T_H;
	}

	mos_mutex_lock(&app->delugeLock);
	app->nSummaries = 0;
	app->nProfiles = 0;
	app->detectedInconsistency = 0;
	if (app->heardRequest) app->heardRequest--;
	if (app->heardData) app->heardData--;
	mos_mutex_unlock(&app->delugeLock);

	//app->state = DELUGE_STATE_MAINTAIN2;
	// wait until it's time to send our summary
	app->sumT = (table[app->index].roundT>>1) + random() % (table[app->index].roundT>>1);
	setTimeout(app, app->sumT, DELUGE_STATE_MAINTAIN2);
}

static void doMaintain2(deluge_app* app)
{
	// Send a summary or a profile
	if (app->heardObsolete) {
		app->heardObsolete = 0;
		if (app->nProfiles < DELUGE_K) {
			// Rule M.4
			deluge_foot_profile* profile = (deluge_foot_profile*)spkt.data;
			mos_mutex_lock(&app->delugeLock);
			profile->crc = app->dcb.programcrc;
			profile->codeSize = app->dcb.codeSize;
			profile->version = app->dcb.version;
			profile->goalPage = app->dcb.goalPage;
			mos_mutex_unlock(&app->delugeLock);
			
			profile->id = mos_node_id_get();
			profile->type = DELUGE_PACKET_PROFILE;
			spkt.size = sizeof(deluge_foot_profile);
			
			net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
		}
	} else if (/*(app->index == 0 && */app->nSummaries < DELUGE_K/*) ||
		(app->index != 0 && app->nSummaries < DELUGE_FORWARD_K)*/)
	{
		// Rule M.1
		deluge_foot_summary* summary = (deluge_foot_summary*)spkt.data;
		mos_mutex_lock(&app->delugeLock);
		#ifdef DELUGE_SYMMETRIC_LINKS
		uint8_t i;
		for (i = 0; i<DELUGE_NEIGHBOR_COUNT; i++)
			summary->neighbors[i] = app->neighbors[i];
		#endif
		summary->version = app->dcb.version;
		summary->highPage = app->dcb.highPage;
		#ifdef DELUGE_NO_FORWARD
		summary->dtb = 0;
		#else
		if (app->index == 0) {
			summary->dtb = 0;
		} else {
			summary->dtb = app->dtb;
		}
		#endif
		mos_mutex_unlock(&app->delugeLock);

		summary->id = mos_node_id_get();
		summary->type = DELUGE_PACKET_SUMMARY;
		spkt.size = sizeof(deluge_foot_summary);
						
		net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
	}
	
	// wait till the round is over
	setTimeout(app, table[app->index].roundT - app->sumT, DELUGE_STATE_MAINTAIN);
}

static void doRx(deluge_app* app)
{
	if (app->nRequests < DELUGE_LAMBDA) {
		// Rule R.1
		deluge_foot_request* request = (deluge_foot_request*)spkt.data;
		mos_mutex_lock(&app->delugeLock);
		request->to = app->updaterNode;
		request->packets[0] = app->incomingPackets[0];
		request->packets[1] = app->incomingPackets[1];
		request->packets[2] = app->incomingPackets[2];
		request->version = app->dcb.version;
		request->page = app->dcb.incomingPage;
		request->rateChange = app->nRequests-1;
		mos_mutex_unlock(&app->delugeLock);

		request->id = mos_node_id_get();
		request->type = DELUGE_PACKET_REQUEST;
		spkt.size = sizeof(deluge_foot_request);
					
		net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
		app->nRequests++;
	} else {
		// TODO: check reception rate
		// Rule R.2
		app->nRequests = 0;
		// Assume parent node is dead; assign the highest possible DTB; 
		// we'll take any node with a lower DTB
		app->dtb = 0xFF;
		/*#ifdef DELUGE_LINK_QUALITY
		app->dtb_votes[app->updater_index] = 0;
		#endif*/
		#ifdef DELUGE_SYMMETRIC_LINKS
		clearSymmetry(app, app->updaterNode);
		/*if (nextSymmetric(app) != 255) {
			app->nRequests = 0;
			stateTransition(app, DELUGE_STATE_RX);
			return;
		}*/
		#endif
		stateTransition(app, DELUGE_STATE_MAINTAIN);
		return;
	}

	// wait a while before requesting again
	setRequestTimeout(app, DELUGE_STATE_RX);
}

static void doTx(deluge_app* app)
{
	deluge_foot_data* foot = (deluge_foot_data*)&spkt.data[DELUGE_PACKET_SIZE];
	mos_mutex_lock(&app->delugeLock);
	foot->version = app->dcb.version;
	#if DELUGE_CACHE_PAGES > 1
	int8_t page = (app->index == 0)? app->outgoingPage : app->outgoingCachePage;
	#else
	int8_t page = app->outgoingPage;
	#endif
	foot->page = app->outgoingPage;
	foot->id = mos_node_id_get();
	foot->type = DELUGE_PACKET_DATA;
		
	uint32_t crc_len = (app->outgoingPage == app->dcb.goalPage-1)?
		app->dcb.codeSize % (uint32_t)DELUGE_PAGE_SIZE :
		(uint32_t)DELUGE_PAGE_SIZE;
	uint16_t crc = mos_file_crc(app->image_file, DELUGE_ADDR(page, 0), crc_len);
	
	uint16_t txpackets[3] = { app->outgoingPackets[0], app->outgoingPackets[1], app->outgoingPackets[2] };
	mos_mutex_unlock(&app->delugeLock);
		
	while (app->state == DELUGE_STATE_TX && (txpackets[0] != 0 || txpackets[1] != 0 || txpackets[2] != 0))
	{
		uint8_t i, ih;
		uint16_t txmask = 1;
		uint8_t txidx = 0;
		uint32_t txaddr = DELUGE_ADDR(page, 0);
		for (i=0; i<DELUGE_PACKETS_PER_PAGE; i++)
		{
			if ((txpackets[txidx] & txmask)!=0)
			{
				mos_mutex_lock(&app->delugeLock);
				mos_file_read(spkt.data, app->image_file, txaddr, DELUGE_PACKET_SIZE);
				mos_mutex_unlock(&app->delugeLock);
							
				foot->packet = i;
				spkt.size = DELUGE_PACKET_SIZE+sizeof(deluge_foot_data);	// com_send may have changed it
							
				// Last two bytes of last packet have page CRC
				if (i == DELUGE_PACKETS_PER_PAGE-1) {
					*(uint16_t*)&(spkt.data[DELUGE_PACKET_SIZE-2]) = crc;
				}
							
				net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
			
				ih = mos_disable_ints();
				app->outgoingPackets[txidx] &= ~txmask;
				txpackets[txidx] = app->outgoingPackets[txidx];
				mos_enable_ints(ih);
				
				mos_thread_sleep(app->txDelay);
			}
			
			txaddr = txaddr + DELUGE_PACKET_SIZE;
			txmask = txmask << 1;
			if (txmask == 0) {
				txmask = 1;
				txidx++;
			}
		}
				
		ih = mos_disable_ints();
		txpackets[0] = app->outgoingPackets[0];
		txpackets[1] = app->outgoingPackets[1];
		txpackets[2] = app->outgoingPackets[2];
		mos_enable_ints(ih);
	}
	// Rule T.2
	//app->state = DELUGE_STATE_MAINTAIN;
	//mos_sem_post(&timeout_sem);
	//deluge_wakeup(&table[app->index]);
	stateTransition(app, DELUGE_STATE_MAINTAIN);
}

/* FORWARD state is a combination of RX and TX states.  The node will fill as
 * many requests as it can from its cache, and forward requests it can't handle
 * to its parent.  The node's state will return to MAINTAIN once the request
 * is fully filled or too many requests time out.  To minimize contention, data
 * is only forwarded when the page is completed or the request times out or a
 * new request is received.
 */
static void doForward(deluge_app* app)
{
	// TX
	deluge_foot_data* foot = (deluge_foot_data*)&spkt.data[DELUGE_PACKET_SIZE];
	mos_mutex_lock(&app->delugeLock);
	foot->version = app->dcb.version;
	#if DELUGE_CACHE_PAGES > 1
	int8_t page = app->outgoingCachePage;
	#else
	int8_t page = 0;
	#endif
	foot->page = app->outgoingPage;
	foot->id = mos_node_id_get();
	foot->type = DELUGE_PACKET_DATA;

	// Packets that have been requested of us
	uint16_t txpackets[3] = { app->outgoingPackets[0], app->outgoingPackets[1], app->outgoingPackets[2] };
	// Packets we have cached
	uint16_t capackets[3] = { app->cachedPackets[0], app->cachedPackets[1], app->cachedPackets[2] };
	mos_mutex_unlock(&app->delugeLock);

	app->nPacketsRequested = 0;
	// Loop as long as we have packets to fill the request
	while (app->state == DELUGE_STATE_FORWARD &&
		((txpackets[0]&capackets[0]) != 0 || (txpackets[1]&capackets[1]) != 0 || (txpackets[2]&capackets[2]) != 0))
	{
		uint8_t i, ih;
		uint16_t txmask = 1;
		uint8_t txidx = 0;
		uint32_t txaddr = DELUGE_ADDR(page, 0);
		for (i=0; i<DELUGE_PACKETS_PER_PAGE; i++)
		{
			if ((txpackets[txidx] & capackets[txidx] & txmask)!=0)
			{
				mos_mutex_lock(&app->delugeLock);
				mos_file_read(spkt.data, app->image_file, txaddr, DELUGE_PACKET_SIZE);
				mos_mutex_unlock(&app->delugeLock);
							
				foot->packet = i;
				spkt.size = DELUGE_PACKET_SIZE+sizeof(deluge_foot_data);	// net_send may have changed it
							
				// Last two bytes of last packet have page CRC
				// We don't compute it ourselves, since we might not have
				// the entire page cached
				if (i == DELUGE_PACKETS_PER_PAGE-1) {
					*(uint16_t*)&(spkt.data[DELUGE_PACKET_SIZE-2]) = app->incomingcrc;
				}
							
				net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
			
				ih = mos_disable_ints();
				app->outgoingPackets[txidx] &= ~txmask;
				txpackets[txidx] = app->outgoingPackets[txidx];
				mos_enable_ints(ih);
				
				mos_thread_sleep(app->txDelay);
			} else if ((txpackets[txidx] & txmask) != 0) {
				app->nPacketsRequested++;
			}
			
			txaddr = txaddr + DELUGE_PACKET_SIZE;
			txmask = txmask << 1;
			if (txmask == 0) {
				txmask = 1;
				txidx++;
			}
		}
				
		ih = mos_disable_ints();
		txpackets[0] = app->outgoingPackets[0];
		txpackets[1] = app->outgoingPackets[1];
		txpackets[2] = app->outgoingPackets[2];
		capackets[0] = app->cachedPackets[0];
		capackets[1] = app->cachedPackets[1];
		capackets[2] = app->cachedPackets[2];
		
		// As long as we are filling requests, don't let incoming requests
		// inflate our total timeout count
		if (app->nRequests) app->nRequests--;
		mos_enable_ints(ih);
	}
	
	if (txpackets[0]==0 && txpackets[1]==0 && txpackets[2]==0) {
		// We are done
		app->nRequests = 0;
		//app->state = DELUGE_STATE_MAINTAIN;
		//deluge_wakeup(&table[app->index]);
		stateTransition(app, DELUGE_STATE_MAINTAIN);
		return;
	}

	// RX
	if (app->nRequests < DELUGE_LAMBDA) {
		// Rule R.1
		deluge_foot_request* request = (deluge_foot_request*)spkt.data;
		mos_mutex_lock(&app->delugeLock);
		request->to = app->updaterNode;
		// Request the packets that were requested of us
		request->packets[0] = (app->incomingPackets[0] |= app->outgoingPackets[0]);
		request->packets[1] = (app->incomingPackets[1] |= app->outgoingPackets[1]);
		request->packets[2] = (app->incomingPackets[2] |= app->outgoingPackets[2]);
		request->version = app->dcb.version;
		request->page = app->dcb.incomingPage;
		mos_mutex_unlock(&app->delugeLock);
		request->rateChange = app->nRequests-1;

		request->id = mos_node_id_get();
		request->type = DELUGE_PACKET_REQUEST;
		spkt.size = sizeof(deluge_foot_request);
					
		net_send(&spkt, DELUGE_PROTO_ID, deluge_portmap[app->index], deluge_portmap[app->index]);
		app->nRequests++;
	} else {
		// TODO: check reception rate
		app->nRequests = 0;
		// Assume parent node is dead; assign the highest possible DTB; 
		// we'll take any node with a lower DTB
		app->dtb = 0xFF;
		/*#ifdef DELUGE_LINK_QUALITY
		app->dtb_votes[app->updater_index] = 0;
		#endif*/
		#ifdef DELUGE_SYMMETRIC_LINKS
		clearSymmetry(app, app->updaterNode);
		/*if (nextSymmetric(app) != 255) {
			app->nRequests = 0;
			stateTransition(app, DELUGE_STATE_FORWARD);
			return;
		}*/
		#endif
		#ifdef DELUGE_CLEAR_OUTGOING
		app->outgoingPackets[0] = 0;
		app->outgoingPackets[1] = 1;
		app->outgoingPackets[2] = 2;
		#endif
		stateTransition(app, DELUGE_STATE_MAINTAIN);
		return;
	}

	// wait a while before requesting again
	setRequestTimeout(app, DELUGE_STATE_FORWARD);
}

void deluge_dispatch(deluge_app* app)
{
	/*if (app->wakeup_reason == PACKET_RECEIVED)
		clearTimeout(app);*/
	switch (app->state)
	{
		case DELUGE_STATE_MAINTAIN:
			doMaintain1(app);
			break;
		case DELUGE_STATE_MAINTAIN2:
			doMaintain2(app);
			break;
		case DELUGE_STATE_RX:
			doRx(app);
			break;
		case DELUGE_STATE_TX:
			doTx(app);
			break;
		case DELUGE_STATE_FORWARD:
			doForward(app);
			break;
	}
}

void handleSummary(deluge_app* app, deluge_foot_summary* summary)
{
	mos_mutex_lock(&app->delugeLock);
	checkNeighbor(app, summary->id);
	mos_mutex_unlock(&app->delugeLock);
	
	switch (app->state)
	{
		case DELUGE_STATE_MAINTAIN:
		case DELUGE_STATE_MAINTAIN2: {
			mos_mutex_lock(&app->delugeLock);
			// Nodes that are far away from the base should not suppress us
			//if (summary->dtb < app->dtb)
				app->nSummaries++;
			if (
				#ifndef DELUGE_NO_FORWARD
				app->index == 0 &&
				#endif
				#ifdef DELUGE_SYMMETRIC_LINKS
				checkSymmetry(app, summary) &&
				#endif
				summary->version == app->dcb.version &&
				summary->highPage > app->dcb.highPage
				#ifndef DELUGE_NO_FORWARD
				#ifndef MEMBER_NO_DTB
				&& summary->dtb < app->dtb
				#else
				&& summary->dtb < 255
				#endif
				#endif
				)
			{
				// We ignore summaries from a node that is farther from the root
				// than our last parent.  If this parent ever fails to update
				// us, we set the DTB to the maximum, in which case we no longer
				// ignore summaries.
				app->updaterNode = summary->id;
				// DTB ensures that we get our updates from the closest source
				app->dtb = summary->dtb + 1;
				// Rule M.5
				if (!app->heardRequest && !app->heardData) {
					app->nRequests = 0;
					stateTransition(app, DELUGE_STATE_RX);
				}
				app->detectedInconsistency = 1;
			}
			#ifndef DELUGE_NO_FORWARD
			else if (app->index != 0 &&
				#ifdef DELUGE_SYMMETRIC_LINKS
				checkSymmetry(app, summary) &&
				#endif
				summary->version == app->dcb.version &&
				(summary->highPage > app->dcb.highPage ||
				(summary->highPage == app->dcb.highPage &&
				/*#ifdef DELUGE_LINK_QUALITY
				checkDtb(app, summary->id, summary->dtb)
				#else*/
				summary->dtb < app->dtb
				//#endif
				)))
			{
				// We will send this page index in our own summaries
				app->dcb.highPage = summary->highPage;
				// DTB ensures that we get our updates from the closest source
				// and prevents routing loops from forming
				app->dtb = summary->dtb + 1;
				app->detectedInconsistency = 1;
				// Take note of our new parent, but don't switch to RX state
				app->updaterNode = summary->id;
				// If we have a previously pending request, switch back to
				// FORWARD state
				// Rule M.5
				if (!app->heardRequest && !app->heardData &&
					((app->incomingPackets[0]&app->outgoingPackets[0]) ||
					(app->incomingPackets[1]&app->outgoingPackets[1]) ||
					(app->incomingPackets[2]&app->outgoingPackets[2])))
				{
					app->nRequests = 0;
					stateTransition(app, DELUGE_STATE_FORWARD);
				}
			}
			#endif
			else if (summary->version != app->dcb.version ||
				summary->highPage != app->dcb.highPage ||
				(summary->highPage == app->dcb.highPage &&
				summary->dtb > app->dtb+1))
			{
				app->detectedInconsistency = 1;
				if (summary->version < app->dcb.version)
					app->heardObsolete = 1;
				stateTransition(app, DELUGE_STATE_MAINTAIN);
			}
			mos_mutex_unlock(&app->delugeLock);
		} break;
		case DELUGE_STATE_RX: break;
		case DELUGE_STATE_TX: break;
		case DELUGE_STATE_FORWARD: break;
	}
}

void handleProfile(deluge_app* app, deluge_foot_profile* profile)
{
	mos_mutex_lock(&app->delugeLock);
	checkNeighbor(app, profile->id);
	mos_mutex_unlock(&app->delugeLock);
	
	switch (app->state)
	{
		case DELUGE_STATE_MAINTAIN:
		case DELUGE_STATE_MAINTAIN2: {
			mos_mutex_lock(&app->delugeLock);
			app->nProfiles++;
			if (profile->version > app->dcb.version)
			{
				#ifdef DELUGE_PRINT_EVENT
				printf_P(sNewVer);
				printf_P(sDisplayTime);		// display time
				#endif
				aqshell_send(AQSHELL_NEWVERSION, &profile->version, 1);
				
				app->dcb.codeSize = profile->codeSize;
				app->dcb.version = profile->version;
				app->dcb.incomingPage = 0;
				app->dcb.highPage = 0;
				app->dcb.goalPage = profile->goalPage;
				app->dcb.programcrc = profile->crc;
				app->outgoingPackets[0] = 0;
				app->outgoingPackets[1] = 0;
				app->outgoingPackets[2] = 0;
				app->dtb = 0xFF;
				
				uint8_t i;
				for (i=0; i<sizeof(app->dcb.pagesNeeded); i++)
					app->dcb.pagesNeeded[i] = 0xFF;

				// Find the next page after the last program image we wrote
				if (app->index == 0) {
					app->image_file = mos_file_create("prg", app->dcb.codeSize);
					app->incomingPackets[0] = -1;
					app->incomingPackets[1] = -1;
					app->incomingPackets[2] = -1;
					app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
				} else {
					char name[4] = { 'a', 'q', '0'+app->index, 0 };
					#ifdef DELUGE_NO_FORWARD
					app->image_file = mos_file_create(name, app->dcb.codeSize);
					app->incomingPackets[0] = -1;
					app->incomingPackets[1] = -1;
					app->incomingPackets[2] = -1;
					app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
					#else
					app->image_file = mos_file_create(name, 
						(uint32_t)DELUGE_PAGE_SIZE*(uint32_t)DELUGE_CACHE_PAGES);
					app->incomingPackets[0] = 0;
					app->incomingPackets[1] = 0;
					app->incomingPackets[2] = 0;
					app->nPacketsRequested = 0;
					#endif
					#if DELUGE_CACHE_PAGES > 1
					int8_t i;
					for (i=0; i<DELUGE_CACHE_PAGES; i++)
						app->cache_map[i] = -1;
					app->outgoingCachePage = 0;
					app->incomingCachePage = 0;
					#endif
					app->cachedPackets[0] = 0;
					app->cachedPackets[1] = 0;
					app->cachedPackets[2] = 0;
				}
				deluge_saveState(app);
			}
			mos_mutex_unlock(&app->delugeLock);
		} break;
		case DELUGE_STATE_RX: break;
		case DELUGE_STATE_TX: break;
		case DELUGE_STATE_FORWARD: break;
	}
}

void handleRequest(deluge_app* app, deluge_foot_request* request)
{
	mos_mutex_lock(&app->delugeLock);
	checkNeighbor(app, request->id);
	mos_mutex_unlock(&app->delugeLock);
	
	switch (app->state)
	{
		case DELUGE_STATE_MAINTAIN:
		case DELUGE_STATE_MAINTAIN2: {
			mos_mutex_lock(&app->delugeLock);
			app->detectedInconsistency = 1;
			if (request->version == app->dcb.version &&
				request->page < app->dcb.highPage)
			{
				if (request->to == mos_node_id_get())
				{
					// Rule M.6
					app->outgoingPage = request->page;
					uint8_t ih = mos_disable_ints();
					app->outgoingPackets[0] = request->packets[0];
					app->outgoingPackets[1] = request->packets[1];
					app->outgoingPackets[2] = request->packets[2];
					mos_enable_ints(ih);
					
					app->txDelay += request->rateChange * DELUGE_TX_DELAY_INC;
					if (app->txDelay < DELUGE_TX_DELAY_MIN) 
						app->txDelay = DELUGE_TX_DELAY_MIN;
					else if (app->txDelay > DELUGE_TX_DELAY_MAX)
						app->txDelay = DELUGE_TX_DELAY_MAX;
					
					#ifdef DELUGE_NO_FORWARD
					stateTransition(app, DELUGE_STATE_TX);
					#else
					if (app->index == 0) {
			 			stateTransition(app, DELUGE_STATE_TX);
					} else if (app->dtb != 255) {
						#if DELUGE_CACHE_PAGES > 1
						// Is the page in the cache?
						int8_t page = cachedPageIndex(app, request->page);
						if (page >= 0) {	// Yes
							app->outgoingCachePage = page;
							stateTransition(app, DELUGE_STATE_TX);
							#ifdef DELUGE_KEEP_STATS
							if (deluge_recordstats) packet_counts[DELUGE_STATS_COUNT-2][0]++;		// cache hit
							#endif
							aqshell_send(AQSHELL_CACHEHIT, NULL, 0);
						} else {			// No
							// Have we already partially recieved this page?
							if (request->page != app->dcb.incomingPage) {
								// No, abandon any partially cached page
								app->dcb.incomingPage = request->page;
								app->incomingPackets[0] = 0;
								app->incomingPackets[1] = 0;
								app->incomingPackets[2] = 0;
								app->nPacketsRequested = 0;
								app->cachedPackets[0] = 0;
								app->cachedPackets[1] = 0;
								app->cachedPackets[2] = 0;
								#ifdef DELUGE_KEEP_STATS
								if (deluge_recordstats) packet_counts[DELUGE_STATS_COUNT-2][1]++;		// cache miss
								#endif
								aqshell_send(AQSHELL_CACHEMISS, NULL, 0);
							} else {
								#ifdef DELUGE_KEEP_STATS
								if (deluge_recordstats) packet_counts[DELUGE_STATS_COUNT-2][3]++;		// cache hit past forward
								#endif
								aqshell_send(AQSHELL_CACHEHITOLDFORWARD, NULL, 0);
							}
								
							// Invalidate current cache entry at this location
							app->cache_map[app->incomingCachePage] = -1;
							app->outgoingCachePage = app->incomingCachePage;
							
							/*#ifdef DELUGE_SYMMETRIC_LINKS
							// Make sure the requester isn't in our list of potential parents
							uint8_t i;
							for (i=0; i<DELUGE_NEIGHBOR_COUNT; i++)
								if (request->id==app->neighbors[i])
									app->symdtbs[i] = 255;
							// Find the node with a symmetric link and the best DTB
							if (nextSymmetric(app) != 255) {
								app->nRequests = 0;
								stateTransition(app, DELUGE_STATE_FORWARD);
							}
							#else*/
							app->nRequests = 0;
							stateTransition(app, DELUGE_STATE_FORWARD);
							//#endif
						}
						#else
						// Is this the page we have cached?
						if (request->page != app->dcb.incomingPage) {
							char name[4] = { 'a', 'q', '0'+app->index, 0 };
							// Open a new cache file for wear-balancing
							app->image_file = mos_file_create(name, (uint32_t)DELUGE_PAGE_SIZE);
							app->dcb.incomingPage = request->page;
							app->incomingPackets[0] = 0;
							app->incomingPackets[1] = 0;
							app->incomingPackets[2] = 0;
							app->nPacketsRequested = 0;
							app->cachedPackets[0] = 0;
							app->cachedPackets[1] = 0;
							app->cachedPackets[2] = 0;
						}
						/*#ifdef DELUGE_SYMMETRIC_LINKS
						// Make sure the requester isn't in our list of potential parents
						uint8_t i;
						for (i=0; i<DELUGE_NEIGHBOR_COUNT; i++)
							if (request->id==app->neighbors[i])
								app->symdtbs[i] = 255;
						// Find the node with a symmetric link and the best DTB
						if (nextSymmetric(app) != 255) {
							app->nRequests = 0;
							stateTransition(app, DELUGE_STATE_FORWARD);
						}
						#else*/
						app->nRequests = 0;
						stateTransition(app, DELUGE_STATE_FORWARD);
						//#endif
						#endif
					}
					#endif
				} else {
					// Rule M.5(i)
					app->heardRequest = 2;
				}
			} else {
				// start new round immediately
				stateTransition(app, DELUGE_STATE_MAINTAIN);
			}
			mos_mutex_unlock(&app->delugeLock);
		} break;
		case DELUGE_STATE_RX: {
			clearTimeout(app);
			setRequestTimeout(app, DELUGE_STATE_RX);
		} break;
		case DELUGE_STATE_TX:
		case DELUGE_STATE_FORWARD: {
			mos_mutex_lock(&app->delugeLock);
			if (request->version == app->dcb.version &&
				request->page == app->outgoingPage &&
				request->to == mos_node_id_get())
			{
				// Rule T.1
				uint8_t ih = mos_disable_ints();
				app->outgoingPackets[0] |= request->packets[0];
				app->outgoingPackets[1] |= request->packets[1];
				app->outgoingPackets[2] |= request->packets[2];
				mos_enable_ints(ih);				
				#ifdef DELUGE_FORWARD_REQUESTS_IMMEDIATELY
				if (app->state == DELUGE_STATE_FORWARD) {
					// handle this request immediately
					stateTransition(app, DELUGE_STATE_FORWARD);
					#ifdef DELUGE_KEEP_STATS
					if (deluge_recordstats) packet_counts[DELUGE_STATS_COUNT-2][2]++;		// cache hit forward
					#endif
					aqshell_send(AQSHELL_CACHEHITFORWARD, NULL, 0);
				}
				#endif /*DELUGE_FORWARD_REQUESTS_IMMEDIATELY*/
			} else if (app->state == DELUGE_STATE_FORWARD) {
				clearTimeout(app);
				setRequestTimeout(app, DELUGE_STATE_FORWARD);
			}				
			mos_mutex_unlock(&app->delugeLock);
		} break; 
	} /*switch(app->state)*/
}

void handleData(deluge_app* app, deluge_foot_data* data, uint8_t* packet)
{
	mos_mutex_lock(&app->delugeLock);
	checkNeighbor(app, data->id);
	mos_mutex_unlock(&app->delugeLock);
	
	uint8_t completedPage = 0;
	uint8_t completedPageCached = 0;
	mos_mutex_lock(&app->delugeLock);
	// Read data packets in any state
	if (data->version == app->dcb.version &&
		data->page == app->dcb.incomingPage &&
		(app->incomingPackets[data->packet/16] & (1<<(data->packet&15))))
	{
		#ifdef DELUGE_NO_FORWARD
		int8_t page = app->dcb.incomingPage;
		#elif DELUGE_CACHE_PAGES > 1
		int8_t page = (app->index == 0)? app->dcb.incomingPage : app->incomingCachePage;
		#else
		int8_t page = (app->index == 0)? app->dcb.incomingPage : 0;
		#endif
		uint32_t addr = (uint32_t)data->packet*DELUGE_PACKET_SIZE +
			(uint32_t)page*DELUGE_PAGE_SIZE;

		// Last two bytes of last packet have page CRC
		if (data->packet == DELUGE_PACKETS_PER_PAGE-1) {
			app->incomingcrc = *(uint16_t*)&(packet[DELUGE_PACKET_SIZE-2]);
			mos_file_write(packet, app->image_file, addr, DELUGE_PACKET_SIZE-2);
			
			dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
			dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_PAGE_CRC_ADDR+(app->dcb.incomingPage*2));
			dev_write(DEV_AVR_EEPROM, (uint8_t *)&(app->incomingcrc), 2);
			dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
		} else {
			mos_file_write(packet, app->image_file, addr, DELUGE_PACKET_SIZE);
		}

		app->incomingPackets[data->packet/16] &= ~(1<<(data->packet&15));
		app->cachedPackets[data->packet/16] |= (1<<(data->packet&15));
		if (app->nPacketsRequested > 0) app->nPacketsRequested--;
		//deluge_saveState(app);
		
		if (app->incomingPackets[0] == 0 && app->incomingPackets[1] == 0 && app->incomingPackets[2] == 0)
		{
			if (
				#ifndef DELUGE_NO_FORWARD
				app->index == 0
				#else
				1
				#endif
				)
			{
				page = app->dcb.incomingPage;
				addr = (uint32_t)page*DELUGE_PAGE_SIZE;
		
				uint32_t crc_len = (app->dcb.incomingPage == app->dcb.goalPage-1)?
					app->dcb.codeSize % (uint32_t)DELUGE_PAGE_SIZE :
					(uint32_t)DELUGE_PAGE_SIZE;
				uint16_t crc = mos_file_crc(app->image_file, addr, crc_len);
				if (app->incomingcrc == crc)
				{
					#ifdef DELUGE_PRINT_EVENT
					printf_P(sPageDone, app->dcb.incomingPage);
					printf_P(sDisplayTime);		// display time
					#endif
					aqshell_send(AQSHELL_COMPLETEPAGE, (uint8_t*)data, 2);	// version and page
					
					moveToNextPage(app);
					app->incomingPackets[0] = -1;
					app->incomingPackets[1] = -1;
					app->incomingPackets[2] = -1;
					app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
					completedPage = 1;
					deluge_saveState(app);
									
					if (app->dcb.incomingPage == app->dcb.goalPage)
					{
						//dev_ioctl(DEV_ATMEL_FLASH, DEV_FLUSH);
						crc = mos_file_crc(app->image_file, (uint32_t)0, app->image_file->length);
						if (app->dcb.programcrc == crc)
						{
							#ifdef DELUGE_COMMANDER
							sendFinished(app, &spkt, deluge_portmap[app->index]);
							#endif
							aqshell_send(AQSHELL_COMPLETEPROG, NULL, 0);
						
							#ifndef DELUGE_NO_REPROGRAM
							#ifdef DELUGE_NO_FORWARD
							if (app->index == 0)
							#endif
								// doesn't return
								runReprogram(app);
							#endif
						} 
						#ifdef DELUGE_NO_FORWARD
						else if (app->index != 0)
						{
							printf_P(sProgErr, crc, app->dcb.programcrc);
							// start over from the beginning
							app->dcb.highPage = 0;
							app->dcb.incomingPage = 0;
							app->incomingPackets[0] = -1;
							app->incomingPackets[1] = -1;
							app->incomingPackets[2] = -1;
							app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
							uint8_t i;
							for (i=0; i<sizeof(app->dcb.pagesNeeded); i++)
								app->dcb.pagesNeeded[i] = 0xFF;
							deluge_saveState(app);
						}
						#endif
						else {
							printf_P(sProgErr, crc, app->dcb.programcrc);
							// start over from the first page that fails its CRC
							findBadPages(app);
							app->incomingPackets[0] = -1;
							app->incomingPackets[1] = -1;
							app->incomingPackets[2] = -1;
							app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
							deluge_saveState(app);
						}
					}
				}
				else {
					printf_P(sPageErr, crc, app->incomingcrc);
					// start over
					app->incomingPackets[0] = -1;
					app->incomingPackets[1] = -1;
					app->incomingPackets[2] = -1;
					app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
				}
			}
			#ifndef DELUGE_NO_FORWARD
			else if (app->index != 0)
			{
				completedPage = 1;
				#if DELUGE_CACHE_PAGES > 1
				// If we happened to have downloaded a whole page
				if (app->cachedPackets[0] == -1 && app->cachedPackets[1] == -1 && app->cachedPackets[2] == -1)
				{
					#if DELUGE_CACHE_PAGES > 1
					page = (app->index == 0)? app->dcb.incomingPage : app->incomingCachePage;
					#else
					page = (app->index == 0)? app->dcb.incomingPage : 0;
					#endif
					addr = (uint32_t)page*DELUGE_PAGE_SIZE;
			
					uint32_t crc_len = (app->dcb.incomingPage == app->dcb.goalPage-1)?
						app->dcb.codeSize % (uint32_t)DELUGE_PAGE_SIZE :
						(uint32_t)DELUGE_PAGE_SIZE;
					uint16_t crc = mos_file_crc(app->image_file, addr, crc_len);
					if (app->incomingcrc == crc)
					{
						#ifdef DELUGE_PRINT_EVENT
						printf_P(sPageDone, app->dcb.incomingPage);
						printf_P(sDisplayTime);		// display time
						#endif
						saveIncomingCachePage(app, app->dcb.incomingPage);
						completedPageCached = 1;
					} else {
						printf_P(sPageErr, crc, app->incomingcrc);
						// assume all cached packets are bad
						app->cachedPackets[0] = 0;
						app->cachedPackets[1] = 0;
						app->cachedPackets[2] = 0;
					}
				}
				#endif
				deluge_saveState(app);
			}
			#endif
			
		}
	}
	mos_mutex_unlock(&app->delugeLock);
	
	switch (app->state)
	{
		case DELUGE_STATE_MAINTAIN:
		case DELUGE_STATE_MAINTAIN2: {
			mos_mutex_lock(&app->delugeLock);
			app->detectedInconsistency = 1;
			if (data->version == app->dcb.version &&
				data->page <= app->dcb.highPage)
			{
				// Rule M.5(ii)
				app->heardData = 1;
			}
			// start new round immediately
			stateTransition(app, DELUGE_STATE_MAINTAIN);
			mos_mutex_unlock(&app->delugeLock);
		} break;
		case DELUGE_STATE_RX: {
			if (completedPage)
			{
				// Rule R.3
				// start new round immediately
				stateTransition(app, DELUGE_STATE_MAINTAIN);
			} else {
				clearTimeout(app);
				setRequestTimeout(app, DELUGE_STATE_RX);
			}
		} break;
		case DELUGE_STATE_TX: break;
		case DELUGE_STATE_FORWARD: {
			clearTimeout(app);
			if (completedPage) {
				// finish filling forwarded request
				#if DELUGE_CACHE_PAGES > 1
				if (completedPageCached) {
					// outgoingCachePage is already set
					stateTransition(app, DELUGE_STATE_TX);
				} else
				#endif
					stateTransition(app, DELUGE_STATE_FORWARD);
			} else {
				#ifdef DELUGE_FORWARD_DATA_IMMEDIATELY
				stateTransition(app, DELUGE_STATE_FORWARD);
				#else
				setRequestTimeout(app, DELUGE_STATE_FORWARD);
				#endif /* DELUGE_FORWARD_DATA_IMMEDIATELY */
			}
		} break;
	}
}

/* Public interface functions: */

int8_t deluge_app_staticinit()
{
	srandom(mos_node_id_get());
	return 0;
}

int8_t deluge_app_init(deluge_app* app, int8_t index)
{
	mos_mutex_init(&app->delugeLock);

	app->index = index;
	table[index].app = app;
	loadState(app);

	if (app->index == 0) {
		app->image_file = mos_file_open("prg");

		app->incomingPackets[0] = -1;
		app->incomingPackets[1] = -1;
		app->incomingPackets[2] = -1;
		app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
	} else {
		char name[4] = { 'a', 'q', '0'+app->index, 0 };
		#ifdef DELUGE_NO_FORWARD
		app->image_file = mos_file_create(name, (uint32_t)0);
		app->incomingPackets[0] = -1;
		app->incomingPackets[1] = -1;
		app->incomingPackets[2] = -1;
		app->nPacketsRequested = DELUGE_PACKETS_PER_PAGE;
		#else
		app->image_file = mos_file_create(name,
			(uint32_t)DELUGE_PAGE_SIZE*(uint32_t)DELUGE_CACHE_PAGES);
		app->incomingPackets[0] = 0;
		app->incomingPackets[1] = 0;
		app->incomingPackets[2] = 0;
		app->nPacketsRequested = 0;
		#endif
		#if DELUGE_CACHE_PAGES > 1
		int8_t i;
		for (i=0; i<DELUGE_CACHE_PAGES; i++)
			app->cache_map[i] = -1;
		app->outgoingCachePage = 0;
		app->incomingCachePage = 0;
		app->cacheSize = 4;
		#endif

		app->cachedPackets[0] = 0;
		app->cachedPackets[1] = 0;
		app->cachedPackets[2] = 0;
	}
	
	app->detectedInconsistency = 0;
	app->heardObsolete = 0;
	app->heardRequest = 0;
	app->heardData = 0;
	app->outgoingPackets[0] = 0;
	app->outgoingPackets[1] = 0;
	app->outgoingPackets[2] = 0;
	app->nRequests = 0;
	//table[app->index].roundT = DELUGE_T_L;
	app->txDelay = 0;
	
	#ifdef DELUGE_SYMMETRIC_LINKS
	uint8_t i;
	for (i = 0; i<DELUGE_NEIGHBOR_COUNT; i++) {
		app->neighbors[i] = 0xFFFF;
		//app->symdtbs[i] = 0xFF;
	}
	app->nextNeighbor = 0;
	#endif

	if (app->index == 0 && app->dcb.highPage == app->dcb.goalPage) {
		app->dtb = 0;
	} else {
		app->dtb = 0xFF;
	}
	
	stateTransition(app, DELUGE_STATE_MAINTAIN);
	return 0;
}

#endif
