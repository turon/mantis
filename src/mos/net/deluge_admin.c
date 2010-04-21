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

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include "com.h"
#include "cc1000.h"
#include "deluge_impl.h"
#include "printf.h"
#include "led.h"
#include "net.h"
#include "sem.h"
#include "mutex.h"
#include "cond.h"
#include "reprogram_commands.h"
#include "node_id.h"
#include <string.h>
#include "deluge_msgs.h"

#ifdef DELUGE_COMMANDER

#ifdef DELUGE_KEEP_STATS
extern uint16_t packet_counts[DELUGE_STATS_COUNT][4];
extern uint8_t deluge_recordstats;
#endif

static uint8_t seqNo = 0;

#define SEQ_CACHE_SIZE 6
static uint8_t nextSeq = 0;
static uint16_t idCache[SEQ_CACHE_SIZE];
static uint8_t seqCache[SEQ_CACHE_SIZE];

static uint8_t sendingStats = 0;
static uint8_t statsPort;
static comBuf spkt;

static uint8_t checkCache(uint16_t id, uint8_t seq)
{
	uint8_t i;
	for (i=0; i<SEQ_CACHE_SIZE; i++)
		if (id == idCache[i] && seq == seqCache[i])
			return 1;
	idCache[nextSeq] = id;
	seqCache[nextSeq] = seq;
	nextSeq = (nextSeq + 1) % SEQ_CACHE_SIZE;
	return 0;
}

void logHeader(comBuf* pkt, uint8_t command)
{
	pkt->data[0] = 0xFF;
	pkt->data[1] = 0xFF;
	pkt->data[2] = 0x07;
	
	uint16_t id = mos_node_id_get();
	pkt->data[3] = (uint8_t)(id >> 8);
	pkt->data[4] = (uint8_t)id;
	pkt->data[5] = command;
	pkt->size = 6;
}

void sendFinished(deluge_app* app, comBuf* pkt, uint8_t port)
{
	/*deluge_foot_command* command = (deluge_foot_command*)pkt->data;
	command->seq = ++seqNo;
	command->command = DELUGE_COMMAND_FINISHED;
	command->to = -1;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	pkt->size = sizeof(deluge_foot_command);
	net_send(pkt, DELUGE_PROTO_ID, port, port);*/
	logHeader(&spkt, DELUGE_COMMAND_FINISHED);
	com_send(IFACE_SERIAL, &spkt);
}

static void statsThread()
{
	/*uint8_t* data = (uint8_t*)packet_counts;
	uint16_t i;
	for (i=0; i<sizeof(packet_counts); i += 32) {
		//mos_thread_sleep(1000);
		logHeader(&spkt, DELUGE_COMMAND_SENDSTATS_REPLY);

		memcpy(spkt.data[spkt.size], &data[i], 32);
		spkt.size += 32;
		spkt.data[spkt.size++] = i;
		com_send(IFACE_SERIAL, &spkt);

		deluge_foot_command* command = (deluge_foot_command*)&spkt.data[33];
		command->seq = ++seqNo;
		command->command = DELUGE_COMMAND_SENDSTATS_REPLY;
		command->to = -1;
		command->id = mos_node_id_get();
		command->type = DELUGE_PACKET_COMMAND;
		spkt.size = 33 + sizeof(deluge_foot_command);

		// Temporarily switch to max power to make sure the command gets through
		uint8_t oldPower;
		com_ioctl(IFACE_RADIO, CC1000_GET_TX_POWER, &oldPower);
		com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 0xFF);
		net_send(&spkt, DELUGE_PROTO_ID, statsPort, statsPort);
		com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
	}*/
	/*uint8_t r = 0;
	uint8_t i = 0;
	
	for (r=0; r<DELUGE_STATS_COUNT;)
	{
		logHeader(&spkt, DELUGE_COMMAND_SENDSTATS_REPLY);
		spkt.data[spkt.size++] = r;
		
		for (i=0; i<4 && r+i<DELUGE_STATS_COUNT; i++)
		{
			uint8_t j;
			for (j=0; j<4; j++)
			{
				uint16_t stat = packet_counts[r+i][j];
				spkt.data[spkt.size++] = (uint8_t)(stat >> 8);
				spkt.data[spkt.size++] = (uint8_t)(stat);
			}
		}
		r += i;
		com_send(IFACE_SERIAL, &spkt);
	}
	sendingStats = 0;*/
}

void handleCommand(deluge_app* app, comBuf* pkt, uint8_t port)
{
	deluge_foot_command* command = (deluge_foot_command*)
		&pkt->data[pkt->size - sizeof(deluge_foot_command)];
	#ifdef DELUGE_PRINT_PACKETS
	printf_P(sRecvComm, command->seq, command->command, command->to);
	#endif
	if (checkCache(command->id, command->seq)) return;
	
	// Temporarily switch to max power to make sure the command gets through
	/*uint8_t oldPower;
	com_ioctl(IFACE_RADIO, CC1000_GET_TX_POWER, &oldPower);
	com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 0xFF);*/
	
	// If the command is not specifically addessed to us, forward it now
	/*if (command->to != mos_node_id_get())
		net_send(pkt, DELUGE_PROTO_ID, port, port);*/
	if (command->to != -1 && command->to != mos_node_id_get()) {
		//com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
		return;
	}
	
	//extern mutex epromLock;
	
	switch(command->command) {
		case DELUGE_COMMAND_HELLO: 
			printf_P(sHello);
			logHeader(&spkt, DELUGE_COMMAND_HELLO);
			com_send(IFACE_SERIAL, &spkt);
			break;
		//case DELUGE_COMMAND_SEQNO: seqNo = 0; break;
		/*case DELUGE_COMMAND_STOPALL: 
			//app->dcb.codeSize;
			//app->dcb.programcrc;
			memset(app->dcb.pagesNeeded, 0, sizeof(app->dcb.pagesNeeded));
			//app->dcb.version;
			app->dcb.incomingPage = app->dcb.highPage = app->dcb.goalPage;
			deluge_saveState(app);
			deluge_app_init(app, app->index);
			break;*/
		#ifdef DELUGE_KEEP_STATS
		case DELUGE_COMMAND_CLEARSTATS:
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sClearStats);
			//printf_P(sClearTimer);		// reset timer
			#endif	
			memset(packet_counts, 0, sizeof(packet_counts));
			logHeader(&spkt, DELUGE_COMMAND_CLEARSTATS);
			com_send(IFACE_SERIAL, &spkt);
			break;
		case DELUGE_COMMAND_STARTRECORD:
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sStartRec);
			//printf_P(sClearTimer);		// reset timer
			#endif
			logHeader(&spkt, DELUGE_COMMAND_STARTRECORD);
			com_send(IFACE_SERIAL, &spkt);
			deluge_recordstats = 1;
			break;
		case DELUGE_COMMAND_ENDRECORD: 
			#ifdef DELUGE_PRINT_EVENT
			//printf_P(sDisplayTime);		// display timer
			printf_P(sStopRec);
			#endif
			deluge_recordstats = 0;
			/*mos_mutex_lock(&epromLock);
			dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_STATS_ADDR);
			dev_write(DEV_AVR_EEPROM, (uint8_t*)packet_counts, sizeof(packet_counts));
			mos_mutex_unlock(&epromLock);*/
			logHeader(&spkt, DELUGE_COMMAND_ENDRECORD);
			com_send(IFACE_SERIAL, &spkt);
			break;
		case DELUGE_COMMAND_SENDSTATS:
			if (-1 != command->to && !sendingStats) {	// it's addressed to us
				sendingStats = 1;
				statsPort = port;
				// This is a slow procedure that requires multiple packets to
				// reply, so run it in another thread.
				//mos_thread_new(statsThread, 256, PRIORITY_NORMAL);

				uint8_t r = 0;
				uint8_t i = 0;
				
				for (r=0; r<DELUGE_STATS_COUNT;)
				{
					logHeader(&spkt, DELUGE_COMMAND_SENDSTATS_REPLY);
					spkt.data[spkt.size++] = r;
					
					for (i=0; i<4 && r+i<DELUGE_STATS_COUNT; i++)
					{
						uint8_t j;
						for (j=0; j<4; j++)
						{
							uint16_t stat = packet_counts[r+i][j];
							spkt.data[spkt.size++] = (uint8_t)(stat >> 8);
							spkt.data[spkt.size++] = (uint8_t)(stat);
						}
					}
					r += i;
					com_send(IFACE_SERIAL, &spkt);
				}
				sendingStats = 0;
			}
			break;
		#endif
		case DELUGE_COMMAND_VERSION:
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sChangeVer, pkt->data[0]);
			printf_P(sClearTimer);		// clear timer
			#endif
			mos_mutex_lock(&app->delugeLock);	
			app->dcb.version = pkt->data[0];
			deluge_saveState(app);
			app->detectedInconsistency = 1;
			stateTransition(app, DELUGE_STATE_MAINTAIN);
			mos_mutex_unlock(&app->delugeLock);	
			if (command->to != -1) {
				/*command->seq = ++seqNo;
				command->command = DELUGE_COMMAND_VERSION_REPLY;
				command->to = command->id;
				command->id = mos_node_id_get();
				command->type = DELUGE_PACKET_COMMAND;
				mos_thread_sleep(1000);
				net_send(pkt, DELUGE_PROTO_ID, port, port);*/
				logHeader(&spkt, DELUGE_COMMAND_VERSION_REPLY);
				com_send(IFACE_SERIAL, &spkt);
			}
			break;
		#ifdef DELUGE_SYMMETRIC_LINKS
		/*case DELUGE_COMMAND_NEIGHBORS:
			if (-1 != command->to) {	// it's addressed to us
				memcpy(pkt->data, app->neighbors, sizeof(app->neighbors));
				pkt->size = sizeof(app->neighbors);
				//memcpy(&pkt->data[pkt->size], app->symdtbs, sizeof(app->symdtbs));
				//pkt->size += sizeof(app->symdtbs);

				command = (deluge_foot_command*)&pkt->data[pkt->size];
				command->seq = ++seqNo;
				command->command = DELUGE_COMMAND_NEIGHBORS_REPLY;
				command->to = -1;
				command->id = mos_node_id_get();
				command->type = DELUGE_PACKET_COMMAND;
				pkt->size += sizeof(deluge_foot_command);
				mos_thread_sleep(1000);
				net_send(pkt, DELUGE_PROTO_ID, port, port);
			}
			break;*/
		#endif
		/*case DELUGE_COMMAND_WIPE: {
			com_mode(IFACE_RADIO, IF_OFF);
			deluge_suspend();
			
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sWipe);
			#endif
			uint16_t zero = 0;
			uint16_t addr;
			mos_mutex_lock(&epromLock);
			for (addr = DELUGE_CONTROL_BLOCK_ADDR; addr < SIMPLE_FS_ADDR; addr += 2)
			{
				dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, addr);
				dev_write(DEV_AVR_EEPROM, (uint8_t*)&zero, 2);
			}
			simple_fs_format();
			
			uint8_t default_portmap[DELUGE_INSTANCE_COUNT];
			uint8_t i;
			for (i=0; i<DELUGE_INSTANCE_COUNT; i++)
				default_portmap[i] = i+1;
			dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_DIRECTORY_ADDR);
			dev_write(DEV_AVR_EEPROM, default_portmap, sizeof (default_portmap));
			mos_mutex_unlock(&epromLock);
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sDone);
			#endif
			
			deluge_init();
			
			com_mode(IFACE_RADIO, IF_LISTEN);
			break;
		}*/
		case DELUGE_COMMAND_BOOT: {
			mos_mutex_lock(&app->delugeLock);
			com_mode(IFACE_RADIO, IF_OFF);
			deluge_suspend();
			
			if (pkt->data[0]==0) {
				// Just reboot
				reboot();
				break;
			}
			
			// Open the requested backup image
			char name[4] = { 'b', 'k', '0'+pkt->data[0], 0 };
			mos_file* file = mos_file_open(name);
			if (!file) {
				printf_P(sFileErr, name);
				
				deluge_resume();
				com_mode(IFACE_RADIO, IF_LISTEN);
				break;
			}
			
			// We keep our current state, but we boot the backup image
			deluge_saveState(app);
			
			// Reboot.  This function will copy the file location into the 
			// boot control block for reprogramming
			app->image_file = file;
			runReprogram(app);
			// We shouldn't return from runReprogram
			mos_mutex_unlock(&app->delugeLock);
			break;
		}
		#if DELUGE_CACHE_PAGES > 1
		case DELUGE_COMMAND_CACHESIZE: {
			if (pkt->data[0] >= DELUGE_CACHE_PAGES) break;
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sChangeCache, pkt->data[0]);
			#endif
			mos_mutex_lock(&app->delugeLock);
			app->cacheSize = pkt->data[0];
			int8_t i;
			for (i=0; i<DELUGE_CACHE_PAGES; i++)
				app->cache_map[i] = -1;
			app->outgoingCachePage = 0;
			app->incomingCachePage = 0;
			stateTransition(app, DELUGE_STATE_MAINTAIN);
			mos_mutex_unlock(&app->delugeLock);
			break;
		}
		#endif
		case DELUGE_COMMAND_POWER: {
			#ifdef DELUGE_PRINT_EVENT
			printf_P(sChangeTx, pkt->data[0]);
			#endif
			//oldPower = pkt->data[0];	// Will get changed at the end
			com_ioctl(IFACE_RADIO, CC1000_TX_POWER, pkt->data[0]);
			break;
		}
		default:
			break;
	}
	//com_ioctl(IFACE_RADIO, CC1000_TX_POWER, oldPower);
}
#endif
#endif
