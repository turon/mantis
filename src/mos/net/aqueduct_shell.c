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

/* Implements a special shell for running Aqueduct experiments.
 * See also src/apps/deluge_test/aqshell.c and src/apps/deluge_test/ExperServer.java
 */

#include "mos.h"

#include "aqueduct_shell.h"

#ifdef AQUEDUCT_SHELL
#ifdef ARCH_AVR

#undef DELUGE_PRINT_PACKETS
#undef DELUGE_PRINT_EVENT

#include <string.h>
#include "com.h"
#include "node_id.h"
#include "printf.h"
#include "deluge_impl.h"
#include "led.h"
#include "plat_dep.h"


/*#ifdef DELUGE_KEEP_STATS
extern uint16_t packet_counts[DELUGE_STATS_COUNT][4];
extern uint8_t deluge_recordstats;
#endif*/
extern uint8_t deluge_portmap[DELUGE_INSTANCE_COUNT];
extern deluge_entry table[DELUGE_INSTANCE_COUNT];

const char sWrongType[] ARCH_PROGMEM = "#Illegal packet type for node";   //: %x\n";
const char sUnkType[] ARCH_PROGMEM = "#Unknown packet type";  //: %x\n";

static comBuf spkt;
static mos_mutex_t sendMutex;
/*static struct {
	struct aqshell_header head;
	uint8_t data[COM_DATA_SIZE - sizeof(struct aqshell_header)];
} aqpkt;*/

static void recvThread()
{
	com_mode(IFACE_SERIAL, IF_LISTEN);
	while (1)
	{
		comBuf* cpkt = com_recv(IFACE_SERIAL);
		mos_led_toggle(0);
		struct aqshell_pkt* pkt = (struct aqshell_pkt*)cpkt->data;
		
		switch (pkt->head.command)
		{
			#if DELUGE_CACHE_PAGES > 1
			case AQSHELL_SETCACHESIZE: {
				if (pkt->data[0] >= DELUGE_CACHE_PAGES) break;
				if (pkt->data[1] >= DELUGE_INSTANCE_COUNT) break;
				deluge_app* app = table[pkt->data[1]].app;
				if (!app) break;

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
			#else
			case AQSHELL_SETCACHESIZE:
				break;
			#endif
			case AQSHELL_SETIMAGESIZE: {
				deluge_app* app = table[0].app;
				if (!app) break;
				
				mos_mutex_lock(&app->delugeLock);
				app->image_file = mos_file_create("prg", 
					pkt->data[0]*(uint32_t)DELUGE_PAGE_SIZE);	
				app->dcb.version = pkt->data[1];
				app->dcb.programcrc = mos_file_crc(app->image_file, 
					(uint32_t)0, app->image_file->length);
				app->dcb.goalPage = app->dcb.incomingPage = app->dcb.highPage = pkt->data[0];
				app->dcb.codeSize = app->image_file->length;
				uint8_t i;
				for (i=0; i<sizeof(app->dcb.pagesNeeded); i++)
					app->dcb.pagesNeeded[i] = 0;

				deluge_saveState(app);
				app->detectedInconsistency = 1;
				stateTransition(app, DELUGE_STATE_MAINTAIN);
				mos_mutex_unlock(&app->delugeLock);	
				break;
			}
			case AQSHELL_SETVERSION: {
				deluge_app* app = table[0].app;
				if (!app) break;
				
				mos_mutex_lock(&app->delugeLock);	
				app->dcb.version = pkt->data[0];
                pkt->data[1] = app->dcb.highPage;       // for reply
				deluge_saveState(app);
				app->detectedInconsistency = 1;
				stateTransition(app, DELUGE_STATE_MAINTAIN);
				mos_mutex_unlock(&app->delugeLock);
				pkt->head.length = 2;   // for reply
				break;
			}
			case AQSHELL_STOPUPDATE: {
				if (pkt->data[0] >= DELUGE_INSTANCE_COUNT) break;
				deluge_app* app = table[pkt->data[0]].app;
				if (!app) break;
				
				mos_mutex_lock(&app->delugeLock);	
				pkt->data[0] = app->dcb.version;
				pkt->data[1] = app->dcb.incomingPage = app->dcb.highPage = app->dcb.goalPage;
				uint8_t i;
				for (i=0; i<sizeof(app->dcb.pagesNeeded); i++)
					app->dcb.pagesNeeded[i] = 0;
				deluge_saveState(app);
				stateTransition(app, DELUGE_STATE_MAINTAIN);
				mos_mutex_unlock(&app->delugeLock);
				pkt->head.length = 2;   // for reply
				break;
			}
			case AQSHELL_GETVERSION: {
				deluge_app* app = table[0].app;
				if (!app) break;
				
				mos_mutex_lock(&app->delugeLock);
				pkt->data[0] = app->dcb.version;
				pkt->data[1] = app->dcb.highPage;
				mos_mutex_unlock(&app->delugeLock);
				printf("#returning version %C, size %C\n", pkt->data[0], pkt->data[1]);
				
				pkt->head.flags |= AQSHELL_F_PLEASEACK;
				pkt->head.length = 2;
				break;
			}
			case AQSHELL_GETID:
			case AQSHELL_NOOP:
				/*pkt->data[0] = AQSHELL_GETID_REPLY;
				*(uint16_t*)&pkt->data[1] = mos_node_id_get();
				pkt->size = 3;
				com_send(IFACE_SERIAL, pkt);*/
				break;
			case AQSHELL_MESSAGE:
			case AQSHELL_START:
			case AQSHELL_NEWVERSION:
			case AQSHELL_COMPLETEPAGE:
			case AQSHELL_COMPLETEPROG:
			case AQSHELL_GETSTATS_REPLY:
			case AQSHELL_SETVERSION_REPLY:
			case AQSHELL_SUMMARY:
			case AQSHELL_PROFILE:
			case AQSHELL_REQUEST:
			case AQSHELL_DATA:
			case AQSHELL_GETID_REPLY:
			case AQSHELL_SUMMARY_SEND:
			case AQSHELL_PROFILE_SEND:
			case AQSHELL_REQUEST_SEND:
			case AQSHELL_DATA_SEND:
				printf_P(sWrongType, pkt->head.command);
				break;
			default:
				printf_P(sUnkType, pkt->head.command);
				break;
		}
		
		if (pkt->head.flags & AQSHELL_F_PLEASEACK) {
			pkt->head.flags |= AQSHELL_F_ACK;
			pkt->head.id = mos_node_id_get();
			cpkt->size = sizeof(struct aqshell_header) + pkt->head.length;
			com_send(IFACE_SERIAL, cpkt);
		}
		com_free_buf(cpkt);
	}
}

void aqshell_init()
{
	mos_mutex_init(&sendMutex);
	mos_thread_new(recvThread, 256, PRIORITY_NORMAL);
}

void aqshell_send(uint8_t type, uint8_t* data, uint8_t len)
{
	if (len > AQSHELL_DATA_SIZE)
		len = AQSHELL_DATA_SIZE;

	mos_mutex_lock(&sendMutex);		
	struct aqshell_pkt* apkt = (struct aqshell_pkt*)spkt.data;
	apkt->head.command = type;
	apkt->head.flags = 0;
	apkt->head.id = mos_node_id_get();
	apkt->head.length = 0;
	spkt.size = sizeof(struct aqshell_header);
	
	if (data && len) {
		memcpy(apkt->data, data, len);
		apkt->head.length = len;
		spkt.size += len;
	}
	com_send(IFACE_SERIAL, &spkt);
	mos_mutex_unlock(&sendMutex);		
}

#endif // ARCH_AVR
#endif /*AQUEDUCT_SHELL*/
