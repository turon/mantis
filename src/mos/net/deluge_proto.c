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
#include "deluge_impl.h"
#include "printf.h"
#include "led.h"
#include "net.h"
#include "sem.h"
#include "mutex.h"
#include "rwlock.h"
#include "cond.h"
#include "reprogram_commands.h"
#include "node_id.h"
#include <string.h>
#include "deluge_msgs.h"
#include "aqueduct_shell.h"

#define debugLeds	0

#ifdef DELUGE_KEEP_STATS
// log packets for each id (0 through DELUGE_STATS_COUNT-2,
// id DELUGE_STATS_COUNT-1 for anything greater) and type
uint16_t packet_counts[DELUGE_STATS_COUNT][4];
uint8_t deluge_recordstats = 0;
#endif

mos_sem_t dispatch_sem;
mos_cond_t suspend_cond;
mos_mutex_t suspend_lock;
mos_rwlock_t applist_lock;
static volatile uint8_t suspended = 0;
//static uint8_t dropped;
static uint8_t deluge_stack[192];

uint8_t deluge_portmap[DELUGE_INSTANCE_COUNT];
deluge_entry table[DELUGE_INSTANCE_COUNT];

#define N_LOADED 2
// We need to hold a read lock while accessing an element of this list
// and a write lock when modifying the list
deluge_app apps[N_LOADED];
static uint8_t next_app_idx = 0;

static deluge_app* last_app;
static uint8_t next_disp_inst = 0;
static uint8_t next_recv_inst = 0;

// Alarms call this function
void deluge_wakeup(deluge_entry* entry)
{
	//mos_sem_post(&entry->app_sem);
	entry->copyState = 1;
	mos_sem_post(&dispatch_sem);
}

#ifdef DELUGE_KEEP_STATS
void deluge_saveStats()
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_STATS_ADDR);
	dev_write(DEV_AVR_EEPROM, (uint8_t*)packet_counts, sizeof(packet_counts));
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
}
#endif

static deluge_app* loadApp(int8_t index)
{
	//printf("loading %C\n", index);
	// The caller needs to hold a write lock on applist_lock, 
	// so we don't unload an application's entry while it is being used
	if (apps[next_app_idx].index != -1) {
		deluge_saveState(&apps[next_app_idx]);
		table[apps[next_app_idx].index].app = NULL;
	}
	deluge_app* app = &apps[next_app_idx];
	next_app_idx = (next_app_idx + 1) % N_LOADED;
	
	deluge_app_init(app, index);

	return app;
}

static void dispatchThread()
{
	while(1)
	{
		// Wait for something to happen (timeout or a change of state)
		mos_sem_wait(&dispatch_sem);
		// Check if we are suspended
		mos_mutex_lock(&suspend_lock);
		while (suspended) mos_cond_wait(&suspend_cond, &suspend_lock);
		mos_mutex_unlock(&suspend_lock);
		//printf("dt1\n");

		// Grab a read lock on the application list
		rwlock_rdlock(&applist_lock);
		mos_led_toggle(1);
		// Quickly check the last application we used, it's most likely to be
		// the one with the event
		deluge_app* app = last_app;
		//if (SEM_SUCCESS == mos_sem_try_wait(&table[last_app->index].app_sem)) {
		/*if (table[app->index].nextState != DELUGE_STATE_IGNORE) {
			//printf("%C -> %C\n", last_app->state, table[last_app->index].nextState);
			app->state = table[app->index].nextState;
			table[app->index].nextState = DELUGE_STATE_IGNORE;
			deluge_dispatch(app);
			rwlock_unlock(&applist_lock);
			continue;
		}*/
		
		// Check everybody else
		uint8_t appid;
		uint8_t i;
		for (i=0; i<DELUGE_INSTANCE_COUNT; i++) {
			appid = (next_disp_inst + i) % DELUGE_INSTANCE_COUNT;
			//if (SEM_SUCCESS == mos_sem_try_wait(&table[appid].app_sem))
			if (table[appid].copyState && table[appid].nextState != DELUGE_STATE_IGNORE)
				break;
		}
		if (i == DELUGE_INSTANCE_COUNT) {
			rwlock_unlock(&applist_lock);
			continue;
		}

		// See if the Deluge instance is already loaded	
		while (!table[appid].app) 
		{
			// Unlock the read lock and grab the write lock
			rwlock_unlock(&applist_lock);
			rwlock_wrlock(&applist_lock);
			// Load the Deluge instance
			loadApp(appid);
			// Unlock the write lock and grab the read lock
			rwlock_unlock(&applist_lock);
			rwlock_rdlock(&applist_lock);
			// Between the time we released the write lock and when we got the
			// read lock, someone else could have unloaded our instance,
			// so we loop and check that it's still loaded
		}
		last_app = app = table[appid].app;
		next_disp_inst = (appid + 1) % DELUGE_INSTANCE_COUNT;

		//printf("%C -> %C\n", app->state, table[appid].nextState);
		app->state = table[appid].nextState;
		table[appid].nextState = DELUGE_STATE_IGNORE;
		table[appid].copyState = 0;
		
		//printf("dispatch %C\n", appid);
		deluge_dispatch(app);
		rwlock_unlock(&applist_lock);
	}
}

/* Net device interface functions */

int8_t deluge_send(comBuf *pkt, va_list args)
{
	if (debugLeds) mos_led_toggle(0);
	uint8_t port = va_arg(args, int);
	uint8_t type = pkt->data[pkt->size - 1];
	uint16_t sender = *(uint16_t*)&pkt->data[pkt->size - 3];
	
	#ifdef DELUGE_KEEP_STATS
	if (deluge_recordstats && type<5)
		packet_counts[sender<DELUGE_STATS_COUNT-1?sender:DELUGE_STATS_COUNT-1][type-1]++;
	#endif
	
	#ifdef DELUGE_PRINT_PACKETS
	printf_P(sPktPortType, port, sender, type);
	switch (type)
	{
		case DELUGE_PACKET_SUMMARY: {
			deluge_foot_summary* summary = (deluge_foot_summary*)pkt->data;
			#ifdef DELUGE_SYMMETRIC_LINKS
			#if DELUGE_NEIGHBOR_COUNT != 5
			#error This code needs to be updated
			#endif
			printf_P(sSendNList,
				summary->neighbors[0], summary->neighbors[1], 
				summary->neighbors[2], summary->neighbors[3], 
				summary->neighbors[4]);
			#endif
			printf_P(sSendSumm, summary->version, summary->highPage, summary->dtb);
		} break;
		case DELUGE_PACKET_PROFILE: {
			deluge_foot_profile* profile = (deluge_foot_profile*)pkt->data;
			printf_P(sSendProf, profile->codeSize, profile->crc, profile->version, profile->goalPage);
		} break;
		case DELUGE_PACKET_REQUEST: {
			deluge_foot_request* request = (deluge_foot_request*)pkt->data;
			printf_P(sSendReq, request->packets[2], request->packets[1], request->packets[0], request->to, request->version, request->page, request->rateChange);
		} break;
		case DELUGE_PACKET_DATA: {
			deluge_foot_data* data = (deluge_foot_data*)&pkt->data[DELUGE_PACKET_SIZE];
			printf_P(sSendData, data->version, data->page, data->packet);
		} break;
		#ifdef DELUGE_COMMANDER
		case DELUGE_PACKET_COMMAND: {
			deluge_foot_command* command = (deluge_foot_command*)&pkt->data[pkt->size-sizeof(deluge_foot_command)];
			printf_P(sSendComm, command->seq, command->command, command->to);
		} break;
		#endif
	}
	#endif
	
	#ifdef AQUEDUCT_SHELL
	uint8_t len;
	switch (type)
	{
		case DELUGE_PACKET_SUMMARY:
			len = sizeof(deluge_foot_summary);
			pkt->data[len] = port;
			aqshell_send(AQSHELL_SUMMARY_SEND, pkt->data, len+1);
			break;
		case DELUGE_PACKET_PROFILE:
			len = sizeof(deluge_foot_profile);
			pkt->data[len] = port;
			aqshell_send(AQSHELL_PROFILE_SEND, pkt->data, len+1);
			break;
		case DELUGE_PACKET_REQUEST:
			len = sizeof(deluge_foot_request);
			pkt->data[len] = port;
			aqshell_send(AQSHELL_REQUEST_SEND, pkt->data, len+1);
			break;
		case DELUGE_PACKET_DATA:
			len = sizeof(deluge_foot_data);
			pkt->data[DELUGE_PACKET_SIZE+len] = port;
			aqshell_send(AQSHELL_DATA_SEND, &pkt->data[DELUGE_PACKET_SIZE], len+1);
			break;
	}
	#endif
	
	return 0;
}

boolean deluge_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
	if (suspended) {
		//dropped++;
		return 0;
	}
	//printf("%d\n", __LINE__);

	uint16_t sender = *(uint16_t*)&pkt->data[pkt->size - 3];
	#ifdef DELUGE_FORCE_MULTIHOP
	uint16_t me = mos_node_id_get();
	if (sender != me+1 && sender != me-1) return 0;
	#endif
	//printf("%d\n", __LINE__);
	
	// Grab a read lock on the application list
	rwlock_rdlock(&applist_lock);
	mos_led_toggle(2);
	// Quickly check the last application we used, it's most likely to be
	// the one using this port
	deluge_app* app = last_app;
	//if (deluge_portmap[app->index] != port)
	//{
		// Find Deluge entry for this port
		uint8_t appid;
		uint8_t i;
		for (i=0; i<DELUGE_INSTANCE_COUNT; i++) {
			appid = (next_recv_inst + i) % DELUGE_INSTANCE_COUNT;
			if (deluge_portmap[appid] == port)
				break;
		}
	//printf("%d\n", __LINE__);
		if (i == DELUGE_INSTANCE_COUNT) {
			rwlock_unlock(&applist_lock);
			return 0;
		}
	//printf("%d\n", __LINE__);
	
		// See if the Deluge instance is already loaded		
		while (!table[appid].app) 
		{
			// Unlock the read lock and grab the write lock
			rwlock_unlock(&applist_lock);
			rwlock_wrlock(&applist_lock);
			// Load the Deluge instance
			loadApp(appid);
			// Unlock the write lock and grab the read lock
			rwlock_unlock(&applist_lock);
			rwlock_rdlock(&applist_lock);
			// Between the time we released the write lock and when we got the
			// read lock, someone else could have unloaded our instance,
			// so we loop and check that it's still loaded
		}
		last_app = app = table[appid].app;
		//printf("recv %C\n", appid);
		next_recv_inst = (appid + 1) % DELUGE_INSTANCE_COUNT;
	//}
	//printf("%d\n", __LINE__);
	
	uint8_t type = pkt->data[pkt->size - 1];
	#ifdef DELUGE_KEEP_STATS
	if (deluge_recordstats && type<5 && (type != DELUGE_PACKET_REQUEST ||
		(type==DELUGE_PACKET_REQUEST && 
		((deluge_foot_request*)pkt->data)->to == mos_node_id_get())))
		packet_counts[sender<DELUGE_STATS_COUNT-1?sender:DELUGE_STATS_COUNT-1][type-1]++;
	#endif
	//printf("%d\n", __LINE__);
	
	#ifdef DELUGE_PRINT_PACKETS
	printf_P(sPktPortType, port, sender, type);
	#endif
	//printf("%d\n", __LINE__);
	
	switch (type)
	{
		case DELUGE_PACKET_SUMMARY: {
	//printf("%d\n", __LINE__);
			deluge_foot_summary* summary = (deluge_foot_summary*)pkt->data;
			#ifdef DELUGE_PRINT_PACKETS
			#ifdef DELUGE_SYMMETRIC_LINKS
			#if DELUGE_NEIGHBOR_COUNT != 5
			#error This code needs to be updated
			#endif
			printf_P(sRecvNList,
				summary->neighbors[0], summary->neighbors[1], 
				summary->neighbors[2], summary->neighbors[3], 
				summary->neighbors[4]);
			#endif
			printf_P(sRecvSumm, summary->version,
				summary->highPage, summary->dtb);
			#endif
			aqshell_send(AQSHELL_SUMMARY, pkt->data, sizeof(deluge_foot_summary)+1);
			handleSummary(app, summary);
		} break;
		case DELUGE_PACKET_PROFILE: {
	//printf("%d\n", __LINE__);
			deluge_foot_profile* profile = (deluge_foot_profile*)pkt->data;
			#ifdef DELUGE_PRINT_PACKETS
			printf_P(sRecvProf, profile->codeSize, profile->crc, 
				profile->version, profile->goalPage);
			#endif
			aqshell_send(AQSHELL_PROFILE, pkt->data, sizeof(deluge_foot_profile)+1);
			handleProfile(app, profile);
		} break;
		case DELUGE_PACKET_REQUEST: {
	//printf("%d\n", __LINE__);
			deluge_foot_request* request = (deluge_foot_request*)pkt->data;
			#ifdef DELUGE_PRINT_PACKETS
			printf_P(sRecvReq, request->packets[2], 
				request->packets[1], request->packets[0], request->to, 
				request->version, request->page, request->rateChange);
			#endif
			aqshell_send(AQSHELL_REQUEST, pkt->data, sizeof(deluge_foot_request)+1);
			handleRequest(app, request);
		} break;
		case DELUGE_PACKET_DATA: {
	//printf("%d\n", __LINE__);
			deluge_foot_data* data = (deluge_foot_data*)&pkt->data[DELUGE_PACKET_SIZE];
			#ifdef DELUGE_PRINT_PACKETS
			printf_P(sRecvData,
				data->version, data->page, data->packet);
			#endif
			aqshell_send(AQSHELL_DATA, (uint8_t*)data, sizeof(deluge_foot_data)+1);
			handleData(app, data, pkt->data);
		} break;
		#ifdef DELUGE_COMMANDER
		case DELUGE_PACKET_COMMAND:
	//printf("%d\n", __LINE__);
			handleCommand(app, pkt, port);
			break;
		#endif
	}
	rwlock_unlock(&applist_lock);
	return 0;
}

int8_t deluge_ioctl(uint8_t request, va_list args)
{
	// TODO maybe this should be the way other applications control Deluge
	// and adjust its settings
	return -1;
}

/* Public interface functions: */

void deluge_suspend()
{
	mos_mutex_lock(&suspend_lock);
	suspended = 1;
	//dropped = 0;
	mos_mutex_unlock(&suspend_lock);
	//mos_sem_post(&dispatch_sem);
	// Give dispatch thread a chance to finish what it's doing
	mos_thread_sleep(250);
}

void deluge_resume()
{
	// May need to reopen image file for home application
	loadApp(0);
	
	mos_mutex_lock(&suspend_lock);
	suspended = 0;
	mos_cond_broadcast(&suspend_cond);
	mos_mutex_unlock(&suspend_lock);
}

int8_t deluge_init()
{
	aqshell_init();
	#ifdef DELUGE_PRINT_EVENT
	printf_P(sInitDeluge);
	printf_P(sDisplayTime);		// display time
	#endif
	aqshell_send(AQSHELL_START, NULL, 0);
	
	mos_sem_init(&dispatch_sem, 0);
	mos_cond_init(&suspend_cond);
	mos_mutex_init(&suspend_lock);
	rwlock_init(&applist_lock);
	suspended = 0;

	// Load stats that were saved before rebooting
	#ifdef DELUGE_KEEP_STATS
	deluge_recordstats = 0;
	memset(packet_counts, 0, sizeof(packet_counts));
	//dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	//dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_STATS_ADDR);
	//dev_read(DEV_AVR_EEPROM, (uint8_t*)packet_counts, sizeof(packet_counts));
	//dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);
	#endif

	// Load port map
	dev_ioctl(DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_DIRECTORY_ADDR);
	dev_read (DEV_AVR_EEPROM, deluge_portmap, sizeof (deluge_portmap));
	dev_ioctl(DEV_AVR_EEPROM, DEV_UNLOCK);

	// Initialize table of applications
	uint8_t i;
	for (i=0; i<DELUGE_INSTANCE_COUNT; i++) {
		table[i].app = NULL;
		//mos_sem_init(&table[i].app_sem, 0);
		table[i].copyState = 0;
		table[i].nextState = DELUGE_STATE_IGNORE;
		table[i].alarm.func = (void(*)(void*))deluge_wakeup;
		table[i].alarm.data = &table[i];
		table[i].roundT = DELUGE_T_L;
	}

	deluge_app_staticinit();
	// Initialized table of loaded Deluge instances
	for (i=0; i<N_LOADED; i++) {
		apps[i].index = -1;
	}
	next_app_idx = 0;
	next_disp_inst = 0;
	next_recv_inst = 0;
	// Go ahead and load the home Deluge instance
	last_app = loadApp(0);

	net_proto_register(DELUGE_PROTO_ID, deluge_send, deluge_recv, deluge_ioctl);
	mos_thread_new_havestack(dispatchThread, 192, deluge_stack, PRIORITY_NORMAL);
	
	return 0;
}

#endif
