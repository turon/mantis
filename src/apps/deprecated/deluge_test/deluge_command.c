//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "command_daemon.h"
#include "deluge_impl.h"
#include "boot.h"
#include "dev.h"
#include "com.h"
#include "net.h"
#include "printf.h"
#include "reprogram_commands.h"
#include "simple_fs.h"
#include "node_id.h"
#include <string.h>
#include <avr/pgmspace.h>
#include "sem.h"
#include "mutex.h"
#include "cond.h"
#include <stdlib.h>
#include "mem.h"
#include "plat_dep.h"

const char sNode[] ARCH_PROGMEM = "node (-1 for all nodes): ";
const char sPort[] ARCH_PROGMEM = "port: ";
const char sVersion[] ARCH_PROGMEM = "version: ";
const char sHigh[] ARCH_PROGMEM = "high page: ";
const char sBoot[] ARCH_PROGMEM = "Choose the image to boot:\n"
		"\t0:\tDon't load an image, just reboot.\n"
		"\t1 and up:\tLoad the backup image that has this index.\n"
		": ";
const char sCache[] ARCH_PROGMEM = "New cache size: ";
const char sPower[] ARCH_PROGMEM = "New transmit power: ";
const char sLogOn[] ARCH_PROGMEM = "\xFF\xFF\4";
const char sSTitle[] ARCH_PROGMEM = "Stats %d\n";
const char sSHead[] ARCH_PROGMEM = " Node Summary Profile Request Data Total\n";
const char sSBar[] ARCH_PROGMEM = "----- ------- ------- ------- ---- -----\n";
const char sSRow[] ARCH_PROGMEM = "%5C %7d %7d %7d %4d %5d\n";
const char sSTotal[] ARCH_PROGMEM = "Total %7d %7d %7d %4d %5d\n";
const char sFinish[] ARCH_PROGMEM = "Finished %d\n";
const char sShowTime[] ARCH_PROGMEM = "\xFF\xFF\6";
const char sLogOff[] ARCH_PROGMEM = "\xFF\xFF\5";
const char sVerAck[] ARCH_PROGMEM = "Version acknowledge %d\n";
const char sNTitle[] ARCH_PROGMEM = "\nNeighbors %d:\n";
const char sNHead[] ARCH_PROGMEM = "Node  DTB\n";
const char sNBar[] ARCH_PROGMEM = "----- ---\n";
const char sNRow[] ARCH_PROGMEM = "%5d n/a\n";
const char sSeed[] ARCH_PROGMEM = "seed node: ";
const char sReceiver[] ARCH_PROGMEM = "receiving node: ";
const char sNNodes[] ARCH_PROGMEM = "number of nodes in the network: ";
const char sWaitOne[] ARCH_PROGMEM = "waiting until a node gets reprogrammed...\n";
const char sQuery[] ARCH_PROGMEM = "querying node %d\n";
const char sWaitLeft[] ARCH_PROGMEM = "waiting till the nodes on the left edge get reprogrammed...\n";
const char sRadioCounts[] ARCH_PROGMEM = "success: %d, mem: %d, sync: %d, size: %d, crc: %d";
const char sFecCount[] ARCH_PROGMEM = ", fec: %d";
const char sSendVersion[] ARCH_PROGMEM = "updating version to %C on node %d\n";

static uint8_t seq;
static comBuf spkt;
static uint8_t giveUp;

static uint16_t statsNode;
static uint8_t statsPort;
static uint16_t packet_counts[DELUGE_STATS_COUNT][4];
#define N_STATS_PACKETS ((sizeof(packet_counts)+31)/32)
static uint8_t received[N_STATS_PACKETS];
static mos_cond_t waitCond;
static mutex waitMutex;

static uint8_t nodeFlags[32];
#define RECEIVER 0x01			// Log this node's finish time
#define FINISHED 0x02			// This node has finished downloading
#define WAIT_FINISH 0x04		// Wait for this node to finish

static uint8_t ver_ack;
static uint8_t sendVersion;
static uint16_t verNode;
static uint8_t verPort;

static uint8_t experPort;
static uint16_t experReceiver;
static uint8_t nNodes;

#define SEQ_CACHE_SIZE 6
static uint8_t nextSeq = 0;
static uint16_t idCache[SEQ_CACHE_SIZE];
static uint8_t seqCache[SEQ_CACHE_SIZE];

extern uint16_t cc1000_success_count;     //completed packets
extern uint16_t cc1000_crc_error_count;  //dropped packets due to crc error
extern uint16_t cc1000_mem_error_count;  //dropped packets due to out of memory
extern uint16_t cc1000_sync_error_count; //dropped packets due to sync failure
extern uint16_t cc1000_size_error_count; //dropped packets due to size out of range
#ifdef RADIO_USE_FEC
extern uint16_t cc1000_fec_error_count;  //dropped packets due to fec errors
#endif

static void printErrors()
{
	printf_P(sRadioCounts, cc1000_success_count, cc1000_mem_error_count,
		cc1000_sync_error_count, cc1000_size_error_count, cc1000_crc_error_count);
	#ifdef RADIO_USE_FEC
	printf_P(sFecCount, cc1000_fec_error_count);
	#endif
	printf("\n");
	
	cc1000_success_count = 0;
	cc1000_crc_error_count = 0;
	cc1000_mem_error_count = 0;
	cc1000_sync_error_count = 0;
	cc1000_size_error_count = 0;
	#ifdef RADIO_USE_FEC
	cc1000_fec_error_count = 0;
	#endif
}

static void monitorThread (void)
{
  while (1) {
	#ifdef DEBUG_MEMORY			// from msched.h
    print_threads(0);
    print_memory(0);
    #endif
    printErrors();
    mos_thread_sleep(60000);    
   }
}

static void send_command(uint8_t cmnd, uint16_t node, uint8_t port)
{
	deluge_foot_command* command = (deluge_foot_command*)spkt.data;
	
	command->seq = ++seq;
	command->command = cmnd;
	command->to = node;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = sizeof(deluge_foot_command);
	spkt.data[spkt.size++] = port;
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}

/*
 * Make all the nodes print "Hello!"
 */	
static void send_hello()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_HELLO, node, port);
}

/*
 * Set all the nodes to the same version and page number.
 * This doesn't always seem to work.  Be careful with this one;
 * it does unnatural things to the protocol state.
 */
/*static void stop_all()
{
	deluge_foot_command* command = (deluge_foot_command*)&spkt.data[2];
	
	printf_P(sVersion);
	spkt.data[0] = prompt_uint8("");
	printf_P(sHigh);
	spkt.data[1] = prompt_uint8("");
	
	command->seq = ++seq;
	command->command = DELUGE_COMMAND_STOPALL;
	command->to = -1;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = 2 + sizeof(deluge_foot_command);
	printf_P(sPort);
	spkt.data[spkt.size++] = prompt_uint8("");
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}*/

/*
 * Clear all the stats on all the nodes.  Do end_record first to make
 * sure nodes aren't still recording stats.
 */
static void clear_stats()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_CLEARSTATS, node, port);
}

/*
 * Start all node recording stats.
 */
static void start_record()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_STARTRECORD, node, port);
}

/*
 * Stop all nodes recording stats.
 */
static void end_record()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_ENDRECORD, node, port);
}

static void printStats()
{
	uint16_t total[4] = { 0, 0, 0, 0 };
	printf_P(sLogOn);	// turn on logging
	printf_P(sSTitle, statsNode);
	printf_P(sSHead);
	printf_P(sSBar);
	uint8_t i;
	for (i = 0; i < DELUGE_STATS_COUNT; i++)
	{
		uint16_t* row = (uint16_t*)packet_counts[i];
		printf_P(sSRow, i, row[0], row[1], row[2], row[3],
			(row[0]+row[1]+row[2]+row[3]));
		total[0] += row[0];
		total[1] += row[1];
		total[2] += row[2];
		total[3] += row[3]; 
	}
	printf_P(sSBar);
	printf_P(sSTotal, total[0], total[1], total[2], total[3],
		(total[0]+total[1]+total[2]+total[3]));
	printf_P(sLogOff);	// turn off logging
}

static void waitForStats()
{
	uint8_t i;
	mos_mutex_lock(&waitMutex);
	giveUp = 0;
	uint8_t retry = 1;
	while (!giveUp) {
		for (i = 0; i<N_STATS_PACKETS; i++)
			if (!received[i])
				break;
		if (i == N_STATS_PACKETS || giveUp)
			break;
		if (retry) {
			printf_P(sQuery, statsNode);
			send_command(DELUGE_COMMAND_SENDSTATS, statsNode, statsPort);
		}
		retry = mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)10, (uint32_t)0);
	}
	if (giveUp) {
		mos_mutex_unlock(&waitMutex);
		return;
	}
	mos_mutex_unlock(&waitMutex);
	printStats();
}

/* 
 * Query one node for packet counts.
 */
static void get_stats()
{
	memset(received, 0, sizeof(received));
		
	printf_P(sNode);
	statsNode = prompt_long("");
	printf_P(sPort);
	statsPort = prompt_uint8("");
	
	//mos_thread_new(waitForStats, 256, PRIORITY_NORMAL);
	send_command(DELUGE_COMMAND_SENDSTATS, statsNode, statsPort);
}

static void set_version_impl(uint8_t version, uint16_t node, uint8_t port)
{
	deluge_foot_command* command = (deluge_foot_command*)&spkt.data[1];
	
	spkt.data[0] = version;
	
	command->seq = ++seq;
	command->command = DELUGE_COMMAND_VERSION;
	command->to = node;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = 1+sizeof(deluge_foot_command);
	spkt.data[spkt.size++] = port;
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}

static void waitForVersion()
{
	mos_mutex_lock(&waitMutex);
	giveUp = 0;
	ver_ack = 0;
	while (!giveUp && !ver_ack) {
		printf_P(sSendVersion, sendVersion, verNode);
		set_version_impl(sendVersion, verNode, verPort);
		mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)2, (uint32_t)0);
	}
	mos_mutex_unlock(&waitMutex);
}

/*
 * Change the version number on a node.  This is how you would start an update.
 */
static void set_version()
{
	printf_P(sNode);
	verNode = prompt_long("");
	printf_P(sPort);
	verPort = prompt_uint8("");
	printf_P(sVersion);
	sendVersion = prompt_uint8("");
	
	//mos_thread_new(waitForVersion, 256, PRIORITY_NORMAL);
	set_version_impl(sendVersion, verNode, verPort);
}

/*
 * Query a node for its current neighbor list.
 */
/*static void get_neighbors()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_NEIGHBORS, node, port);
}*/

/*
 * More drastic than stop_all.  Completely erase Deluge state from the
 * EEPROM, format the file system, and restart Deluge.  Same as calling
 * format_deluge in the deluge_setup application, except Deluge keeps running,
 * which might not be a good thing.
 */
/*static void wipe()
{
	printf_P(sNode);
	uint16_t node = prompt_long("");
	printf_P(sPort);
	uint8_t port = prompt_uint8("");
	send_command(DELUGE_COMMAND_WIPE, node, port);
}*/

/*
 * Reboot one or all the nodes and load a backup copy of the OS into
 * program flash.  See the set_image command in deluge_setup to see how to 
 * store a backup image.  If a backup image can't be found, then nothin
 * happens.
 */
static void reboot_net()
{
	deluge_foot_command* command = (deluge_foot_command*)&spkt.data[1];
	
	printf_P(sBoot);
	spkt.data[0] = prompt_uint8("");
	
	command->seq = ++seq;
	command->command = DELUGE_COMMAND_BOOT;
	printf_P(sNode);
	command->to = prompt_long("");
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = 1+sizeof(deluge_foot_command);
	printf_P(sPort);
	spkt.data[spkt.size++] = prompt_uint8("");
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}

static void cachesize()
{
	deluge_foot_command* command = (deluge_foot_command*)&spkt.data[1];
	
	printf_P(sCache);
	spkt.data[0] = prompt_uint8("");
	
	command->seq = ++seq;
	command->command = DELUGE_COMMAND_CACHESIZE;
	command->to = -1;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = 1+sizeof(deluge_foot_command);
	printf_P(sPort);
	spkt.data[spkt.size++] = prompt_uint8("");
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}

static void power()
{
	deluge_foot_command* command = (deluge_foot_command*)&spkt.data[1];
	
	printf_P(sPower);
	spkt.data[0] = prompt_uint8("");
	
	command->seq = ++seq;
	command->command = DELUGE_COMMAND_POWER;
	command->to = -1;
	command->id = mos_node_id_get();
	command->type = DELUGE_PACKET_COMMAND;
	
	spkt.size = 1+sizeof(deluge_foot_command);
	printf_P(sPort);
	spkt.data[spkt.size++] = prompt_uint8("");
	spkt.data[spkt.size++] = DELUGE_PROTO_ID;
	
	com_send(IFACE_RADIO, &spkt);
}

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

static void recvThread()
{
	com_mode(IFACE_RADIO, IF_LISTEN);
	while(1)
	{
		comBuf* pkt = com_recv(IFACE_RADIO);
		//printf("!");
		if (pkt->data[pkt->size-1] != DELUGE_PROTO_ID ||
			pkt->data[pkt->size-3] != DELUGE_PACKET_COMMAND)
		{
			com_free_buf(pkt);
			continue;
		}
		//printf("*");
		
		deluge_foot_command* command = (deluge_foot_command*)
			&pkt->data[pkt->size-2-sizeof(deluge_foot_command)];
		if (checkCache(command->id, command->seq)) {
			com_free_buf(pkt);
			continue;
		}
		//printf(".");
		
		switch (command->command)
		{
			case DELUGE_COMMAND_SENDSTATS_REPLY:
			{
				if (command->id != statsNode) break;
				uint8_t* data = (uint8_t*)packet_counts;
				memcpy(&data[pkt->data[32]], pkt->data, 32);
				
				mos_mutex_lock(&waitMutex);
				received[pkt->data[32]/32] = 1;
				mos_cond_broadcast(&waitCond);
				mos_mutex_unlock(&waitMutex);
				break;
			}
			case DELUGE_COMMAND_FINISHED:
				if (nodeFlags[command->id] & RECEIVER) {
					printf_P(sLogOn);	// turn on logging
				}
				printf_P(sFinish, command->id);
				printf_P(sShowTime);	// display time
				if (nodeFlags[command->id] & RECEIVER) {
					printf_P(sLogOff);	// turn off logging
				}
				
				mos_mutex_lock(&waitMutex);
				nodeFlags[command->id] |= FINISHED;
				if (nodeFlags[command->id] & WAIT_FINISH)
					mos_cond_broadcast(&waitCond);
				mos_mutex_unlock(&waitMutex);						
				break;
			case DELUGE_COMMAND_VERSION_REPLY:
				if (command->id != verNode) break;
				printf_P(sLogOn);	// turn on logging
				printf_P(sVerAck, command->id);
				printf_P(sShowTime);	// display time
				printf_P(sLogOff);	// turn off logging
				
				mos_mutex_lock(&waitMutex);
				ver_ack = 1;
				mos_cond_broadcast(&waitCond);
				mos_mutex_unlock(&waitMutex);						
				break;
			case DELUGE_COMMAND_NEIGHBORS_REPLY:
				break;
		}
		com_free_buf(pkt);
	}
}

static void give_up()
{
	mos_mutex_lock(&waitMutex);
	giveUp = 1;
	mos_cond_broadcast(&waitCond);
	mos_mutex_unlock(&waitMutex);
}

static void allStatsThread()
{
	for (statsNode = nNodes; statsNode>0; statsNode--)
	{
		uint8_t i;
		mos_mutex_lock(&waitMutex);
		memset(received, 0, sizeof(received));
		giveUp = 0;
		uint8_t retry = 1;
		while (!giveUp) {
			for (i = 0; i<N_STATS_PACKETS; i++)
				if (!received[i])
					break;
			if (i == N_STATS_PACKETS || giveUp)
				break;
			if (retry) {
				printf_P(sQuery, statsNode);
				send_command(DELUGE_COMMAND_SENDSTATS, statsNode, statsPort);
			}
			retry = mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)12, (uint32_t)0);
		}
		if (giveUp) {
			mos_mutex_unlock(&waitMutex);
			return;
		}
		mos_mutex_unlock(&waitMutex);
		printStats();
	}
}

/*
 * Run an experiment where one node gets reprogrammed.  This command automates
 * the following process:
 * 1. Stop all nodes recording stats.
 * 2. Clear stats on all nodes.
 * 3. Change the version number on one node.
 * 4. Start all nodes recording stats.
 * 5. Wait until one node gets reprogrammed.
 * 6. Query all nodes for stats and neighbor lists.  (Nodes must be numbered 1 
 * to N if there are N nodes in the network.  There should be no gaps.)
 * 
 * Start time, completion time, packet counts, and neighbor lists will be
 * logged to a file, if you start mos_shell with the -l option.
 */
static void runExper()
{
	// Stop recording stats
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');
	// Clear stats
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	putchar('\n');	
	
	// Set version on seed node
	mos_mutex_lock(&waitMutex);
	ver_ack = 0;
	giveUp = 0;
	while (!giveUp && !ver_ack) {
		printf_P(sSendVersion, sendVersion, verNode);
		set_version_impl(sendVersion, verNode, experPort);
		mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)2, (uint32_t)0);
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Start recording stats now
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	putchar('\n');	
	
	// Wait for the receiving node to finish
	mos_mutex_lock(&waitMutex);
	memset(nodeFlags, 0, sizeof(nodeFlags));
	nodeFlags[experReceiver] = RECEIVER | WAIT_FINISH;
	giveUp = 0;
	while (!giveUp && !(nodeFlags[experReceiver] & FINISHED)) {
		mos_cond_wait(&waitCond, &waitMutex);
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Record for one more minute so we get an accurate count of data packets
	mos_thread_sleep(60000);
	
	// Stop recording
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	
	// Query all nodes for stats
	statsPort = experPort;
	allStatsThread();
}

static void exper()
{
	experPort = 2;
	verNode = 25;
	experReceiver = 1;
	printf_P(sVersion);
	sendVersion = prompt_uint8("");
	nNodes = 25;

	mos_thread_new(runExper, 256, PRIORITY_NORMAL);
}

static void runExperDeluge()
{
	// Stop recording stats
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');
	// Clear stats
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	putchar('\n');	
	
	// Set version on seed node
	mos_mutex_lock(&waitMutex);
	ver_ack = 0;
	giveUp = 0;
	while (!giveUp && !ver_ack) {
		printf_P(sSendVersion, sendVersion, verNode);
		set_version_impl(sendVersion, verNode, experPort);
		mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)2, (uint32_t)0);
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Start recording stats now
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	putchar('\n');	
	
	// Wait for the receiving node to finish
	mos_mutex_lock(&waitMutex);
	memset(nodeFlags, WAIT_FINISH, sizeof(nodeFlags));
	nodeFlags[experReceiver] = RECEIVER | WAIT_FINISH;
	giveUp = 0;
	while (!giveUp) {
		uint8_t i;
		for (i=nNodes; i>0; --i)
			if (!(nodeFlags[i] & FINISHED))
				break;
		if (i)
			mos_cond_wait(&waitCond, &waitMutex);
		else
			break;
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Stop recording
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	
	// Query all nodes for stats
	statsPort = experPort;
	allStatsThread();
}

static void exper_del()
{
	experPort = 2;
	verNode = 25;
	experReceiver = 1;
	printf_P(sVersion);
	sendVersion = prompt_uint8("");
	nNodes = 25;

	mos_thread_new(runExperDeluge, 256, PRIORITY_NORMAL);
}

static void runCacheTest()
{
	// Stop recording stats
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');
	// Clear stats
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_CLEARSTATS, -1, experPort);
	putchar('\n');	
	
	// Set version on seed node
	mos_mutex_lock(&waitMutex);
	ver_ack = 0;
	giveUp = 0;
	while (!giveUp && !ver_ack) {
		printf_P(sSendVersion, sendVersion, verNode);
		set_version_impl(sendVersion, verNode, experPort);
		mos_cond_timedwait(&waitCond, &waitMutex, (uint32_t)2, (uint32_t)0);
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Start recording stats now
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	mos_thread_sleep(2000);
	putchar('.');	
	send_command(DELUGE_COMMAND_STARTRECORD, -1, experPort);
	putchar('\n');	
	
	// Wait for the receiving node to finish
	mos_mutex_lock(&waitMutex);
	memset(nodeFlags, 0, sizeof(nodeFlags));
	nodeFlags[1] = RECEIVER | WAIT_FINISH;
	nodeFlags[6] = RECEIVER | WAIT_FINISH;
	nodeFlags[11] = RECEIVER | WAIT_FINISH;
	nodeFlags[16] = RECEIVER | WAIT_FINISH;
	nodeFlags[21] = RECEIVER | WAIT_FINISH;
	giveUp = 0;
	while (!giveUp) {
		uint8_t i;
		for (i=1; i<nNodes; i+=5)
			if (!(nodeFlags[i] & FINISHED))
				break;
		if (i < nNodes)
			mos_cond_wait(&waitCond, &waitMutex);
		else
			break;
	}
	mos_mutex_unlock(&waitMutex);
	if (giveUp) return;
	
	// Stop recording
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	send_command(DELUGE_COMMAND_ENDRECORD, -1, experPort);
	mos_thread_sleep(1000);
	
	// Query all nodes for stats
	statsPort = experPort;
	allStatsThread();
}

static void cache_test()
{
	experPort = 2;
	cachesize();
	verNode = 25;
	printf_P(sVersion);
	sendVersion = prompt_uint8("");
	nNodes = 25;

	mos_thread_new(runCacheTest, 256, PRIORITY_NORMAL);
}

/*
 * Query all nodes for packet counts.  (Nodes must be numbered 1 
 * to N if there are N nodes in the network.  There should be no gaps.)
 * Packet counts will be logged to a file, if you start mos_shell with 
 * the -l option.
 */
void all_stats()
{
	printf_P(sPort);
	statsPort = prompt_uint8("");
	printf_P(sNNodes);
	nNodes = prompt_long("");
	
	mos_thread_new(allStatsThread, 256, PRIORITY_NORMAL);
}

void start(void)
{
	seq = (uint8_t)rand();
	memset(nodeFlags, 0, sizeof(nodeFlags));
	
	mos_command_daemon_init();
	mos_register_function("send_hello", send_hello);
	//mos_register_function("clear_seq", clear_seq);
	//mos_register_function("stop_all", stop_all);
	mos_register_function("set_version", set_version);
	mos_register_function("clear_stats", clear_stats);
	mos_register_function("start_record", start_record);
	mos_register_function("end_record", end_record);
	mos_register_function("get_stats", get_stats);
	//mos_register_function("get_neighbors", get_neighbors);
	//mos_register_function("wipe", wipe);
	mos_register_function("reboot_net", reboot_net);
	mos_register_function("exper", exper);
	mos_register_function("exper_del", exper_del);
	mos_register_function("all_stats", all_stats);
	mos_register_function("cachesize", cachesize);
	//mos_register_function("cache_test", cache_test);
	mos_register_function("power", power);
	mos_register_function("give_up", give_up);
	
	mos_thread_new(recvThread, 256, PRIORITY_NORMAL);
	mos_thread_new(monitorThread, 256, PRIORITY_NORMAL);
	mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
}
