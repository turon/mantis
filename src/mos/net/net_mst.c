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
/*   File: net_mst.c                                                      */
/* Author: Rafael Salomon                                                 */
/*   Date: 2007-04-06                                                     */
/*                                                                        */
/* net layer plugin: MST protocol                                         */
/**************************************************************************/

#ifdef NET_MST
#include <stdlib.h>
#include <string.h>

#include "led.h"
#include "node_id.h"
#include "mutex.h"
#include "net_mst.h"
#include "net.h"

#ifdef PLATFORM_MICA2
	#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
	#include "cc2420.h"
#endif

/* is_base: */
/* whether or not i am a base station (initialzed to false) */
static boolean is_base;

/* addr */
/* my network address (initialized to node id) */
/* !!! NODE ID RETURNS uint16_t !!! */
static uint8_t addr;

/* hop_count[] */
/* my distance to base, aka hop count, indexed into by base network address (initialized to MAX_NETWORK_DIAMETER + 1) */
static uint8_t hop_count[NET_MST_MAX_ADDR];

/* packet_seq_no[] */
/* the sequence number of the last data packet i sent/forwarded, indexed into by network address (initialized to 0) */
/* !!! ACCESSED BY MULTIPLE THREADS.  SHOULD HAVE MUTEX !!! */
static uint8_t packet_seq_no[NET_MST_MAX_ADDR];

/* next_addr[] */
/* the next hop for sending and forwarding packets, index by destination network address (initialized to broadcast address) */
static uint8_t next_addr[NET_MST_MAX_ADDR];

/* beacon */
/* buffer for sending out beacon packets if this node is a base station */
static comBuf beacon;

/* getters */
boolean net_mst_is_base() {
	return is_base;
}
uint8_t net_mst_addr() {
	return addr;
}
uint8_t net_mst_hop_count(uint8_t dest_addr) {
	return hop_count[dest_addr];
}
uint8_t net_mst_next_addr(uint8_t dest_addr) {
	return next_addr[dest_addr];
}

/** @brief net_mst_proto_init called by application to initialize mst protocol.
 */
void net_mst_proto_init() {

	uint8_t i;

	/* register mst with net layer */
	net_proto_register(MST_PROTO_ID, net_mst_proto_send, net_mst_proto_recv, net_mst_proto_ioctl);
	
	/* initialize variables */
	is_base = false;

	if(mos_node_id_get() <= NET_MST_MAX_ADDR) {
		addr = mos_node_id_get();
	}
	else {
		/* should return error code but we can't return anything */
		/* !!! NEED TO DO SOMETHING HERE !!! */
		addr = 0;
	}
	
	for(i=0; i<NET_MST_MAX_ADDR; i++) {
		hop_count[i] = NET_MST_MAX_NETWORK_DIAMETER + 1;
	}

	for(i=0; i<NET_MST_MAX_ADDR; i++) {
		packet_seq_no[i] = 0;
	}

	for(i=0; i<NET_MST_MAX_ADDR; i++) {
		next_addr[i] = NET_MST_BROADCAST_ADDR;
	}
}


/** @brief net_mst_proto_send called by net_send.  appends mst info structure and hands packet back to net_send for sending.
 * @param pkt comBuf from application
 * @param dest_addr network address of the node that the packet is being sent to
 * @param packet_type mst_test packet type defined in net_mst.h
 * @param add_mst_header, we only need to add the header if it's a new packet from an app, not if it's a packet that we're forwarding
 * @return int8_t 0 on success, error code on failure.  return value will be passed on to application by net_send
 */
 /* !!! NEED FROM_APP/APPEND_MST_INFO BUT CAN WE HIDE IT FROM USER? !!! */
int8_t net_mst_proto_send(comBuf *pkt, va_list args) {

	uint8_t dest_addr;
	uint8_t packet_type;
	boolean add_mst_header;
	net_mst_packet_t *mst_info;

	/* grab function parameters */
	dest_addr = va_arg(args, int);
	packet_type = va_arg(args, int);
	add_mst_header = va_arg(args, int);
  
	if(add_mst_header) {

		/* bail if packet is too big */
		if((pkt->size + sizeof(net_mst_packet_t)) > COM_DATA_SIZE) {
			return NET_MST_ERROR_PACKET_TOO_BIG;
		}
	
		/* increase size to accomodate metadata */
		pkt->size = pkt->size + sizeof(net_mst_packet_t);
	
		/* set metadata values */
		mst_info = (net_mst_packet_t*) &(pkt->data[pkt->size-sizeof(net_mst_packet_t)]);
		mst_info->packet_type = packet_type;
		mst_info->packet_seq_no = ++packet_seq_no[addr];
		mst_info->src_addr = addr;
		mst_info->prev_addr = addr;
		if(packet_type == NET_MST_BEACON_PACKET) {
			mst_info->next_addr = NET_MST_BROADCAST_ADDR;
			mst_info->dest_addr = NET_MST_BROADCAST_ADDR;
		}
		else {
			mst_info->next_addr = next_addr[dest_addr];
			mst_info->dest_addr = dest_addr;
		}
		mst_info->hop_count = 0;
	}

	/* send it */
	return 0;
}

/* called by net_mst_proto_recv */
/* return value of false will make net_recv_packet free the packet memory */
/* call is_app_waiting_on to pass packet to application */
static boolean handle_beacon_packet(comBuf *pkt, net_mst_packet_t *mst_info, uint8_t port) {

	/* don't do anything if it's my own beacon */
	if(mst_info->src_addr == addr) {
		return false;
	}

	/* update seq no if it's bigger than what we have */
	if(mst_info->packet_seq_no > packet_seq_no[mst_info->src_addr]) {
		packet_seq_no[mst_info->src_addr] = mst_info->packet_seq_no;
	}

	/* update routing tables and forward if the info is new or better */
	if((mst_info->packet_seq_no >= packet_seq_no[mst_info->src_addr]) || (mst_info->hop_count + 1 < hop_count[mst_info->src_addr])) {
		/* update routing info */
		hop_count[mst_info->src_addr] = mst_info->hop_count + 1;
		next_addr[mst_info->src_addr] = mst_info->prev_addr;
		/* update packet info and forward */
		mst_info->prev_addr = addr;
		mst_info->next_addr = NET_MST_BROADCAST_ADDR;
		mst_info->hop_count++;
		net_send(pkt, MST_PROTO_ID, port, mst_info->dest_addr, mst_info->packet_type, NET_MST_FORWARD_PACKET);
	}

	/* free packet memory */
	return false;
}

/* called by net_mst_proto_recv */
/* return value of false will make net_recv_packet free the packet memory */
/* call is_app_waiting_on to pass packet to application */
static boolean handle_data_packet(comBuf *pkt, net_mst_packet_t *mst_info, uint8_t port) {

	/* update seq no if it's bigger than what we have */
	if(mst_info->packet_seq_no > packet_seq_no[mst_info->src_addr]) {
		packet_seq_no[mst_info->src_addr] = mst_info->packet_seq_no;
	}

	/* update routing tables if the info is new or better */
	if((mst_info->packet_seq_no >= packet_seq_no[mst_info->src_addr]) || (mst_info->hop_count + 1 < hop_count[mst_info->src_addr])) {
		/* update routing info */
		hop_count[mst_info->src_addr] = mst_info->hop_count + 1;
		next_addr[mst_info->src_addr] = mst_info->prev_addr;
	}

	/* if the packet is meant for me, pass it on to app */
	if(mst_info->dest_addr == addr) {
		return is_app_waiting_on(port);
	}
	/* otherwise consider forwarding if we're the next hop or if it's broadcast */
	else if(mst_info->next_addr == addr || mst_info->dest_addr == NET_MST_BROADCAST_ADDR) {

		/* only forward if we haven't done so already */
		if(mst_info->packet_seq_no >= packet_seq_no[mst_info->src_addr]) {

			/* update packet info */
			mst_info->prev_addr = addr;
			mst_info->next_addr = next_addr[mst_info->dest_addr];
			mst_info->hop_count++;
			/* forward */
			net_send(pkt, MST_PROTO_ID, port, mst_info->dest_addr, mst_info->packet_type, NET_MST_FORWARD_PACKET);
		}
	}

	/* free packet memory */
	return false;
}

/* called by net_recv_packet which is called by background net_thread whenever a packet arrives */
/* return value of false will make net_recv_packet free the packet memory */
/* !!! WHAT IS FOOTER FOR ? !!! */
boolean net_mst_proto_recv(comBuf *pkt, uint8_t **footer, uint8_t port) {

	/* grab mst meta data from end of packet */
	net_mst_packet_t* mst_info;
	mst_info = (net_mst_packet_t*) &(pkt->data[pkt->size - sizeof(net_mst_packet_t)]);

	/* figure out if it's data or beacon packet and hand off */
	if(mst_info->packet_type == NET_MST_BEACON_PACKET) {
		/* beacon */
		return handle_beacon_packet(pkt, mst_info, port);
	}
	else if(mst_info->packet_type == NET_MST_DATA_PACKET) {
		/* data */
		return handle_data_packet(pkt, mst_info, port);
	}

	/* invalid packet type.  discard. */
	return 0;
}

void beacon_thread() {

	/* wake up transmitter */
	/* !!! WHAT DOES THIS DO ? !!! */
	#ifdef PLATFORM_MICA2
		com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x00);
	#endif
	#ifdef PLATFORM_TELOSB
		com_ioctl_IFACE_RADIO(CC2420_TX_POWER, 0x00);
	#endif

	while(1) {
		
		/* send beacon to broadcast address */
		beacon.size = 0;
		net_send(&beacon, MST_PROTO_ID, NET_MST_PORT, NET_MST_BROADCAST_ADDR, NET_MST_BEACON_PACKET, NET_MST_NEW_PACKET);
		
		/* nap */
		mos_thread_sleep(NET_MST_BEACON_FREQUENCY_MILLIS);
	}
}

int8_t net_mst_proto_ioctl(uint8_t request, va_list args) {

	uint8_t new_addr;

	/* do requested action */
	switch(request) {
		/* make this node a base station, which means we need to start a thread for sending out beacon packets */
		case NET_MST_SET_IS_BASE:
			/* only do this if we're not a base station yet.  we only want to launch beacon thread once. */
			if(!is_base) {
				is_base = true;
				mos_thread_new(beacon_thread, 128, PRIORITY_NORMAL);
			}
			break;
		/* change the node's network address */
		case NET_MST_SET_ADDR:
			/* get new address and make sure it's valid */
			new_addr = va_arg(args, int);
			if(new_addr > NET_MST_MAX_ADDR) {
				return NET_MST_ERROR_INVALID_ADDR;
			}
			/* reset values for our previous address */
			hop_count[addr] = NET_MST_MAX_NETWORK_DIAMETER + 1;
			packet_seq_no[addr] = 0;
			next_addr[addr] = NET_MST_BROADCAST_ADDR;
			/* change address */
			addr = new_addr;
			/* set values for our new address */
			hop_count[addr] = 0;
			packet_seq_no[addr] = 0;
			next_addr[addr] = addr;
			break;
		default:
			return NET_MST_ERROR_INVALID_IOCTL_OPTION;
	}

	return 0;
}
#endif