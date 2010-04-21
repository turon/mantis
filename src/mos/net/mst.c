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
/* File:       mst.c                                                      */
/* Author:     Anmol Sheth: anmol.sheth@colorado.edu                      */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* MST protocol example                                                   */
/**************************************************************************/

/** @file mst.c
 * @brief MST protocol example * @author Carl Hartung
 * @date Created: 11/12/2004
 */

#include <stdlib.h>
#include <string.h>

#include "led.h"
#include "node_id.h"
#include "mutex.h"
#include "mst.h"
#include "net.h"

#ifdef MST
#if defined(MST_USE_CRC)
#include "crc.h"
static uint16_t mst_s_crc;
static uint16_t mst_r_crc;
#endif

#if defined(TESTBED_DIAGNOSTICS)
#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

#warning "Testbed Diagnostics are enabled!"
#warning "Testbed Diagnostics are enabled!"
#include "testbed_event.h"
static recv_pkt_event_t recv_event;
static send_pkt_event_t send_event;

#endif

static uint8_t myDtb; // my distance (hop count) to the base station
//uint8_t myParentID; //upstream parent ID ( == rtable[0])
static uint8_t myID;
static uint8_t seqNo;
static uint8_t beaconSeqNo;
static uint8_t lstctrlfwd;   //the seqno of the last control pkt this node forwarded

#define MAX_NETWORK_DIAMETER 20
#define MAX_NODES 256   //1-byte address allows for 256 - 1 nodes(1 broadcast)

uint8_t rtable [MAX_NODES] = {0}; 	//Downstream routing table indexed by 
				 	// dest address	
uint8_t mst_dtb(){   return myDtb;}
uint8_t mst_parent(){ return rtable[0];}
void mst_proto_init()
{
  /* must be called by all protocols */
   net_proto_register(MST_PROTO_ID, mst_proto_send, mst_proto_recv, mst_proto_ioctl);
   myDtb = MAX_NETWORK_DIAMETER;
   myID = 0xFF;
   seqNo = 0;
   beaconSeqNo = 0;
   lstctrlfwd = 0;
}


/*Send function for MST
 * Required args: 
 *      uint8_t fromApp: 1 if sender is source
 *      uint8_t pkt_type: Type of packet
 *      uint8_t dest: Final destination of packet
 */
int8_t mst_proto_send(comBuf *pkt, va_list args)
{
   uint8_t fromApp;
   uint8_t pkt_type;
   uint8_t dest;

   mst_packet_t *mst_info;

   fromApp = va_arg(args, int);
   pkt_type = va_arg(args, int);
   dest = va_arg(args, int);
   
   if(!fromApp) {
      mst_info = (mst_packet_t *)&(pkt->data[pkt->size-sizeof(mst_packet_t)]);
#if defined(MST_USE_CRC)
      // compute crc over packet, last two bytes are the crc
      mst_s_crc = crc_compute(pkt->data, pkt->size - 2);
      mst_info->crc_l = mst_s_crc & 0x00FF;;
      mst_info->crc_h = (mst_s_crc & 0xFF00) >> 8;
#endif

#if defined(TESTBED_DIAGNOSTICS)
      if(mst_info->type == MST_CONTROL)
	 com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x00);
      else if(mst_info->type == MST_DATA)
	 com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x02);
      else
	 printf("unknown packet type!\n");
#endif
      //This is a packet in mid path.  Just send it off.
      return 0;
   }
   else{
      //It is a data packet from the application. Initialize it and send
      //it off
     pkt->size = pkt->size + sizeof(mst_packet_t);
     mst_info = (mst_packet_t *)&(pkt->data[pkt->size-sizeof(mst_packet_t)]);
     
     mst_info->sender_dtb = myDtb;  //Source_DTB
     mst_info->last_hop = myID;     //Last Hop 
     mst_info->src = myID;          //Source
     mst_info->dest = dest;         //Destination
     mst_info->type = pkt_type;     //Type

     //SeqNo 
     if (pkt_type == MST_CONTROL)
	mst_info->seqno = beaconSeqNo++;
     else       	
	mst_info->seqno = seqNo++;
     
     mst_info->pkt_dtb = myDtb;            //DTB
     mst_info->ttl = MAX_NETWORK_DIAMETER; //TTL
     mst_info->next_hop = rtable[dest];    //Next Hop

     //sent packet event
#if defined(TESTBED_DIAGNOSTICS)
     send_event.dest = dest;
     send_event.size = pkt->size;
     send_testbed_event(EVENT_NETWORK, PACKET_SENT,
			(uint8_t *)&(send_event), 3);
#endif

     //set output power (optimization)
#if defined(TESTBED_DIAGNOSTICS)
      if(mst_info->type == MST_CONTROL)
	 com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x00);
      else if(mst_info->type == MST_DATA)
	 com_ioctl_IFACE_RADIO(RADIO_TX_POWER, 0x02);
      else
	 printf("unknown packet type!\n");
#endif
      
#if defined(MST_USE_CRC)
      // compute crc over packet, last two bytes are the crc
      mst_s_crc = crc_compute(pkt->data, pkt->size - 2);
      mst_info->crc_l = mst_s_crc & 0x00FF;
      mst_info->crc_h = (mst_s_crc & 0xFF00) >> 8;
#endif      
      return 1;
   }
}

static uint8_t handle_control_packet(comBuf *pkt, mst_packet_t *mst_info, uint8_t port)
{
        //Control packet update the Dtb if needed. Change the parent
     //only if the new DTB is < Current DTB. Do not change otherwise
     //(prevent ping-ponging between 2 parent nodes with same DTB)
      
      //this is a copy of my own packet, do nothing with it
      if(mst_info->src == myID)
	 return 0;

      // see if we've sent this packet. 
      if(mst_info->seqno == lstctrlfwd &&
	 myDtb != MAX_NETWORK_DIAMETER)
	 return 0;

      //packet is about to expire or invalid ttl
      if(mst_info->ttl <= 1 || mst_info->ttl > MAX_NETWORK_DIAMETER)
	 return 0;

      
     //Node lost its parent and has no better candidate
     //myDtb = max to find best available parent
     //(seqno - 5): 5 missed cycles of broadcast to indicate a lost link 
      if (mst_info->seqno > 5 &&
	  mst_info->seqno - 5 > lstctrlfwd &&
	  myID != 0){
	myDtb = MAX_NETWORK_DIAMETER;
     }


      //test to see if the sequence number is
      //drastically too high
#if defined(TESTBED_DIAGNOSTICS)
      if(mst_info->seqno > lstctrlfwd &&
	 mst_info->seqno - lstctrlfwd > 10)
      {
	 printf("lcf>>");
	 return 0;
      }
      
#endif
      
      //If control packet is from parent or better, store seqno and forward
      if(mst_info->pkt_dtb + 1 < myDtb ||
	mst_info->last_hop == rtable[mst_info->src]) {
	
	rtable[mst_info->src] = mst_info->last_hop; //Update routing table
	lstctrlfwd = mst_info->seqno; //store beacon no
	
	if(mst_info->pkt_dtb + 1 < myDtb && myID != 0)  //Found a better path
	   if(mst_info->src == 0)
	   {
	      myDtb = mst_info->pkt_dtb +1;	       
#if defined(TESTBED_DIAGNOSTICS)
	      uint16_t parent = mst_info->last_hop;
	      send_testbed_event(EVENT_NETWORK,NEW_PARENT, &parent, 2);
#endif
	   }
     
	 
	 
	 mst_info->ttl = mst_info->ttl -1; //Dec TTL
	 mst_info->pkt_dtb = myDtb; //Inc DTB
	 mst_info->last_hop = myID; //Change the last hop address
	 
	 //most nodes shouldn't be sending control
	 //packets except for the basestation(0)
	 //atleast for now
#if defined(TESTBED_DIAGNOSTICS)
	 //debugging!!!
	 if(mst_info->src != 0)
      {
	 printf("c nfb.");
	 return 0;
      }
#endif
      
      net_send(pkt, MST_PROTO_ID, port, false, mst_info->type, mst_info->dest);
      
     }
     //A return value of 0 causes the net thread to return the buffer
      //to the common pool.
     return 0;
}

static uint8_t forward_data_packet(comBuf *pkt, mst_packet_t *mst_info, uint8_t port)
{
   //I may be on the path. net_send would check if I am on the
   //path and forward the packet upstream if necessary.
   
   if(mst_info->ttl != 1 && mst_info->next_hop == myID) {
      
#if defined(TESTBED_DIAGNOSTICS)
      recv_event.src = mst_info->last_hop;
      recv_event.size = pkt->size;
      send_testbed_event(EVENT_NETWORK, PACKET_RECV, (uint8_t *)&recv_event, 3);
#endif	    
      //Update routing table
      rtable[mst_info->src] = mst_info->last_hop;
      
      //need to forward the packet upstream
      
      mst_info->last_hop = myID;    //Add self as last hop
      mst_info->pkt_dtb = mst_info->pkt_dtb - 1;//decrement the DtB
      mst_info->next_hop = rtable[mst_info->dest];//change the next hop ID
      mst_info->ttl = mst_info->ttl - 1;//decrement the ttl
      
      net_send(pkt, MST_PROTO_ID, port, false, mst_info->type, mst_info->dest);
      return 0; // Free the buffer after sending it out
   }
}

static uint8_t handle_data_packet(comBuf *pkt, mst_packet_t *mst_info, uint8_t port)
{
      //Data frame so we need to send it up to the app only if the
      //dest addr == myaddr. If not check if I am on the path and
      //forward it upstream if necessary
      
      if(mst_info->dest == myID) { //dest is the final dest of the packet
	 //Update routing table
	 
	 rtable[mst_info->src] = mst_info->last_hop;
	      
	 /* must be called if the protocol wants to try to send the packet
	  * to the app */

#if defined(TESTBED_DIAGNOSTICS)
	    recv_event.src = mst_info->last_hop;
	    recv_event.size = pkt->size;
	    send_testbed_event(EVENT_NETWORK, PACKET_RECV, (uint8_t *)&recv_event, 3);
#endif	    
	 return is_app_waiting_on(port);
      }
      else  
	 return forward_data_packet(pkt, mst_info, port);
      return 0;
}

boolean mst_proto_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
  mst_packet_t *mst_info;
  mst_info = (mst_packet_t *)&(pkt->data[pkt->size-sizeof(mst_packet_t)]);

   //We have received a packet from the net_thread. We need to check
   //whether it is a control or a data packet and send it off to the
   //higher app layer.
#if defined(MST_USE_CRC)    //test the CRC!
   mst_r_crc = crc_compute(pkt->data, pkt->size - 2);
   if(mst_info->crc_l != (mst_r_crc & 0x00FF) ||
      mst_info->crc_h != ( (mst_r_crc & 0xFF00) >> 8) )   {
      printf("crc.");
      return 0;
   }
#endif
   
   if (mst_info->type == MST_CONTROL)
      return handle_control_packet(pkt, mst_info, port);
   else if (mst_info->type == MST_DATA)
      return handle_data_packet(pkt, mst_info, port);
   else //shouldn't get here
      printf("unknown packet type: %d\n", mst_info->type);

   return 0;
}

int8_t mst_proto_ioctl(uint8_t request, va_list args)
{
   switch(request) {
   case SET_ADDR:
      myID = va_arg(args, int);
      printf("Set my ID to %C\n", myID);
      break;
   case SET_DTB:
      myDtb = va_arg(args, int);
      printf("Set my DTB to %C\n", myDtb);
      break;
   default:
      return 0;
      break;
   }
   return 0;
}
 
#endif