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
/* File:      flood.c                                                     */
/* Author     Jeff Rose   :  rosejn@colorado.edu                          */
/*   Date: 04/15/04                                                       */
/*                                                                        */
/* A broadcast flooding implementation for the mos net layer.             */
/**************************************************************************/

#include "flood.h"
#include "node_id.h"

#ifdef FLOOD

#define FLOOD_BROADCAST_ADDR 0xffff	// Broadcast address
#define FLOOD_HISTORY_SIZE   10	// Packet cache size for flooding

typedef struct cache_entry_t {
   struct cache_entry_t *next;
   uint16_t src;
   uint8_t seq_num;
} cache_entry;

cache_entry pkt_cache_mem[FLOOD_HISTORY_SIZE];
cache_entry *pkt_cache;

int8_t cache_check(flood_pkt * pkt);

void flood_init()
{
   uint8_t i;
   cache_entry *cache_ptr;
   
   // Setup the packet cache
   pkt_cache_mem[0].src = 0;
   pkt_cache_mem[0].seq_num = 0;
   pkt_cache = &pkt_cache_mem[0];
   cache_ptr = pkt_cache;
   
   for (i = 1; i < FLOOD_HISTORY_SIZE; i++) { 
      pkt_cache_mem[i].src = 0;
      pkt_cache_mem[i].seq_num = 0;
      cache_ptr->next = &pkt_cache_mem[i];
  }

   net_proto_register(FLOOD_PROTO_ID, flood_send, flood_recv, flood_ioctl);
}

int8_t flood_send(comBuf * pkt, va_list args)
{
   flood_pkt *flood_ptr;

   // First make sure the packet isn't too large
   if (pkt->size > (COM_DATA_SIZE - sizeof(flood_pkt)))
      return NET_BUF_FULL;

   // Fill in the flood packet info for the broadcast addr &
   //  specified port
   flood_ptr = (flood_pkt*)&(pkt->data[pkt->size]);
   flood_ptr->src = mos_node_id_get();
   flood_ptr->dest = FLOOD_BROADCAST_ADDR;
   pkt->size += sizeof(flood_pkt);
   pkt->data[pkt->size++] = FLOOD_PROTO_ID;
   
   // Save it in our packet cache so we don't send it again
   cache_check(flood_ptr);
   
   com_send(IFACE_RADIO, pkt);

   return pkt->size;
}

boolean flood_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
   flood_pkt *flp;

   flp = (flood_pkt*)&(pkt->data[pkt->size - sizeof(flood_pkt)]);
   *footer = (uint8_t *)flp;

   // We can just dump the packet if we already have it cached
   if(cache_check(flp))
      return true;

   // Forward it on if its a broadcast or its not for us
   if(flp->dest == FLOOD_BROADCAST_ADDR || flp->dest != mos_node_id_get())
      com_send(IFACE_RADIO, pkt);

   // Pull our header off
   pkt->size -= sizeof(flood_pkt);

   // Now tell the net layer whether this is an event packet or not
   if(flp->dest == FLOOD_BROADCAST_ADDR || flp->dest == mos_node_id_get())
      return true;

   return false;
}

int8_t flood_ioctl(uint8_t request, va_list args)
{
  /*  int arg; */
   
/*    switch(request) */
/*    { */
/*    case DUMMY_CONTROL: // Adjust transmit power */
      
/*       /\* First get the argument and then call the specific function. *\/ */
/*       arg = va_arg(ap, int); */
/*       dummy_func(arg); */
/*       break; */
/*    } */
   
   return 0;
}

/** Functions private to this file. **/

/** @brief Check the cache for the given packet, and cache it if we haven't
 * already seen it. 
 * @param pkt Packet 
 * @return FALSE if this is an old or non-existent packet, else return TRUE
*/
int8_t cache_check(flood_pkt * pkt)
{
   cache_entry *cache_ptr, *last = NULL;

   // Run through the cache to determine if we have already seen this packet
   for(cache_ptr = pkt_cache; cache_ptr->next != NULL;
       cache_ptr = cache_ptr->next) {
      if(cache_ptr->src == pkt->src) {
	 // This is a duplicate
	 if (cache_ptr->seq_num == pkt->seq_num)
	    return TRUE;
	 else if (cache_ptr->seq_num < pkt->seq_num){
	    // This is an old packet
	    cache_ptr->seq_num = pkt->seq_num;
	    return FALSE;
	 }
      }
      last = cache_ptr;
   }

   // If we get here than the cache doesn't have any packets from
   // this address.  Copy the pkt data into the tail of the list and
   // move it up to the front
   
   cache_ptr->src = pkt->src;
   cache_ptr->seq_num = pkt->seq_num;

   last->next = NULL;
   cache_ptr->next = pkt_cache;
   pkt_cache = cache_ptr;

   return FALSE;
}

#endif