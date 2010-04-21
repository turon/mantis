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
/* File:       net.c                                                      */
/* Author:     Jeff Rose               :  rosejn@colorado.edu             */
/* Author:     Charles Gruenwald III   :  gruenwal@colorado.edu           */
/*   Date: 04/15/04                                                       */
/* Modified:   Cyrus Hall              :  hallcp@colorado.edu             */
/*   Date: 07/22/04                                                       */
/*                                                                        */
/* Top level event based networking layer.                                */
/**************************************************************************/

/** @file net.c
 * @brief Top level event based networking layer.
 * @author Jeff Rose, Charles Gruenwald III
 * @author Modified: Cyrus Hall
 * @date Created: 04/15/2004
 * @date Modified: 07/22/2004
 */

#include "net.h"
#include "msched.h"
#include "cond.h"
#include <string.h>

/** @brief List of current protocols. */
net_proto protocols[NET_PROTO_MAX];

/** @brief default_protocol **/
uint8_t default_proto;

/** @brief pointer to last received comBuf **/
comBuf * currentComBuf;

/** @brief list of threads blocked on recv **/
static mos_thread_t *waitingThreads;

static mos_mutex_t netMutex;
static mos_mutex_t currentBufReadLock;
static mos_cond_t  currentBufCond;

#ifdef MOS_NO_USE_DYNAMIC_MEMORY
static uint8_t net_stack[192];
#endif

/** @brief Initialize the net layer variables  
 */
void net_init()
{
   uint8_t i;
   
   for(i = 0; i < NET_PROTO_MAX; i++)
      protocols[i].proto_id = 0;
   
   currentComBuf = NULL;
   waitingThreads = NULL;
   mos_mutex_init(&netMutex);
   mos_mutex_init(&currentBufReadLock);
   mos_cond_init(&currentBufCond);

#ifdef MOS_NO_USE_DYNAMIC_MEMORY
   mos_thread_new_havestack(net_thread, 192, net_stack, PRIORITY_NORMAL);
#else
   mos_thread_new(net_thread, 384, PRIORITY_NORMAL);
#endif
}

/** @brief net_send method called by applications 
 * @param pkt comBuf from application
 * @param proto_id protocol's ID
 * @param port destination port
 * @return ret not useful so far.. should probably return number of bytes sent?
 */
int8_t net_send(comBuf *pkt, uint8_t proto_id, uint8_t port, ...)
{
    uint8_t i;
    va_list ap;
    uint8_t ret;
    uint8_t old_size;
    
    if(proto_id == 0)
      return NET_PROTO_INVALID;

    /* store the old size since we modify it */
    old_size = pkt->size;
    
    /* Make sure we have a valid protocol id, then send. */
    for(i = 0; i < NET_PROTO_MAX; i++)
    {
      if(protocols[i].proto_id == proto_id)
      {
        /* Setup the va_list. */
        va_start(ap, port);
        /* Now send to protocol to add protocol specific footers. */
        ret = protocols[i].sfunc(pkt, ap);
        va_end(ap);

        /* add the dest port to the packet footer*/
        pkt->data[pkt->size] = port;
        pkt->size++;
        /* add the protocol id to the packet footer*/
        pkt->data[pkt->size] = proto_id;
        pkt->size++;

        /* the net layer actually sends the packet */
#ifdef PLATFORM_AVRDEV
        com_send(IFACE_SERIAL2, pkt);
#else 
        // Only CC2420 currently provides link layer unicast
#ifdef CC2420
        /* MOS CTP protocal requires uni-cast on link layer */
        if (proto_id == CTP_PROTO_ID)
        {
          /* CTP's sfunc() stores the MAC address of nextHop in pkt->source */
          /* if nextHop is found, send it */ 
          if(ret == 0) {
            ret = com_sendto(IFACE_RADIO, pkt, pkt->source);

#ifdef CTP_PLUS_DEBUGt
   if(ret!=0xff)
      printf("\nNET MSG: sent frame type %C, len %C to %d, ret %C\n", pkt->data[0], pkt->size, pkt->source, ret);
#endif
          }

          /* To be compatible with TinyOS, MOS CTP protocal inserts the net layer header
           * in front of its payload. So, in order to keep the intactness of 
           * the passed-in pkt, we need to remove the inserted header
           *
           * restore the payload */
if(ret < 11)
          if(pkt->size > old_size + 2) 
            memcpy(pkt->data, &(pkt->data[pkt->size - 2 - old_size]), old_size);
        }
        else
#endif
        ret = com_send(IFACE_RADIO, pkt);
#endif

        // set the packet size to be its original size
        // since we modified it
        pkt->size = old_size;
        return ret;
      }
    }

    /* Not a valid protocol. */
    return NET_PROTO_INVALID;
}


/*
 * @param sfunc Protocol send function
 * @param rfunc Protocol receive function
 * @param ifunc Protocol io control function
 * @return NET_PROTO_INVALID if protocol invalid, -1 if no space left, else return
 */
int8_t net_proto_register(uint8_t proto, net_proto_send sfunc,
        net_proto_recv rfunc, net_proto_ioctl ifunc)
{
    uint8_t i;

    /* Make sure the passed in ID is valid, i.e., not 0. */
    if(proto == 0)
  return NET_PROTO_INVALID;

    /* Find the next available slot. */
    for(i = 0; i < NET_PROTO_MAX; i++)
    {
  if(protocols[i].proto_id == proto ||
     protocols[i].proto_id == 0)
  {
      protocols[i].proto_id = proto;
      protocols[i].sfunc = sfunc;
      protocols[i].rfunc = rfunc;
      protocols[i].ifunc = ifunc;
   
      return 0;
  }
    }

    /* No space left. */
    return -1;
}

/** @brief Set some protocol specific options. 
 * @param proto Protocol
 * @param request IO Request
 * @return NET_PROTO_INVALID if protocol invalid, else return retval
 */
int8_t net_ioctl(uint8_t proto, uint8_t request, ...)
{
    uint8_t i;
    uint8_t retval;
    va_list ap;

    if(proto == 0)
      return NET_PROTO_INVALID;
   
    /* Make sure we have a valid protocol id, then call the func. */
    for(i = 0; i < NET_PROTO_MAX; i++)
    {
      if(protocols[i].proto_id == proto)
      {
      /* Now ship it off to the protocol. */
        va_start(ap, request);
        retval = protocols[i].ifunc(request, ap);
    
        va_end(ap);
     
        return retval;
      }
    }

    /* Not a valid protocol. */
    return NET_PROTO_INVALID;
}


/** @brief A background thread that listens on everything for event traffic.*/
void net_thread()
{
#ifdef PLATFORM_AVRDEV
    com_mode(IFACE_SERIAL2, IF_LISTEN);
#else
    com_mode(IFACE_RADIO, IF_LISTEN);
#endif

    while(1)
    {
       net_recv_packet();
    }
}

void net_recv_packet()
{
   
   comBuf *pkt;
   uint8_t i;
   uint8_t proto;
   uint8_t port;
   uint8_t *footer;
   boolean toApp;
   
#ifdef PLATFORM_AVRDEV
   pkt = com_recv(IFACE_SERIAL2);
#else
   pkt = com_recv(IFACE_RADIO);
#endif
   
   if(pkt == NULL)
      return;

   /* Now that we have a packet we pull the protocol ID and port and send it
      to the correct protocol handler. */
   proto = pkt->data[pkt->size - 1];
   pkt->size--;
   port = pkt->data[pkt->size - 1];
   pkt->size--;   

#ifdef CTP_PLUS_DEBUGt
   i = cc2420_num_suppressed_dup();
   printf("\nNET MSG: recvd type %C, len %C (%C, %C) from %d\n", pkt->data[0], pkt->size+2, proto, port, pkt->source);
   printf("\nLINK MSG: duplicates %C\n",  i); 
#endif

   /* If the proto is 0, we've received an invalid packet. */
   if(proto == 0)
   {
      com_free_buf(pkt);
      return;
   }
   
   for(i = 0; i < NET_PROTO_MAX && protocols[i].proto_id != proto; i++);
   
   /* Bogus protocol id... */
   if(i == NET_PROTO_MAX)
   {
      com_free_buf(pkt);
      return;
   }

   mos_mutex_lock(&currentBufReadLock);
   while (currentComBuf != NULL) {
      mos_cond_wait(&currentBufCond, &currentBufReadLock);
   }

   /* set the current comBuf to the currently received packet */
   currentComBuf = pkt;    
  
   /* check to see if the protocol is going to send the packet to the app */
   toApp = protocols[i].rfunc(pkt, &footer, port);
   /* if it doesn't free the buffer.  If it does, it's the Apps job to free
      the buffer */
   if (!toApp){
      com_free_buf(pkt);
      currentComBuf = NULL;

//printf("\n\ncom free buf 1 %x\n\n", pkt);
//comBuf * bf = check_free_combuf(); 
//printf("\nfree comBuf pool:\t%x -> %x -> %x\n", bf, bf==NULL?NULL:bf->next, (bf==NULL || bf->next==NULL)?NULL:bf->next->next);
   }
   mos_mutex_unlock(&currentBufReadLock);
}

/** @brief is_app_waiting_on() method called by protocols
 * @param port check all waiting threads for the specified port
 * @return true if it sent the packet to an app, false if it did not.
 */
boolean is_app_waiting_on(uint8_t port)
{
   mos_thread_t *current;
   mos_thread_t *previous;
 
   mos_mutex_lock(&netMutex);

   if(waitingThreads == NULL)
   {
      mos_mutex_unlock(&netMutex);
      return false;
   }

   previous = waitingThreads;
   current = waitingThreads;

   /* first check to see if the first thread is waiting on the specified port */
   if (previous->port == port)
   {
      waitingThreads = waitingThreads->next;
      //give current packet back to app
      mos_thread_resume(current);
      mos_mutex_unlock(&netMutex);
      return true;
   }

   /* it wasn't the first thread, so check the rest */
   current = current->next;   
   while (current != NULL)
   {
      /* found a waiting thread */
      if (current->port == port)
      {
        /* remove it from list */
         previous->next = current->next;
        /* resume the app (gives the current packet to the app) */
         mos_thread_resume(current);
         mos_mutex_unlock(&netMutex);
         return true;
      }
      current = current->next;
      previous = previous->next;
   }

   mos_mutex_unlock(&netMutex);
   return false;     
}
 
/** @brief net_recv method called by applications
 * @param port listening on specified port
 * @return returns comBuf to application
 */
comBuf *net_recv(uint8_t port)
{
   mos_thread_t *head;
   comBuf * buf;
 
   /* set the port the thread will wait on */
   mos_thread_set_port(port);
   
   mos_mutex_lock(&netMutex);
   /* if no threads waiting, make this the first in the list */
   if (waitingThreads == NULL){ 
      waitingThreads = mos_thread_current();
   }
   /* otherwise, add it to the end of the list of waiting threads */
   else
   {
      head = waitingThreads;
      while (head->next != NULL){
        head = head->next;
      }
      head->next = mos_thread_current();
   }
   
   mos_mutex_unlock(&netMutex); 

   /* suspend the thread until a packet with the specified port is received */
   mos_thread_suspend();
   
   mos_mutex_lock(&currentBufReadLock);
   buf = currentComBuf;
   currentComBuf = NULL;
   mos_cond_signal(&currentBufCond);
   mos_mutex_unlock(&currentBufReadLock);

   /* we've been woken up by the is_app_waiting_on(port) call, so the current
      packet must be for this app */
   return buf;
   
}
