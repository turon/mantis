//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "com.h"
#include "node_id.h"
#include "printf.h"
#include "net.h"

#include "rts.h"

extern comBuf rts_pkt;
extern comBuf *recv_pkt;
extern comBuf *recv_pool;
extern comBuf *recv_pool_end;

extern uint8_t rts_enable;

int8_t rts_send (comBuf *pkt, va_list ap)
{
   if (rts_enable) {
      com_send (IFACE_RADIO, &rts_pkt);
      
      while (1) {
	 recv_pkt = com_recv (IFACE_RADIO);
	 if (recv_pkt->data[0] == CTS_BYTE) {
	    if (recv_pkt->data[1] == CTS_BYTE) {
	       if (recv_pkt->data[2] == CTS_BYTE) {
		  if (*((uint16_t *)&recv_pkt->data[3]) == mos_node_id_get ()) {
		     printf ("Clear to Send!\n");
		     break;
		  } else {
		     printf ("Someone else's CTS\n");
		  }
	       } else {
		  printf ("Expected another CTS_BYTE\n");
	       }
	    } else {
	       printf ("Expected another CTS_BYTE\n");
	    }
	 } else {
	    if (!recv_pool) {
	       recv_pool = recv_pkt;
	       recv_pool_end = recv_pkt;
	    } else {
	       recv_pool_end->next = recv_pkt;
	       recv_pool_end = recv_pkt;
	    }   
	 }
	 com_free_buf (recv_pkt);
      }
      com_free_buf (recv_pkt);
   }

   com_send (IFACE_RADIO, pkt);

   return 0;
}

int8_t rts_ioctl (uint8_t request, va_list ap)
{

   return 0;
}

boolean rts_recv (comBuf *pkt, uint8_t **footer, uint8_t port) 
{
   comBuf *ret;

   if (rts_enable) {
      if (recv_pool) {
	 ret = recv_pool;
	 recv_pool = recv_pool->next;
	 if (recv_pool == recv_pool_end) {
	    recv_pool = recv_pool_end = NULL;
	 }
      } else
	 ret = com_recv (IFACE_RADIO);
   } else
      ret = com_recv (IFACE_RADIO);
   
   return true;
}

void rts_init (void)
{
   net_proto_register (RTS_PROTO_ID, rts_send, rts_recv, rts_ioctl);
}
