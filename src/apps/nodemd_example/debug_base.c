//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*
  This is an example debugging base station for the NodeMD
  debugger. It is designed for the default parameters (8 bit trace
  codes, headers defined in mos_debugging.h) and will work for the
  mica2 and micaz platforms.

  The app waits for debugging data and forwards the data to the serial
  line. It's recommended that the verbatim trace codes 1-15 be sent to
  the serial line and parsed at the receiving end. Using the
  DEBUG_VERBOSE flag will print a text string in place of the verbatim
  trace codes, however using this option considerably slows packet
  processing.

*/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "mos_debugging.h"
#include "com.h"
#include "cc2420.h"

/*

//#define DEBUG_VERBOSE

void recv_thread()
{
   
   debug_net_packet_t* recvednetpkt;
   debug_packet_t* recveddebugpkt;
   uint8_t to_print;
   uint16_t count;
   uint16_t curr_offset;
   uint8_t i,j;
   comBuf *recvbuf;

   printf("debug base\n");

   while(1){
      com_mode(IFACE_RADIO, IF_LISTEN);
      recvbuf = com_recv(IFACE_RADIO);
      recvednetpkt = (debug_net_packet_t*)&(recvbuf->data[0]);
      recveddebugpkt = (debug_packet_t*)&(recvbuf->data[sizeof(debug_net_packet_t)]);
      
      if(recvednetpkt->type == 254)
      {
	 if(recveddebugpkt->offset == 0)
	 {
	    printf("**%d DETECTED IN NODE %d**\n", recveddebugpkt->error_code, recvednetpkt->src);
	 }
	 
	 i = 0;
	 count = 1;
	 curr_offset = recveddebugpkt->offset*DEBUG_DATA_SIZE*2;
	 while(i < recveddebugpkt->size)
	 {
	    //printf("%d\n%d\n", (recveddebugpkt->data[i] & 0x0F), ((recveddebugpkt->data[i++] & 0xF0)) >> 4);
	    for(j=0; j<(8/DBCODE_SIZE); j++)
	    {
	       if(j == 0)
	       {
		  to_print = ((recveddebugpkt->data[i] & (0x0F)));
	       }
	       else
	       {
		  to_print = (((recveddebugpkt->data[i]) & 0xF0) >> 4);
	       }
	       
	       if(to_print != 0)
	       {
		  //printf("%d\t", curr_offset + (count++));

#ifdef DEBUG_VERBOSE
		  switch(to_print)
		  {
		  case DBCODE_CONTEXT_SWITCH: printf("context switch\t"); break;
		  case DBCODE_PROCEDURE_CALL: printf("procedure call\t"); break;
		  case DBCODE_PROCEDURE_RETURN: printf("procedure return\t");break;
		  case DBCODE_INTERRUPT: printf("interrupt\t\t");break;
		  case DBCODE_THREAD_BLOCK: printf("thread block\t");break;
		  case DBCODE_THREAD_UNBLOCK: printf("thread unblock\t");break;
		  case DBCODE_TIMER_SET: printf("timer set\t\t");break;
		  case DBCODE_TIMER_FIRED: printf("timer fired\t\t");break;
		  case DBCODE_THREAD_SLEEP: printf("thread sleep\t");break;
		  case DBCODE_THREAD_WAKEUP: printf("thread wakeup\t");break;
		  case DBCODE_NODE_SLEEP: printf("deep sleep\t\t");break;
		  case DBCODE_NODE_WAKEUP: printf("deep wakeup\t\t");break;
		  case DBCODE_THREAD_NEW: printf("thread new\t\t");break;
		  case DBCODE_THREAD_EXIT: printf("thread exit\t\t");break;
		  case DBCODE_BREAKPOINT: printf("breakpoint\t\t");break;
		  case DBCODE_THREAD1: printf("thread 1\t\t");break;
		  case DBCODE_THREAD2: printf("thread 2\t\t");break;
		  case DBCODE_THREAD3: printf("thread 3\t\t");break;
		  case DBCODE_THREAD4: printf("thread 4\t\t");break;
		  case DBCODE_THREAD5: printf("thread 5\t\t");break;
		  case DBCODE_THREAD6: printf("thread 6\t\t");break;
		  default: break;
		  }
#else
		  printf("%d", to_print);
#endif
		  
		  printf("\n");
	       }
	       
	    }
	    i++;
	 }
	 
      }
      
      
      com_free_buf(recvbuf);
   }
   
   
}

*/
void start(void)
{


  //   mos_thread_new(recv_thread, 128, PRIORITY_NORMAL);

}
