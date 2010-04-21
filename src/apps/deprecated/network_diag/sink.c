//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>

#include "led.h"
#include "com.h"     //give us the communication layer
#include "printf.h"
#include "msched.h"
//#include "dev.h"

#define CC1000_FREQ     2

static comBuf send_pkt;
static comBuf ctrl_pkt;

#define RTS_TYPE 1
#define CTS_TYPE 2
#define DATA_TYPE 3

void rtscts_sink_thread();
void sink_thread();
void beacon_thread();
void send_cts(uint8_t src, uint8_t seqno, comBuf *ctrl_pkt);
void sniffer();
void rssi_sniffer();

#ifndef GET_RSSI
#error "Define GET_RSSI in com.h and recompile everything"
#endif
#ifndef TS_PACKET
#error "Define TS_PACKET in com.h and recompile everything"
#endif

void start(void)
{
   //mos_thread_new(sink_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(rtscts_sink_thread, 128, PRIORITY_NORMAL);
//   mos_thread_new(beacon_thread, 128, PRIORITY_NORMAL);
//   mos_thread_new(sniffer, 128, PRIORITY_NORMAL);
//   mos_thread_new(rssi_sniffer, 128, PRIORITY_NORMAL);
}


void sink_thread(){
   comBuf *recv_pkt;                     //give us a packet pointer
   com_mode(IFACE_RADIO, IF_LISTEN);

   send_pkt.size =1;
   send_pkt.data[0] = 99;
   com_send(IFACE_RADIO, &send_pkt);
   
//   printf("before entering the while loop\n");
   
   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      printf("Src = %C Seq = %C Type = %C Size = %C TS = %l TCNT = %C RSSI = %l\n",
	     recv_pkt->data[0],
	     recv_pkt->data[1],
	     recv_pkt->data[2],
	     recv_pkt->size,
	     recv_pkt->ts,
	     recv_pkt->tcnt,
	     recv_pkt->signal);
      com_free_buf(recv_pkt);             //free the recv'd packet to the pool
      mos_led_toggle(0);
   }
}



void rtscts_sink_thread(){
   comBuf *recv_pkt;                     //give us a packet pointer
   com_mode(IFACE_RADIO, IF_LISTEN);

   printf("before entering the while loop\n");
   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      printf("Src = %C Seq = %C Type = %C Size = %C TS = %l TCNT = %C RSSI = %l\n",
	     recv_pkt->data[0],
	     recv_pkt->data[1],
	     recv_pkt->data[2],
	     recv_pkt->size,
	     recv_pkt->ts,
	     recv_pkt->tcnt,
	     recv_pkt->signal);
      if(recv_pkt->data[2] == RTS_TYPE) {
	 mos_thread_sleep(100);
	 send_cts(recv_pkt->data[0], recv_pkt->data[1], &ctrl_pkt);
	 printf("Send out CTS to %C\n", recv_pkt->data[0]);
      }
      com_free_buf(recv_pkt);             //free the recv'd packet to the pool
      mos_led_toggle(0);
      
   }
}

void sniffer() {
   comBuf *recv_pkt;
   com_mode(IFACE_RADIO, IF_LISTEN);
   while (1) {
      recv_pkt = com_recv(IFACE_RADIO);
      com_free_buf(recv_pkt);
      mos_led_toggle(2);
   }
}

void rssi_sniffer() {
   uint16_t rssi;
   while (1) {
      cc1000_rssi_on();
      rssi = rssi_poll();
      printf ("rssi: %d\n", rssi);
      //rssi_off();
      mos_thread_sleep(500);
   }
}
   
   

void send_cts(uint8_t src, uint8_t seqno, comBuf *ctrl_pkt) {
   

   ctrl_pkt->size = 0;

   ctrl_pkt->data[ctrl_pkt->size]=src;
   ctrl_pkt->size++;
   
   ctrl_pkt->data[ctrl_pkt->size]=seqno;
   ctrl_pkt->size++;

   ctrl_pkt->data[ctrl_pkt->size]=CTS_TYPE;
   ctrl_pkt->size++;
   
   com_send(IFACE_RADIO, ctrl_pkt);
}

void beacon_thread ()
{
   //send_pkt.size = 2; //2 bytes
   uint8_t ctr =0;
   while(1) {
      mos_led_toggle(2);
      
      //set up the packet
      send_pkt.size=0;

      send_pkt.data[send_pkt.size]=1;
      send_pkt.size++;

      send_pkt.data[send_pkt.size]=ctr++;
      send_pkt.size++;

      com_send(IFACE_RADIO, &send_pkt);
      mos_thread_sleep(5000);
   }
}

