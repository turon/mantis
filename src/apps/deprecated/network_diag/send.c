//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "node_id.h"
#include "printf.h"

static comBuf send_pkt; //comBuf goes in heap
static comBuf ctrl_pkt;


#define PAYLOAD 10
#define NO_PACKETS 100
#define MY_ID 18

#define RTS_TYPE 1
#define CTS_TYPE 2
#define DATA_TYPE 3

#define CC1000_TX_POWER 0

void send_thread ();
void simple_send_thread();
void rts_send(uint8_t id, uint8_t seqno, comBuf *ctrl_pkt);
void set_up_data_packet(uint8_t myid, uint8_t ctr, uint8_t pload);
void rtscts_send_thread ();
void rtscts_send_thread_no_select();

void start (void)
{
   //mos_thread_new (simple_send_thread, 128, PRIORITY_NORMAL);
   //mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
   mos_thread_new (rtscts_send_thread_no_select, 128, PRIORITY_NORMAL);
   //mos_thread_new (rtscts_send_thread_select, 128, PRIORITY_NORMAL);
}

void send_thread ()
{
   //send_pkt.size = 2; //2 bytes
   uint8_t ctr =0, i,j,k, pload;
   uint8_t myid = mos_node_id_get();
   uint8_t START = 0;
   comBuf *commandPkt;
   
   //step down the transmit power
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x01);

   commandPkt = com_recv(IFACE_RADIO);
   printf("Got a packet\n");
   com_free_buf(commandPkt);
   START = 1;
   mos_thread_sleep(500);
   
   if (START) {
      pload = PAYLOAD;
      for(j=0;j<5;j++) {
	 ctr=0;
	 for(i=0;i<NO_PACKETS;i++) {
	    mos_led_toggle(1);
	 
	    send_pkt.size=0;
	    send_pkt.data[send_pkt.size]=myid;
	    send_pkt.size++;
	    send_pkt.data[send_pkt.size]=ctr++;
	    send_pkt.size++;
	    send_pkt.data[send_pkt.size]=DATA_TYPE;
	    send_pkt.size++;
	 
	    for(k=0;k<pload;k++) {
	       send_pkt.data[send_pkt.size]=k;
	       send_pkt.size++;
	    }
	 
	    com_send(IFACE_RADIO, &send_pkt);
	    mos_thread_sleep(500);
	 
	 }
	 pload = pload + 10;
      }
   }
}

#define SECS  1
#define USECS 0

void rtscts_send_thread_no_select()
{
   //send_pkt.size = 2; //2 bytes
   uint8_t ctr =0, i, j,pload;
   uint8_t clear_channel;
   uint8_t myid = mos_node_id_get();
   IF_SET iset;
   comBuf *pkt;
   uint32_t ticks = SECS * TICKS_PER_SEC + USECS / USECS_PER_TICK;
   pload = 10;
   
   //step down the transmit power
   com_ioctl_IFACE_RADIO( CC1000_TX_POWER, 0x01);


/*    pload = PAYLOAD; */
/*    for(j=0;j<5;j++) { */
/*       ctr=0; */
   for(i=0;i<NO_PACKETS;i++) {
      mos_led_toggle(0);
      //set up the packet to be sent
      set_up_data_packet(myid, ctr, pload);
      
      clear_channel = 0;
      while(!clear_channel) {
	 IF_ZERO(&iset);
	 IF_SET(IFACE_RADIO, &iset);
	 rts_send(myid, ctr, &ctrl_pkt);
	 //com_select(&iset, 1000);
	 pkt = com_recv_timed(IFACE_RADIO, ticks);
	 //if(IF_ISSET(IFACE_RADIO, &iset)) {
	 if(pkt) {
	    //pkt = com_recv(IFACE_RADIO);
	    if(pkt->data[0] == myid &&
	       pkt->data[1] == ctr  && pkt->data[2] == CTS_TYPE ) {
	       
	       clear_channel = 1;
	       com_free_buf(pkt);
	       com_send(IFACE_RADIO, &send_pkt);
	       mos_thread_sleep(1000);
	       
	    } else {
	       //Got a packet but its not the CTS that we are waiting for
	       //We should defer our transmission.
	       com_free_buf(pkt);
	       mos_thread_sleep(1000);
	       continue;
	    }
	 } else {
	    //Timer expired. Send out the RTS again
	    clear_channel = 0;
	    printf("ReTx RTS!\n:");
	    continue;
	 }
      }
      ctr++;
   }
    /*   pload = pload+10; */
/*    } */
}

void rtscts_send_thread_select ()
{
   //send_pkt.size = 2; //2 bytes
   uint8_t ctr =0, i, j,pload;
   uint8_t clear_channel;
   uint8_t myid = mos_node_id_get();
   IF_SET iset;
   comBuf *pkt;
   uint32_t ticks = SECS * TICKS_PER_SEC + USECS / USECS_PER_TICK;
   pload = 10;
   
   //step down the transmit power
   com_ioctl_IFACE_RADIO( CC1000_TX_POWER, 0x01);


/*    pload = PAYLOAD; */
/*    for(j=0;j<5;j++) { */
/*       ctr=0; */
   for(i=0;i<NO_PACKETS;i++) {
      mos_led_toggle(0);
      //set up the packet to be sent
      set_up_data_packet(myid, ctr, pload);
      
      clear_channel = 0;
      while(!clear_channel) {
	 IF_ZERO(&iset);
	 IF_SET(IFACE_RADIO, &iset);
	 rts_send(myid, ctr, &ctrl_pkt);
	 com_select(&iset, 1000);
	 if(IF_ISSET(IFACE_RADIO, &iset)) {
	    pkt = com_recv(IFACE_RADIO);
	    if(pkt->data[0] == myid &&
	       pkt->data[1] == ctr  && pkt->data[2] == CTS_TYPE ) {
	       
	       clear_channel = 1;
	       com_free_buf(pkt);
	       com_send(IFACE_RADIO, &send_pkt);
	       mos_thread_sleep(1000);
	       
	    } else {
	       //Got a packet but its not the CTS that we are waiting for
	       //We should defer our transmission.
	       com_free_buf(pkt);
	       mos_thread_sleep(1000);
	       continue;
	    }
	 } else {
	    //Timer expired. Send out the RTS again
	    clear_channel = 0;
	    printf("ReTx RTS!\n:");
	    continue;
	 }
      }
      ctr++;
   }
   /*   pload = pload+10; */
   /*    } */
}

void set_up_data_packet(uint8_t myid, uint8_t ctr, uint8_t pload) {
   uint8_t k;
   
   send_pkt.size=0;
   
   send_pkt.data[send_pkt.size]=myid;
   send_pkt.size++;

   send_pkt.data[send_pkt.size]=ctr;
   send_pkt.size++;

   send_pkt.data[send_pkt.size]=DATA_TYPE;
   send_pkt.size++;
   
   for(k=0;k<pload;k++) {
      send_pkt.data[send_pkt.size]=k;
      send_pkt.size++;
   }
}

void rts_send(uint8_t id, uint8_t seqno, comBuf *ctrl_pkt)
{

   mos_led_toggle(1);

   ctrl_pkt->size = 0;

   ctrl_pkt->data[ctrl_pkt->size]=id;
   ctrl_pkt->size++;

   ctrl_pkt->data[ctrl_pkt->size]=seqno;
   ctrl_pkt->size++;

   ctrl_pkt->data[ctrl_pkt->size]=RTS_TYPE;
   ctrl_pkt->size++;
   
   com_send(IFACE_RADIO, ctrl_pkt);
//   printf("Send out RTS\n");
} 


void simple_send_thread ()
{
   //send_pkt.size = 2; //2 bytes
   uint8_t ctr =0, i,k, pload;
   pload = 10;

   //step down the transmit power
   com_ioctl_IFACE_RADIO( CC1000_TX_POWER, 0x01);

   
   for(i=0;i<NO_PACKETS;i++) {
      mos_led_toggle(0);
      
      //set up the packet
      send_pkt.size=0;
      
      send_pkt.data[send_pkt.size]=mos_node_id_get();
      send_pkt.size++;
      send_pkt.data[send_pkt.size]=ctr++;
      send_pkt.size++;
      
      for(k=0;k<pload;k++) {
	 send_pkt.data[send_pkt.size]=k;
	 send_pkt.size++;
      }
      com_send(IFACE_RADIO, &send_pkt);
      mos_thread_sleep(500);
      
   }
   
}
