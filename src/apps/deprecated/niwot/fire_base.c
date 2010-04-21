//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**********************************
NOTE:
In order to get the application to function properly, you must uncomment the line:
#define MICA2_PROTO 1
from the file:
$MANTIS_HOME/src/mos/dev/include/mica2-light-temp.h
***********************************/ 

#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"
#include "niwot.h"
#include "cc1000.h"
#include "sem.h"

#define YELLOW 0
#define GREEN 1
#define RED 2

static mos_sem_t send_sem;
static mos_sem_t go_state_sem;

static comBuf sendBuf;
static uint8_t myID;
static uint8_t baseID;
static uint8_t myParentID;
static uint8_t myDTB;
static uint8_t mySeqNo;

static mos_alarm_t sleep_timer;
uint32_t sleep_timer_ms = TO_NEXT_SLEEP;
void sleep_timer_callback();

static mos_alarm_t send_timer;
uint32_t send_timer_ms = BEACON_INTERVAL;
void send_timer_callback();

static uint8_t state;

void send();
void recv();

void start (void)
{
    printf("base\n");
    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);

    //com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x01);

    mos_sem_init(&go_state_sem, 0);
    mos_sem_init(&send_sem, 0);

    myID = mos_node_id_get(); //preset
    baseID = myID; //I'm the base
    myParentID = 0; //no parents
    myDTB = 0; //I am the base
    mySeqNo = 0;

    //set timer functions
    sleep_timer.func = sleep_timer_callback;  
    sleep_timer.msecs = sleep_timer_ms;
    sleep_timer.reset_to = 0;
    sleep_timer.data = 1;
    send_timer.func = send_timer_callback;
    send_timer.msecs = send_timer_ms;
    send_timer.reset_to = send_timer_ms;
    send_timer.data = 2;

    //start sleep alarm
    mos_alarm (&sleep_timer);

    state = GO;

    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);

}

void sleep_timer_callback()
{
   mos_led_on(2);
   state = SLEEPNODE;

}

void send_timer_callback()
{
   mos_led_on(1);
   mos_sem_post(&send_sem);
}

void send()
{ 
    net_packet_t * np;
    mos_alarm(&send_timer);

    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);

    while (1)
    {
       //mos_led_toggle (GREEN);

       mos_led_off(1);
	if (state == SLEEPNODE)
	{
	   // need to remove the alarm before sleeping
   	   mos_remove_alarm(&send_timer);
	   
	   //printf("send sleeps\n"); 
	   mos_thread_sleep(SLEEP_INTERVAL);
	   mos_led_display(0);
	   // printf("send awake\n"); 


	   // synchronize with recv thread
	   mos_sem_wait(&go_state_sem);
	   send_timer.msecs = send_timer_ms;
	   mos_alarm (&send_timer);
	   
	   mySeqNo = 0;
	}

	// debugging stuff
	/*if(!mos_find_alarm(&sleep_timer))
	{
	   printf("no more sleep timer?\n");
	   print_clock_list();
	   printf("----\n");
	   state = SLEEPNODE;
	}
	else
	   printf(".");
	*/
	
	//first populate the buffer with the net packet since this will
	//always be the header
	np = (net_packet_t *)&(sendBuf.data[0]);
	np->type = CONTROL;
	np->src = myID;
	np->dest = 0;
	np->next_hop = 0;
	np->last_hop = 0;
	np->seqno = mySeqNo;
	np->pkt_dtb = 0;
	np->sender_dtb = 0;
	sendBuf.size = sizeof(net_packet_t);

	sendBuf.size += sizeof(wildfire_packet_t);
	com_send(IFACE_RADIO, &sendBuf);

	mySeqNo++;
	//printf("state = %d\n", state);
	
	// wait for send_timer
	mos_sem_wait(&send_sem);
   }
}


void recv()
{
    comBuf * recvBuf;
    wildfire_packet_t * wp;
    net_packet_t * np;    
    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    
    while (1)
    {
       //mos_led_toggle (RED); 
	if (state == SLEEPNODE)
	{
	    com_mode(IFACE_RADIO, IF_OFF);

	    //printf("recv sleeps\n"); 
	    mos_thread_sleep(SLEEP_INTERVAL);
	    //printf("recv awake\n");

	    com_mode(IFACE_RADIO, IF_LISTEN);

	    sleep_timer.msecs = sleep_timer_ms;
	    mos_alarm (&sleep_timer);
	    state = GO;
 	    mos_sem_post(&go_state_sem);
	}

	//wait 1/20th awake time to recv before timeout
	recvBuf = com_recv_timed(IFACE_RADIO, 255);

	if (recvBuf == NULL)
	{
	    //probably should do something here... don't really care, though.
	}
	else
	{
	    //pull off the net packet to see what type it is.
	    np = (net_packet_t *)&(recvBuf->data[0]);
	 
	    if (np->type == DATA)
	    {
		wp = (wildfire_packet_t *)&(recvBuf->data[sizeof(net_packet_t)]);
		//printf ("received a data packet from %d.. %d hops\n", np->src, np->sender_dtb);
		//printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
//		printf("%d\n", wp->battery);
		//printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
	    }
	    else if (np->type == CONTROL)
	    {
		//printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
		//printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
	    }
	    else
	    {
		//printf ("received a RANDOM packet\n");
		//INVALID PACKET!!!  -- We should never get here.
	    }
	    com_free_buf(recvBuf);
	}
      
    }//end while

}//end recv
