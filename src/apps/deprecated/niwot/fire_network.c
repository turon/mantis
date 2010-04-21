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

#define RECONNECT_INTERVAL 10 //10 seconds
#define WIND_INTERVAL 2

static mos_sem_t readynode_state_sem;
//static mos_sem_t go_state_sem;

static comBuf sendBuf;
static uint8_t myID;
static uint8_t baseID;
static uint8_t myParentID;
static uint8_t myDTB;
static uint8_t mySeqNo;
static uint8_t found;
static uint8_t next_sleep;

static uint8_t state;

static mos_alarm_t sleep_timer;
static mos_alarm_t reconnect_timer;

uint32_t reconnect_timer_ms = 500;
void reconnect_timer_callback();

uint32_t sleep_timer_ms = TO_NEXT_SLEEP;
void sleep_timer_callback();

void alarm_set();
void send();
void recv();

void start (void)
{
   printf("network\n");
    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);
    mos_thread_sleep(200);
    found = 0;

    //com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x01);

    mos_sem_init(&readynode_state_sem, 0);
    //mos_sem_init(&go_state_sem, 0);
    
    myID = mos_node_id_get(); //preset
    baseID = 0;  //updated with the first packet
    myParentID = 0; //updated with the first packet
    myDTB = 255; //updated with the first packet
    mySeqNo = 0;
    state = INIT;
    //state = GO;

    //initialize timer for the wind speed 
    sleep_timer.func = sleep_timer_callback;
    reconnect_timer.func = reconnect_timer_callback;  

    reconnect_timer.reset_to = reconnect_timer_ms;
    sleep_timer.reset_to = 0;
    
    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL); 
}

//running in the background.. times out every 10 seconds and sees if we've
//received a control packet from our parent.  If not.. find a new one.
void reconnect_timer_callback()
{
   mos_led_on(1);
    if (found == 0)
    {  // lost our parent... need a new one
	myDTB = 255;
    }
    else {
	// printf ("still got it!\n");
    }
    found = 0;   
}


void sleep_timer_callback()
{
   mos_led_on(2);
   state = SLEEPNODE;
}                              

void send()
{   
    wildfire_packet_t * wp;
    net_packet_t * np;
    uint16_t sample;
    
    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    //just wait while in init state

    while (state != GO)
    {	
        mos_led_toggle (LED_GREEN); 
	mos_thread_sleep(1000);
	mos_led_toggle (LED_GREEN);
    }
    printf("starting to send\n");

    while (1)
    {
       mos_led_off(1);
       if (state == SLEEPNODE)
       {
	  mos_thread_sleep(SLEEP_INTERVAL);
	  mos_led_display(0);
	  
	  // wait for the recv thread to wake up
	  mos_sem_wait(&readynode_state_sem);
	  // synchronize the threads
	  //mos_sem_post(&go_state_sem);
       }

        //first populate the buffer with the net packet since this will
	//always be the header
	np = (net_packet_t *)&(sendBuf.data[0]);
	np->type = DATA;
	np->src = myID;
	np->dest = baseID;
	np->next_hop = myParentID;
	np->last_hop = myID;
	np->seqno = mySeqNo;
	np->pkt_dtb = myDTB;
	np->sender_dtb = myDTB;
	sendBuf.size = sizeof(net_packet_t);
	

	//**************** sensor data code here ********************
	// need to get the data and fill our packet buffer

	

        //for data packets, we now populate the buffer with the 
	//wildfire packet information.  Adding it to the end.
	wp = (wildfire_packet_t *)&(sendBuf.data[sendBuf.size]);


	//battery
	dev_open(DEV_MICA2_BATTERY);
	dev_read(DEV_MICA2_BATTERY, &sample, sizeof(sample));
	dev_close(DEV_MICA2_BATTERY);
	wp->battery = sample;
	sendBuf.size += sizeof(wildfire_packet_t);

	//printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
	//printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
//        printf("sent?\n");



	//*************** end sensor data code **********************
	
	com_send(IFACE_RADIO, &sendBuf);

	mySeqNo++;	
	if (mySeqNo > 254)
	   mySeqNo = 0;


	// debugging stuff
	/*if(state == GO) {
	   if(!mos_find_alarm(&sleep_timer))
	   {
	      printf("no more sleep timer?\n");
	      print_clock_list();
	      printf("----\n");
	      state = SLEEPNODE;
	   }
	   else
	   {
	      printf(".");
	   }
	   }*/
	
	   
	mos_thread_sleep(SEND_INTERVAL);
    }
}


void alarm_set()
{
    sleep_timer.msecs = sleep_timer_ms;
    reconnect_timer.msecs = reconnect_timer_ms;
}

      
void recv()
{
    comBuf * recvBuf;
    net_packet_t * np;
    wildfire_packet_t * wp;

    printf("my id = %d\n", myID);

    //initial state
    np = (net_packet_t *)&(sendBuf.data[0]);
    np->type = FINDME;
    np->src = myID;
    np->dest = 0;
    np->next_hop = 0;
    np->last_hop = 0;
    np->seqno = 0;
    np->pkt_dtb = 0;
    np->sender_dtb = 0;
    sendBuf.size = sizeof(net_packet_t);

    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);

    while (state == INIT)
    {
	
	recvBuf = com_recv_timed(IFACE_RADIO, 16000);    
	if (recvBuf  == NULL)
	{
	    com_send(IFACE_RADIO, &sendBuf);
	    mos_led_off(LED_GREEN);
	    mos_led_off(LED_YELLOW);
	    mos_led_toggle(LED_RED);
	    
	}
	else{
	    np = (net_packet_t *)&(recvBuf->data[0]);
	    if (np->type == FINDME)
	    {

		np->dest = np->src;
		np->src = myID;
		mos_led_off(LED_RED);
		mos_led_off(LED_YELLOW);
		mos_led_toggle(LED_GREEN);
		mos_thread_sleep(400);
		com_send(IFACE_RADIO, &sendBuf);
	    }
	    else if (np->type == CONTROL)
	    {
		printf ("received control 1\n");
		//must be put into an already running network
		myDTB = np->sender_dtb +1;
		np->sender_dtb = myDTB;
		com_send(IFACE_RADIO, recvBuf);
		next_sleep = np->seqno;
		found = 1;
		sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * BEACON_INTERVAL;
		//printf("setting 1 sleep for %d\n", sleep_timer_ms);
		alarm_set();
		mos_alarm (&sleep_timer);
		mos_alarm (&reconnect_timer);
		state = GO;
	    }
	    else if (np->type == STANDBY)
	    {
		state = STANDBY;
		printf ("got a standby\n");
	    }
	    com_free_buf(recvBuf);
	}
    }

    mos_led_off(LED_GREEN);
    mos_led_off(LED_RED);
    while (state == STANDBY)
    {
	recvBuf = com_recv(IFACE_RADIO);    
	np = (net_packet_t *)&(recvBuf->data[0]);
	if (np->type == CONTROL)
	{
	    //must be put into an already running network
	    myDTB = np->sender_dtb +1;
	    np->sender_dtb = myDTB;
	    com_send(IFACE_RADIO, recvBuf);
	    next_sleep = np->seqno;
	    found = 1;
	    sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * BEACON_INTERVAL;
	    //printf("setting 1 sleep for %d\n", sleep_timer_ms);
	    alarm_set();
	    mos_alarm (&sleep_timer);
	    mos_alarm (&reconnect_timer);
	    state = GO;
	}
	com_free_buf(recvBuf);
    }
    
    while (1)
    {
	//check to see if I should be sleeping
        if (state == SLEEPNODE)
	{
	    // need to remove the alarm before we sleep
	    mos_remove_alarm(&reconnect_timer);
	   
	    com_mode(IFACE_RADIO, IF_OFF);
	    mos_thread_sleep(SLEEP_INTERVAL);
	    com_mode(IFACE_RADIO, IF_LISTEN);

	    state = READYNODE;

	    // signal readynode to the send thread
	    mos_sem_post(&readynode_state_sem);
	}

        //wait just over 5 seconds to recv before timeout
	//5125 is Carl's original value, might need to check

	//should be 255 now 1/20th of awake time
	recvBuf = com_recv_timed(IFACE_RADIO, 255);

	if (recvBuf == NULL)
	{
	   //printf("recv timed out\n");
	}
	else
	{
	    //pull off the net packet to see what type it is.
	    np = (net_packet_t *)&(recvBuf->data[0]);
	    if (np->type == DATA)
	    {
		//printf("received data packet\n");
		wp = (wildfire_packet_t *) & (recvBuf->data[sizeof(net_packet_t)]);
		//printf("received a data packet from %d at %d hops\n", np->src, np->sender_dtb);
		//printf("mydtb = %d and pkt_dtb = %d\n", myDTB, np->pkt_dtb);
		if (myDTB < np->pkt_dtb && np->src != myID && np->last_hop != myID && np->sender_dtb != 255)
		{
			np->last_hop = myID;
			np->pkt_dtb -= 1;
			com_send(IFACE_RADIO, recvBuf);
		}
		if (np->sender_dtb == 255)
		{
		    np->type = CONTROL;
		    np->last_hop = myID;
		    np->src = myID;
		    np->pkt_dtb = myDTB;
		    np->sender_dtb = myDTB;
		}
	    }
	    else if (np->type == CONTROL)
	    {
		//printf ("received control\n");
		if (state == READYNODE)
		{
		    //this is the first control packet since i woke up
		    //get the time until I need to sleep again.
		    //printf ("received control 2\n");
		    next_sleep = np->seqno;
		    if(next_sleep*BEACON_INTERVAL >= TO_NEXT_SLEEP)
		    {
		       // our sleep cycle is out of phase
		       // need to stay in the READYNODE state and wait for another control packet
		    }
		    else
		    {
		       //printf("seqno %d\n", next_sleep);
		       sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * BEACON_INTERVAL;
		       //printf("setting sleep for %d\n", sleep_timer_ms);
		       alarm_set();
		       mos_alarm (&sleep_timer);
		       mos_alarm (&reconnect_timer);
                       // wait for sync
		       //mos_sem_wait(&go_state_sem);
		       state = GO;
		    }
		}

		//it's a control packet.
		//update the information and send it on.
		//printf("received control packet from %d, dtb %d\n", np->last_hop, np->pkt_dtb);
		//printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
		//printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
	
		if (np->pkt_dtb < myDTB)
		{
		    np->last_hop = myID;
		    np->pkt_dtb += 1;
		    myDTB = np->pkt_dtb;
		    found = 1;  //race condition.. but I don't think we care.
		    if (np->sender_dtb == 0)
		    {
			baseID = np->src;
		    }
		    com_send(IFACE_RADIO, recvBuf);
		}
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



