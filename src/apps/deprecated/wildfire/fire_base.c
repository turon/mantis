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

//change this to enable variable rate routing
//0 is off
//the metric is packetsends=SENDRATE*nodehop and maxes out around 44 packets/min 
#define SENDRATE 0



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
#include "fire.h"
#include "cc1000.h"
#include <string.h>

static comBuf sendBuf;
static uint8_t myID;
static uint8_t baseID;
static uint8_t myParentID;
static uint8_t myDTB;
static uint8_t mySeqNo;

static mos_alarm_t sleep_timer;
void sleep_timer_callback();

static uint8_t state;

void send();
void recv();
void timerlist();

void start (void)
{
    comBuf * ptr;

    //turn the radio and serial on
    com_mode(IFACE_RADIO, IF_LISTEN);
    com_mode(IFACE_SERIAL, IF_LISTEN);

    //change the radio power?
    //com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x50);

    myID = mos_node_id_get();     //preset
    baseID = myID;                //I'm the base
    myParentID = 0;               //no parents
    myDTB = 0;                    //I am the base
    mySeqNo = 0;                  //start at 0

    //set timer functions
    sleep_timer.func = sleep_timer_callback;  

    //set timer times
    sleep_timer.msecs = TO_NEXT_SLEEP;
    sleep_timer.reset_to = 0;
 
    state = GO;

    //just to let us know it's on
    printf ("waiting for start signal\n");

    while (1)
    {
	ptr = com_recv(IFACE_SERIAL);
	if (strncmp(ptr->data, "startstartstartstartstartstartstart", 35) == 0)
	{
	    printf ("received start\n");  //this is for the python script
	    com_free_buf(ptr);
	    break;
	}
	else
	{
	    printf ("got %s\n", ptr->data);
            printf ("which not the start value.  Still waiting...\n");
	    com_free_buf(ptr);
	}
	
    }
    //kill the serial to save power
    com_mode(IFACE_SERIAL, IF_OFF);

    //start sleep alarm
    mos_alarm (&sleep_timer);

    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);
}

//thread for debugging
void timerlist()
{
    while (1)
    {
	mos_mdelay(500);
	print_clock_list();
    }
}


void sleep_timer_callback()
{
    state = SLEEPNODE;
}


void send()
{ 
    net_packet_t * np;

    while (1)
    {

	if (state == SLEEPNODE)
	{
	    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //must set this for power save mode
	    mos_thread_sleep(SLEEP_INTERVAL);
	    mos_thread_set_suspend_state(SUSPEND_STATE_IDLE);  //just seems to work better when 'on'
	    while(state != GO)
		mos_mdelay(500);
	    mySeqNo = 0;
	}

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
	np->rate = SENDRATE;         //change this for adaptive rate sending.
	sendBuf.size = sizeof(net_packet_t);

        //make the beacons the same size as data to help ensure connectivity
	sendBuf.size += sizeof(wildfire_packet_t);  
	//debug info:
	//printf("CP:\ttype\tsrc\tdst\tnext\tlast\tseq\tpdtb\tsdtb\n");
	//printf("CP:\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
	com_send(IFACE_RADIO, &sendBuf);
	mySeqNo++;

	mos_mdelay(SEND_TIMER);
   }
}


void recv()
{
    comBuf * recvBuf;
    wildfire_packet_t * wp;
    net_packet_t * np;
    uint16_t sample;
    comBuf * ptr;

    //header info
    printf("src\tseq\tp_dtb\ts_dtb\tlast\ttemp\thumid\tw_dir\tw_spd\tbat\n");



    while (1)
    {
	if (state == SLEEPNODE)
	{
	    
	    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //power save mode
	    com_mode(IFACE_RADIO, IF_OFF);                      //kill radio
	    com_mode(IFACE_SERIAL, IF_OFF);                     //kill serial
	    mos_thread_sleep(SLEEP_INTERVAL - COM_RECV_TIMEOUT);
	    mos_thread_set_suspend_state(SUSPEND_STATE_IDLE);   //idle mode
	    com_mode(IFACE_SERIAL, IF_LISTEN);                  //turn serial on
	  
	    while (1)
	    {
		ptr = com_recv(IFACE_SERIAL);
		if (strncmp(ptr->data, "startstartstartstartstartstartstart", 35) == 0)
		{
		    printf ("got it\n");  //for the python script
		    com_free_buf(ptr);
		    break;
		}
		else
		    com_free_buf(ptr);
	    }
	    com_mode(IFACE_SERIAL, IF_OFF);  //kill the serial
	    com_mode(IFACE_RADIO, IF_LISTEN);//turn radio on

	    //set timer when to begin next sleep cycle
	    sleep_timer.msecs = TO_NEXT_SLEEP;
	    mos_alarm (&sleep_timer);
	    state = GO;
	    
            //print out battery
	    dev_open(DEV_MICA2_BATTERY);
	    dev_read(DEV_MICA2_BATTERY, &sample, sizeof(sample));
	    dev_close(DEV_MICA2_BATTERY);
	    printf("base battery = %d\n", sample);
	}


	//wait just 5 seconds to recv before timeout
	recvBuf = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);

	if (recvBuf == NULL)
	{
	    //probably should do something here... don't really care, though.
	}
	else
	{
	    //pull off the net packet header to see what type it is.
	    np = (net_packet_t *)&(recvBuf->data[0]);
	 
	    if (np->type == DATA)
	    {
		wp = (wildfire_packet_t *)&(recvBuf->data[sizeof(net_packet_t)]);
		printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->src, np->seqno, np->pkt_dtb, np->sender_dtb, np->last_hop, wp->temp, wp->humidity, wp->wind_direction, wp->wind_speed, wp->battery);
	    }
	    else if (np->type == CONTROL)
	    {
		//we don't do anything with control packets
		//debug info:
		//printf ("received control packet\n");
		//printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
		//printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);
	    }
	    else
	    {
		//we don't care about other packets
		//debug info:
		//printf ("received a MOS packet not associated with our system\n");
		//INVALID PACKET!!!  -- We should never get here.
	    }
	    //printf ("freeing buffer\n");
	    com_free_buf(recvBuf);
	}
      
    }//end while

}//end recv
