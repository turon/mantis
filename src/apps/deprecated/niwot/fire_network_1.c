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
#include "fire.h"
#include "cc1000.h"
#include "sem.h"

#define RECONNECT_INTERVAL 10 //10 seconds
#define WIND_INTERVAL 2

static comBuf sendBuf;
static uint8_t myID;
static uint8_t baseID;
static uint8_t myParentID;
static uint8_t myDTB;
static uint8_t mySeqNo;
static uint8_t found;
static uint8_t next_sleep;

static uint8_t state;


uint16_t wind_ticks;
uint16_t wind_speed;
mos_alarm_t wind_timer;
mos_alarm_t reconnect_timer;
mos_alarm_t sleep_timer;

mos_sem_t state_transition_sem;

uint32_t wind_timer_ms = 225; /*-------------new*/
//uint32_t wind_timer_ms = 2250; /*-------------old*/
void wind_timer_callback();

uint32_t reconnect_timer_ms = 1000; /*-----------new*/
//uint32_t reconnect_timer_ms = 10000; /*-----------old*/
void reconnect_timer_callback();

uint32_t sleep_timer_ms = 6000; /*----------new*/
//uint32_t sleep_timer_ms = 60000; /*----------old*/
void sleep_timer_callback();

void alarm_set();
void send();
void recv();


//method to initialize devices
void init_devs(){
   HUMIDITY_DIRE |= HUMIDITY_MASK;
   WIND_DIR_DIRE |= WIND_DIR_MASK;
   WIND_SPEED_DIRE |= WIND_SPEED_MASK;
}


void humidity_on() { HUMIDITY_PORT |=  HUMIDITY_MASK; }
void humidity_off(){ HUMIDITY_PORT &= ~HUMIDITY_MASK; }

void wind_dir_on() { WIND_DIR_PORT |=  WIND_DIR_MASK; }
void wind_dir_off(){ WIND_DIR_PORT &= ~WIND_DIR_MASK; }


//method to power wind speed 
void init_wind_speed(){

   WIND_SPEED_PORT |= WIND_SPEED_MASK;

   handle_t ints = mos_disable_ints();

   //enable int0 in external int. mask
   EIMSK |= (1 << 0);

   //set PORTD PIN0 to input
   DDRD &= ~(1 << 0);

   //set intterrupt to fire on falling edge
   EICRA |= (1 << ISC01);
   EICRA &= ~(1 << ISC00);
   
   mos_enable_ints(ints);
}


uint16_t convert_direction(uint16_t sample)
{
  return (uint16_t)(((float)sample)/(1024) * 360);
}

uint16_t convert_temp(uint16_t sample)
{
  uint16_t ret;
  ret = pgm_read_byte(&temps_vals[sample]);
  return ret; 
}

uint16_t convert_humidity(uint16_t sample)
{
    //uint32_t humid;
    //humid = 117 * sample;
    //humid = humid / 1024;
    //humid -= 38;
    //return (uint16_t)humid; 
    return sample;
}



void start (void)
{

    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);
    //mos_thread_sleep(40); /*------------new*/
    //mos_thread_sleep(200); /*------------old*/
    found = 0;

    //com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x01);

    myID = mos_node_id_get(); //preset
    baseID = 0;  //updated with the first packet
    myParentID = 0; //updated with the first packet
    myDTB = 255; //updated with the first packet
    mySeqNo = 0;
    state = INIT;
    //state = GO;

    //initialize timer for the wind speed 
    wind_timer.func = wind_timer_callback;  
    sleep_timer.func = sleep_timer_callback;
    reconnect_timer.func = reconnect_timer_callback;  

    wind_timer.reset_to = 0;
    reconnect_timer.reset_to = 0;
    sleep_timer.reset_to = 0;

    mos_sem_init(&state_transition_sem, 0);

    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);
 
}




void wind_timer_callback()
{
    printf("wind_timer_callback() entered\n"); //--------------------------------------------------------------------------------
    wind_speed = wind_ticks;
    wind_ticks = 0;

    if (state == SLEEPNODE)
    {
       wind_timer.reset_to = 0;
       return;
    }
    
 
    wind_timer.reset_to = wind_timer_ms;
}

//running in the background.. times out every 10 seconds and sees if we've
//received a control packet from our parent.  If not.. find a new one.
void reconnect_timer_callback()
{
    printf("reconnect_timer_callback() entered\n"); //--------------------------------------------------------------------------------
    if (found == 0)
    {  // lost our parent... need a new one
	myDTB = 255;
    }
    else {
	// printf ("still got it!\n");
    }
    found = 0;
    
    if (state == SLEEPNODE)
    {
       reconnect_timer.reset_to = 0;
       return;
    }
    
    reconnect_timer.reset_to = reconnect_timer_ms;

    
}


void sleep_timer_callback()
{
    printf("sleep_timer_callback() entered\n"); //--------------------------------------------------------------------------------
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
        mos_thread_sleep(100); /*------------new*/
	//mos_thread_sleep(1000); /*------------old*/
    }

    init_devs();
    init_wind_speed();
    
    while (1)
    {
	//printf ("send thread is working\n");
	//check to see if I should be sleeping
	if (state == SLEEPNODE)
	{   
	    mos_led_toggle (LED_RED); //-----------------------------------------------------
	    printf("send thread is asleep\n"); //---------------------------------------------------
	    mos_thread_sleep(SLEEP_INTERVAL);
	    printf("send thread is awake\n"); //---------------------------------------------------
	    while (state != READYNODE)
	    {
	       mos_thread_sleep(50); /*---------new*/
	       //mos_thread_sleep(500); /*---------old*/
	    }   
	    mos_sem_post(&state_transition_sem);
	    mos_led_toggle (LED_RED); //-----------------------------------------------------
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
	
	//for data packets, we now populate the buffer with the 
	//wildfire packet information.  Adding it to the end.
	wp = (wildfire_packet_t *)&(sendBuf.data[sendBuf.size]);
	//temp
	dev_open(DEV_MICA2_TEMP);
	dev_read(DEV_MICA2_TEMP, &sample, sizeof(sample));
	dev_close(DEV_MICA2_TEMP);
	wp->temp = convert_temp(sample);

	//humidity
	humidity_on();
	//mos_mdelay(40); /*----------------new*/
	//mos_mdelay(200); //1 second settling time /*----------------old*/
	dev_open(DEV_ADC);
	dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_2);
	dev_read(DEV_ADC, &sample, sizeof(sample));
	dev_close(DEV_ADC);
	humidity_off();
	wp->humidity = convert_humidity(sample);

	//wind direction
	wind_dir_on();
	dev_open(DEV_ADC);
	dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_1);
	dev_read(DEV_ADC, &sample, sizeof(sample));
	dev_close(DEV_ADC);
	wind_dir_off();
	wp->wind_direction = convert_direction(sample);

	//wind speed
	wp->wind_speed = wind_speed;

	//battery
	dev_open(DEV_MICA2_BATTERY);
	dev_read(DEV_MICA2_BATTERY, &sample, sizeof(sample));
	dev_close(DEV_MICA2_BATTERY);
	wp->battery = sample;
	sendBuf.size += sizeof(wildfire_packet_t);

//4 lines below, comment later//-------------------------------------------------------------------------------------------------------	
	
	printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
	printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);

	printf("temp\thmdty\tw_dir\tw_spd\tbat\n");
	printf("%d\t%d\t%d\t%d\t%d\n", wp->temp, wp->humidity, wp->wind_direction, wp->wind_speed, wp->battery);
  
//top 4 lines above, comment later//---------------------------------------------------------------------------------------------------    
    
	
	com_send(IFACE_RADIO, &sendBuf);
	mySeqNo++;
	if (mySeqNo > 254)
	    mySeqNo = 0;
	mos_thread_sleep(100); /*----------------new*/
	//mos_thread_sleep(1000); /*----------------old*/
    }
}


void alarm_set()
{
    sleep_timer.msecs = sleep_timer_ms;
    wind_timer.msecs = wind_timer_ms;
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
	
	recvBuf = com_recv_timed(IFACE_RADIO, 1600); /*--------------new*/
	//recvBuf = com_recv_timed(IFACE_RADIO, 16000); /*--------------old*/    
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
		mos_thread_sleep(40); /*-----------new*/
		//mos_thread_sleep(400); /*-----------old*/
		com_send(IFACE_RADIO, &sendBuf);
	    }
	    else if (np->type == CONTROL)
	    {
		printf ("received control\n");
		//must be put into an already running network
		myDTB = np->sender_dtb +1;
		np->sender_dtb = myDTB;
		com_send(IFACE_RADIO, recvBuf);
		next_sleep = np->seqno;
		found = 1;
		sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 600; /*------------new*/
		//sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 1000 * 4; /*------------old*/
		printf("setting 1 sleep for %d\n", sleep_timer_ms); //-----------------------------comment later
		alarm_set();
		mos_alarm (&sleep_timer);
		mos_alarm (&wind_timer);
		mos_alarm (&reconnect_timer);
		printf("all timers posted\n"); //--------------------------------------------------------------------------------
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
	    sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 600; /*------------new*/
            //sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 1000 * 4; /*------------old*/
	    printf("setting 1 sleep for %d\n", sleep_timer_ms); //--------------------------comment later
	    alarm_set();
	    mos_alarm (&sleep_timer);
	    mos_alarm (&wind_timer);
	    mos_alarm (&reconnect_timer);
	    printf("all timers posted\n"); //--------------------------------------------------------------------------------
	    state = GO;
	}
	com_free_buf(recvBuf);
    }
    
    while (1)
    {
	//check to see if I should be sleeping
	if (state == SLEEPNODE)
	{   mos_led_toggle (LED_YELLOW);
	    com_mode(IFACE_RADIO, IF_OFF);
	    printf("recv thread is asleep\n"); //---------------------------------------------------
	    mos_thread_sleep(SLEEP_INTERVAL);
	    printf("recv thread is awake\n"); //---------------------------------------------------
	    com_mode(IFACE_RADIO, IF_LISTEN);
	    state = READYNODE;
	    mos_led_toggle (LED_YELLOW);
	}

        //wait just over 5 seconds to recv before timeout
	//5125 is Carl's original value, might need to check
	recvBuf = com_recv_timed(IFACE_RADIO, 512); /*--------------new*/
	//recvBuf = com_recv_timed(IFACE_RADIO, 5125); /*--------------old*/

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
		printf("received data packet\n"); //-------------------------------------------------------comment later
		wp = (wildfire_packet_t *) & (recvBuf->data[sizeof(net_packet_t)]);
		printf("received a data packet from %d at %d hops\n", np->src, np->sender_dtb);//------------------------------comment later
		printf("mydtb = %d and pkt_dtb = %d\n", myDTB, np->pkt_dtb);//-------------------------------------------------comment later
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
		
	    
		//just debugging stuff
		//printf("temp\thmdty\tw_dir\tw_spd\tbat\n");
		//printf("%d\t%d\t%d\t%d\t%d\n", wp->temp, wp->humidity, wp->wind_direction, wp->wind_speed, wp->battery);


	    }
	    else if (np->type == CONTROL)
	    {
		//printf ("received control\n");
		if (state == READYNODE)
		{
		    //this is the first control packet since i woke up
		    //get the time until I need to sleep again.
		    next_sleep = np->seqno;
		    sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 600; /*------------new*/
                    //sleep_timer_ms = TO_NEXT_SLEEP - next_sleep * 1000 * 4; /*------------old*/
		    printf("setting sleep for %d\n", sleep_timer_ms); //-------------------------comment later
		    alarm_set();
		    mos_alarm (&sleep_timer);
		    mos_alarm (&wind_timer);
		    mos_alarm (&reconnect_timer);
		    printf("all timers posted\n"); //--------------------------------------------------------------------------------
		    mos_sem_wait(&state_transition_sem);
		    state = GO;
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


SIGNAL (SIG_INTERRUPT0)
{
    wind_ticks++;
}


