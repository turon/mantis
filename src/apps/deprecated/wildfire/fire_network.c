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

void wind_timer_callback();
void reconnect_timer_callback();
void sleep_timer_callback();

void send();
void recv();

uint8_t rate; //for variable rate routing



//method to initialize sensing devices
void init_devs(){
    HUMIDITY_DIRE |= HUMIDITY_MASK;
    WIND_DIR_DIRE |= WIND_DIR_MASK;
    WIND_SPEED_DIRE |= WIND_SPEED_MASK;
    // ddrc |= (1 << 5)
}


void humidity_on() { HUMIDITY_PORT |=  HUMIDITY_MASK; }
void humidity_off(){ HUMIDITY_PORT &= ~HUMIDITY_MASK; }

void wind_dir_on() { WIND_DIR_PORT |=  WIND_DIR_MASK; }
void wind_dir_off(){ WIND_DIR_PORT &= ~WIND_DIR_MASK; }

void temp_on()  {TEMPERATURE_PORT |= TEMPERATURE_MASK; }
void temp_off() {TEMPERATURE_PORT &= ~TEMPERATURE_MASK; }

void light_off() {LIGHT_PORT |= LIGHT_MASK;}

//method to power wind speed 
void init_wind_speed(){

    wind_ticks = 0;
    wind_speed = 0;
    
    WIND_SPEED_PORT |= WIND_SPEED_MASK;
    // PORTC |= (1 << 5)
    
    handle_t ints = mos_disable_ints();
    
    //clear interrupt flag
    EIFR |= (1 << INTF0);
    
    //enable int0 in external int. mask
    EIMSK |= (1 << 0);
    
    //set PORTD PIN0 to input
    DDRD &= ~(1 << 0);
    
    //set intterrupt to fire on falling edge
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    
    mos_enable_ints(ints);
}

//needed because external interrupts wake the processor from sleep mode
void disable_wind_speed(){
    
    handle_t ints = mos_disable_ints();

    WIND_SPEED_PORT &= ~WIND_SPEED_MASK;
    
    //disable int0 in external int. mask
    EIMSK &= ~(1 << 0);

    //set intterrupt to fire on falling edg
    EICRA &= ~(1 << ISC01);
    EICRA |= (1 << ISC00);

    //clear interrupt flag
    EIFR |= (1 << INTF0);
    
    mos_enable_ints(ints);
}

//converts direction from ADC reading to 0-360 degrees
uint16_t convert_direction(uint16_t sample)
{
    return (uint16_t)(((float)sample)/(1024) * 360);
}

//converts temp from ADC reading to temp in F
uint16_t convert_temp(uint16_t sample)
{
    uint16_t ret;
    ret = pgm_read_byte(&temps_vals[sample]);
    return ret; 
}

//converts humidity from ADC reading to RH in %
uint16_t convert_humidity(uint16_t sample, uint16_t battery)
{
    uint32_t humid;
    humid = (uint32_t)battery * (uint32_t)sample;
    humid = humid / 1024;
    humid = humid * 195;
    humid = humid / 5;
    humid = humid / 1000;
    humid = humid - 38.5;
    return (uint16_t)humid;
}



void start (void)
{

    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);
    mos_mdelay(200);
    found = 0;
    rate = 0;

    //com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x01);

    myID = mos_node_id_get();     //preset
    baseID = 0;                   //updated with the first packet.. should be 0
    myParentID = 0;               //updated with the first packet
    myDTB = 255;                  //lost value -- updated with the first packet
    mySeqNo = 0;                  //start on 0
    state = INIT;                 //initial state

    //initialize timer callbacks 
    wind_timer.func = wind_timer_callback;  
    sleep_timer.func = sleep_timer_callback;
    reconnect_timer.func = reconnect_timer_callback;  
   
    //set timer values
    wind_timer.msecs = WIND_DELAY;
    reconnect_timer.msecs = RECONNECT_DELAY;  
    sleep_timer.msecs = TO_NEXT_SLEEP;

    //we set the timers manually
    wind_timer.reset_to = 0;
    reconnect_timer.reset_to = 0;
    sleep_timer.reset_to = 0;

    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);
}



//updates the wind total
/*** this doesn't completely work.  sometimes get bogus wind values.  esp after sleeping ***/
void wind_timer_callback()
{
    wind_speed = wind_ticks;
    wind_ticks = 0;

    if (state == SLEEPNODE)
    {
	//if we're supposed to sleep, turn off the timer
	wind_timer.reset_to = 0;
	return;
    }

    wind_timer.reset_to = WIND_DELAY;
}

//running in the background.. times out every 10 seconds and sees if we've
//received a control packet from our parent.  If not.. find a new one.
void reconnect_timer_callback()
{
    if (found == 0)
    {  // lost our parent... need a new one
	myDTB = 255;
    }
    else {
	// printf ("still got it!\n");
    }
    found = 0;  //reset our found value
    
    if (state == SLEEPNODE)
    {
	reconnect_timer.reset_to = 0;
	return;
    }
    
    reconnect_timer.reset_to = RECONNECT_DELAY;
}


void sleep_timer_callback()
{
    state = SLEEPNODE;
}


void send()
{ 
    wildfire_packet_t * wp;
    net_packet_t * np;
    uint16_t sample;
    uint8_t sends;
    
    sends = 0;

    init_devs();
    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //sets power save mode

    //just wait while in init state
    while (state != GO)
    {
	mos_mdelay(500);
    }

    while (1)
    {
	if (state == SLEEPNODE)
	{

	    disable_wind_speed();   //must disable the ext interrupts for power save sleep
	    //mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //sets power save mode
	    mos_thread_sleep(SLEEP_INTERVAL - 1000); //extra 1000 to make sure it wakes up early
	    //mos_thread_set_suspend_state(SUSPEND_STATE_IDLE); //idle mode
	    
	    while (state != GO)  //awake, but not ready yet.. so just wait
		mos_mdelay(250);
	    sends = 0;

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

	//battery
	dev_open(DEV_MICA2_BATTERY);
	dev_read(DEV_MICA2_BATTERY, &sample, sizeof(sample));
	dev_close(DEV_MICA2_BATTERY);
	wp->battery = sample;

	//humidity
	humidity_on();
	mos_mdelay(200); //settling time
	dev_open(DEV_ADC);
	dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_2);
	dev_read(DEV_ADC, &sample, sizeof(sample));
	dev_close(DEV_ADC);
	humidity_off();
	wp->humidity = convert_humidity(sample, wp->battery);

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

	sendBuf.size += sizeof(wildfire_packet_t);

	//debug info:
        /*	printf("type\tsrc\tdest\tnext\tlast\tseq\tpktdtb\ts_dtb\n");
  printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", np->type, np->src, np->dest, np->next_hop, np->last_hop, np->seqno, np->pkt_dtb, np->sender_dtb);

  printf("temp\thmdty\tw_dir\tw_spd\tbat\n");
  printf("%d\t%d\t%d\t%d\t%d\n", wp->temp, wp->humidity, wp->wind_direction, wp->wind_speed, wp->battery);
        */    
    
	if (rate == 0 || sends < rate*myDTB)
	{
	    com_send(IFACE_RADIO, &sendBuf);
	    mySeqNo++;
	    sends++;
	}
	if (mySeqNo > 254)
	    mySeqNo = 0;
	mos_mdelay(1000);
    }
}


      
void recv()
{
    comBuf * recvBuf;
    net_packet_t * np;
    wildfire_packet_t * wp;

    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //sets power save mode
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
    sendBuf.size += sizeof(wildfire_packet_t);  //make the same size as a data packet
 
    //this is the initial stage.. blinks green in range of another node
    //blinks red when not in range of another node
    mos_led_off(LED_YELLOW);
    
    while (state == INIT)
    {
	recvBuf = com_recv_timed(IFACE_RADIO, COM_RECV_INIT_TIMEOUT);    
	if (recvBuf  == NULL)
	{  
	    //didn't hear any packets, so send one of my own
	    np = (net_packet_t *)&(sendBuf.data[0]);
	    np->type = FINDME;
	    np->src = myID;
	    np->dest = 255;
	    com_send(IFACE_RADIO, &sendBuf);
	    mos_led_off(LED_GREEN);
	    mos_led_toggle(LED_RED);
	}
	else{
	    //heard a packet
	    np = (net_packet_t *)&(recvBuf->data[0]);
	    if (np->type == FINDME && np->dest != myID)
	    {
		//heard a broadcast findme packet, return it
		np->dest = np->src;
		np->src = myID;
		com_send(IFACE_RADIO, recvBuf);
	    }
	    else if (np->type == FINDME && np->dest == myID)
	    {
		//heard a findme packet response for me.. blink green LED, tell them I heard them too
		np->dest = np->src;
		np->src = myID;
		mos_led_off(LED_RED);
		mos_led_toggle(LED_GREEN);
		mos_mdelay(500);
		com_send(IFACE_RADIO, recvBuf);
	    }
	    else if (np->type == CONTROL)
	    {
		//must be put into an already running network so start sensing and routing
		//first forward the control packet on
		myDTB = np->sender_dtb +1;
		np->sender_dtb = myDTB;
		com_send(IFACE_RADIO, recvBuf);
		next_sleep = np->seqno;
		rate = np->rate;
		found = 1;  //heard a control, so I know I'm connected

		//set next sleep timer based on what control seqno i heard.
		sleep_timer.msecs = TO_NEXT_SLEEP - next_sleep * 1024 * 4;

		//start timers
		wind_timer.msecs = WIND_DELAY;
		reconnect_timer.msecs = RECONNECT_DELAY;  
		init_wind_speed();
    		mos_alarm (&sleep_timer);
		mos_alarm (&wind_timer);
		mos_alarm (&reconnect_timer);
		state = GO;
	    }
	    com_free_buf(recvBuf);
	}
    }
  
    mos_led_off(LED_GREEN);
    mos_led_off(LED_RED);
    
    while (1)
    {
	if (state == SLEEPNODE)
	{
	    
	    //mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);  //power save sleep mode
	    com_mode(IFACE_RADIO, IF_OFF);                      //radio off
	    mos_thread_sleep(SLEEP_INTERVAL - COM_RECV_TIMEOUT);
	    com_mode(IFACE_RADIO, IF_LISTEN);                   //radio on
	    //mos_thread_set_suspend_state(SUSPEND_STATE_IDLE);   //idle mode
	    state = READYNODE;  //ready node state is awake but waiting for a control packet before sending
	}

        //wait just over 5 seconds to recv before timeout
	recvBuf = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);

	if (recvBuf == NULL)
	{
	    //should probably do something here, but just means we haven't heard a packet
	}
	else
	{
	    //pull off the net packet to see what type it is.
	    np = (net_packet_t *)&(recvBuf->data[0]);
	    if (np->type == DATA)
	    {
		wp = (wildfire_packet_t *) & (recvBuf->data[sizeof(net_packet_t)]);
		if (myDTB < np->pkt_dtb && np->src != myID && np->last_hop != myID && np->pkt_dtb != 255)
		{
		    //normal case:
		    //my dtb is less than the packet
		    //I'm not the last hop (someone sending me my packet)
		    //I'm not resending one of my packets (!= src)
		    //the sender is not lost (pkt 255)
		    //so forward the packet on to base

		    np->last_hop = myID;
		    np->pkt_dtb -= 1;
		    com_send(IFACE_RADIO, recvBuf);
		}
		else if (myDTB < np->pkt_dtb && np->src != myID && np->last_hop != myID && np->pkt_dtb == 255)
		{
		    //if the sender is lost, and I'm not lost, and this is the first hop of the lost packet
		    //update the packet_dtb and send to base
		    np->last_hop = myID;
		    np->pkt_dtb = myDTB;
		    com_send(IFACE_RADIO, recvBuf);
		}

		//debugging info:
		//printf("temp\thmdty\tw_dir\tw_spd\tbat\n");
		//printf("%d\t%d\t%d\t%d\t%d\n", wp->temp, wp->humidity, wp->wind_direction, wp->wind_speed, wp->battery);
	    }
	    else if (np->type == CONTROL)
	    {
		if (state == READYNODE)
		{
		    //this is the first control packet since i woke up
		    //get the time until I need to sleep again.
		    next_sleep = np->seqno;
		    rate = np->rate;
		    sleep_timer.msecs = TO_NEXT_SLEEP - next_sleep * 1024 * 4;
		    //restart all my timers
		    wind_timer.msecs = WIND_DELAY;
		    reconnect_timer.msecs = RECONNECT_DELAY;  
		    init_wind_speed();     //re-enable wind speed measurement
		    mos_alarm (&wind_timer);
		    mos_alarm (&reconnect_timer);
		    mos_alarm (&sleep_timer);
		    state = GO; //set state to GO so send thread starts
		}
		if (np->pkt_dtb < myDTB)
		{
		    //it's a control packet from a closer node to base
		    //update the information and send it on.
		    np->next_hop = np->last_hop;
		    np->last_hop = myID;
		    np->pkt_dtb += 1;
		    myDTB = np->pkt_dtb;
		    found = 1;  //race condition.. but I don't think we care.
		    //this isn't used... yet.
		    if (np->sender_dtb == 0)
		    {
			//baseID = np->src;
		    }
		    com_send(IFACE_RADIO, recvBuf);
		}
	    }
	    else
	    {
		//printf ("received a mos packet from somewhere else\n");
		//we don't care about other packets
	    }
	    com_free_buf(recvBuf);
	}
	
    }//end while
    
}//end recv


SIGNAL (SIG_INTERRUPT0)
{
    wind_ticks++;
}


