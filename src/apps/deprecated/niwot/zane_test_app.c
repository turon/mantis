//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

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
// #include "mica2-light-temp.h"
// #include "fire.h"
#include "cc1000.h"
#include "sem.h"

#define GO 35
#define SLEEP 14

static comBuf test_pkt;
static mos_alarm_t sleep_timer;

uint32_t sleep_ms = 10000;  //sleeps for 10 seconds, will wake for 5 seconds, giving roughly a 33% duty cycle
uint32_t sleep_timer_ms = 5000;

uint8_t battery;

void sleep_timer_callback();

static uint8_t state;

void send();
void recv(); //for this app, will not really receive

void start (void)
{

    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);
    mos_thread_sleep(200);

    com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0xff); //max radio power = 0xff, min = 0x01

       
    sleep_timer.func = sleep_timer_callback;  
    sleep_timer.msecs = sleep_timer_ms;
    sleep_timer.reset_to = 0;
    
    mos_alarm (&sleep_timer);
    
    state = GO;
    
    mos_thread_new (send, 128, PRIORITY_NORMAL);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);
 
}

void sleep_timer_callback()
{
    state = SLEEP;
}

void send()
{
    test_pkt.size = 1;//1 byte
    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    mos_led_toggle(0);
    
    while(1)
    {
        mos_thread_sleep(50);
    	if(state == SLEEP)
	{
	    printf("send sleeps\n");
	    mos_led_off(0);	    
	    mos_thread_sleep(sleep_ms);
	    sleep_timer.msecs = sleep_timer_ms;
	    mos_alarm(&sleep_timer);
	    state = GO;
	    printf("send wakes\n");
	    mos_led_toggle(0);
	}
	
	
//      dev_open(DEV_MICA2_BATTERY);
// 	dev_read(DEV_MICA2_BATTERY, &battery, sizeof(battery));
// 	dev_close(DEV_MICA2_BATTERY);
// 	
 	test_pkt.data[0] = 0;
// 	
// 	//mos_led_toggle(2);
	com_send(IFACE_RADIO, &test_pkt);
	printf("sent packet\n");
	mos_thread_sleep(500);
// 	//mos_led_toggle(2);
    }    
}

void recv()
{   
    mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    mos_led_toggle(1);
    
    while(1)
    {
    
    mos_thread_sleep(50);
    
        if(state == SLEEP)
	{
	    printf("recv sleeps\n");
	    mos_led_off(1);
	    mos_thread_sleep(sleep_ms);
	    while(state != GO)
	    {
	        mos_thread_sleep(50);
	    }
	    printf("recv wakes\n");
	    mos_led_toggle(1);    
	}
	
	mos_mdelay(100);
	
    }
}







