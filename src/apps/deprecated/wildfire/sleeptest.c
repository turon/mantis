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

static mos_sem_t send_sem;
void recv();
void sem();
void sem_callback();
void sleeper();

static mos_alarm_t send_timer;

void start (void)
{
    //mos_sem_init(&send_sem, 0);

    /*  send_timer.kbus = 1024;
    send_timer.reset_to = 1024;
    send_timer.data = &send_timer;
    send_timer.func = sem_callback;
    mos_alarm(&send_timer);*/

    //mos_thread_new (recv, 128, PRIORITY_NORMAL);
    //mos_thread_new (sem, 128, PRIORITY_NORMAL);
    mos_thread_new (sleeper, 128, PRIORITY_NORMAL);

}

void sleeper()
{
      mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    while (1)
    {
	printf ("thread sleeping for 14 minutes\n");
	mos_thread_sleep(860160);
	printf ("thread awake");
	mos_thread_sleep(1024);
    }
    
}


void recv()
{
    while (1)
    {
	mos_led_toggle(2);
	mos_thread_sleep(1020);
	mos_show_and_clear_ext_timer();
    }

}

void sem_callback()
{
    mos_led_toggle(1);
    mos_sem_post(&send_sem);
}

void sem()
{
    while (1)
    {
	mos_sem_wait(&send_sem);
	mos_show_and_clear_ext_timer();
    }
}
