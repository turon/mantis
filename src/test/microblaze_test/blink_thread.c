//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "printf.h"
#include "led.h"
#include "sem.h"

static uint8_t blink1_continue;
static uint8_t blink2_continue;

void blink_1 (void)
{
    while (blink1_continue) {
        //        mos_sem_wait(&sem_b);
        printf("%sBlink 1!!%s\r\n", COLOR_GREEN, COLOR_NONE);
        mos_led_toggle (2);
        mos_thread_sleep(2000);
        //        mos_sem_post(&sem_a);
    }  
}


void blink_2 (void)
{
    uint32_t start_time = mos_get_realtime();

    unsigned int j;
    printf("starting blink2\r\n");
    while (blink2_continue) {
        //        uint32_t end = mos_get_realtime();
        //        mos_sem_wait(&sem_a);
        //mos_led_toggle (1);
        mos_show_and_clear_ext_timer();
        //printf("%sBlink 2!!%s\r\n", COLOR_YELLOW, COLOR_NONE);
        //uint32_t time = end-start_time;
        //printf("time:\t%d ms\r\n", time);
        //start_time = mos_get_realtime();
        mos_udelay(4000);
        //        mos_thread_sleep(1024);
        //        mos_sem_post(&sem_b);
    }
}

void start_blink1()
{
    blink1_continue = true;
    if(mos_thread_new (blink_1, 1024, PRIORITY_NORMAL) != THREAD_OK)
        show_thread_err();
}


void start_blink2()
{
    blink2_continue = true;
    if(mos_thread_new (blink_2, 1024, PRIORITY_NORMAL) != THREAD_OK)
        show_thread_err();
}

void stop_blink1() { blink1_continue = false; }
void stop_blink2() { blink2_continue = false; }

