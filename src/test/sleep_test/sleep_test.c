//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sleep_test.c                                                  */
/* Author      Charles Gruenwald   :   gruenwal@colorado.edu              */
/* Edited by   Adam Torgerson      :   adam.torgerson@colorado.edu        */
/*   Date: 7/2/04                                                         */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "dev.h"
#include "com.h"
#include "plat_clock.h"
#include "clock.h"
#include "printf.h"
#include "realtime.h"
#include "tlist.h"

uint16_t get_last_sleep_time (void);
void thread_a (void);
void thread_b (void);
void sleep_q_thread (void);
void send_my_byte();
void read_dev();
static comBuf send_pkt;

uint32_t elapsed_time;

/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start (void)
{
   //real_timer_init();
   mos_thread_new (thread_a, 128, PRIORITY_NORMAL);
   mos_thread_new (thread_b, 128, PRIORITY_NORMAL);
   mos_thread_new (sleep_q_thread, 128, PRIORITY_NORMAL);
   //printf("SLEEP TEST INITIALIZED.\n");

}

void thread_a (void)
{
   uint32_t sleep_time_a = 5000;
   uint8_t state_flag;
   printf ("thread_a: %x\n", mos_thread_current ());
   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while (1) {
      state_flag = mos_thread_get_suspend_state ();
      mos_thread_sleep (sleep_time_a);
      printf ("Thread A %l %C reading from a dev, sleeping 9000\n",
           sleep_time_a, state_flag);
      //printf("b4 read dev\n");
      read_dev();
      //printf("a4 read dev\n");
   }
}

void thread_b (void)
{
   uint32_t sleep_time_b = 7000;
   uint8_t state_flag;
   printf ("thread_b: %x\n", mos_thread_current ());
   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
   while (1) {
      state_flag = mos_thread_get_suspend_state ();
      mos_thread_sleep (sleep_time_b);
      printf ("Thread B %l %C sending a byte sleeping 20000\n",
           sleep_time_b, state_flag);
      //printf("before sb\n");
      send_my_byte();
      //printf("after sb\n");
   }  
}

void sleep_q_thread (void)
{
   printf ("sleep_q_thread: %x\n", mos_thread_current ());
   //printf ("CLOCK_SPEED_1024/100=%d\n", CLOCK_SPEED_1024/100);
   mos_thread_set_suspend_state (SUSPEND_STATE_SLEEP);
   while (1) {
      //front = mos_tlist_ptrtothread (sleep_q);
      
//      while (front != NULL) {
//	 printf ("sleepq ptr %x: %l\n", front, front->st);
//	 front = front->next;
//      }
      //front = mos_tlist_ptrtothread (ready_q);
      
      //while (front != NULL) {
	 //printf ("readyq ptr %x: %l\n", front, front->st);
	 //front = front->next;
      //}

      //    mos_led_toggle (2);
      //printf ("last_sleep_time: %d\n", get_last_sleep_time ());
      // mos_led_toggle (0);
      printf("sleepq thread, sleeping 1000 only\n");
      mos_thread_sleep (1000);
      // mos_led_toggle (1);
   }
   
   
}


void send_my_byte ()
{
   send_pkt.size = 1;
   send_pkt.data[0] = 0;
   com_send (IFACE_RADIO, &send_pkt);
}

void read_dev ()
{
   uint8_t input;

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ)
   dev_read (DEV_MICA2_LIGHT, &input, 1);
#else
   input = -1;
#endif
      
}
