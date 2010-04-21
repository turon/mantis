//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     dvdrp_test.c      
 * Author:   Cyrus Hall - hallcp@cs.colorado.edu
 * Date:     07-20-2004
 */

#include <inttypes.h>
#include <stdio.h>

#include "led.h"
#include "msched.h"
#include "dvdrp.h"
#include "net.h"
#include "clock.h"
#include "cc1000.h"
#include "clock.h"
#include "mem.h"
#include "dev.h"
#include "sem.h"
#include "node_id.h"

#include "dvdrp_deployment.h"

static comBuf stack_buf;

#define ATTRIB_COUNT 3

dvdrp_attrib attribs[ATTRIB_COUNT];
mos_alarm_t alarm;

Thread *main_t;
Thread *stk_chk_t;
extern Thread *net_t;
static sem wakeup_sem;
void generator ();

mos_alarm_t alarm_stk;

void wakeup_stk(void *n_data) {
   mos_thread_resume(stk_chk_t);

   mos_alarm(&alarm_stk, 5, 0);
}

void stack_check() {
   stk_chk_t = mos_thread_current();

   alarm_stk.func = wakeup_stk;
   mos_alarm(&alarm_stk, 1, 0);

   while(1) {
//	printf("ge: %d\n", mos_check_stack(main_t));
//	printf("nt: %d\n", mos_check_stack(net_t));
//	printf("ts: %d\n", mos_check_stack(stk_chk_t));

      mos_thread_suspend();
   }
}

void start (void)
{
   mos_sem_init(&wakeup_sem, 1);
   //give us control over serial/rf
   //mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   //start this thread
   mos_thread_new(generator, 192, PRIORITY_NORMAL);
   mos_thread_new(net_thread, 256, PRIORITY_NORMAL);
//    mos_thread_new(stack_check, 128, PRIORITY_NORMAL);
}

void wakeup(void *n_data) {
   mos_sem_post(&wakeup_sem);
//    mos_thread_resume(main_t);

//    mos_alarm(&alarm, 1, 0);
}

void generator ()
{
   int8_t ret;
   uint8_t i;

   printf("Hello from DV/DRP test!\n");
   net_init();
   dvdrp_init();

//    main_t = mos_thread_current();

   com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 1);

   //indexes don't neet to match name values
   attribs[TEMP_VALUE].name = TEMP_VALUE;
   attribs[LIGHT_VALUE].name = LIGHT_VALUE;
   attribs[NODEID_VALUE].name = NODEID_VALUE;


   //fill in data
   attribs[NODEID_VALUE].value = mos_node_id_get ();

   ret = net_proto_set(DVDRP_PROTO_ID);

   if(ret == NET_PROTO_INVALID) {
      printf("Invalid proto\n");
   }

   alarm.func = wakeup;
   i = 0;
   while(1) {
      stack_buf.size = 0; 

      dev_mode (DEV_MICA2_TEMP, DEV_MODE_ON);
      attribs[TEMP_VALUE].value = 0;
      dev_read (DEV_MICA2_TEMP, &attribs[TEMP_VALUE].value, 1);
      attribs[LIGHT_VALUE].value = 0;
      dev_read (DEV_MICA2_LIGHT, &attribs[LIGHT_VALUE].value, 1);
      dev_mode (DEV_MICA2_LIGHT, DEV_MODE_OFF);

      //1 = # of attributes
      printf("sending, light is %C.\n", attribs[LIGHT_VALUE].value);
      ret = net_event_send(1, &stack_buf, ATTRIB_COUNT, &attribs);
      if(ret)  //ret == 0 if no valid routes
	 mos_led_on(0);
      //mos_led_toggle(1);
//	printf("crc  errs: %d ",cc1000_get_crc_errors());
//        printf("sync errs: %d \n",cc1000_get_sync_errors());

	
//	timeout_attach(t, wakeup);
//	mos_thread_suspend();
      mos_alarm(&alarm, 1, 0);
      mos_sem_wait(&wakeup_sem);
	
   }

   printf("End test.\n");
}
