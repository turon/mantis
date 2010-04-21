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
#include "clock.h"
#include "net.h"
#include "cc1000.h"
#include "mem.h"
#include "dvdrp_deployment.h"

void m_thread();

Thread *recv_thread;
extern Thread *net_t;

Thread *stk_chk_t;
mos_alarm_t alarm;

void wakeup(void *n_data) {
   //mos_thread_resume(stk_chk_t);

    mos_alarm(&alarm, 5, 0);
}

void stack_check() {
    stk_chk_t = mos_thread_current();
    
    alarm.func = wakeup;
    mos_alarm(&alarm, 1, 0);
	
    while(1) {
	printf("rc: %d\n", mos_check_stack(recv_thread));
//	printf("nt: %d\n", mos_check_stack(net_t));
	printf("ts: %d\n", mos_check_stack(stk_chk_t));

	mos_thread_suspend();
    }
}

void start(void) {
    //give us control over serial/rf
    //mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
    //start this thread
    mos_thread_new(m_thread, 192, PRIORITY_NORMAL);
    mos_thread_new(net_thread, 160, PRIORITY_NORMAL);
//    mos_thread_new(net_thread, 192, PRIORITY_NORMAL);
//    mos_thread_new(stack_check, 128, PRIORITY_NORMAL);
}

int8_t recv(uint8_t event_id, comBuf *pkt, uint8_t proto, uint8_t *footer) {
    dvdrp_mesg_pkt *hdr = (dvdrp_mesg_pkt *)footer;

    mos_led_toggle(0);
    
    //printf(".d;%d(htl=%C)\n", hdr->attributes[0].value, hdr->header.htl);
    printf ("l: %d t: %d\n", hdr->attributes[LIGHT_VALUE].value,
	    hdr->attributes[TEMP_VALUE].value);
    
    return 0;
}

void m_thread() {
    dvdrp_filter_list *pred;
    int8_t ret;
    dvdrp_constraint_list *clist_ptr;
    printf("Hello from DV/DRP test recv!\n");

    net_init();
    dvdrp_init();

    //turn down the power to make it easy to test multihop
    com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 1);

    
    ret = net_proto_set(DVDRP_PROTO_ID);
    if(ret == NET_PROTO_INVALID) {
	printf("Invalid proto\n");
    }

    net_event_register(1, recv);

    pred = pool_malloc(&filter_pool);
    pred->next = NULL;

    clist_ptr = pool_malloc(&constraint_pool);
    clist_ptr->name = TEMP_VALUE;
    clist_ptr->value = 0;
    clist_ptr->compare_type = DVDRP_COMPARE_GT;
    pred->constraints = clist_ptr;

    clist_ptr = pool_malloc(&constraint_pool);
    clist_ptr->name = LIGHT_VALUE;
    clist_ptr->value = 150;
    clist_ptr->compare_type = DVDRP_COMPARE_GT;
    clist_ptr->next = NULL;

    pred->constraints->next = clist_ptr;

    net_ioctl(DVDRP_PROTO_ID, DVDRP_IOCTL_SETPRED, pred, 0);

    mos_led_toggle(2);
    
    printf("Waiting for mesgs.\n\n");
}
