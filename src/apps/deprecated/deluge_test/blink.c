//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    blink.c                                                       */
/* Author      Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 12/11/03                                                       */
/* Edited by   Adam Torgerson: adam.torgerson@colorado.edu                */
/*   Date: 12/11/03                                                       */
/* Edited by   Charles Gruenwald  :  gruenwal@colorado.edu                */
/*   Date: 04/14/0/04                                                     */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "command_daemon.h"
#include "cc1000.h"
#include "com.h"
#include "net.h"
#include "deluge.h"
#include "dev.h"
#include "simple_fs.h"
#include "mem.h"
#include <avr/pgmspace.h>
//#include "debug_locking.h"
#include "sem.h"
#include "mutex.h"
#include "rwlock.h"
#include "cond.h"
#include "deluge_impl.h"
#include "plat_dep.h"

#ifndef BLINK_VERSION
#define BLINK_VERSION 0
#endif

extern uint16_t cc1000_success_count;     //completed packets
extern uint16_t cc1000_crc_error_count;  //dropped packets due to crc error
extern uint16_t cc1000_mem_error_count;  //dropped packets due to out of memory
extern uint16_t cc1000_sync_error_count; //dropped packets due to sync failure
extern uint16_t cc1000_size_error_count; //dropped packets due to size out of range
#ifdef RADIO_USE_FEC
extern uint16_t cc1000_fec_error_count;  //dropped packets due to fec errors
#endif

const char sRadioCounts[] ARCH_PROGMEM = "success: %d, mem: %d, sync: %d, size: %d, crc: %d";
const char sFecCount[] ARCH_PROGMEM = ", fec: %d";

/*static void printErrors()
{
	printf_P(sRadioCounts, cc1000_success_count, cc1000_mem_error_count,
		cc1000_sync_error_count, cc1000_size_error_count, cc1000_crc_error_count);
	#ifdef RADIO_USE_FEC
	printf_P(sFecCount, cc1000_fec_error_count);
	#endif
	printf("\n");
	
	cc1000_success_count = 0;
	cc1000_crc_error_count = 0;
	cc1000_mem_error_count = 0;
	cc1000_sync_error_count = 0;
	cc1000_size_error_count = 0;
	#ifdef RADIO_USE_FEC
	cc1000_fec_error_count = 0;
	#endif
}
void blinkThread (void)
{
  mos_led_display(0);
  while (1) {
    mos_led_display(7);
    mos_thread_sleep (750);
    mos_led_display(0);
    mos_thread_sleep (750);
    
	#ifdef DEBUG_MEMORY			// from msched.h
    print_threads(0);
    print_memory(0);
    mos_thread_sleep(120000);
    #endif
    
    //printErrors();
    
    #ifdef DEBUG_LOCKING
	//debug_print_locking(1);
	// from deluge_proto.c
	extern sem dispatch_sem;
	extern mos_cond_t suspend_cond;
	extern mutex suspend_lock;
	extern mos_rwlock_t applist_lock;
	debug_print_sem(&dispatch_sem, 1);
	debug_print_cond(&suspend_cond, 1);
	debug_print_mutex(&suspend_lock, 1);
	debug_print_rwlock(&applist_lock, 1);
	extern deluge_app apps[2];
	if (apps[0].index != -1) debug_print_mutex(&apps[0].delugeLock, 1);
	if (apps[1].index != -1) debug_print_mutex(&apps[1].delugeLock, 1);
	#endif
   }
}

static uint8_t blink_stack[256];*/
//static uint8_t commander_stack[MOS_COMMANDER_STACK_SIZE];

void start(void)
{
  // delay to give the shell a chance to know we are running
  uint8_t i = 0;
  for (; i<6; i++) {
  	mos_led_display(7);
	mos_thread_sleep(100);
  	mos_led_display(0);
	mos_thread_sleep(100);
  }

  //printf("Blink version %C\n", BLINK_VERSION);
  com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 6);
  simple_fs_init();
  net_init();
  deluge_init();
  //mos_thread_new_havestack(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, commander_stack, PRIORITY_NORMAL);
  //mos_thread_new_havestack(blinkThread, 256, blink_stack, PRIORITY_NORMAL);
}
