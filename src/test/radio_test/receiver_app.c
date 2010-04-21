//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     receiver.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: receiver will sit in a tight while loop looking for
 * values received over the radio. Any value it receives it will then
 * show on the leds and send over the serial line using the uart. It is
 * to be used with compatable hardware (nymph/mica2).
 * 
 */

#include <inttypes.h>

#include "led.h"
#include "msched.h"
#include "printf.h"
#include "com.h"
#include "command_daemon.h"

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
#include "cc1000.h"
#endif

#ifdef RADIO_USE_FEC
extern uint16_t cc1000_fec_error_count;
#endif

void receiver();

void show_stats()
{
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
   printf("crc  errs: %d ",cc1000_crc_error_count);
#ifdef RADIO_USE_FEC
   printf("fec  errs: %d ",cc1000_fec_error_count);
#endif
   printf("size errs: %d ", cc1000_size_error_count);
   printf("mem errs: %d ", cc1000_mem_error_count);
   printf("sync errs: %d \n",cc1000_sync_error_count);
#endif
}

void start(void)
{
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   mos_register_function("stats",show_stats);
   mos_thread_new(receiver, 256, PRIORITY_NORMAL);
}


