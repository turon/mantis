//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     generator.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: generator will repeatedly send the numbers 1,2,3...255 
 * over the radio and display the last 3 bits in binary on the LEDs. it 
 * is to be used with receiver.c to test the radios on compatable 
 * hardware (nymph/mica2).
 */

#include <inttypes.h>

#include "led.h"
#include "msched.h"
#include "tlist.h"
#include "com.h"
#include "command_daemon.h"
#include "node_id.h"
#include "printf.h"
#include "sem.h"

extern uint8_t packet_size;

void set_size()
{
   packet_size = prompt_long("new size of packet:");
}

void generator(); //in generator.c file

void start (void)
{
   //give us control over serial/rf
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   //mos_register_function("set_size", set_size);
   //start this thread
   mos_thread_new(generator, 128, PRIORITY_NORMAL);

   //while(1);
   
}
