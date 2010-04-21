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
#include "adc.h"
#include "com.h"
#include "command_daemon.h"

void generator();


void start(void){
  mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
  mos_thread_new(generator, 128, PRIORITY_NORMAL);
}

void generator(){
   comBuf send_pkt;
   uint8_t value;

   send_pkt.size=1;

  while(1){
     send_pkt.data[0] = value++;
     com_send(IFACE_RADIO, &send_pkt);
  }
}
