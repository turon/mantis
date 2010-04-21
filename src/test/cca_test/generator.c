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



#include "mos.h"

#include "printf.h"
#include "led.h"
#include "msched.h"
#include "com.h"
#include "command_daemon.h"
#include "clock.h"

void generator();

void start(void){
   //give us control over serial/rf
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   //start this thread
   mos_thread_new(generator, 128, PRIORITY_NORMAL);
}

void generator(){
   comBuf send_pkt;
   uint8_t j;
   send_pkt.size=COM_DATA_SIZE; //make our packet as large as possible
   printf("\n");
   while(1){
      //turn on red led while sending
      
      mos_led_display(4);
      printf("sending.\n");
      for(j=0;j<50;j++)
	 com_send(IFACE_RADIO, &send_pkt);

      printf("clear.\n");
      //turn on green led while not sending
      mos_led_display(2);
      mos_thread_sleep(64);
   }
}
