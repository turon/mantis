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
#include "uart.h"
#include "msched.h"
#include "command_daemon.h"
#include "com.h"


void receiver();

void start(void){
   //give us control over the serial/rf
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   //gives us start
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
}

void receiver(){

   int i;
   comBuf *recv_pkt;
   printf("Radio to uart initialized....\n"); //hello
   com_mode(IFACE_RADIO, IF_LISTEN);
   while(1){
      mos_led_toggle(1);
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet

      //print out as a string.
      //printf("received [%d] bytes:%s\n",recv_pkt->size,recv_pkt->data);

      //print out as data
      printf("Packet[%C]:",recv_pkt->size);
      for(i=0;i<recv_pkt->size;i++)
	 printf("%C ",recv_pkt->data[i]);
      printf("\n");
      com_free_buf(recv_pkt);//free the packet back to the pool
   }
}

