//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     receiver.c      
 * Author:   Charles Gruenwald III
 * Date:     03-24-2004
 *
 * Description: receiver will sit in a tight while loop looking for
 * values received over the radio. Any value it receives it will then
 * show on the leds and send over the serial line using the uart. It is
 * to be used with compatable hardware (nymph/mica2).
 * 
 */
#include <inttypes.h>

#include "led.h"
#include "com.h"     //give us the communication layer
#include "printf.h"
#include "msched.h"

void receiver();

void start(void)
{
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
}


void receiver(){
   comBuf *recv_pkt;                     //give us a packet pointer
   
   com_mode(IFACE_RADIO, IF_LISTEN);
   
   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      printf("%d\n",recv_pkt->data[0]); //show packet's content as a string over serial port
      com_free_buf(recv_pkt);             //free the recv'd packet to the pool
      
      mos_led_toggle(0);
   }
}
