//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    simple_proto_example.c                                        */
/* Author      Carl Hartung   :  carl.hartung@colorado.edu                */
/*   Date: 11/12/04                                                       */
/*                                                                        */
/* This is a simple program that demonstrates the ability of the net      */
/* Layer.                                                                 */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "simple_proto.h"


static comBuf mybuf;
static comBuf mybuf2;

void send();
void recv();
void recv2();
void recv3();
void send2();


/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void){
  /* must start the net thread */
  mos_thread_new(net_thread, 128, PRIORITY_NORMAL);


  /* our sending and receiving threads */
  mos_thread_new(recv, 128, PRIORITY_NORMAL);
  mos_thread_new(recv2, 128, PRIORITY_NORMAL);
  mos_thread_new(recv3, 128, PRIORITY_NORMAL);
  //mos_thread_new(send, 128, PRIORITY_NORMAL);
  //mos_thread_new(send2, 128, PRIORITY_NORMAL);
}


/* listens on port 40 and toggles led 0 on recv */
void recv()
{
   comBuf * buffer;
   
   simple_proto_init();
   
   while (1)
   {
      buffer = net_recv(40);
      mos_led_toggle (0);
      com_free_buf(buffer);	
   }
}

/* listens on port 30 and tottles led 1 on recv */
void recv2()
{
   comBuf * buffer;
   simple_proto_init();
   
   while (1)
   {
      buffer = net_recv(30);
      mos_led_toggle (1);
      com_free_buf(buffer);
   }
}

/* listens on port 20 and toggles led 2 on recv */
void recv3()
{
   comBuf * buffer;
   simple_proto_init();
   
   while (1)
   {
      buffer = net_recv(20);
      mos_led_toggle (2);
      com_free_buf(buffer);
   }
}



/* alternates sending on ports 40, 30, 20 (in that order).  Sends 1 packet every 2 seconds */
void send()
{
  uint8_t i;
  uint32_t sleep_time = 2000;
  uint8_t port;

  
  mybuf.size=0;
  port = 40;
  for (i = 0; i < 10; i++)
  {
     mybuf.data[mybuf.size] = i;
     mybuf.size++;
  }
  
  simple_proto_init();

  while (1) {
     mos_led_toggle (0);
     if (port == 40)
       port = 30;
     else if (port == 30)
       port = 20;
     else if (port == 20)
       port = 40;
     net_send(&mybuf, SIMPLE_PROTO_ID, port, NULL);     
     mos_thread_sleep (sleep_time);
  }
}
 

/* sends only on port 30.  Sends 1 packet every 2 seconds */
void send2()
{
   
  uint8_t i;
  uint32_t sleep_time = 2000;
  uint8_t port;

  
  mybuf2.size=0;
  port = 30;
  for (i = 0; i < 10; i++)
  {
     mybuf2.data[mybuf.size] = i;
     mybuf2.size++;
  }
  
  simple_proto_init();

  while (1) {
     mos_led_toggle (1);
     net_send(&mybuf2, SIMPLE_PROTO_ID, port, NULL);     
     mos_thread_sleep (sleep_time);
  }
}
 
