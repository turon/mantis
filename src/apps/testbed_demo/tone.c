//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


#include <inttypes.h>
#include "led.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "adc.h"
#include "led.h"
#include "printf.h"
#include "sem.h"
#include "dev.h"
//#include "serial.h"
#include "command_daemon.h"
#include "uart.h"
#include "net.h"
comBuf send_pkt;
comBuf b_pkt;

uint16_t count=0;
uint16_t delay=1000;
uint16_t length=1;


int8_t tone[] = {65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65, 65};

//                 127, 126, 124, 121, 117, 112, 105, 98, 89, 80, 70, 59, 48, 36, 24, 12, 0, -12, -24, -36, -48, -59, -70, -80, -89, -98, -105, -112, -117, -121, -124, -126, -127, -126, -124, -121, -117, -112, -105, -98, -89, -80, -70, -59, -48, -36, -24, -12, 0, 12, 24, 36, 48, 59, 70, 80, 89, 98, 105, 112, 117, 121, 124, 126};



void recv_thread()
{
   com_mode(IFACE_SERIAL2, IF_LISTEN);
   comBuf *recv_pkt;
   while(1)
   {
      
      recv_pkt = com_recv(IFACE_SERIAL2);
      mos_led_toggle(2);
      if (((uint32_t)recv_pkt->data[0])==0)
      {
         mos_led_toggle(1);
         com_free_buf(recv_pkt);
#if defined(PLATFORM_TELOSB)
         com_send(IFACE_SERIAL2,&b_pkt);
#else
         com_send(IFACE_SERIAL,&b_pkt);
#endif
      }
      else
      {
         
         delay = ((uint16_t*)& recv_pkt->data)[0];
         length = ((uint16_t*)& recv_pkt->data)[1];
         printf ("Delay %d, Length %d\n", delay,length);
         com_free_buf(recv_pkt);
      }
      
   }
}

/*void test()
{
   printf("this thread still running.\n");
}

void set_delay()
{
   delay = prompt_long("#:");
}
void set_length()
{
   send_pkt.size = prompt_uint8("#:");
//   =length;
}*/


void send_thread()
{
   send_pkt.size=length;
   for (count=0;count< 64;count++)
      send_pkt.data[count]=tone[count];
      
   while(1) 
   {    
      mos_led_toggle(0);
      send_pkt.size = length>64?64:length;
#if defined(PLATFORM_TELOSB)
      com_send(IFACE_SERIAL2,&send_pkt);
#else
      com_send(IFACE_SERIAL,&send_pkt);
#endif
      mos_mdelay(delay);
   }

}

void start (void)
{
   b_pkt.size = 1;
   b_pkt.data[0]='B';
   mos_thread_new(recv_thread,128,PRIORITY_NORMAL);
   mos_thread_new(send_thread, 128, PRIORITY_NORMAL);

}
