//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "adc.h"
#include "led.h"
#include "printf.h"
#include "sem.h"
#include "mica2-mic.h"


mos_sem_t do_send;


comBuf send_pkt;

uint8_t i=0, count=0,sample;
void sense_thread()
{
   //sleep to let the initialization on the reader finish
   mos_thread_sleep(1000);

#if defined(ARCH_AVR)
   dev_mode(DEV_MICA2_MIC, DEV_MODE_ON);
#endif
   while(1) 
   {
      
    
#if defined(ARCH_AVR)
      dev_read (DEV_MICA2_MIC,&sample,sizeof(sample));
#elif defined(PLATFORM_TELOSB)
      sample = adc_get_conversion16(4); // the other light chanel is
      sample >>=2;
#endif
      send_pkt.data[count]=(uint8_t)(sample);
       
      count++;
      if(count>63)
      {
          
	 mos_led_toggle(1);
	 count=0;
	 mos_sem_post(&do_send);
      }
   }
}


void send_thread()
{
   send_pkt.size=64;

   while(1)
   {
      mos_sem_wait(&do_send);
      mos_led_toggle(2);
#if defined(PLATFORM_TELOSB)
      com_send(IFACE_SERIAL2,&send_pkt);
#else
      com_send(IFACE_SERIAL,&send_pkt);
#endif
   }
}


void start (void)
{

   mos_sem_init(&do_send,0);
   mos_thread_new (sense_thread, 128, PRIORITY_NORMAL);
   mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
}

