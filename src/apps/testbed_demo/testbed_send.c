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
#include "node_id.h"
#include "testbed_event.h"

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#include "msp-humidity.h" // ioctl arguments
#endif

static uint16_t seq = 0;
static comBuf outpkt;

extern comBuf event_buf;

void send_thread()
{

   mos_mdelay(1000);
   printf("SYSTEM: NODE: Started.\n");
   
   uint16_t id = mos_node_id_get();
   send_testbed_event(EVENT_NETWORK, NETWORK_ID, (uint8_t *)&id, 2);
   com_mode(IFACE_RADIO, IF_LISTEN);
//   com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);

   outpkt.size = 4;
   *(uint16_t *)&(outpkt.data[0]) = id;

   while(1)
   {
      mos_led_toggle(2);
      *(uint16_t *)&(outpkt.data[2]) = seq++;
      com_send(IFACE_RADIO, &outpkt);
      outpkt.data[2] = outpkt.size;
      send_testbed_event(EVENT_NETWORK, PACKET_SENT,
		 (uint8_t *)&(outpkt.data[0]), 3);
      mos_thread_sleep(1000);
   }
}

void sense_thread()
{
#if defined(PLATFORM_TELOSB)
   uint16_t temp;
   dev_open(DEV_MSP_HUMIDITY);
   
   dev_mode(DEV_MSP_HUMIDITY, DEV_MODE_ON);
   // turn off OTP reload for faster response time and lower power consumption
//   dev_ioctl(DEV_MSP_HUMIDITY, SHT11_RELOAD_OFF);
#endif


   while(1)
   {
#if defined(PLATFORM_TELOSB)
      dev_read(DEV_MSP_TEMPERATURE, (void*)&temp, sizeof(uint16_t));
      send_testbed_event(EVENT_SENSE, TEMPERATURE, &temp, sizeof(uint16_t));
#endif
      mos_thread_sleep(5000);
   }

#if defined(PLATFORM_TELOSB)
   dev_mode(DEV_MSP_HUMIDITY, DEV_MODE_OFF);
   dev_close(DEV_MSP_HUMIDITY);
#endif

   
}


void start (void)
{

   mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
   mos_thread_new (sense_thread, 128, PRIORITY_NORMAL);
}

