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

static comBuf send_pkt; //comBuf goes in heap

void send_thread ();

void start (void)
{
   mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
}

void send_thread ()
{
   send_pkt.size = 2; //2 bytes

   com_mode(IFACE_RADIO, IF_LISTEN);
   
   while(1) {
      mos_led_toggle(0);
#ifdef PLATFORM_TELOSB
      dev_read (DEV_MSP_TEMPERATURE, &send_pkt.data[0], 2);
#else
      dev_read (DEV_MICA2_TEMP, &send_pkt.data[0], 1);
#endif
      com_send(IFACE_RADIO, &send_pkt);
      mos_thread_sleep(1000);
   }
}
