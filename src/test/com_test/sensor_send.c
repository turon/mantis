//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sensor_test.c                                                 */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/08/04                                                       */
/*                                                                        */
/* Testing the mnet network stack                                         */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "com.h"

#include "clock.h"

#include "cc2420.h"

comBuf packet;

void send()
{
    uint8_t counter;

    counter = 0;
    packet.size = 1;

    while(1) { 
        packet.data[0] = counter++;
        com_send(IFACE_RADIO, &packet);
        mos_mdelay(500);
    //    mos_thread_sleep(1000);
    }
}

/* The start function is automatically called by the operating system,
   so this is where the application begins execution.  The thread used to
   call start has a limited amount of stack space so it is best to create
   threads here and then return. */
void start(void)
{
#if defined(PLATFORM_TELOSB) || defined(PLATFORM_MICAZ)
    cc2420_init();
#endif
    mos_led_toggle(0);
    mos_thread_new(send, 128, PRIORITY_NORMAL);
}
