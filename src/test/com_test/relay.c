//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    test.c                                                        */
/* Author     Jeff Rose & Brian Shucker: rosejn & shucker@cs.colorado.edu */
/*   Date: 03/09/04                                                       */
/*                                                                        */
/* Radio relay for testing the mnet network stack                         */
/**************************************************************************/

#include "mos.h"

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "com.h"
#include "mos.h"
#include "led.h"
#include "cc2420.h"
#include "printf.h"

#include "uart.h"

comBuf *packet;

void relay(void)
{
    //IF_SET set;

    com_mode(IFACE_RADIO, IF_LISTEN);
    //com_mode(IFACE_SERIAL, IF_LISTEN);

    while(1) {
//        IF_ZERO(&set);
//        IF_SET(IFACE_RADIO, &set);
//        IF_SET(IFACE_SERIAL, &set);

//        mos_led_display(1);
//        com_select(&set, COM_BLOCK);
//        mos_led_display(2);

//        if(IF_ISSET(IFACE_RADIO, &set)) {
            packet = com_recv(IFACE_RADIO);

            if(packet != NULL) {
                //com_send(IFACE_SERIAL, packet);
                printf("%d\n", packet->data[0]);
                com_free_buf(packet);
            }
//        } else if(IF_ISSET(IFACE_SERIAL, &set)) {
//            packet = com_recv(IFACE_SERIAL);

//            if(packet != NULL) {
//                packet->data[0]++;
//                com_send(IFACE_SERIAL, packet);
//                com_free_buf(packet);
//            }
//        }
    }
}

void start(void)
{
#if defined(PLATFORM_TELOSB) || defined(PLATFORM_MICAZ)
    cc2420_init();
#endif
    mos_led_toggle(1);
    mos_thread_new(relay, 128, PRIORITY_NORMAL);
}
