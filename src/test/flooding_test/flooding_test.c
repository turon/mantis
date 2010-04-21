//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**  
 * @File:     flooding_test.c
 * @Brief:    A file to test the flooding net layer.
 * @Author:   Charles Gruenwald III
 * @Date:     09-07-2004
 */

#include <inttypes.h>

#include "led.h"
#include "msched.h"
#include "net.h"
#include "clock.h"
#include "command_daemon.h"
#include "cc1000.h"
#include "flood.h"
#include "node_id.h"

static flood_pkt outpacket;
static comBuf out_buf;
void generator ();

void ledb ()
{
   int8_t ret;
   uint8_t led_to_broadcast = prompt_uint8 ("led to bcast:");
   printf ("bcasting %C.\n", led_to_broadcast);
   out_buf.size = 1;
   out_buf.data[0] = led_to_broadcast;
   ret = net_event_send (1, &out_buf);
}

void clickb ()
{
   int8_t ret;
   uint16_t *p = (uint16_t *)&out_buf.data[0];
   out_buf.size = 2;
   *p = 500;
   ret = net_event_send (2, &out_buf);
}

void beepb ()
{
   int8_t ret;
   uint16_t *p = (uint16_t *)&out_buf.data[0];
   out_buf.size = 2;
   *p = 2000;
   ret = net_event_send (2, &out_buf);
}

void start (void)
{
    //give us control over serial/rf
    mos_thread_new (mos_command_daemon, 192, PRIORITY_NORMAL);
    //start this thread
    mos_thread_new (generator, 224, PRIORITY_NORMAL);
    mos_register_function ("ledb",ledb);
    mos_register_function ("clickb",clickb);
    mos_register_function ("beepb",beepb);
}

void generator ()
{
   int8_t ret;

    printf ("Flooding test Generator.\n");
    net_init ();
    flood_init ();

    ret = net_proto_set (FLOOD_PROTO_ID);
    
    if (ret == NET_PROTO_INVALID) {
	printf ("Invalid proto\n");
    } else {
       printf ("\nNet proto-id is valid.\n");
    }

    outpacket.src = mos_node_id_get();
    
    while (1) {
       //       out_buf.size=1;
       //       out_buf.data[0]=i++;
       //       i %= 8;
       //       ret = net_event_send(1, &out_buf);
       mos_led_toggle (1);
       mos_thread_sleep (1000);
       clickb ();
       //       mos_thread_sleep(1000);
    }
}
