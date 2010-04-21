//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "sem.h"
#include "com.h"

static uint8_t generator_continue;
static uint8_t packet_size = 4;

void generator(void)
{
    comBuf send_pkt;
    uint8_t value = 0;
    uint16_t seq = 0;
    uint8_t i;


    com_mode(IFACE_RADIO, IF_LISTEN);

    //   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    send_pkt.size = packet_size;
    for(i = 0; i < send_pkt.size; i++)
        send_pkt.data[i] = value++;


    while(generator_continue) {
        mos_led_toggle(2);


        printf("seq: %d\r\n", seq);
        *(uint8_t *)(&send_pkt.data[1]) = (uint8_t)(seq >> 8);
        *(uint8_t *)(&send_pkt.data[0]) = (uint8_t)seq;

        ++seq;

        //fake a dropped packet
        //if(seq % 10 == 0)
        // seq++;


        volatile comBuf *sp = &send_pkt;

        com_send(IFACE_RADIO, sp);
        //      printf("sent\n");

        mos_thread_sleep(200);
        //mos_mdelay(20);
    }
}

void start_generator()
{
    generator_continue = true;
    if(mos_thread_new (generator, 2048, PRIORITY_NORMAL) != THREAD_OK)
        show_thread_err();
}

void stop_generator()
{
    generator_continue = false;
}

