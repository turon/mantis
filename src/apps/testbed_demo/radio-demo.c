//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    blink.c                                                       */
/* Author      Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 12/11/03                                                       */
/* Edited by   Adam Torgerson: adam.torgerson@colorado.edu                */
/*   Date: 12/11/03                                                       */
/* Edited by   Charles Gruenwald  :  gruenwal@colorado.edu                */
/*   Date: 04/14/0/04                                                     */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"

#include "com.h"

#if defined(PLATFORM_MICA2)
#include "cc1000.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

void blink (void)
{
   uint32_t sleep_time_a = 2000;
   int8_t a;
   
   mos_led_display(1);
   mos_thread_sleep (sleep_time_a);
   mos_led_display(2);
   mos_thread_sleep (sleep_time_a);
   mos_led_display(4);
   mos_thread_sleep (sleep_time_a);
   mos_led_display(2);
   mos_thread_sleep (sleep_time_a);
   mos_led_display(1);
   mos_thread_sleep (sleep_time_a);
   mos_led_display(0);
   
   while (1) {
       for(a=0;a<=7;a++)
       {
           mos_led_display(a);
           mos_thread_sleep (sleep_time_a);
           printf("The value is %d\n",a);
       }
       for(a=6;a>=1;a--)
       {
           mos_led_display(a);
           mos_thread_sleep (sleep_time_a);
           printf("The value is %d\n",a);
       }
   }
}


//This is a standard debugging thread
void debug(void)
{
    mos_thread_sleep(500);//wait for the computer to get ready for us.

    while (1)
    {
        printf("Node is up");
    }
}


static comBuf send_pkt; //comBuf goes in heap

void radio_send(void)
{
    send_pkt.size = 4; //2 bytes
    send_pkt.data[0]=1;
    send_pkt.data[1]=1;
    send_pkt.data[2]=1;
    send_pkt.data[3]=1;
    
      
    while(1) 
    {
        com_send(IFACE_RADIO, &send_pkt);
        mos_led_toggle(0);
        mos_thread_sleep(100);
    }
}

void radio_recieve(void)
{
    comBuf *recv_pkt;                     //give us a packet pointer
    uint8_t a=0;
    
    while(1)
    {
        recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
        a++;
        if(a>=2)
        {
            a=0;
            mos_led_toggle(2);
        }
        mos_led_toggle(1);
        com_free_buf(recv_pkt);             //free the recv'd packet to the pool
    }
}




void start(void)
{
//  mos_thread_new (blink, 128, PRIORITY_NORMAL);
// mos_thread_new (debug,128, PRIORITY_SLEEP);

//put a radio thread here
    com_ioctl(IFACE_RADIO,RADIO_TX_POWER,0);

    com_mode(IFACE_RADIO, IF_LISTEN);
    

    //    mos_thread_new (radio_send, 128, PRIORITY_NORMAL);
    mos_thread_new (radio_recieve,128, PRIORITY_NORMAL);
   
}
