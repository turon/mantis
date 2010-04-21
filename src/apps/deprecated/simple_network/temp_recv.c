//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     receiver.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: receiver will sit in a tight while loop looking for
 * values received over the radio. Any value it receives it will then
 * show on the leds and send over the serial line using the uart. It is
 * to be used with compatable hardware (nymph/mica2).
 * 
 */

#include <inttypes.h>
#include <stdio.h>

#include "led.h"
#include "uart.h"
#include "msched.h"
#include "flooding.h"
#include "radio.h"
#include "netconst.h"
#include "net_util.h"

#ifndef PLATFORM_LINUX
  #include "printf.h"
#endif

/* some defines for our hard-coded network... */
#define NET_FULL_POWER      0xff
#define NET_LOCAL_ADDRESS   0x04
#define NET_FREQUENCY       0x03
#define NET_PORT            0x02

void receiver();

Packet data;
app_buf_t app_buf;

void start(void){
   mos_set_addr(NET_LOCAL_ADDRESS);
   mos_set_net_power(NET_FULL_POWER);
   flooding_init(NET_FREQUENCY);

   mos_thread_new(mos_net_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new(receiver, 128, PRIORITY_NORMAL);
}
void lprintf(int input)
{
  if(input>99)
    { printf("%d",input);
      return;
    }
  if(input>9)
    { printf("0%d",input);
      return;
    }
  printf("00%d",input);
}
void receiver(){

   uint8_t i, size;
   uint8_t dataRecvd;
   uint8_t fromAddr;
   mos_led_display(7);
   mos_uart_open(UART0);
   printf("\nThermal Network Initialized.\n\nPlease start the thermal send \nprogram on a few sensor-nodes\n(with different net addresses).\n");
   mos_uart_close(UART0);
   mos_led_display(0);
   while(1){

     size = flooding_recv((char *)&data, NET_PORT);

     //for(i=0; i<size; i++){
      dataRecvd = ((char *)&data)[0];
      fromAddr = ((char *)&data)[1];
      mos_led_display(dataRecvd);
      mos_uart_open(UART0);
      printf("node %d is ", fromAddr  );
      lprintf(dataRecvd);
      printf("deg.\n");
		// }
     mos_uart_close(UART0);
   }
}

