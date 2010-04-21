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

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "led.h"     // LED control
#include "dev.h"
#include "clock.h"
#include "printf.h"
#include "uart.h"

static char* send_string = "Hello!\n";
comBuf send_Buf;

void send_uart1 (void)
{

   uint8_t len = sizeof("Hello!\n");
   uint8_t i;
   send_Buf.size = len;
   for(i = 0; i < len; i++){
      send_Buf.data[i] = send_string [i];
   }
   printf("sending HELLO over uart test.\n");
   while (1) {      
      mos_led_toggle (0);
      printf("sending out...");
      com_send(IFACE_SERIAL2, &send_Buf);
      printf("sent.\n");
      mos_thread_sleep (500);
   }
}

   
/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void){

   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);

   printf("Maxstream radio test!\n");

//   uart1_set_baud(B9600);
   uart_set_baud(UART1, 23);
   mos_thread_new (send_uart1, 128, PRIORITY_NORMAL);
   mos_led_display(0xFF);

}
