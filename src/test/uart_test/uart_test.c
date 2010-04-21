//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* For simple testing of the UART driver, this app starts a thread and    */
/*   prints a message over the UART using packetized MOS comBufs. It then */
/*   goes into an infinite loop, receiving data over the UART and sending */
/*   anything received right back out.                                    */
/**************************************************************************/

#include "mos.h"

#include <string.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"
#include "com.h"
#include "uart.h"

#if defined (ARCH_AVR) || defined(PLATFORM_TELOSB) || defined(PLATFORM_MICROBLAZE)

comBuf sendbuf;

#define TEXT_MSG "hello world!\n"

void uart_test(void)
{
   comBuf *inbuf;
   sendbuf.size = sizeof(TEXT_MSG);
   strcpy(sendbuf.data, TEXT_MSG);
   
   com_mode(IFACE_SERIAL, IF_LISTEN);
   
   com_send(IFACE_SERIAL, &sendbuf);

   while(1) {
      mos_led_toggle(0);
      inbuf = com_recv(IFACE_SERIAL);
      com_send(IFACE_SERIAL, inbuf);
      com_free_buf(inbuf);
   }
}

#endif

void start(void)
{
   mos_thread_new(uart_test, 128, PRIORITY_NORMAL);
}
