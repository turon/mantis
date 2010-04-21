//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* This shows an example of using the UART in raw-mode, rather than the   */
/*   packetized protocol which the com layer uses. If you need to         */
/*   communicate with something that isn't running MOS, this is probably  */
/*   what you want to look at. If you want to use this, note that you     */
/*   uncomment the appropriate NEED_RAW_MODE define in uart.h             */
/**************************************************************************/

#include "mos.h"

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"
#include "com.h"
#include "clock.h"
#include "uart.h"

uint8_t buffer[] = "hello world!\n";
uint8_t read_buf;

comBuf send_buf;

void uart_test(void)
{
   memcpy(send_buf->data, "adamo!", sizeof("adamo!"));
   send_buf->size = sizeof("adamo!");

   com_ioctl(IFACE_SERIAL, UART_IOCTL_RAW_MODE0);
   
   uart_write(0, buffer, sizeof(buffer));
   
   while(1) {
      uart_read(0, &read_buf, sizeof(read_buf));
      uart_write(0, &read_buf, sizeof(read_buf));
   }
}

void start(void)
{
   mos_thread_new(uart_test, 128, PRIORITY_NORMAL);
}
