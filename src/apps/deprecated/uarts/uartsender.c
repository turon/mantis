//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     uartsender.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-04-2003
 * Edited:   Adam Torgerson: adam.torgerson@colorado.edu
 *
 * Description: uartsender can be used to to send single bytes over 
 * the serial line (uart). Should be platform indepented if hardware
 * supports serial communication over the uart (nymph, xmos currently)
 */


#include <inttypes.h>

#include "msched.h"
#include "led.h"
#include "com.h"
#include "clock.h"

comBuf send_packet;

void uart_sender(void);

void start(void)
{
  mos_thread_new(uart_sender, 128, PRIORITY_NORMAL);
}

void uart_sender()
{
   uint8_t value=0;

  send_packet.size =1;
  
  while(1)
  {
     send_packet.data[0]=value++;         //set the data value
     com_send(IFACE_SERIAL, &send_packet); //send the packet
     mos_led_display(value);              //display value sent
     mos_mdelay(750);
  }
}
