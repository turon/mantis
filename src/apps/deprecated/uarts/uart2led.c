//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     uart2led.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     11-20-2003
 *
 * Description: uart2led will listen for bytes over the serial line
 * and display any byte it receives in binary on the LEDs, to be used
 * with uartsender.c on compatable hardware (nymph/xmos).
 */


#include <inttypes.h>

#include "msched.h"
#include "led.h"
#include "com.h"

void start(void)
{
   comBuf *recv_packet;
   com_mode(IFACE_SERIAL, IF_LISTEN);
   while(1) //recv over uart, show on leds
   {
     recv_packet = com_recv(IFACE_SERIAL);
     mos_led_display(recv_packet->data[0]);
     com_free_buf(recv_packet);
   }
}


