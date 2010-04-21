//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


/**
 * File:     cc2420_test.c      
 * Author:   Adam Torgerson, Charles Gruenwald III
 * Date:     9-10-2004
 *
 * Description: This app is meant to test the functionality of the
 * cc2420 low-level code.
 *
 */

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "clock.h"

#include "node_id.h"
#include "cc2420.h"
uint16_t spi_get_register(uint8_t reg);
void spi_set_register(uint8_t reg, uint16_t val);

comBuf send_buf;
comBuf *recv_buf;

void other(void)
{
   
   printf("CC2420 Radio Test Sender\n");

   com_mode(IFACE_RADIO, IF_LISTEN);

   //send_buf.size = 6;
   //send_buf.data[0] = 'h';
   //send_buf.data[1] = 'e';
   //send_buf.data[2] = 'l';
   //send_buf.data[3] = 'l';
   //send_buf.data[4] = 'o';
   //send_buf.data[5] = '\0';
   
   send_buf.size = 3;
   //send_buf.data[0] = 'f';
   //send_buf.data[1] = '\0';

   buf_insert16(send_buf.data, 0, mos_node_id_get());
   send_buf.data[2] = '\0';
   
   printf("Entering loop\n");

   
   
   while(1) {
      com_send(IFACE_RADIO, &send_buf);
//      mos_mdelay(500);
      mos_thread_sleep(500);
      printf("Sent a packet\n");
      
      mos_led_toggle(0);
      //send_buf.size--;
      //if(send_buf.size == 0 ) break;
      //send_buf.data[send_buf.size - 1] = '\0';
    }   
   
}

void start(void)
{
   //mos_thread_new(mos_command_daemon, 192, PRIORITY_NORMAL);
   mos_thread_new(other, 128, PRIORITY_NORMAL);
}
