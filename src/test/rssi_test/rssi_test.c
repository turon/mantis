//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    tdma_test.c                                                   */
/* Author      Brian Shucker shucker@cs.colorado.edu                      */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Simple app used for testing cc1000 TDMA MAC                            */
/**************************************************************************/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "com.h" //com layer
#include "dev.h"
#include "printf.h"

#ifndef PLATFORM_MICA2
//pc thread that receives data over serial and prints it

comBuf *pkt;

void start (void)
{
   uint8_t i;

   com_mode(IFACE_SERIAL, IF_LISTEN);

   while(1) {
      pkt = com_recv (IFACE_SERIAL);
      printf("Got packet, size=%d, data: ", pkt->size);
      for(i = 0; i < pkt->size; i++)
	 printf("%d ", pkt->data[i]);
      putchar('\n');
      com_free_buf(pkt);
   }
}

#else
//radio receiver running on mica2, sending data back over serial

comBuf pkt;

void start(void)
{
   uint8_t index;
   uint16_t delay;
   uint16_t val;

   pkt.size = 16;
  
   //turn on rssi
   dev_mode(DEV_AVR_RSSI, DEV_MODE_ON);

   while(1) {
      //receive a bunch of samples, then send
      for (index = 0; index < 16; index++) {
	 //take sample
	 dev_read (DEV_AVR_RSSI, &val, 2);
	 printf("%d ", val);
	 //wait
	 //mos_mdelay(200);
      }
      printf("\n");
      
   }
}

#endif
