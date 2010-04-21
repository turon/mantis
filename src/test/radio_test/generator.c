//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     generator.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     12-14-2003
 *
 * Description: generator will repeatedly send the numbers 1,2,3...255 
 * over the radio and display the last 3 bits in binary on the LEDs. it 
 * is to be used with receiver.c to test the radios on compatable 
 * hardware (nymph/mica2).
 */


#include "msched.h"
#include "mos.h"
#include "com.h"
#include "clock.h"
#include "led.h"

uint8_t packet_size = 64;
comBuf send_pkt;

void generator(void)
{
   uint8_t value = 0;
   uint16_t seq = 0;
   uint8_t i;

   com_mode(IFACE_RADIO, IF_LISTEN);
   
   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);

   send_pkt.size = packet_size;
   for(i = 0; i < send_pkt.size; i++)
      send_pkt.data[i] = value++;

   while(1) {
      mos_led_toggle(2);

      *(uint16_t *)(&send_pkt.data[0]) = seq++;

      //fake a dropped packet
      //if(seq % 10 == 0)
      // seq++;

      com_send(IFACE_RADIO, &send_pkt);      

      //printf("sent\n");
      
      //mos_thread_sleep(5);
      mos_mdelay(50);
   }
}
