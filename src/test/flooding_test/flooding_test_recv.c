//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * @Brief: a file to test flooding routing.
 * @File:     flooding_test_recv.c
 * @Author:   Charles Gruenwald III
 * @Date:     09-07-2004
 */


#include "led.h"
#include "msched.h"
#include "clock.h"
#include "net.h"
#include "dev.h"
#include "printf.h"
#include "flood.h"
#include "command_daemon.h"

void recv_thread();

int8_t recv(uint8_t event_id, comBuf *pkt, uint8_t proto, uint8_t *footer)
{
   uint8_t val;
   
   printf("event: %C\tdata: %C\n",event_id, pkt->data[0]);

   switch(event_id) {
   case 1:
      printf("turning on leds.\n");
      mos_led_display(pkt->data[0]);
      break;
   case 2:
      printf("sounder on for: %d\n",(uint16_t)pkt->data[0]);
      val = 1;
      dev_write (DEV_MICA2_SOUNDER, &val, sizeof (val));
      mos_thread_sleep((uint16_t)pkt->data[0]);
      val = 0;
      dev_write (DEV_MICA2_SOUNDER, &val, sizeof (val));
      break;
       
   default:
      break;
   }
   return 0;
}

void start(void)
{
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new(recv_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(net_thread, 192, PRIORITY_NORMAL);
}

void recv_thread()
{
   int8_t ret;
    
   printf("This is Flooding Test Receiver!\n");
   net_init();
   flood_init();

   ret = net_proto_set(FLOOD_PROTO_ID);
   if(ret == NET_PROTO_INVALID) {
      printf("Invalid proto\n");
   }

   net_event_register(1, recv);
   net_event_register(2, recv);
    
   while(1) {
      mos_thread_sleep(1000);
   }
}
