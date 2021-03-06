//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "msched.h"
#include "node_id.h"
#include "led.h"
#include "printf.h"
#include "clock.h"
#include "command_daemon.h"

#include "rts.h"

comBuf send_pkt;
comBuf rts_pkt;
comBuf *recv_pkt;
comBuf *recv_pool;
comBuf *recv_pool_end;

uint8_t rts_enable;

void start (void)
{
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		  PRIORITY_NORMAL);

   send_pkt.size = 5;
   send_pkt.data[0] = CTS_BYTE;
   send_pkt.data[1] = CTS_BYTE;
   send_pkt.data[2] = CTS_BYTE;
   *((uint16_t *)&send_pkt.data[3]) = 4/*mos_node_id_get ()*/;

   rts_enable = 1;
   
   while (1) {
      com_send (IFACE_RADIO, &send_pkt);
      mos_led_toggle (0);

      mos_mdelay (80);
      //mos_thread_sleep (100);
   }
}
