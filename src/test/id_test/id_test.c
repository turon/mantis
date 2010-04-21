//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"

/**
 * File:     id_test.c      
 * Author:   Jeff Rose  rosejn@colorado.edu
 * Date:     05-18-2004
 *
 * Description: This app is meant to test the 1-wire serial id
 * chip on the mica2 nodes.  It simply reads the id and sends
 * it out over the serial port.   
 *
 */
uint8_t id_read (void);

static void read_id_callback (void)
{
   uint8_t i;
   uint8_t my_id[6];
   uint8_t len;

   //if (!(len = id_read ())) {
   //   printf ("Device not present1: %C?\n", len);
   //  return;
   //}   

   if ((len = dev_read (DEV_HARDWARE_ID, &my_id[0], 6)) != 6) {
      printf ("Device not present2: %C?\n", len);
      //return;
   }

   printf ("The ID: ");
   for (i = 0; i < 6; i++)
      printf("%C", my_id[i]);

   printf ("\n");
   
}

void start (void)
{
   mos_thread_new (mos_command_daemon, 196, PRIORITY_NORMAL);
   mos_thread_new (read_id_callback, 128, PRIORITY_NORMAL);
}
