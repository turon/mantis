//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    crc_test.c                                                    */
/* Author      Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 12/11/03                                                       */
/* Edited by   Adam Torgerson: adam.torgerson@colorado.edu                */
/*   Date: 12/11/03                                                       */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/

#include "mos.h"

#include "printf.h"

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "crc.h"
#include "com.h"

comBuf *buf;

char *crc_str = "run a crc on this str";

void print_crc()
{
   uint16_t crc;

   //com_mode(IFACE_TERMINAL, IF_LISTEN);

   while(1)
   {
      //printf("Enter a string to compute its CRC16-CCITT:\n");
      //buf = com_recv(IFACE_TERMINAL);
      
      crc = crc_compute(crc_str, sizeof(crc_str));

      printf("CRC: %d\n\n", crc);

      com_free_buf(buf);
   }
}
/* The start function is automatically called by the operating system,
so this is where the application begins execution.  The thread used to
call start has a limited amount of stack space so it is best to create
threads here and then return. */
void start(void)
{
   mos_thread_new(print_crc, 128, PRIORITY_NORMAL);
}
