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
#include "atmel-flash.h" //devines the DEV_AVR_EEPROM_SEEK ioctl

/**
 * File:     ext_flash_test.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     05-20-2004
 *
 * Description: This app is meant to test reading and writing
 * to the external flash using the dev-layer.
 */

uint8_t data[534];

void ext_flash_write()
{
   uint32_t address = 0;
   //uint8_t data[] = "mantis";
   uint16_t i;
 
   for (i = 0; i < sizeof (data); i++) {
      data[i] = (i % 10) + '0';
   }
 
   dev_ioctl (DEV_ATMEL_FLASH, DEV_SEEK, address);
   dev_write (DEV_ATMEL_FLASH, data, sizeof (data));
}

void ext_flash_read()
{
   uint32_t address = 0;
   //uint8_t data [534];
   //uint8_t data[80];
   uint16_t i;
   comBuf pkt;

   for (i = 0; i < sizeof (data); i++) {
      data[i] = '0';
   }
   
   dev_ioctl (DEV_ATMEL_FLASH, DEV_SEEK, address);
   dev_read (DEV_ATMEL_FLASH, data, sizeof (data));

   //printf ("%s", (char *)data);

   pkt.size = 3;
   pkt.data[0] = 245;
   for (i = 0; i < sizeof (data); i++) {
      //printf ("%c", data[i]);
      *((uint16_t *)&pkt.data[1]) = data[i];
      com_send (IFACE_SERIAL, &pkt);
   }
   //printf('\n');
}

void start(void)
{
   mos_thread_new (mos_command_daemon, 196, PRIORITY_NORMAL);
   mos_register_function ("write", ext_flash_write);
   mos_register_function ("read", ext_flash_read);
}
