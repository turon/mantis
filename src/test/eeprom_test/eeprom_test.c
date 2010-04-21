//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include <string.h>

#include "mos.h"
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "avr-eeprom.h" //devines the DEV_AVR_EEPROM_SEEK ioctl

/**
 * File:     eeprom_test.c      
 * Author:   Charles Gruenwald III - gruenwal@colorado.edu
 * Date:     03-28-2004
 *
 * Description: This app is meant to test reading and writing
 * to the onboard eeprom, this can be used to save small bits
 * of data (perhaps network id's).
 *
 */


void eep_write ()
{
   // boot control block is at addr 0, so start somewhere else
   uint16_t address = 100;
   uint8_t data[] = "mantis rocks yo";
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, address);
   dev_write (DEV_AVR_EEPROM, data, sizeof (data));
}

void eep_read ()
{
   uint16_t address = 100;
   uint8_t data [20];
   
   memset (data, 0, sizeof (data));
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, address);
   dev_read (DEV_AVR_EEPROM, data, sizeof (data));
   printf ("%s", (char *)data);
}

void start (void)
{
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   printf ("AVR-EEPROM Test\n");
   
   mos_register_function ("write", eep_write);
   mos_register_function ("read", eep_read);   
}
