//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    mosix_test.c                                                 */
/* Author(s)      Adam Torgerson                                          */
/*                                                                        */
/*   Date:  03/01/04                                                      */
/* Edited:   Charles Gruenwald III    :    gruenwal@colorado.edu          */
/*                                                                        */
/* mosix_test :                                                           */
/* Demonstrates the "MOSIX" device driver calls                           */
/*                                                                        */
/**************************************************************************/

#include "mos.h"

#include "msched.h"
#include "dev.h"
#include "printf.h"
#include "command_daemon.h"

void get_light ();
void get_temp ();
void get_vol ();
void test_ioctl ();

void start (void)
{  
  printf ("\n\nWelcome to the sensor test application.\n");
  printf ("type 'light' 'temp' or 'vol' to test the sensors.\n\n");
  mos_thread_new(mos_command_daemon, 192, PRIORITY_NORMAL);

  mos_register_function ("light", get_light);
  mos_register_function ("temp", get_temp);
  mos_register_function ("vol", get_vol);
  mos_register_function ("ioctl", test_ioctl);
}

void test_ioctl ()
{
   int8_t ret = dev_ioctl (DEV_MICA2_LIGHT, DEV_SEEK);
   printf ("Status is %d\n", ret);
}

void get_light ()
{
   int8_t ret;
   int8_t light;
   ret = dev_read (DEV_MICA2_LIGHT, &light, sizeof(light));
   printf ("Light is '%d'\n", light);
}

void get_temp ()
{
   int8_t ret;
   int8_t temp;
   ret = dev_read (DEV_MICA2_TEMP, &temp, sizeof (temp));
   printf ("Temp is '%d'\n", temp);
}

void get_vol ()
{
   uint16_t counter = 0;
   uint8_t loop = 0;
   int8_t ret;
   int8_t vol;
   for (loop = 0; loop < 10; loop++) {
      ret = dev_read (DEV_MICA2_MIC, &vol, sizeof (vol));
      printf ("Vol is '%d'\n", vol);
      for (counter = 0; counter < 0xffff; counter++)
	 ;
      for (counter = 0; counter < 0xffff; counter++)
	 ;
   }
}
