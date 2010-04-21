//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "msched.h"
#include "dev.h"
#include "command_daemon.h"
#include "printf.h"

uint16_t x_a;
uint16_t x_b;
uint16_t y_a;
uint16_t y_b;

void accel_x_a(void)
{
   dev_read(DEV_MICA2_ACCEL_X, &x_a, sizeof(x_a));
   printf("Accel X A reading: %d\n", x_a);   
}

void accel_x_b(void)
{
   dev_read(DEV_MICA2_ACCEL_X, &x_b, sizeof(x_b));
   printf("Accel X B reading: %d\n", x_b);}


void accel_y_a(void)
{
   dev_read(DEV_MICA2_ACCEL_Y, &y_a, sizeof(y_a));
   printf("Accel Y A reading: %d\n", y_a);
}

void accel_y_b(void)
{
   dev_read(DEV_MICA2_ACCEL_Y, &y_b, sizeof(y_b));
   printf("Accel Y B reading: %d\n", y_b);
}

void accel_final(void)
{
   uint16_t sensitivity_x, sensitivity_y;
   uint16_t offset_x, offset_y;
   uint16_t accel_g_x, accel_g_y;
   
   sensitivity_x = (x_a - x_b) / 2;
   offset_y = (x_a + x_b) / (x_a - x_b);
   
   printf("X sensitivity: %d offset: %d accel: %d\n", sensitivity_x, offset_x);

   sensitivity_y = (y_a - y_b) / 2;
   offset_y = (y_a + y_b) / (y_a - y_b);
   
   printf("Y sensitivity: %d offset: %d\n", sensitivity_y, offset_y);

   while(1) {
      dev_read(DEV_MICA2_ACCEL_X, &x_a, sizeof(x_a));
      dev_read(DEV_MICA2_ACCEL_Y, &y_a, sizeof(y_a));

      accel_g_x = (x_a / sensitivity_x) + offset_x;
      accel_g_y = (y_a / sensitivity_y) + offset_y;

      printf("accel_g_x: %d, accel_g_y: %d\n", accel_g_x, accel_g_y);
   }
}

void start(void)
{
   dev_mode(DEV_MICA2_ACCEL_X, DEV_MODE_ON);
   
   mos_register_function("accel_x_a", accel_x_a);
   mos_register_function("accel_x_b", accel_x_b);
   mos_register_function("accel_y_a", accel_y_a);
   mos_register_function("accel_y_b", accel_y_b);
   mos_register_function("accel_final", accel_final);
   
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE + 64,
		  PRIORITY_NORMAL);
}
