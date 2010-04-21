//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    calibrate.c  (bionet)                                         */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Calibration application for bionet sensor nodes.                       */
/**************************************************************************/

#include "mos.h"
//#include "config.h"
#include "msched.h"
#include "led.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"
#include "node_net_event.h"
#include "node_id.h"
#include "mutex.h"
#include "queue.h"
#include "avr-adc.h"
#include "dev.h"

#define ACCEL_BUF_SIZE 20

uint8_t showvals = false;

/* sense and send thread */
void sense_and_send();
uint16_t accel_calc();
void accel_sample();
void read_calibration();
uint16_t do_avg(uint8_t accel, uint8_t show)
{
   uint32_t sum = 0;
   uint16_t j;
   uint8_t i;

/*   if(accel == DEV_MICA2_ACCEL_X)
      dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_3);
   else
   dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_4);*/
   
   mos_thread_sleep (100);
      
   for(i = 0; i < ACCEL_BUF_SIZE; i++)
   {
      if(accel == DEV_MICA2_ACCEL_X)
	 dev_read (DEV_MICA2_ACCEL_X, &j, 2);
      else
	 dev_read (DEV_MICA2_ACCEL_Y, &j, 2);

      sum += j;
      if(show)
	 printf("%d\n", j);
   }
   return sum/ACCEL_BUF_SIZE;

}

uint16_t get_avg(uint8_t accel, uint8_t show)
{
   //calculate average
   showvals = true;
   prompt_char("hit [enter] to continue.\n");
   showvals = false;
   return do_avg(accel, show);
}

void calibrate(){
   uint16_t Xa,Xb, Ya, Yb;
   dev_mode(DEV_MICA2_ACCEL_X, DEV_MODE_ON);

   printf("Mantis calibration for accelerometer.\n");

   printf("Rotate the node into the Xa position (maximize x).\n");
   Xa = get_avg(DEV_MICA2_ACCEL_X, true);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 14);
   dev_write(DEV_AVR_EEPROM, (uint8_t *)&Xa, sizeof(Xa));
   
   printf("Rotate the node into the Xb position (minimize x).\n");
   Xb = get_avg(DEV_MICA2_ACCEL_X, true);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 16);
   dev_write(DEV_AVR_EEPROM, (uint8_t *)&Xb, sizeof(Xb));

   printf("Rotate the node into the Ya position (maximize y).\n");
   Ya = get_avg(DEV_MICA2_ACCEL_Y, true);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 18);
   dev_write(DEV_AVR_EEPROM, (uint8_t *)&Ya, sizeof(Ya));

   printf("Rotate the node into the Yb position (minimize y).\n");
   Yb = get_avg(DEV_MICA2_ACCEL_Y, true);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 20);


   if( (Xb > Xa) || (Yb > Ya) ){
      printf("Scale is incorrect, CALIBRATION FAILED!\n");
   }
   else
       dev_write(DEV_AVR_EEPROM, (uint8_t *)&Yb, sizeof(Ya));


}

void read_calibration(){
   uint16_t Xa,Xb, Ya, Yb;
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 14);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Xa, sizeof(Xa));
   
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 16);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Xb, sizeof(Xb));

   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 18);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Ya, sizeof(Ya));

   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 20);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Yb, sizeof(Yb));

   float scaley = (Ya - Yb)/(float)2;
   float offsety = ((float)(Ya+Yb))/((float)(Ya-Yb));

   float scalex = (Xa - Xb)/(float)2;
   float offsetx = ((float)(Xa+Xb))/((float)(Xa-Xb));

   printf("Scale(x): %d\n", (uint16_t)scalex);
   printf("Scale(y): %d\n", (uint16_t)scaley);
   printf("Offset(x): %d\n", (uint16_t)offsetx);
   printf("Offset(y): %d\n", (uint16_t)offsety);
   
   printf("Xa\tXb\tYa\tYb\t\n");
   printf("%d\t%d\t%d\t%d\t\n", Xa, Xb, Ya, Yb);


}

void print_zeroes(uint16_t value)
{
   if(value > 999){
      printf("%d", value);
      return;
   }
   else if(value > 99){
      printf("0%d", value);
      return;
   }
   else {
      printf("00%d", value);
      return;
   }
      
}
void show_vals()
{
   uint16_t x;
   uint16_t y;
   while(1){
      if(showvals){
	 x = do_avg(DEV_MICA2_ACCEL_X, false);
	 y = do_avg(DEV_MICA2_ACCEL_Y, false);
	 printf("\b\b\b\b\b\b\b\b\b");
	 print_zeroes(x);
	 printf(" ");
	 print_zeroes(y);
      }
      mos_thread_sleep(10);
   }
}

void start(void)
{

   // command daemon, normal user interaction
   mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   mos_register_function("calibrate",calibrate);
   mos_register_function("read_calibration",read_calibration);

   mos_thread_new(show_vals, 128, PRIORITY_NORMAL);

}

