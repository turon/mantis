//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**  
 * @File:     wb_accel.c
 * @Brief:    A file to test the accelerometer on the weather-sensor board.
 * @Author:   Charles Gruenwald III
 * @Date:     1-31-2004
 */

#include "mos.h"
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "avr-i2c.h"
#include "avr-eeprom.h" //devines the DEV_AVR_EEPROM_SEEK ioctl
#include "avr-adc.h"
#include "adc.h"

#define ADG715_PWR_ADDR 72 //address of ADG715 which controls the power

void accel_on ()
{
   uint8_t i;
   
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR); //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i)); //get current value
  
   i |= (1 <<5);                         //turn on accel

   dev_write(DEV_AVR_I2C, &i, sizeof(i));//write value back

   printf("Weatherboard accelerometer on.\n");

}

void accel_off ()
{
   uint8_t i;
   
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR);              //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i));    //get current value

   i &= ~(1 <<5);                           //turn off accel

   dev_write(DEV_AVR_I2C, &i, sizeof(i));   //write value back
   printf("Weatherboard accelerometer off.\n");
   
}

//retrieve the x&y values of the accelerometer
void accel_get_val ()
{
   uint8_t x, y;

   accel_on(); //turn accel on to read value
   dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 1);
   dev_read(DEV_ADC, &x, sizeof(x));

   dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 1);
   dev_read(DEV_ADC, &y, sizeof(y));
   accel_off(); //turn off accel
   
   printf("%C\t%C\n",x,y);
}

void start (void)
{
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE + 60, PRIORITY_NORMAL);
   printf ("Weatherboard Accelerometer Test\n");
   mos_register_function("getval", accel_get_val);
   
   accel_get_val();
   
}
