//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**  
 * @File:     wb_light.c
 * @Brief:    A file to test the photometer on the weather-sensor board.
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

void light_on ()
{
   uint8_t i;
   
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR); //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i)); //get current value
  
   i |= (1);                         //turn on light sensor

   dev_write(DEV_AVR_I2C, &i, sizeof(i));//write value back

   printf("Weatherboard photometer on.\n");

}

void light_off ()
{
   uint8_t i;
   
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR);              //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i));    //get current value

   i &= ~(1);                           //turn off light sensor

   dev_write(DEV_AVR_I2C, &i, sizeof(i));   //write value back
   printf("Weatherboard photometer off.\n");
   
}

//retrieve the x&y values of the accelerometer
void light_get_val ()
{
   uint16_t x, y;

   light_on(); //turn accel on to read value
   dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 1);
   dev_read(DEV_ADC, &x, 1);
   dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 2);
   dev_read(DEV_ADC, &y, 2);
   light_off(); //turn off accel
   
   printf("%C\t%C\n",x,y);
}

void start (void)
{
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE + 60, PRIORITY_NORMAL);
   printf ("Weatherboard Photometer Test\n");
   mos_register_function("getval", light_get_val);
   
   light_get_val();
   
}
