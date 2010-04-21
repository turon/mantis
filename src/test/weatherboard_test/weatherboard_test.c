/*
  This file is part of MANTIS OS, Operating System for Nymph.
  See http://mantis.cs.colorado.edu/

  Copyright (C) 2003 University of Colorado, Boulder

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  (See http://www.gnu.org/copyleft/gpl.html)
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
  USA, or send email to mantis-users@cs.colorado.edu.
*/

/**  
 * @File:     weatherboard_test.c
 * @Brief:    A file to test the weather-sensor board.
 * @Author:   Charles Gruenwald III
 * @Date:     12-03-2004
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

void i2c_write ()
{

   uint8_t i = prompt_uint8("#:");
   dev_write(DEV_AVR_I2C, i, sizeof(i));
   printf("Wrote: %C\n",i);
}

void i2c_read ()
{
   uint8_t i;
   dev_read(DEV_AVR_I2C, &i, sizeof(i));
   printf("i2c Value: %C\n",i);
}


void i2c_setbrr ()
{
   uint8_t i = prompt_uint8("#:");
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, i);
   printf("BRR: %C\n",i);
}


void i2c_dest ()
{
   uint8_t i = prompt_uint8("#:");
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, i);
}

void i2c_slave ()
{
   uint8_t i = prompt_char("#:");
   dev_ioctl(DEV_AVR_I2C, I2C_SLAVE_ADDR, i);
}

void i2c_enable_ack ()
{
   dev_ioctl(DEV_AVR_I2C, I2C_ENABLE_ACK);
   printf("Auto-Ack enabled.\n");

}

void i2c_disable_ack ()
{
   dev_ioctl(DEV_AVR_I2C, I2C_DISABLE_ACK);
   printf("Auto-Ack disabled.\n");
}

void setup()
{
   uint8_t i;
   uint8_t j;
   i = (1 << 5);

   uint8_t retval;
   
   mos_led_display(7);
   printf("Setting bit rate reg to 10.\n");
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50);
   
   printf("Setting dest address to 72.\n");
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, 72);

   
   printf("Writing %C to ADG715!\n", i);
   retval = dev_write(DEV_AVR_I2C, &i, sizeof(i));

   printf("Wrote %C bytes.\n", retval);
   printf("Retrieving current value.\n");

   retval = dev_read(DEV_AVR_I2C, &j, sizeof(j));

   printf("retval: %C\nCurr Val: %C\n", retval, j);

   if(i == j)
   {
      printf("ADG715 is working properly.\n");
   } else {
      printf("ADG715 returned incorrect value.\n");
   }
   
   printf("done.\n");
   
   mos_led_display(0);
}

void poll()
{
   uint8_t i;
   uint16_t j;
   uint8_t x, y;

   dev_mode(DEV_ADC, DEV_MODE_ON);
   
   for(i = 0; i < 40; i++)
   {
      dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 1);
      dev_read(DEV_ADC, &x, sizeof(x));
      dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, 2);
      dev_read(DEV_ADC, &y, sizeof(y));
      
      printf("%C\t%C\n",x,y);
      for(j=0;j<0xffff;j++);
      for(j=0;j<0xffff;j++);
      for(j=0;j<0xffff;j++);
      for(j=0;j<0xffff;j++);
   }
}

void start (void)
{
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE + 60, PRIORITY_NORMAL);
   printf ("AVR-I2C Test\n");
   
   mos_register_function ("i2c_write", i2c_write);
   mos_register_function ("i2c_read", i2c_read);
   mos_register_function ("i2c_setbrr",i2c_setbrr);
   mos_register_function ("i2c_setbrr",i2c_setbrr);
   mos_register_function ("i2c_dest",i2c_dest);
   mos_register_function ("i2c_slave",i2c_slave);
   mos_register_function ("i2c_enable_ack",i2c_enable_ack);
   mos_register_function ("i2c_disable_ack",i2c_disable_ack);
   mos_register_function ("setup",setup);
   mos_register_function ("poll",poll);
//   setup ();
}
