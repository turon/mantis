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
#include "uart.h"
#include "led.h"
#include "dev.h"
#include "avr-i2c.h"
#include "avr-eeprom.h" //devines the DEV_AVR_EEPROM_SEEK ioctl
#include "avr-adc.h"

#define ADG715_PWR_ADDR 72 //address of ADG715 which controls the power
#define ADG715_PWR_ADDR2 73 //address of ADG715 which controls the power

static comBuf outbuf;
static uint8_t inbuf [64];

static void gps_read_param();
static void gps_thread();

void gps_on ()
{
   uint8_t i;


   printf("turning on GPS_ENABLE switch\n");
   //turn on the GPS_ENABLE switch
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR); //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i)); //get current value
  
   i |= (1 <<7);                         //turn on gps_enable

   dev_write(DEV_AVR_I2C, &i, sizeof(i));//write value back


   //connect UART1 to gps
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR2); //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i)); //get current value
  
   i |= 3;                         //connect gps

   dev_write(DEV_AVR_I2C, &i, sizeof(i));//write value back

   
   printf("GPS on.\n");

}

void gps_off ()
{
   uint8_t i;
   
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50); //set speed
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR);              //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i));    //get current value
   i &= ~(1 <<7);                           //turn off gps_enable

   dev_write(DEV_AVR_I2C, &i, sizeof(i));   //write value back
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR,
	     ADG715_PWR_ADDR2);              //set dest address

   dev_read(DEV_AVR_I2C, &i, sizeof(i));    //get current value
   i &= ~(3);                           //turn off gps_enable
   dev_write(DEV_AVR_I2C, &i, sizeof(i));   //write value back
   
   
   printf("GPS off.\n");
   
}

comBuf buf;

void minicom_test(void)
{
   com_mode(IFACE_RADIO, IF_LISTEN);
   com_mode(IFACE_SERIAL, IF_LISTEN);
   com_ioctl(IFACE_SERIAL, UART_IOCTL_RAW_MODE);	
   com_ioctl(IFACE_SERIAL, UART_IOCTL_BAUD_RATE, B9600);
   buf.size = 1;

   buf.data[0] = 'a';
   
   
   while(1) {
      mos_led_display(1);
      uart_read(UART0, buf.data, 1);
      uart_send_buf(UART0, &buf);
      mos_led_display(2);
      com_send(IFACE_RADIO, &buf);
   }
}

void start (void)
{
   
   //mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   //mos_thread_new (gps_thread, 128, PRIORITY_NORMAL);

   mos_thread_new(minicom_test, 128, PRIORITY_NORMAL);
}

static void gps_thread()
{
   printf ("GPS Test\n");
   mos_led_toggle(0);
   mos_led_toggle(1);
   mos_led_toggle(2);
   
   while (1) {
      printf("turning the gps on...\n");
      gps_on();
      printf("reading params\n");
      gps_read_param();
      gps_off();
      mos_thread_sleep(800);
      //mos_led_toggle(0);
   }
   //outbuf.size = 40;
   //for (k=0; k < 40; k++) {
   //  outbuf.data[k] = (int)inbuf[k];
   //}
   //com_send(IFACE_RADIO, &outbuf);
}

static void gps_read_param()
{

   
   com_mode(IFACE_SERIAL2, IF_LISTEN);
   com_ioctl(IFACE_SERIAL2, UART_IOCTL_RAW_MODE);	
   com_ioctl(IFACE_SERIAL2, UART_IOCTL_BAUD_RATE, B4800);
   //To poll GPS parameters from GPS unit

   memcpy(outbuf.data, "$PSRF103,00,01,00,01*25\r\n", sizeof("$PSRF103,00,01,00,01*25\r\n") - 1);
   
   outbuf.size = sizeof("$PSRF103,00,01,00,01*25\r\n") - 1;
   /*
   outbuf.data[0] = 0xA0;
   outbuf.data[1] = 0xA2;
   outbuf.data[2] = 0x00;
   outbuf.data[3] = 0x02;
   outbuf.data[4] = 0x98;
   outbuf.data[5] = 0x00;
   outbuf.data[6] = 0x00;
   outbuf.data[7] = 0x98;
   outbuf.data[8] = 0xB0;
   outbuf.data[9] = 0xB3;
   */
   printf("Sending parameter request.\n");
   com_send(IFACE_SERIAL2, &outbuf);

   printf("Waiting for response.\n");
   //retrieve the parameters
   uart_read(UART1, inbuf, 1);
   printf("Response received.\n");
}
