//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


#include "mos.h"

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "printf.h"
#include "com.h"
#include "led.h"
#include "dev.h"
#include "clock.h"


#if defined(PLATFORM_MICA_ANY)
#include "mica2-sounder.h"
#endif


#if defined(PLATFORM_MICA_ANY)
static void click()
{
   uint8_t arg;

   dev_open(DEV_MICA2_SOUNDER);
   
   arg = 1;
   dev_write(DEV_MICA2_SOUNDER, &arg, sizeof(arg));

   mos_udelay(0xffff);
   mos_udelay(0xffff);

   arg = 0;
   dev_write(DEV_MICA2_SOUNDER, &arg, sizeof(arg));

   dev_close(DEV_MICA2_SOUNDER);
}

typedef void (*reboot_func_t)(void);

void restart()
{
   // Pointer to normal RESET vector
   SPCR &= ~(1 << SPE);
   reboot_func_t reset_func = (reboot_func_t)0x0000; 
   reset_func();
}

void reboot()
{
   SPCR &= ~(1 << SPE);
   reboot_func_t reboot_func = (reboot_func_t)0xF000;
   reboot_func();
}

#define get_val(dev)				\
   do {						\
      uint8_t result;				\
      dev_open_##dev();				\
      dev_read_##dev(&result, 1);		\
      dev_close_##dev();			\
      printf ("val: %d\n", result);		\
   } while (0)

void get_light()
{
   dev_mode(DEV_MICA2_LIGHT, DEV_MODE_ON);
   get_val(DEV_MICA2_LIGHT);  
}

void get_temp()    { get_val(DEV_MICA2_TEMP); }
void get_mic()     { get_val(DEV_MICA2_MIC); }
void get_accelx()  { get_val(DEV_MICA2_ACCEL_X); }
void get_accely()  { get_val(DEV_MICA2_ACCEL_Y); }
void get_eeprom()  { get_val(DEV_AVR_EEPROM); }
void get_magnetx() { get_val(DEV_MICA2_MAGNET_X); }
void get_magnety() { get_val(DEV_MICA2_MAGNET_Y); }

void get_battery()
{
   uint16_t voltage;
   uint16_t high_val;
   uint16_t low_val;
   dev_open(DEV_MICA2_BATTERY);
   dev_mode(DEV_MICA2_BATTERY, DEV_MODE_ON);
   //mos_mdelay(200);
   dev_read(DEV_MICA2_BATTERY, &voltage, sizeof(voltage));
   high_val = voltage / 1000;
   low_val = (uint16_t)(voltage - (1000 * high_val));
   dev_mode(DEV_MICA2_BATTERY, DEV_MODE_OFF);
   dev_close(DEV_MICA2_BATTERY);
   printf("Battery is at %d.%d volts, %d\n", high_val, low_val, voltage);
   //printf("Battery is ADC level is at %d bits\n", voltage);
}
#endif


#ifdef PLATFORM_MICA2
void get_rssi()
{
   uint16_t rssi_val;
   dev_open(DEV_AVR_RSSI);
   dev_read(DEV_AVR_RSSI, &rssi_val, sizeof(rssi_val));
   dev_close(DEV_AVR_RSSI);
   printf("rssi %d\n",rssi_val);
}
#endif

#if defined(PLATFORM_MICA_ANY)
void radio_poll()
{
   comBuf *recvdbuf;
   while(1) {
      recvdbuf = com_recv(IFACE_RADIO);
      mos_led_toggle(1);
      com_free_buf(recvdbuf);
   }
}

void poll()
{
   uint8_t times;
   for(times = 0; times < 20;times++) {
      get_battery();
      mos_mdelay(750);
   }
}
#endif


void register_mica_functions()
{
#if defined(PLATFORM_MICA_ANY)
   mos_register_function("light", get_light);
   mos_register_function("temp", get_temp);
   mos_register_function("mic", get_mic);
   mos_register_function("accelx", get_accelx);
   mos_register_function("accely", get_accely);
   mos_register_function("click", click);
   mos_register_function("reboot", reboot);
   mos_register_function("restart", restart);
   mos_register_function("poll", poll);
   mos_register_function("magnetx", get_magnetx);
   mos_register_function("magnety", get_magnety);
   mos_register_function("battery", get_battery);
   mos_register_function("eeprom", get_eeprom);
#endif
#ifdef PLATFORM_MICA2
   mos_register_function("rssi", get_rssi);
#endif
}
