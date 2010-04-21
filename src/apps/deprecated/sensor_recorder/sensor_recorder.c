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
#include "clock.h"

//initial addr to begin writing to in EEPROM
#define EEPROM_START_ADDR 100

static uint8_t input_device;
static uint8_t output_device;

static uint8_t sending_commands;
static uint16_t msec_delay;
static uint16_t count;
static uint16_t clock_counter;
static uint16_t sample_counter;
static uint8_t device_count;

static dev_read_func_t device_func_ptr;

static mos_alarm_t alarm;

static comBuf send;

const char *device_list[] = {
   "Photocell (light)",
   "Thermistor (temp)",
   "Microphone",
   "Accelerometer - X",
   "Accelerometer - Y",
   "Magnetometer - X",
   "Magnetometer - Y",
   "Eeprom Reader"
};
const char *output_list[] = {"Screen", "Eeprom", "Radio"};


void stop_sending ()
{
   sending_commands = 0;
}

typedef void (*reboot_func_t)(void);
void reboot ()
{
   reboot_func_t reboot_func = (reboot_func_t)0xF000;
   reboot_func();
}

static void click () 	 
{ 	 
#ifdef PLATFORM_MICA_ANY
   uint8_t arg; 	 
  	 
   arg = 1; 	 
   dev_write(DEV_MICA2_SOUNDER, &arg, sizeof(arg)); 	 
   mos_thread_sleep (1); 	 
   arg = 0; 	 
   dev_write(DEV_MICA2_SOUNDER, &arg, sizeof(arg)); 	 
#endif 	 
}

void change_sensor ()
{
   uint8_t selection;
   uint8_t i;
   printf("Sensor List:\n");

   for (i = 0; i < device_count - 1; i++) {
      if (i == input_device) //we're on the current selection
	 printf (" * ");
      else
	 printf ("   ");
      printf ("(%d)  %s\n", i, device_list[i]);
   }
   printf ("\n");

   selection = prompt_uint8 ("New Sensor Selection:");
   if (selection > device_count - 1) {
      printf ("Error, number out of range.\n");
      printf ("Selecting photocell for default.\n");
      input_device = 0;
   }
   else if(selection == 7 && output_device == 2) {
      printf ("Input device and output devices cannot ");
      printf ("both be set to %s.",output_list[2]);
      selection = device_count + 1;
   }
   else if (selection < device_count - 1) {
      input_device = selection;
   }

}

void set_delay ()
{
   uint16_t entered_delay = 0;
   printf ("Current Delay: %d msec\n",msec_delay);
   if (input_device < 7){
      while (entered_delay < 50) { //loop until sane input
	 entered_delay = prompt_long("New Delay (msec):");
	 if (entered_delay < 50){
	    printf ("Error: delay too short.");
	    printf ("Device needs time to record/output values.\n");
	 } else
	    msec_delay = entered_delay;
      }
   } else {
      printf ("Device doesn't take a delay.");
   }
}

void set_count()
{
   uint16_t entered_count = 0;
   printf ("Current sample count: %d samples\n", count);
   while (entered_count < 1) {
      entered_count = prompt_long ("New count (samples):");
      if (entered_count < 1)
	 printf ("Error: Value too low.\n");
      else
	 count = entered_count;
   }
}

void set_output()
{
   uint8_t output_selection = 4;
   uint8_t i;
   printf ("Output Devices:\n");

   for (i = 0; i < 3; i++){
      if (i == output_device)
	 printf (" * ");
      else
	 printf ("   ");
      printf ("(%d) %s", i, output_list[i]);
   }

   while (output_selection > 2) {
      output_selection = prompt_long ("New Output Device:");
      if (output_selection >= 3){
	 printf ("Value too high.");
      }
      if (output_selection == 2 && input_device == 7) {
	 printf("Input device and output devices cannot ");
	 printf("both be set to %s.", output_list[2]);
	 output_selection = 4;
      }
      if (output_selection < 3)
	 output_device = output_selection;
   }
}

void sense_service ()
{
   uint8_t input_value;
   device_func_ptr (&input_value, sizeof (input_value));
   if (output_device == 0)
      printf ("val: %d", input_value);
   if (output_device == 1)
      dev_write (DEV_AVR_EEPROM, &input_value, 1);
   if (output_device == 2) {
      send.data[0] = input_value;
      com_send (IFACE_RADIO, &send);
   }
}

void clock_service()
{
   if (clock_counter == 0)
      return;
   clock_counter--;
   if (clock_counter == 0){ //time is up, record sample
      sense_service ();
      if (--sample_counter != 0)
	 clock_counter = msec_delay;
   }

   alarm.reset_to = msec_delay;
}

void start_sensing()
{
   if (input_device < 8) {
      switch (input_device) {
      case 0: //photocell
	 device_func_ptr = dev_read_DEV_MICA2_LIGHT;
	 break;
      case 1:
	 device_func_ptr = dev_read_DEV_MICA2_TEMP;
	 break;
      case 2:
	 device_func_ptr = dev_read_DEV_MICA2_MIC;
	 break;
#if defined(MICA2_PLATFORM_ANY)
      case 3:
	 device_func_ptr = dev_read_DEV_MICA2_ACCEL_X;
	 break;
      case 4:
	 device_func_ptr = dev_read_DEV_MICA2_ACCEL_Y;
	 break;
#endif
      case 5:
	 device_func_ptr = dev_read_DEV_MICA2_MAGNET_X;
	 break;
      case 6:
	 device_func_ptr = dev_read_DEV_MICA2_MAGNET_Y;
	 break;
      case 7:
	 device_func_ptr = dev_read_DEV_AVR_EEPROM;
	 break;
      }
   }
   
   // initialize timer 3
   sample_counter = count;
   alarm.func = clock_service;
   alarm.msecs = msec_delay;
   alarm.reset_to = msec_delay;
   mos_alarm (&alarm);
}

void start(void)
{
   msec_delay = 1;
   count = 15;
   device_count = 9;
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, EEPROM_START_ADDR);
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
   //mos_thread_sleep(32);
   mos_register_function ("set delay", set_delay);
   mos_register_function ("set count", set_count);
   mos_register_function ("change sensor", change_sensor);
   mos_register_function ("change output", set_output);
   mos_register_function ("start", start_sensing);
   mos_register_function ("click", click);
   mos_register_function ("reboot", reboot);
}
