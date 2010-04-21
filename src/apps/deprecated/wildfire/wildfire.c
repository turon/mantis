//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "sem.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"

#define HUMIDITY_PORT PORTC
#define HUMIDITY_MASK (1 << 4)
#define HUMIDITY_DIRE DDRC

#define WIND_DIR_PORT PORTC
#define WIND_DIR_MASK (1 << 3)
#define WIND_DIR_DIRE DDRC

#define WIND_SPEED_PORT PORTC
#define WIND_SPEED_MASK (1 << 5)
#define WIND_SPEED_DIRE DDRC

uint16_t wind_ticks;
uint16_t wind_speed;
mos_alarm_t wind_timer;

uint32_t wind_timer_s = 4;
uint32_t wind_timer_us = 0;
mos_sem_t wind_speed_sem;

//static comBuf send_pkt; //comBuf goes in heap

void send_thread ();
void speed_thread ();

void start (void)
{
   mos_sem_init(&wind_speed_sem, 0);

   mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
   mos_thread_new (speed_thread, 128, PRIORITY_NORMAL);
}

void init_humidity(){
   HUMIDITY_DIRE |= HUMIDITY_MASK;
   HUMIDITY_PORT |= HUMIDITY_MASK;
}

void init_wind_dir(){
   WIND_DIR_DIRE |= WIND_DIR_MASK;
   WIND_DIR_PORT |= WIND_DIR_MASK;
}

void init_wind_speed(){
   //power wind speed 
   WIND_SPEED_DIRE |= WIND_SPEED_MASK;
   WIND_SPEED_PORT |= WIND_SPEED_MASK;

   handle_t ints = mos_disable_ints();

   //enable int0 in external int. mask
   EIMSK |= (1 << 0);

   //set PORTD PIN0 to input
   DDRD &= ~(1 << 0);

   //set intterrupt to fire on falling edge
   EICRA |= (1 << ISC01);
   EICRA &= ~(1 << ISC00);
   
   mos_enable_ints(ints);
}

uint16_t convert_direction(uint16_t sample)
{
  return (uint16_t)(((float)sample-60.0)/(944.0-60.0) * 360);
}

void send_thread ()
{

   uint16_t sample;
   init_humidity();
   init_wind_dir();
   init_wind_speed();

   uint8_t reset_header = 1;
   printf("lt\ttemp\thmdty\tw_dir\tw_dir_c\tw_spd\n");
   
   while(1) {
      if(((reset_header++) % 15) == 0){
	 printf("lt\ttemp\tw_dir\tw_dir_c\tw_spd\n");
	 reset_header = 1;
      }

      //light
      dev_read(DEV_MICA2_LIGHT, &sample, sizeof(sample));
      printf("%d\t", sample);

      //temp
      dev_read(DEV_MICA2_TEMP, &sample, sizeof(sample));
      printf("%d\t", sample);

      //humidity
      dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_2);
      dev_read(DEV_ADC, &sample, sizeof(sample));
      printf("%d\t", sample);

      //wind direction
      dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_1);
      dev_read(DEV_ADC, &sample, sizeof(sample));
      printf("%d\t", sample);
      sample = convert_direction(sample);
      printf("%d\t", sample);

      printf("%d\n", wind_speed);
      
      mos_thread_sleep(4000);
   }
}

void wind_timer_callback(){
   mos_sem_post(&wind_speed_sem);   
   mos_alarm (&wind_timer, wind_timer_s, wind_timer_us);
}

void speed_thread()
{
   wind_timer.func = wind_timer_callback;
   mos_alarm (&wind_timer, wind_timer_s, wind_timer_us);
   while(1){
      mos_sem_wait(&wind_speed_sem);
      wind_speed = wind_ticks;
      wind_ticks = 0;
      mos_led_toggle(1);
   }
}

SIGNAL (SIG_INTERRUPT0)
{
   wind_ticks++;
   mos_led_toggle(2);
}
