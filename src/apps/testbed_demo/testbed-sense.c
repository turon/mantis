//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "adc.h"
#include "led.h"
#include "printf.h"
#include "sem.h"
#include "user_button.h"

uint16_t light1[8];

uint16_t light2[8];
uint16_t temp[8];
uint16_t humid[8];

mos_sem_t button;



//0.upto(255) do |i| printf "#{(-40 + 0.018 * (i<<6)).to_i}, \n" end
static int8_t temp_conversion[] = {
-39, -38, -38, -37, -37, -36, -35, -35, -34, -33, -33, -32, -31, -31,
 -30, -30, -29, -28, -28, -27, -26, -26, -25, -24, -24, -23, -22, -22,
 -21, -21, -20, -19, -19, -18, -17, -17, -16, -15, -15, -14, -14, -13,
 -12, -12, -11, -10, -10, -9, -8, -8, -7, -6, -6, -5, -5, -4, -3, -3,
 -2, -1, -1, 0, 0, 0, 1, 2, 2, 3, 3, 4, 5, 5, 6, 7, 7, 8, 9, 9, 10,
 10, 11, 12, 12, 13, 14, 14, 15, 16, 16, 17, 18, 18, 19, 19, 20, 21,
 21, 22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32,
 32, 33, 34, 34, 35, 35, 36, 37, 37, 38, 39, 39, 40, 41, 41, 42, 42,
 43, 44, 44, 45, 46, 46, 47, 48, 48, 49, 50, 50, 51, 51, 52, 53, 53,
 54, 55, 55, 56, 57, 57, 58, 58, 59, 60, 60, 61, 62, 62, 63, 64, 64,
 65, 66, 66, 67, 67, 68, 69, 69, 70, 71, 71, 72, 73, 73, 74, 74, 75,
 76, 76, 77, 78, 78, 79, 80, 80, 81, 82, 82, 83, 83, 84, 85, 85, 86,
 87, 87, 88, 89, 89, 90, 90, 91, 92, 92, 93, 94, 94, 95, 96, 96, 97,
 98, 98, 99, 99, 100, 101, 101, 102, 103, 103, 104, 105, 105, 106,
 106, 107, 108, 108, 109, 110, 110, 111, 112, 112, 113, 114, 114, 115,
 115, 116, 117, 117, 118, 119, 119, 120, 121, 121, 122, 122, 123

};


// 0.upto(127) do |i| printf "#{(-4 + 0.0405 * (i<<5) + -2.8 * 10 ** -6 * (i<<5) ** 2).to_i}, \n" end

uint8_t humid_conversion[] = {
    0,0,0,0,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19,20,21,23,24,25,26,27,28,30,31,32,
    33,34,35,36,37,38,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,
    61,62,63,64,65,66,67,68,69,69,70,71,72,73,74,75,76,77,77,78,79,80,81,82,82,83,84,
    85,86,87,87,88,89,90,90,91,92,93,93,94,95,96,96,97,98,99,99,100,100,100,100,100,
    100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100
};


void button_thread()
{
  while(1)
    {
      mos_sem_wait(&button);
      mos_led_display(7);
      mos_thread_sleep(1000);
      mos_led_display(0);
      
    }
}

void blink_a (void)
{
   uint32_t sleep_time_a = 800;
   
   while (1) {
      mos_led_toggle (0);
      mos_thread_sleep (sleep_time_a);
   }
}

void blink_b (void)
{

   uint32_t sleep_time_b = 400;
   while (1) {
      mos_led_toggle (1);
      mos_thread_sleep (sleep_time_b);
   }  
}

void blink_c (void)
{

   uint32_t sleep_time_c = 200;
   while (1) {
      mos_led_toggle (2);
      mos_thread_sleep (sleep_time_c);
   }  
}

void sense_thread()
{
  uint8_t i=0,a;
  uint32_t hu,te,lt1,lt2;

  dev_open(DEV_MSP_HUMIDITY);
  
  if (dev_mode(DEV_MSP_HUMIDITY, DEV_MODE_ON) == DEV_FAILURE)
  {
     printf("SHT15 failed to initialize\n");
     //Start rapid blinking to indicate failure
     mos_thread_new (blink_a, 128, PRIORITY_NORMAL);
     mos_thread_new (blink_b, 128, PRIORITY_NORMAL);
     mos_thread_new (blink_c, 128, PRIORITY_NORMAL);
     return;
  }  
    
//  mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);

    
  //sleep to let the initialization on the reader finish
  mos_thread_sleep(1000);
    
  //  printf("Temperature, Humidity, Light1, Light2, Possible temp, Possible humid,\n");


  while(1) {
      dev_read(DEV_MSP_TEMPERATURE, (void*)&(temp[i]), sizeof(uint16_t));
      dev_read(DEV_MSP_HUMIDITY, (void*)&(humid[i]), sizeof(uint16_t));
      

      light1[i] = adc_get_conversion16(4);
      light2[i] = adc_get_conversion16(5);
      
      
  
      if(i==7)//every 8th time, agregate and send.
        {
          hu=0;
          te=0;
          
          lt1=0;
          lt2=0;

          for(a=0;a<8;a++)
            {
              te+=temp[a];
              hu+=humid[a];
              lt1+=light1[a];
              lt2+=light2[a];
            }
          //divide by 8
          lt1=lt1>>3;
          lt2=lt2>>3;
          te=te>>3;
          hu=hu>>3;
        
          //CSV format
          if(temp_conversion[te>>6]>=0)
              printf("%l,%l,%l,%l,%d,%d\n", te, hu, lt1, lt2,temp_conversion[te>>6],humid_conversion[hu>>5]);
          else
              printf("%l,%l,%l,%l,-%d,%d\n", te, hu, lt1, lt2,-temp_conversion[te>>6],humid_conversion[hu>>5]);
        }
      i=(i+1) & 7;
      mos_thread_sleep(7500);// 1/8 of a minute.
      // 1 minute is about 59053 ticks here
    }
}

void button_handler(void)
{
   mos_sem_post(&button);
}


void start (void)
{

    mos_mdelay(1000);
    printf("SYSTEM: NODE: Started.\n");
    
    mos_sem_init(&button,0);
    mos_enable_user_button(button_handler);
 
    mos_thread_new (button_thread, 128, PRIORITY_NORMAL);
    mos_thread_new (sense_thread, 128, PRIORITY_NORMAL);
}

