//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "msched.h" //provides Thread definition

#include "mos.h"
#include "led.h"
#include "msched.h"
#include "sem.h"

#include <stdlib.h>
#include "dev.h"    //provides access to rssi
#include "clock.h"  //provides us usleep



mos_sem_t empty1;
mos_sem_t full1;
mos_sem_t mux1;

mos_sem_t empty2;
mos_sem_t full2;
mos_sem_t mux2;

static mos_alarm_t alarm1;  //holds alarm for backoff_alarm_func
static mos_alarm_t alarm2;  //holds alarm for backoff_alarm_func
static mos_alarm_t alarm3;  //holds alarm for backoff_alarm_func
static mos_alarm_t alarm4;  //holds alarm for backoff_alarm_func
static uint32_t rand_val;
void sem_alarm_func(void *data)
{
  *(uint8_t*)data = 1;
}

//#define verbose


uint8_t fired1;
uint8_t fired2;
uint8_t fired3;
uint8_t fired4;
uint16_t seed;

//uint8_t verbose = true;

void start(void)
{
 

  mos_led_on(1);
#ifdef PLATFORM_MICA_ANY
  dev_mode(DEV_MICA2_LIGHT, DEV_MODE_ON);  
  dev_read(DEV_MICA2_LIGHT,&seed,1);
  dev_read(DEV_MICA2_LIGHT,(&seed)+1,1);
  dev_mode(DEV_MICA2_LIGHT, DEV_MODE_OFF);  
#elif PLATFORM_TELOSB
  seed = adc_get_conversion16(4);
#endif
  printf("SEED: %x\n", seed);
  mos_led_off(1);
  srand(seed);
 
  fired1=1;
  fired2=1;
  fired3=1;
  fired4=1;
  
  alarm1.func = sem_alarm_func;
  alarm2.func = sem_alarm_func;
  alarm3.func = sem_alarm_func;
  alarm4.func = sem_alarm_func;

  alarm1.data = (void*)&fired1;
  alarm2.data = (void*)&fired2;
  alarm3.data = (void*)&fired3;
  alarm4.data = (void*)&fired4;

  alarm1.reset_to = 0;
  alarm2.reset_to = 0;
  alarm3.reset_to = 0;
  alarm4.reset_to = 0;
  
  while(1)
    {
        if(fired1 == 1)
        {
            alarm1.msecs = (((uint16_t)rand() % 100) + 1);
            fired1=0;
            mos_alarm(&alarm1);
            mos_led_toggle(0);
#ifdef verbose
            printf("1 fired!\n");
#endif
        }
        if(fired2 == 1)
        {
            alarm2.msecs = (((uint16_t)rand() % 100) + 1);
            fired2=0;
            mos_alarm(&alarm2);
            mos_led_toggle(1);
#ifdef verbose
            printf("2 fired!\n");
#endif
        }
        if(fired3 == 1)
        {
            alarm3.msecs = (((uint16_t)rand() % 100) + 1);
            fired3=0;
            mos_alarm(&alarm3);
            mos_led_toggle(2);
#ifdef verbose
            printf("3 fired!\n");
#endif
        }
        if(fired4 == 1)
        {
            alarm4.msecs = (((uint16_t)rand() % 100) + 1);
            fired4=0;
            mos_alarm(&alarm4);
//	 mos_led_toggle(2);
#ifdef verbose
            printf("4 fired!\n");
#endif
        }
#ifdef verbose
        print_clock_list();
#endif
    }
}

