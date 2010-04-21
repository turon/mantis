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


#include "dev.h"    //provides access to rssi
#include "clock.h"  //provides us usleep



mos_sem_t empty1;
mos_sem_t full1;
mos_sem_t mux1;

mos_sem_t empty2;
mos_sem_t full2;
mos_sem_t mux2;

static mos_alarm_t producer1_alarm;  //holds alarm for backoff_alarm_func
static mos_alarm_t consumer1_alarm;  //holds alarm for backoff_alarm_func
static mos_alarm_t producer2_alarm;  //holds alarm for backoff_alarm_func
static mos_alarm_t consumer2_alarm;  //holds alarm for backoff_alarm_func
void sem_alarm_func(void *data)
{
  mos_sem_post((mos_sem_t*)data);
}

void producer1()
{
    while(1)
    {
        mos_sem_wait(&empty1);

        mos_sem_wait(&mux1);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux1);

        mos_sem_wait(&mux1);
        mos_thread_sleep( ((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux1);
        producer1_alarm.msecs = ((uint16_t)rand() % 100) + 1;
        mos_alarm(&producer1_alarm);
        
        //        mos_sem_post(&full1);
        mos_led_toggle(0);
    }
}

void consumer1()
{
    while(1)
    {
        
        mos_sem_wait(&full1);

        mos_sem_wait(&mux1);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux1);

        mos_sem_wait(&mux1);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux1);

        consumer1_alarm.msecs = ((uint16_t)rand() % 100) + 1;
        mos_alarm(&consumer1_alarm);

        //        mos_sem_post(&empty1);
        mos_led_toggle(1);
    }
    

}
void producer2()
{
    while(1)
    {
        mos_sem_wait(&empty2);

        mos_sem_wait(&mux2);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux2);

        mos_sem_wait(&mux2);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux2);

        producer2_alarm.msecs =  ((uint16_t)rand() % 100) + 1;;
        mos_alarm(&producer2_alarm);

        //        mos_sem_post(&full2);
        mos_led_toggle(1);
    }
}

void consumer2()
{
    while(1)
    {
        
        mos_sem_wait(&full2);

        mos_sem_wait(&mux2);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux2);

        mos_sem_wait(&mux2);
        mos_thread_sleep(((uint16_t)rand() % 100) + 1);//copy something to the buffer
        mos_sem_post(&mux2);

        consumer2_alarm.msecs = ((uint16_t)rand() % 100) + 1;
        mos_alarm(&consumer2_alarm);

        //        mos_sem_post(&empty2);
        mos_led_toggle(2);
    }
    

}


  uint16_t seed;
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
  
  producer1_alarm.func = sem_alarm_func;
  consumer1_alarm.func = sem_alarm_func;
  producer2_alarm.func = sem_alarm_func;
  consumer2_alarm.func = sem_alarm_func;


  producer1_alarm.data = (void*)&full1;
  consumer1_alarm.data = (void*)&empty1;
  producer2_alarm.data = (void*)&full2;
  consumer2_alarm.data = (void*)&empty2;
  
  mos_sem_init(&empty1,5);
  mos_sem_init(&full1,0);
  mos_sem_init(&mux1,1);
  mos_sem_init(&empty2,5);
  mos_sem_init(&full2,0);
  mos_sem_init(&mux2,1);
  mos_thread_new(consumer2, 128, PRIORITY_NORMAL);
  mos_thread_new(consumer1, 128, PRIORITY_NORMAL);
  mos_thread_new(producer2, 128, PRIORITY_NORMAL);
  mos_thread_new(producer1, 128, PRIORITY_NORMAL);
}

