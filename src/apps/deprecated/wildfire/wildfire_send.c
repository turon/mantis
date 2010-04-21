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
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"
#include "wildfire.h"
#include "net.h"
#include "mst.h"


uint16_t wind_ticks;
uint16_t wind_speed;
mos_alarm_t wind_timer;

uint32_t wind_timer_ms = 1000;

static comBuf sendBuf;

void send_thread ();
void wind_timer_callback();

void start (void)
{
  uint16_t myaddr = mos_node_id_get();
  
  net_init();
  mst_proto_init();

  printf("\n\n my addr: %d\n", myaddr);
  
  net_ioctl(MST_PROTO_ID, SET_ADDR, (uint8_t)myaddr);
  net_ioctl(MST_PROTO_ID, SET_DTB, 20);  

  wind_timer.func = wind_timer_callback;
  wind_timer.msecs = wind_timer_ms;
  wind_timer.reset_to = wind_timer_ms;

  mos_alarm(&wind_timer);
  
//  mos_alarm (&wind_timer, wind_timer_ms);

  mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
}


void init_devs(){
   HUMIDITY_DIRE |= HUMIDITY_MASK;
   WIND_DIR_DIRE |= WIND_DIR_MASK;
   WIND_SPEED_DIRE |= WIND_SPEED_MASK;
}

void humidity_on() { HUMIDITY_PORT |=  HUMIDITY_MASK; }
void humidity_off(){ HUMIDITY_PORT &= ~HUMIDITY_MASK; }


void wind_dir_on() { WIND_DIR_PORT |=  WIND_DIR_MASK; }
void wind_dir_off(){ WIND_DIR_PORT &= ~WIND_DIR_MASK; }

void init_wind_speed(){
   //power wind speed 

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
  return (uint16_t)(((float)sample)/(1024) * 360);
}

uint16_t convert_temp(uint16_t sample)
{
  uint16_t ret;
  ret = pgm_read_byte(&temps_vals[sample]);
  return ret; 
}

uint16_t convert_humidity(uint16_t sample)
{
  return sample; 
}




void send_thread ()
{
  wildfire_packet_t *packet;
  uint16_t sample;
  uint8_t reset_header;
  
  init_devs();
  init_wind_speed();

  packet = (wildfire_packet_t *)&(sendBuf.data[0]);
  
  reset_header=0;
  
  while(1) {
    if(((reset_header++) % 15) == 0){
      printf("temp\ttemp_c\thmdty\thmdty_c\tw_dir\tw_dir_c\tw_speed\n");
      reset_header = 1;
    }
    
    //light, not used
    //dev_read(DEV_MICA2_LIGHT, &sample, sizeof(sample));
    //printf("%d\t", sample);

    //temp
    dev_read(DEV_MICA2_TEMP, &sample, sizeof(sample));
    packet->temp = sample;
    packet->temp_c = convert_temp(sample);
    
    //humidity
    humidity_on();
    mos_mdelay(200); //1 second settling time
    //    dev_open(DEV_ADC);
    dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_2);
    dev_read(DEV_ADC, &sample, sizeof(sample));
    packet->humidity = sample;
    packet->humidity_c = convert_humidity(sample);
    //dev_close(DEV_ADC);
    humidity_off();
    
    //wind direction
    wind_dir_on();
    //dev_open(DEV_ADC);
    dev_ioctl(DEV_ADC, ADC_SET_CHANNEL, AVR_ADC_CH_1);
    dev_read(DEV_ADC, &sample, sizeof(sample));
    //dev_close(DEV_ADC);
    wind_dir_off();
    packet->wind_direction = sample;
    packet->wind_direction_c = convert_direction(sample);
    
    //wind speed
    packet->wind_speed = wind_speed;
    

    //printf("temp\ttemp_c\thmdty\thmdty_c\tw_dir\tw_dir_c\tw_speed\n");
    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
	   packet->temp,
	   packet->temp_c,
	   packet->humidity,	   
	   packet->humidity_c,
	   packet->wind_direction,
	   packet->wind_direction_c,
	   packet->wind_speed);

    
    sendBuf.size = sizeof(wildfire_packet_t);

    net_send(&sendBuf, MST_PROTO_ID, 20, true, MST_DATA, 0);
    mos_led_toggle(0);
    mos_thread_sleep(2000);
  }
}


void wind_timer_callback(){
  wind_speed = wind_ticks;
  wind_ticks = 0;
  //mos_alarm (&wind_timer, wind_timer_ms);
}

SIGNAL (SIG_INTERRUPT0)
{
   wind_ticks++;
   mos_led_toggle(2);
}
