//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

//#include <unistd.h>
#include "mos.h"
#include "printf.h"
#include "realtime.h"
#include "clock.h"
#include "avr-adc.h"
/**
 * File:     time_elapsed_test.c      
 * Author:   Charles
 * Date:     05-29-2004
 *
 * Description: This app is for testing the real time timer interface
 *
 */

void start(void)
{
   uint32_t *ms_ptr;

   uint16_t i;
   uint8_t j;
   uint16_t samp_count = 1000;

   uint16_t rssi_value;
   printf("Time Elapsed test.\n");

   real_timer_init();
   for(j=0;j<15;j++)
   {
      real_timer_clear();
      for (i=0; i < samp_count; i++)
      {
	 rssi_value = adc_read_channel16(AVR_ADC_CH_0);
	 if(1==1);
      }
      ms_ptr=real_timer_get_ticks();
      printf("trial:%d\t samples:%d\t time:%l ms\n",j,samp_count,*ms_ptr);
   }
   
}
