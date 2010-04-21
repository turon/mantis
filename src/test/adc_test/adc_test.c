//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "dev.h"
#include "adc.h"
#include "avr-adc.h"
#include "msp-adc.h"
#include "mutex.h"
#include "printf.h"

#define NUM_ADC_CHANS 3

void adc_test (void)
{
   uint16_t adc_vals[NUM_ADC_CHANS];
   uint8_t i;

   
   while (1) {
      printf ("adc readings: \n");
      for (i = 0; i < NUM_ADC_CHANS; i++) {
	 uint16_t val;
	 dev_ioctl (DEV_ADC, ADC_SET_CHANNEL, i);
	 dev_read(DEV_ADC, &val, sizeof(val));
	 printf ("\t%d", val);
      }
      uint8_t samp;
      uint16_t value;
      dev_read(DEV_MICA2_TEMP, &samp, 1);
      printf("\ttemp: %C ",samp);

      dev_read(DEV_MICA2_LIGHT, &value, 2);
      printf("\tlight: %d", value);
      printf ("\n");
      mos_thread_sleep(1000);
      }


}

void start (void)
{
   mos_thread_new (adc_test, 128, PRIORITY_NORMAL);
}
