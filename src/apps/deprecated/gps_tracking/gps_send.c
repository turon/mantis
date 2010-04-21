//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <ctype.h>

#include "led.h"
#include "msched.h"
#include "uart.h"
#include "adc.h"
#include "gps.h"


#define PHOTO_CHANNEL ADC_CH_2

void gps_send_data();

void start(void){
  mos_thread_new(gps_send_data, 128, PRIORITY_NORMAL);
}

/** out put all the data to the base station 
 * The receiver side is the Java 
 */
void gps_send_data(){

  uint8_t i;
  TSIPPKT *pkt;
  
  gps_init(TSIP_PROTO);
  mos_uart_open(0);
  gps_set_mode(TSIP_AUTO_REPORT);
  mos_adc_open();

  while(1){
         mos_led_toggle(2);
	 pkt = (TSIPPKT *)gps_get_position();
	 mos_uart_send(0,'#');

	 for(i=0; i<12; i++){ //12 bytes for the lon, lat and alt
	   mos_uart_send(0, pkt->buf[i]);
	 }

	 mos_uart_send(0, '$'); //the begin symbol for the photo sensor readings
	 i = mos_adc_eight_polling(PHOTO_CHANNEL);
 	 mos_uart_send(0, i);
	 // uart_send(0, 'a');
	 mos_uart_send(0,'|');
  }//while1

}
