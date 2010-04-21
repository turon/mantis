//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "adc.h"
#include "led.h"
#include "printf.h"
#include "sem.h"
#include "dev.h"
#include "uart.h"
#include "mica2-sounder.h"

mos_sem_t do_send;

#define PACKET_COUNT 4
comBuf send_pkt[PACKET_COUNT];
uint8_t sending[PACKET_COUNT];
uint8_t read_sample;

uint8_t skip1;
#define SKIP 1
/*
 *  0 - 7400
 *  1 - 7900
 *  2 - 6000
 *  3 - 5653
 *  4 - 5242
 *  5 - 4900
 *
 * 10 - 3820
 * 
 * 
 * 15 - 3090
*/


/*
  No skip, tuning
  4 - 8000
  5 - 6600
  6 - 4800
  7 - 3100
*/

void reading_init()
{
   read_sample=0;
   int a;

   for(a=0;a<PACKET_COUNT;a++)
   {
      send_pkt[a].size=64;
      sending[a] = 0;
   }
}


uint8_t which_sending, which_reading;

uint16_t i=0, count=0,sample;


static uint16_t adc_val;

//port definitions
/** @brief Mic directional port. */
#define MIC_PORT_DIRE DDRC
/** @brief Mic primary port. */
#define MIC_PORT PORTC
/** @brief Mic pin mask port. */
#define MIC_PIN_MASK 0x08
//static mos_sem_t adc_sem;

void adc_start_channel8(uint8_t ch)
{

   // grab the conversion value
//      while (ADCSRA & (1 << ADSC));
   MIC_PORT_DIRE |= MIC_PIN_MASK;
   MIC_PORT &= ~MIC_PIN_MASK;
   MIC_PORT |= MIC_PIN_MASK;

   mos_thread_sleep(5);
   
//   SFIOR |= (1 << ADHSM);
   ADMUX = ch /*| (1 << REFS0) | (1 << REFS1) */; //set the channel
   ADMUX |= (1 << ADLAR);
//   ADCSRA |= (1 << ADIF); //clear any old conversions...   
   ADCSRA = (1 << ADSC) | (1 << ADFR) | (1 << ADEN) | ( 1 << ADIE) | 5;
   
}
#define adc_off() ADCSRA &= ~(1 << ADEN)



uint8_t read_channel8()
{
   //mos_sem_wait(&adc_sem);
   
   return adc_val;
}


SIGNAL(SIG_ADC)
{
/*   if(++skip1>SKIP)
   {
   skip1=0;*/
         if(sending[which_reading]==0)
         {


            adc_val = ADCH;  
//      adc_val |= (ADCH << 8);

            send_pkt[which_reading].data[read_sample] = adc_val;
      
            read_sample++;
            if(read_sample>63)
            {
               mos_led_toggle(0);
               read_sample=0;
               which_reading=(which_reading+1)&(PACKET_COUNT - 1);
               mos_sem_post(&do_send);
            }
         }
         else
            mos_led_toggle(1);
//   }
} 







void sense_thread()
{
   //sleep to let the initialization on the reader finish
   mos_thread_sleep(1000);

#if defined(ARCH_AVR)
//   dev_mode(DEV_MICA2_MIC, DEV_MODE_ON);
   adc_start_channel8(AVR_ADC_CH_2);   
#endif
   while(1) 
   {
      
    
#if defined(ARCH_AVR)
      mos_led_toggle(2);
      
      // send_pkt.data[count]=read_channel8();
/*#elif defined(PLATFORM_TELOSB)
      sample = adc_get_conversion16(4); // the other light chanel is
      sample >>=2;
      send_pkt.data[count]=sample;*/
#endif
       
      count++;
      if(count>63)
      {
          
	 mos_led_toggle(0);
	 count=0;
	 //mos_sem_post(&do_send);
      }
   }
}

static void uart_set_baud(uint8_t uart_num, uint16_t baud_rate)
{
   if(uart_num == UART0) {
      UBRR0H = (uint8_t)(baud_rate >> 8);
      UBRR0L = (uint8_t)(baud_rate);
   } else {
      UBRR1H = (uint8_t)(baud_rate >> 8);
      UBRR1L = (uint8_t)(baud_rate);
   }
}

void send_thread()
{
      //sleep to let the initialization on the reader finish
   mos_thread_sleep(1000);

   reading_init();
   
#if defined(ARCH_AVR)
//   dev_mode(DEV_MICA2_MIC, DEV_MODE_ON);
   adc_start_channel8(AVR_ADC_CH_2);
   uint8_t a = 0;
   dev_write(DEV_MICA2_SOUNDER,&a,1);
//   mica2_souunder_on();
#endif

   uart_set_baud( UART0, B115200);
   printf("starting\n");
   
   while(1)
   {
      mos_sem_wait(&do_send);
      which_sending=(which_sending+1)&(PACKET_COUNT - 1);
      sending[which_sending]=1;
      
      mos_led_toggle(2);
#if defined(PLATFORM_TELOSB)
      com_send(IFACE_SERIAL2,&send_pkt[which_sending]);
#else
      com_send(IFACE_SERIAL,&send_pkt[which_sending]);
#endif
      sending[which_reading]=0;
   }
}


void start (void)
{
   mos_led_toggle(0);
   adc_off();

//   mos_sem_init(&adc_sem, 0);
   mos_sem_init(&do_send,0);

   
//   mos_thread_new (sense_thread, 128, PRIORITY_NORMAL);
   mos_thread_new (send_thread, 128, PRIORITY_NORMAL);
}

