//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    sense.c  (bionet)                                             */
/* Author     Charles Gruenwald III   :   gruenwal@colorado.edu           */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* Sensing application for bionet sensor nodes.                           */
/**************************************************************************/


#include "mos.h"
#include "msched.h"
#include "led.h"
#include "avr-adc.h"
//#include "adc.h"
#include "dev.h"
#include "com.h"
#include "printf.h"
#include "command_daemon.h"

#include "cc1000.h"


#include "node_net_event.h"
#include "node_id.h"
#include "clock.h"


#include "bedrest_config.h"
#include "bedrest_shared.h"

#include "dev.h"
#include "sem.h"

#define ACCEL_BUF_SIZE 40



static comBuf outpacket;
static comBuf configPacket;
comBuf *recv_pkt;


static uint8_t net_addr;
static uint8_t accelx, accely;

static uint32_t send_sleep;

static uint16_t accel_cb_x [ACCEL_BUF_SIZE];
static uint16_t accel_cb_y [ACCEL_BUF_SIZE];
static uint8_t accel_buf_len;

static   uint16_t accelx_avg;
static   uint16_t accely_avg;
static   uint16_t accelx_std;
static   uint16_t accely_std;

static uint16_t Xa;
static uint16_t Xb;
static uint16_t Ya;
static uint16_t Yb;

net_event_t *event;
bedrest_t *packet;
sem dtpa_sem;


net_event_t *configEvent;

uint32_t sum = 0;

/* sense and send thread */
void sense_and_send();
uint16_t accel_calc();
void accel_sample();

void dtpa_process_ack();
void dtpa_process_nack();
void dtpa_thread();




// DTPA Initialization:

uint16_t TxPower            = INIT_POWER;
uint8_t  TxState            = TX_POWER_FIND_FLOOR;
uint8_t  TxEvent            = 0;
uint8_t  nacks              = 0;
uint8_t  consecutive_acks   = 0;




void init()
{
   mos_sem_init(&dtpa_sem,0);
   
   send_sleep = 2000;

   accel_buf_len = 0;

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ) || defined(PLATFORM_MICA2DOT)
   
   //   dev_open(DEV_AVR_EEPROM);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 14);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Xa, sizeof(Xa));
   
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 16);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Xb, sizeof(Xb));

   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 18);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Ya, sizeof(Ya));

   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 20);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&Yb, sizeof(Yb));
   //   dev_close(DEV_AVR_EEPROM);

#endif   

   configEvent = (net_event_t *) &configPacket.data;

   configEvent -> from = net_addr;
   configEvent -> to = 0;
   configEvent -> event = BEDREST_CONFIG_PACKET;

   config_t *configs = (config_t *)&(configPacket.data[6]);
   configs->sample_delay = SAMPLE_DELAY_MS;
   configs->sample_n = SAMPLE_N;
   configs->Xa = Xa;
   configs->Xb = Xb;
   configs->Ya = Ya;
   configs->Yb = Yb;
   configs->resistor = THERMISTER_OHMS;
   configs->config_interval = CONFIG_PACKET_INTERVAL;
   configs->smooth_num = SMOOTHING_NUM_READINGS;
   configs->smooth_disc = SMOOTHING_DISCARDS;
#if defined(RAW_DATA_MODE)
   configs->raw_data = 1;
#else
   configs->raw_data = 0;
#endif
   configs->svn_id = SOFTWARE_VERSION;
   
   configPacket.size = 6+sizeof(config_t);//offset plus number of config items



}


#define MICA2_BATTERY_PORT PORTC
#define MICA2_BATTERY_DIRE DDRC
#define TEMP_LO (1 << 6)
#define BATT_LO (1 << 7)

#define mica2dot_battery_on() MICA2_BATTERY_DIRE |= TEMP_LO | BATT_LO;	\
   MICA2_BATTERY_PORT |= TEMP_LO;					\
   MICA2_BATTERY_PORT &= ~BATT_LO;

#define mica2dot_battery_off() MICA2_BATTERY_DIRE &= ~(TEMP_LO | BATT_LO); 

#define mica2dot_temp_on() MICA2_BATTERY_DIRE |= TEMP_LO | BATT_LO;	\
   MICA2_BATTERY_PORT |= BATT_LO;					\
   MICA2_BATTERY_PORT &= ~TEMP_LO;

#define mica2dot_temp_off() MICA2_BATTERY_DIRE &= ~(TEMP_LO | BATT_LO); 


#define BATTERY_CHANNEL AVR_ADC_CH_1
#define TEMP_CHANNEL AVR_ADC_CH_1



void start(void)
{
   
   net_addr = mos_node_id_get ();
   
   init();
#ifdef DTPA_ENABLED
   mos_thread_new(dtpa_thread,128, PRIORITY_NORMAL);
#endif
   mos_thread_new(sense_and_send, 196, PRIORITY_NORMAL);
}


uint8_t a;
uint16_t* values;
uint8_t seqNum;
uint8_t sampleCount;
uint8_t sentCount;
uint16_t i, j;
uint16_t value,reading_sum;

//#define ADC_READ adc_read_channel16
#define ADC_READ avr_adc_read_channel16

uint16_t average_read(uint8_t to_read)
{
   reading_sum=0;
   for(a=0;a<SMOOTHING_NUM_READINGS;a++)
   {
      switch(to_read)
      {
      case DEV_MICA2_TEMP:
         value = ADC_READ (TEMP_CHANNEL);
         break;
      case DEV_MICA2_BATTERY: 
         value = ADC_READ (BATTERY_CHANNEL);
         break;
      case DEV_MICA2_ACCEL_X: 
         dev_read (DEV_MICA2_ACCEL_X, &value, 2);
         break;
      case DEV_MICA2_ACCEL_Y: 
         dev_read (DEV_MICA2_ACCEL_Y, &value, 2);
         break;
      default:
         printf ("ERROR: NO SUCH DEVICE %d\n",to_read);
         return -1;
         break;
      }
        
      if (a>= SMOOTHING_DISCARDS)
         reading_sum += value;
   }
   reading_sum /= (SMOOTHING_NUM_READINGS - SMOOTHING_DISCARDS);
   return reading_sum;
}


void sense_and_send()
{
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, TxPower);


   com_send(IFACE_RADIO,&configPacket);

   event = (net_event_t *) &outpacket.data;
   event->from = net_addr; //from us
   event->to = 0;          //to the base stattion
   event->event = BEDREST_PACKET;
#if defined(RAW_DATA_MODE)
   event->event = BEDREST_RAW_PACKET;
#else
   event->event = BEDREST_PACKET;
#endif   

   packet = (bedrest_t *)&(outpacket.data[6]);

   mos_thread_set_suspend_state(SUSPEND_STATE_SLEEP);
    
   seqNum=0;
   sampleCount=0;
   sentCount=5; //ofset them

   mos_thread_sleep(100);
   
   
   while(1) 
   {
      sampleCount++;
      sentCount++;
       

      
      mos_thread_sleep(SAMPLE_DELAY_MS/2);
     if(sentCount>CONFIG_PACKET_INTERVAL)
      {
         sentCount=0;
#ifdef SHOW_LEDS
         mos_led_toggle(2);
#endif
           
         com_send(IFACE_RADIO,&configPacket);
/*#ifdef DTPA_ENABLED
  mos_sem_post(&dtpa_sem);
  #endif*/
      }

      
/* I need to have some sleep time between turning on the accelerometer
* and starting to read from it, otherwise the startup conditions
* cause the reading to be completly messed up.
*/
         dev_mode(DEV_MICA2_ACCEL_X, DEV_MODE_ON);
         mos_thread_sleep(SAMPLE_DELAY_MS/2);
         i = average_read(DEV_MICA2_ACCEL_X);
         j = average_read(DEV_MICA2_ACCEL_Y);
         dev_mode(DEV_MICA2_ACCEL_X, DEV_MODE_OFF);
      

      
 
         if(accel_buf_len < ACCEL_BUF_SIZE)
         {
            accel_cb_x[accel_buf_len] = i;
            accel_cb_y[accel_buf_len] = j;
            accel_buf_len++;
         }      

         if(sampleCount>=SAMPLE_N)
         {
            seqNum++;
            sampleCount=0;
#ifdef SHOW_LEDS
            mos_led_toggle(2);
#endif
        
//            dev_open(DEV_MICA2_BATTERY);
//            dev_open(DEV_MICA2_TEMP);

            accel_calc();


      
            packet->avg_accelx = accelx_avg;
            packet->avg_accely = accely_avg;
            packet->std_accelx = accelx_std;
            packet->std_accely = accely_std;
            packet->seqNum = seqNum;
      
            mica2dot_temp_on();

            packet->temp = average_read(DEV_MICA2_TEMP);

            mica2dot_battery_on();
//battery
            packet->battery = average_read(DEV_MICA2_BATTERY);

            dev_mode(DEV_MICA2_BATTERY, DEV_MODE_OFF);
            mica2dot_battery_off();

#ifdef RAW_DATA_MODE
            outpacket.size = 6+sizeof(bedrest_t)+SAMPLE_N*4;
            values = &packet->temp;
            values+=2;
            for(a=0;a<SAMPLE_N;a++)
               *values++ = accel_cb_x[a];

            for(a=0;a<SAMPLE_N;a++)
               *values++ = accel_cb_y[a];
#else
            outpacket.size = 6+sizeof(bedrest_t);
#endif

#ifdef DTPA_ENABLED
            cc1000_set_power(TxPower);
#endif
            //tx power
            packet->txpower = cc1000_get_power();

#ifdef DTPA_ENABLED
            mos_sem_post(&dtpa_sem);
#else
            com_send(IFACE_RADIO, &outpacket);
#endif

         }
      }
}

void accel_calc_individual(uint16_t *buf, uint16_t *avg, uint16_t *std)
{
   uint8_t i;

   if (accel_buf_len == 0)
      return;

   sum = 0;

   //calculate average
   for(i = 0; i < accel_buf_len; i++)
   {
      sum += buf[i];
   }

   *avg = (sum / accel_buf_len);

   //calculate std-dev
   sum = 0;
   for(i = 0; i < accel_buf_len; i++) {
      sum += (buf[i] - *avg) * (buf[i] - *avg);
   }

   *std = (sum / accel_buf_len);
}

uint16_t accel_calc()
{

   uint8_t cnt;
   uint16_t rms;

   if(accel_buf_len == 0)
   {
      return accelx_std + accely_std;
   }
   //calculate the rms
   accel_calc_individual(accel_cb_x, &accelx_avg, &accelx_std);
   accel_calc_individual(accel_cb_y, &accely_avg, &accely_std);

   //empty stored points
   cnt = accel_buf_len;
   accel_buf_len = 0;
  

   accelx = accelx_avg;
   accely = accely_avg;
   rms = accelx_std + accely_std;

   return rms;
}


void dtpa_thread()
{
   IF_SET iset;

   net_event_t *event;
   uint8_t acked = 0;
   uint8_t a;
   while(1)
   {
      acked=0;
      
      mos_sem_wait(&dtpa_sem);
      printf("sending   ");
      com_send(IFACE_RADIO, &outpacket);
      com_mode (IFACE_RADIO, IF_LISTEN);
      // Loop/wait/timeout for ACK from basestation
      for (a = 0; a < TIME_STEPS; a++)
      {
         mos_thread_sleep(TIMEOUT/TIME_STEPS);
         IF_ZERO(&iset);
         IF_SET(IFACE_RADIO, &iset);
         com_select(&iset, 0);
         
         if (IF_ISSET(IFACE_RADIO, &iset))
         {
            
            recv_pkt = com_recv (IFACE_RADIO);
            
            
//         recv_pkt = com_recv_timed (IFACE_RADIO, TIMEOUT/2);
//             if (recv_pkt!=NULL) 
//             { 
            
            event = (net_event_t *)recv_pkt->data;

            if (event->to == net_addr && event->from == 0)
            {
               //         dtpa_process_ack();
               printf("acked\n");
               acked = 1;
               com_free_buf (recv_pkt);
//               com_mode (IFACE_RADIO, IF_OFF);
               break;
            }
            com_free_buf (recv_pkt);
         }
      }
      com_mode (IFACE_RADIO, IF_OFF);
      if(acked==0)
      {
         // FIXME: do I want to resend the packet to the relay node?
         // send packet to basestation relay
         // Need sequence number handling in the HAB before we retransmit
         com_send(IFACE_RADIO, &outpacket);
         printf("not acked\n");
         
         //   dtpa_process_nack();
      }

   }
   
}


void dtpa_process_ack(void) {

   nacks = 0;
   consecutive_acks++;

   switch (TxState) {
      case TX_POWER_MAXIMUM:
	 TxEvent = 1;
	 if (consecutive_acks > ACK_THRESHHOLD) {
	    TxEvent = 2;
	    TxState = TX_POWER_FIND_FLOOR;
	    TxPower -= POWER_STEP_DECREASE;
	    if ((int16_t) TxPower < MIN_POWER) 
	       TxPower = MIN_POWER;
	    consecutive_acks = 0;
	 }
	 break;

      case TX_POWER_FIND_FLOOR:
	 TxEvent = 3;
	 if (consecutive_acks > ACK_THRESHHOLD) {
	    TxEvent = 4;
	    TxPower -= POWER_STEP_DECREASE;
	    if ((int16_t) TxPower < MIN_POWER) 
	       TxPower = MIN_POWER;
	    consecutive_acks = 0;
	 }
	 break;
	 
      case TX_POWER_FLOOR:
	 TxEvent = 5;
	 if (consecutive_acks > FLOOR_ACKS) {
	    TxEvent = 6;
	    TxPower -= POWER_STEP;
	    if ((int16_t) TxPower < MIN_POWER) 
	       TxPower = MIN_POWER;
	    TxState = TX_POWER_PROBE;
	    consecutive_acks = 0;
	 }
	 break;
	 
      case TX_POWER_PROBE:
	 TxEvent = 7;
	 if (consecutive_acks > ACK_THRESHHOLD) {
	    TxEvent = 8;
	    TxState = TX_POWER_FLOOR;
	    if ((int16_t) TxPower < MIN_POWER) 
	       TxPower = MIN_POWER;
	    consecutive_acks = 0;
	 }
	 break;
	 
      default:
	 break;
   }
}


void dtpa_process_nack(void) {

   nacks++;
   consecutive_acks = 0;

   // If miss MAX_NACKS consective ACKS then reset to high power
   if (nacks >= MAX_NACKS)
   {
      TxEvent = 9;
      TxPower = MAX_POWER;
      TxState = TX_POWER_MAXIMUM;
   }
   else
   {
      switch (TxState) {
      default:
      case TX_POWER_MAXIMUM:
         TxEvent = 10;
         TxPower = MAX_POWER;
         break;

      case TX_POWER_FIND_FLOOR:
         TxEvent = 11;
         if (nacks > NACK_THRESHHOLD) {
            TxEvent = 12;
            TxPower += POWER_STEP_INCREASE;
            if (TxPower > MAX_POWER) TxPower = MAX_POWER;
            nacks = 0;
            TxState = TX_POWER_FLOOR;
         }
         break;
	    
      case TX_POWER_FLOOR:
         TxEvent = 13;
         if (nacks > FLOOR_NACKS) {
            TxEvent = 14;
            TxPower += POWER_STEP*nacks;
            if (TxPower > MAX_POWER) TxPower = MAX_POWER;
            nacks = 0;
         }
         break;
	    
      case TX_POWER_PROBE:
         TxEvent = 15;
         if (nacks > NACK_THRESHHOLD) {
            TxEvent = 16;
            TxPower += POWER_STEP;
            if (TxPower > MAX_POWER) TxPower = MAX_POWER;
            nacks = 0;
            TxState = TX_POWER_FLOOR;
         }
         break;
      }

   }
}
