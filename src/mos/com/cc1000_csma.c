//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

/*
  Project Mantis
  File:   cc1000_csma.c
  Authors: Charles Gruenwald III & Jeff Rose
  Date:   05-13-04
  
*/

#include "mos.h"
#include "com.h"

#ifdef CC1000_CSMA

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include <stdlib.h>
#include "adc.h"
#include "avr-adc.h" //proveds AVR_ADC_CH_0
#include "cc1000.h"
#include "cc1000_csma.h"
#include "msched.h" //provides Thread definition
#include "mutex.h"
#include "dev.h"    //provides access to rssi
#include "crc.h"
#include "sem.h"
#include "clock.h"  //provides us usleep
#include "avr-rssi.h"
extern uint8_t cc1000_com_mode;
extern void (*cc1000_state)(void);
void state_recv_idle(void);

static mos_sem_t timer_sem;

//carrier sensing
#define CCA_FLOOR  300
#define CCA_NUM_SAMPLES 6
#define INIT_BACKOFF 7 //initial backoff max period in ms
#define CONGEST_BACKOFF 10 //congestion backoff max period 

static mos_alarm_t backoff_alarm;  //holds alarm for backoff_alarm_func
static uint32_t rand_val;
static uint16_t rssi_value;

/* check to see if the air is clear for transmission */
static boolean is_clear(void)
{
   uint8_t i;

   //if we're idle, use rssi to assess channel
   for(i = 0; i < CCA_NUM_SAMPLES; i++) {
      //if we're sending or receiving a packet, the channel isn't clear
      if(cc1000_state != state_recv_idle)
	 return false;

      //rssi_value = adc_read_channel16(AVR_ADC_CH_0);
      rssi_value = rssi_poll();
      if(rssi_value < CCA_FLOOR)
	 return false;
   }
   
   return true;
}

//this function called when backoff timer expires
//THIS IS IN AN INTERRUPT CONTEXT
//just wake up the semaphore (waiting in cc1000_csma_send)
static void backoff_alarm_func(void *user_data)
{
   mos_sem_post(&timer_sem);
}

/** @brief Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Returns 0 on successful transmission
 */
uint8_t com_send_IFACE_RADIO(comBuf* buf)
{
   uint8_t old_mode = cc1000_com_mode;
   uint8_t old_pri = mos_thread_current()->priority;
   uint16_t exponential_backoff = 0;
   
   if(buf->size == 0)
      return -1;

   mos_mutex_lock(&if_send_mutexes[IFACE_RADIO]);

   mos_thread_current()->priority = PRIORITY_HIGH;
   com_mode_cc1000(IF_LISTEN);

   cc1000_rssi_on();
   //mos_udelay(250);

   //start counting initial backoff period
   //send state machine will get kicked off when backoff is
   //complete and channel is clear
   rand_val = (random() % INIT_BACKOFF) + 1;
   backoff_alarm.data = mos_thread_current();
   backoff_alarm.msecs = rand_val;
   backoff_alarm.reset_to = 0;

   mos_alarm(&backoff_alarm);
   
#ifdef RADIO_EXTRA_CRC
   // Compute a CRC that will be checked after error correction has been run
   // FIXME find a better place to store this, perhaps in a new field in the
   //    packet see com.h
   *(uint16_t *)&buf->data[buf->size] = crc_compute(buf->data, buf->size);
   buf->size += 2;
#endif
   


//suspend thread until send completes
   mos_sem_wait(&timer_sem);
  
   //is_clear() read the ADC so lock it until rssi is clear
   dev_open(DEV_ADC);
   exponential_backoff = 12;
   while(!is_clear()) {
      rand_val = (random() % CONGEST_BACKOFF) + 1;
      backoff_alarm.msecs = exponential_backoff;
      mos_alarm(&backoff_alarm);
      exponential_backoff *= 2;
      mos_sem_wait(&timer_sem);
      }



   cc1000_rssi_off();
   dev_close(DEV_ADC);

   cc1000_start_transmit(buf);

   //if state machine turned us on, turn back off
   com_mode_cc1000(old_mode);
   mos_thread_current()->priority = old_pri;
   
   mos_mutex_unlock(&if_send_mutexes[IFACE_RADIO]);
   
   return 0;
}

/** @brief Init function. */
void cc1000_csma_init(void)
{
   cc1000_default_init();

   mos_sem_init(&timer_sem, 0);
   
   //set the backoff function
   backoff_alarm.func = backoff_alarm_func;
}

/** @brief Interrupt handler is called when a byte is received or sent.
 *
 * Basically implements a state machine.
 */
SIGNAL(SIG_SPI)
{
   //  mos_led_toggle(0);
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   
   cc1000_state();
}

#endif
#endif
