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
  File:   cc1000_bmac.c
  Authors: Charles Gruenwald III & Jeff Rose
  Date:   05-13-04
  
*/

#include "mos.h"
#include "com.h"

#ifdef CC1000_BMAC

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include <stdlib.h>
#include "avr-adc.h"

#include "cc1000.h"
#include "cc1000_bmac.h"
#include "msched.h" //provides Thread definition
#include "dev.h"    //provides access to rssi
#include "crc.h"
#include "sem.h"
#include "clock.h"  //provides us usleep

#define MODE_ON 0 
#define MODE_OFF 1 

/** @brief Number of preamble bytes you need to see. */
#define PREAMBLE_THRESH 6
/** @brief Number of preamble bytes you need to send. */ 
#define PREAMBLE_LEN 16

#define FLUSH_BYTE 0xff

extern comBuf *cc1000_sendBuf;
extern comBuf *cc1000_recvBuf;
extern uint8_t cc1000_com_mode;
extern uint8_t cc1000_crcLow;
extern uint8_t cc1000_crcHigh;
extern uint8_t cc1000_state;

static sem timer_sem;
static sem noise_sem;

//carrier sensing
#define CCA_NUM_SAMPLES 6
#define CCA_FLOOR_INIT  250

// Init noise floor to sane value
static uint16_t cca_noise_floor = CCA_FLOOR_INIT;
// Samples for computing the cca estimate
static uint16_t cca_sample_queue[CCA_NUM_SAMPLES];
static uint8_t cca_index = 0;
static uint8_t init_backoff = 5; //initial backoff max period in ms
static uint8_t congest_backoff = 7; //congestion backoff max period
#define NOISE_SAMPLE_PERIOD 1

// hold the alarm for the backoff alarm func
static uint32_t rand_val;
static uint16_t rssi_value;

static mos_alarm_t backoff_alarm;  //holds the alarm for the backoff_alarm_func
static mos_alarm_t noise_alarm;  //holds the alarm for noise_sample_alarm_func

/* check to see if the air is clear for transmission */
static boolean is_clear ()
{
   uint8_t i;
   
   //if we're idle, use bmac algorithm to assess channel
   for (i = 0; i < CCA_NUM_SAMPLES; i++) {
      //if we're sending of receiving a packet, the channel isn't clear
      if (cc1000_state != STATE_RECV_IDLE)
	 return false;
      
      rssi_value = rssi_poll ();
      
      if (rssi_value < cca_noise_floor)
	 return false;
   }
   
   return true;
}

/* store a new sample in our rssi queue */
static void new_sample(uint16_t val)
{
   if(++cca_index == CCA_NUM_SAMPLES)
      cca_index = 0;

   cca_sample_queue[cca_index] = val;
}

/* get the median rssi value from our queue */
static uint16_t compute_median ()
{
   uint8_t i,j;
   uint16_t sort_q[CCA_NUM_SAMPLES];
   uint16_t median_index = 0;
   uint16_t median_value = 0xffff;

   // get a copy of the array 
   for(i = 0; i < CCA_NUM_SAMPLES; i++)
      sort_q[i] = cca_sample_queue[i];

   // loop only 1/2 of the num cycles to get the median 
   for(i = 0; i < (CCA_NUM_SAMPLES >> 1); i++) {
      median_value=0xffff; //init large to get minimum again...
      // loop through entire (copied) array 
      for(j = 0; j < CCA_NUM_SAMPLES; j++) {
	 if(sort_q[j] < median_value)
	    median_index = j; //get the next minimum
      }
      median_value = sort_q[median_index];
      sort_q[median_index] = 0xffff; //mark the minimum index as visited
   }
   
   return median_value;
}

/* take a sample and use it to compute the noise floor */
static void update_noise_floor ()
{
   cc1000_rssi_on ();
   uint16_t rssi_val = avr_adc_read_channel16 (AVR_ADC_CH_0);
   cc1000_rssi_off ();
//   printf ("new adc reading is %d\n", rssi_val);
   
   new_sample (rssi_val);
   rssi_val = compute_median ();

   // the noise floor is computed by taking an average of the new
   // median with the old
   // the 4 shift is multiplied by 16, the 1 is by 2
   // so 18 is the weighted number to divide by
   cca_noise_floor = ((cca_noise_floor << 4) + (rssi_val << 1)) / 18;
}


//this function called when backoff timer expires
//THIS IS IN AN INTERRUPT CONTEXT
//just wake up the semaphore (waiting in cc1000_bmac_send)
static void backoff_alarm_func (void *user_data)
{
   mos_sem_post (&timer_sem);
}

//this function called when noise sample alarm expires
//THIS IS IN AN INTERRUPT CONTEXT!
static void noise_sample_alarm_func (void* user_data)
{
   mos_sem_post (&noise_sem);
}

static void bmac_noise_thread (void)
{
   while (1) {
      //if we aren't sending or receiving, try to update the noise floor
      mos_thread_sleep (1);
      if(cc1000_state == STATE_RECV_IDLE)
	 update_noise_floor();

      //if we haven't been turned off, reset alarm for next time
      if(cc1000_com_mode == MODE_ON) {
	 mos_alarm(&noise_alarm, NOISE_SAMPLE_PERIOD, 0);
	 mos_sem_wait (&noise_sem);
      }
   }
}

/** @brief Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Always returns 0
 */
uint8_t com_send_IFACE_RADIO (comBuf* buf)
{
   uint8_t old_mode = cc1000_com_mode;

   if(buf->size == 0)
      return -1;

   mos_mutex_lock (&ifSendMutex[IFACE_RADIO]);

   //if size is zero, we're done
   com_mode (IFACE_RADIO, IF_LISTEN);

   cc1000_rssi_on ();

   //start counting initial backoff period
   //send state machine will get kicked off when backoff is
   //complete and channel is clear
   rand_val = random () % init_backoff + 1;
   mos_alarm(&backoff_alarm, 0, 1024 * rand_val);

   //suspend thread until send completes
   mos_sem_wait (&timer_sem);

   while (!is_clear ()) {
      rand_val = (random () % congest_backoff) + 1;
      mos_alarm (&backoff_alarm, 0, 1024 * rand_val);
      mos_sem_wait (&timer_sem);
   }
   cc1000_start_transmit (buf);

   cc1000_rssi_off ();
   //if state machine turned us on, turn back off
   com_mode (IFACE_RADIO, old_mode);
   
   mos_mutex_unlock (&ifSendMutex[IFACE_RADIO]);
   
   return 0;
}

void cc1000_bmac_init ()
{
   uint8_t i;
   uint16_t rssi_value;

   mos_sem_init (&noise_sem, 0);
   noise_alarm.func = noise_sample_alarm_func;

   cc1000_default_init ();

   // Initialize the clear channel assesment queue
   rssi_value = avr_adc_read_channel16 (AVR_ADC_CH_0);
   for(i = 0; i < CCA_NUM_SAMPLES; i++)
      cca_sample_queue[i] = rssi_value;


   
   backoff_alarm.func = backoff_alarm_func;
}

/** @brief Interrupt handler is called when a byte is received or sent.
 *
 * Basically implements a state machine.
 */
SIGNAL(SIG_SPI)
{
   cc1000_state_machine();
}

#endif
#endif
