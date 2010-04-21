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
  File:   cc1000_csma_ack.c
  Authors: Cyrus Hall <hallcp@gmail.com>
  Based on code from: Charles Gruenwald III & Jeff Rose
  Date:   09-22-04

  Version of MANTIS CSMA with ACK.
*/

#include "mos.h"
#include "com.h"

#ifdef CC1000_CSMA_ACK
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include <stdlib.h>

#include "avr-adc.h" //proveds AVR_ADC_CH_0
#include "cc1000.h"
#include "cc1000_csma.h"
#include "msched.h" //provides Thread definition
#include "mutex.h"
#include "dev.h"    //provides access to rssi
#include "crc.h"
#include "sem.h"
#include "clock.h"  //provides us usleep
#include "led.h"


#define STATE_SEND_ACK_NUM_H 30
#define STATE_SEND_ACK_NUM_L 31

#define STATE_RECV_ACK_NUM_H 32
#define STATE_RECV_ACK_NUM_L 33

//cc1000 basic externs
extern uint8_t spi_data;
extern uint8_t preamble_count;
extern uint8_t prev_data;
extern uint8_t offset;
extern uint16_t sync_error_count;

extern uint8_t cc1000_com_mode;
extern uint8_t cc1000_state;
//extern mutex   cc1000_lock;

//csma carrier sensing variables
#define CCA_FLOOR_INIT  250
#define CCA_NUM_SAMPLES_INIT 6
static uint16_t cca_noise_floor = CCA_FLOOR_INIT;//init noise floor to sane val
static uint16_t cca_num_samples = CCA_NUM_SAMPLES_INIT;
static uint8_t initBackoff = 7; //initial backoff max period in ms
static uint8_t congestBackoff = 10; //congestion backoff max period 

static uint8_t int_handle;
static sem timer_sem;
static mos_alarm_t backoff_alarm;  //holds alarm for backoff_alarm_func
static uint32_t rand_val;
static uint16_t rssi_value;

//ACK specific stuff
static uint16_t ack_num, sent_ack_num;
static boolean wait_ack, recv_ack;

uint8_t adc_channel; //hopefully this grabs it from the dev layer...

inline static void cc1000_csma_ack_state_machine();

/* check to see if the air is clear for transmission */
static boolean is_clear()
{
   uint16_t i;

   //if we're idle, use rssi to assess channel
   for (i=0; i < cca_num_samples; i++)
   {
      //if we're sending or receiving a packet, the channel isn't clear
      if (cc1000_state != STATE_RECV_IDLE)
	 return false;

      //rssi_value = avr_adc_read_channel16(AVR_ADC_CH_0);
      rssi_value = rssi_poll();
      if (rssi_value < cca_noise_floor)
	 return false;
   }
   
   return true;
}

//this function called when backoff timer expires
//THIS IS IN AN INTERRUPT CONTEXT
//just wake up the semaphore (waiting in cc1000_csma_send)
static void backoff_alarm_func(void *user_data)
{
    mos_sem_post (&timer_sem);
    //mos_thread_resume ((Thread *)user_data);
}

/** @brief Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Always returns 0
 */
uint8_t com_send_IFACE_RADIO (comBuf* buf)
{
    uint8_t old_mode = cc1000_com_mode;
    uint8_t ret = 0;

    mos_mutex_lock (&ifSendMutex[IFACE_RADIO]);
    
    //if size is zero, we're done
    if(buf->size == 0)
	return 0;  //TODO: we could indicate an error here...

    //mos_mutex_lock(&cc1000_lock);
    com_mode (IFACE_RADIO, IF_LISTEN);

    cc1000_rssi_on (); //turn on rssi

    //start counting initial backoff period
    //send state machine will get kicked off when backoff is
    //complete and channel is clear
    rand_val = (random () % initBackoff) + 1;
    backoff_alarm.data = mos_thread_current ();
    mos_alarm (&backoff_alarm, 0, 1000 * rand_val);
   
    //suspend thread until send completes
    mos_sem_wait (&timer_sem);
    //mos_thread_suspend ();
   
    while (!is_clear ()) {
	rand_val = (random () % congestBackoff) + 1;
	mos_alarm (&backoff_alarm, 0, 1000 * rand_val);
	mos_sem_wait (&timer_sem);
	//mos_thread_suspend ();
    }

    //can we do this here?  change from default CSMA.  was below tx
    //finishing.
    cc1000_rssi_off ();  //turn of cc1000_rssi
    
    //do actuall transmit
    cc1000_init_ec(buf);
    cc1000_mode(CC1000_MODE_TX); //go to transmit mode

    int_handle = mos_disable_ints();
    cc1000_state = STATE_SEND_PRE;       //init send state machine
    preamble_count = 1;
    SPDR = PREAMBLE_BYTE;        //start sending first preamble byte
   
    //we use polling only for now - simplifys state machine
    SPCR &= ~(1 << SPIE);        //turn off spi interrupt
    while(cc1000_state != STATE_RECV_IDLE) {
	while(!(SPSR & (1 << SPIF)));   //wait for clear

	cc1000_csma_ack_state_machine();
    }
    mos_enable_ints(int_handle);

    if(cc1000_com_mode == IF_LISTEN) {
	SPCR |= (1 << SPIE);
	cc1000_mode(CC1000_MODE_RX);
    }
    //end transmit

    //now we check for the ACK
    if(recv_ack == TRUE) {
	//we got the ACK
	ret = 1;
	recv_ack = FALSE;
    } //else we got screwed!
   
    //if state machine turned us on, turn back off
    com_mode(IFACE_RADIO, old_mode);

    mos_mutex_unlock(&ifSendMutex[IFACE_RADIO]);
    
    return ret;
}

/** @brief Init function. */
void cc1000_csma_ack_init()
{
    //register with the com layer
    //com_register_iface(IFACE_RADIO, cc1000_csma_send,
    //	       cc1000_cmode, cc1000_ioctl);

    //init radio
    cc1000_default_init();

    mos_sem_init (&timer_sem, 0);
    //mos_mutex_init(&cc1000_lock);
    //set the backoff function
    backoff_alarm.func = backoff_alarm_func;

    recv_ack = wait_ack = FALSE;
    sent_ack_num = ack_num = 0;
}


inline void cc1000_csma_ack_state_machine() {
   switch(cc1000_state) {

	//send states
    case STATE_SEND_SYNC:
	if(ack_num) {
	    SPDR = 0xAF;              //send ACK sync byte
	    cc1000_state = STATE_SEND_ACK_NUM_H;
	} else {
	    cc1000_state_machine();
	}
	break;
    case STATE_SEND_ACK_NUM_H:
	SPDR = ack_num >> 0x4;
	cc1000_state = STATE_SEND_ACK_NUM_L;
	break;
    case STATE_SEND_ACK_NUM_L:
	SPDR = ack_num & 0xFF;
	cc1000_state = STATE_SEND_FLUSH;
	break;

    case STATE_SEND_DONE:
	//if we are really recv'ing and have only just sent an ACK
	if(ack_num) {
	    //go back to recv mode
	    sent_ack_num = ack_num;
	    ack_num = 0;
	    cc1000_state = STATE_RECV_IDLE;
	}
	else {
	    //we have sent and now need to listen for the ACK
	    wait_ack = TRUE;
	}
	break;

    case STATE_RECV_SYNC:
	if(wait_ack) {
	    spi_data = SPDR;
	    //Figure out the bit offset by shifting until we find the sync byte
	    while(prev_data != 0xAF) {
		prev_data = (prev_data << 1) | (spi_data >> (7 - offset));
		offset++;
		if(offset >= 8)	{
		    //didn't get the sync byte... Something is wrong
		    sync_error_count++; //log the error
		    cc1000_state = STATE_RECV_IDLE;
		    return;
		}
	    }
	    cc1000_state = STATE_RECV_ACK_NUM_H;
	} else {
	    cc1000_state_machine();
	}
	
	break;
	
    case STATE_RECV_DATA_FEC:
    case STATE_RECV_CRC_L:
	//this is the last RECV state - we need to now recv/bounce the ACK
	cc1000_state_machine();

	//overload cc1000_state_machine
	if(cc1000_state == STATE_RECV_IDLE) {
	    cc1000_state = STATE_RECV_ACK_NUM_H;
	}
	break;

    case STATE_RECV_ACK_NUM_H:
	spi_data = SPDR;
	ack_num |= spi_data << 4;
	cc1000_state = STATE_RECV_ACK_NUM_L;
	break;
	
    case STATE_RECV_ACK_NUM_L:
	spi_data = SPDR;
	ack_num |= spi_data & 0xFF;

	if(!wait_ack) {
	    //send ACK
	    SPCR &= ~(1 << SPIE);//disable the spi device
	    cc1000_mode(CC1000_MODE_TX);
	    cc1000_state = STATE_SEND_PRE; //init send state machine
	    preamble_count = 1;
	    //need to send out strobe, start the send here
	    SPDR = PREAMBLE_BYTE; //start sending first preamble byte
	    SPCR |= (1 << SPIE);//enable the spi device

	    return;
	} else {
	    if(sent_ack_num == ack_num) {
		recv_ack = TRUE;
	    }
	}

	cc1000_state = STATE_RECV_IDLE;
	break;
	
    default:
	cc1000_state_machine();
    }
}


/** @brief Interrupt handler is called when a byte is received or sent.
 *
 * Basically implements a state machine.
 */
SIGNAL(SIG_SPI)
{
    cc1000_csma_ack_state_machine();
}
    
#endif
#endif
