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

/**
   Project Mantis
   File:   cc1000_tdma.c
   Authors: Brian Shucker
   Date:   03-18-04
  
   This is a derivative of cc1000_raw.
**/


/*TODO: This driver uses the SPI bus.  It is NOT compatible with the spi.c that's already
  in the AVR kernel--they can't both be active at the same time.*/

/*TODO: we're still using cc1000.c that's in the share directory...that's bad!  There should
  be a cc1000 low-level file that's bundled with this driver*/

#include "mos.h"
#include "com.h"

#ifdef CC1000_TDMA
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include "msched.h"
#include "sem.h"
#include "cc1000.h"
#include "cc1000_tdma.h"
#include "crc.h"
#include "led.h"
#include "clock.h"  //provides us usleep

/** @brief seeing frame number */
#define STATE_RECV_FRAMENO 20


/** @brief Sending frame number */
#define STATE_SEND_FRAMENO 21 

//needed to edit the state machine
extern uint8_t spi_data;
extern uint8_t prev_data;
extern uint8_t offset; //recv bit offset
extern uint8_t cc1000_com_mode;
extern uint8_t cc1000_state;
extern comBuf *cc1000_sendBuf;
extern uint8_t preamble_count; //preamble bytes received
//tdma-related
#define FRAME_UNKNOWN 255 //id for an invalid frame

#ifdef PLATFORM_MICA2 //clock at 7.3728MHz
#define FRAME_COUNTS 5760 //frame length in clocks/64 (set to 50ms)
#define FRAME_SYNC_OFFSET 900 //how far into frame 0 the sync strobe is located
#endif

#ifdef PLATFORM_NYMPH //clock at 4.0000MHz
#define FRAME_COUNTS 3125 //frame length in clocks/64 (set to 50ms)
#define FRAME_SYNC_OFFSET 488 //how far into frame 0 the sync strobe is located
#endif

#define FRAME_SYNC_OFFSET_US 7813

extern mos_sem_t cc1000_sem;

//tdma-related
static uint8_t myFrame; //my frame number
static uint8_t curFrame; //current frame number
static uint8_t waiting; //true if waiting to send
static uint8_t strobing; //true if sending a strobe packet
static uint8_t tmp;

//alarm for timing
#define FRAME_SIZE_US 50000
static mos_alarm_t frame_alarm;
static void frame_alarm_func(void *p);
inline void tdma_state_machine();

static mos_sem_t cc1000_sem;
static uint8_t preamble_count;
static uint8_t prev_data;
static uint8_t offset;
static uint8_t spi_data;
static uint16_t sync_error_count;

/*com layer required functions*/

/** Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Always returns 0
*/
uint8_t com_send_IFACE_RADIO (comBuf* buf)
{
   uint8_t int_handle;

   mos_mutex_lock (&if_send_mutexes[IFACE_RADIO]);
   
   //if size is zero, we're done
   if(buf->size == 0)
      return 0;  //TODO: we could indicate an error here...

   cc1000_init_ec(buf);//initialize error correction/checking
   
   waiting = TRUE;//we have to wait until our frame

   cc1000_mode(CC1000_MODE_RX);//we need to see strobe before sending
   SPCR |= (1 << SPIE);//enable the spi device

   //suspend thread until it is our turn (slot)
   mos_sem_wait (&cc1000_sem);
   
   cc1000_mode(CC1000_MODE_TX);//go to transmit mode to send the packet
   int_handle = mos_disable_ints();
      
   cc1000_state = state_send_pre; //init send state machine
   SPDR = PREAMBLE_BYTE; //start sending first preamble byte
   preamble_count = 1;

   //INTERRUPT DRIVEN I/O
//   SPCR |= (1 << SPIE); //turn on spi interrupt 
//   mos_enable_ints (int_handle);
//   mos_sem_wait (&cc1000_sem);

   //POLLING METHOD:
   SPCR &= ~(1 << SPIE); //turn off spi interrupt
   while (cc1000_state != STATE_RECV_IDLE) {
      while (!(SPSR & (1 << SPIF)))
	 ;
      tdma_state_machine ();
   }

   mos_enable_ints (int_handle);
   
   if(cc1000_com_mode == IF_LISTEN) { //turn back off if necessary
      cc1000_mode(CC1000_MODE_RX);
      SPCR |= (1 << SPIE);//enable the spi device
   } else if (cc1000_com_mode == IF_OFF) {
      cc1000_mode(CC1000_MODE_PD);
   }

   mos_mutex_unlock (&if_send_mutexes[IFACE_RADIO]);
 
   return 0;
}

void cc1000_tdma_init(uint8_t id)
{
   //set frame number
   //TODO: should actually get this from base station, not count on app to have it
   myFrame = id;
   curFrame = FRAME_UNKNOWN;
   waiting = FALSE;
   strobing = FALSE;

   //register with the com layer
   //com_register_iface(IFACE_RADIO, cc1000_tdma_send,
   //	      cc1000_cmode, cc1000_ioctl);
   
   cc1000_default_init();
   mos_sem_init (&cc1000_sem, 0);
   
   //set up the frame clock
   frame_alarm.func = frame_alarm_func;
   if(id == 0) { //we are the base station, start the clock
      curFrame = MAX_FRAMES - 1; //define our frame to be next
      mos_alarm(&frame_alarm, 0, FRAME_SIZE_US);
      //base station has to send the strobe even if it's mode is off,
      //because other nodes might be on and need to sync.
   }
}

static void frame_alarm_func(void *p)
{
   boolean reset_alarm = true;

   curFrame++;//advance to next frame number
   if(curFrame == MAX_FRAMES) {
      if(myFrame == 0) //I am the base station, just keep going
	 curFrame = 0;
      else {
	 curFrame = FRAME_UNKNOWN; //past the maximum, wait for strobe
	 reset_alarm = false;
      }
   }

   //if I'm the base station and not sending in my frame, send a strobe
   if(curFrame == 0 && myFrame == 0 && !waiting) {
      mos_led_toggle (0);
      strobing = TRUE;
   }

   if(curFrame == myFrame) {//could be our turn to do something
      if(waiting) {
	 waiting = FALSE;//reset waiting flag
	 mos_sem_post (&cc1000_sem);
      } else if(strobing){
	 SPCR &= ~(1 << SPIE);//disable the spi device
	 cc1000_mode(CC1000_MODE_TX);
	 cc1000_state = state_send_pre; //init send state machine
	 preamble_count = 1;
	 //need to send out strobe, start the send here
	 SPDR = PREAMBLE_BYTE; //start sending first preamble byte
	 SPCR |= (1 << SPIE);//enable the spi device
      }
   }
   if(reset_alarm)
      mos_alarm(&frame_alarm, 0, FRAME_SIZE_US);
}

inline void tdma_state_machine()
{
   switch(cc1000_state)
   {
//SEND STATES
   case STATE_SEND_SYNC:
      SPDR = 0x33; //send sync byte
      cc1000_state = STATE_SEND_FRAMENO;
      break;
   case STATE_SEND_FRAMENO:
      SPDR = myFrame; //send id byte
      cc1000_state = STATE_SEND_SIZE;
      break;
   case STATE_SEND_SIZE:
      if(strobing){ //if this is a strobe, we can skip the actual packet
	 SPDR = 0; //send zero size, to indicate strobe
	 cc1000_state = STATE_SEND_FLUSH;
      } else {
	 cc1000_state_machine();
      }
      break;
   case STATE_SEND_DONE:
      //go back to recv mode
      cc1000_state = STATE_RECV_IDLE;
      //resume sending thread
      if(strobing){
	 cc1000_mode(CC1000_MODE_RX); //put back in receive mode
	 strobing = FALSE; //clear strobing flag
      }
      //else
      //mos_thread_resume(sendThread);
      break;
//RECV STATES
      
   case STATE_RECV_FRAMENO:
      spi_data = SPDR;
      //look for frame 0 (base station) to sync up
      tmp = (prev_data << offset) | (spi_data >> (8-offset));
      if (tmp == 0){
	 curFrame = 0; //we're in the middle of frame 0
	 mos_remove_alarm(&frame_alarm);//remove pending alarms
	 mos_alarm(&frame_alarm, 0, FRAME_SYNC_OFFSET_US);
      }
      cc1000_state = STATE_RECV_SIZE;
      prev_data = spi_data;
      break;
   case STATE_RECV_SYNC:
      spi_data = SPDR;
      // Figure out the bit offset by shifting until we find sync byte
      while(prev_data != 0x33){
	 prev_data = (prev_data << 1) | (spi_data >> (7-offset));
	 offset++;
	 if(offset >= 8){ //didn't get sync byte... Something is wrong
	    sync_error_count++; //log the error
	    cc1000_state = STATE_RECV_IDLE;
	    return;
	 }
      }
      cc1000_state = STATE_RECV_FRAMENO;//synced and ready to get the packet.
      prev_data = spi_data;
      break;

   default:
      cc1000_state_machine();
   }
}


/** @brief Interrupt handler is called when a byte is received or sent.
 */
SIGNAL(SIG_SPI)
{
   tdma_state_machine();
}

#endif
#endif
