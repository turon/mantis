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
   File:   cc1000_raw.c
   Authors: Brian Shucker
   Date:   03-3-04
  
**/

/*TODO: This driver uses the SPI bus.  It is NOT compatible with the spi.c that's already
  in the AVR kernel--they can't both be active at the same time.*/

/*TODO: we're still using cc1000.c that's in the share directory...that's bad!  There should
  be a cc1000 low-level file that's bundled with this driver*/

#include "mos.h"
#include "com.h"

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#ifdef CC1000_RAW

#include "msched.h"
#include "sem.h"
#include "cc1000.h"
#include "cc1000_raw.h"
#include "crc.h"

/** @brief Radio is on. */
#define MODE_ON 0 
/** @brief Radio is off.*/ 
#define MODE_OFF 1 

extern comBuf *cc1000_sendBuf;
extern comBuf *cc1000_recvBuf;
extern uint8_t cc1000_com_mode;

extern void (*cc1000_state)(void);

/*com layer required functions*/

/** @brief Sends out a com buffer using this driver (blocking).
 * @param buf Buffer
 * @return Always returns 0
 */
uint8_t com_send_IFACE_RADIO (comBuf* buf)
{
   mos_mutex_lock (&if_send_mutexes[IFACE_RADIO]);

   //if size is zero, we're done
   if(buf->size == 0)
     return 0;  //TODO: we could indicate an error here...

   cc1000_start_transmit (buf);

   //thread will get resumed when send is complete
   //state machine will put us back in recv mode

   if(cc1000_com_mode == MODE_OFF) { //turn back off
      SPCR &= ~0x80;
   }

   mos_mutex_unlock (&if_send_mutexes[IFACE_RADIO]);
   
   return 0;
}

void cc1000_raw_init()
{
   /*
   uint8_t int_handle;
   //register with the com layer
   //com_register_iface(IFACE_RADIO, cc1000_raw_send,
   //	      cc1000_cmode, cc1000_ioctl);
   
   //get initial receive buffer
   cc1000_recvBuf = com_swap_bufs(IFACE_RADIO, NULL);

   //init radio
   cc1000_init(FREQ);
  
   //now init the SPI bus
   int_handle = mos_disable_ints();
   DDRB &= ~( (1<<DDB0) | (1<<DDB1) );// clock pin to input
   SPCR &= ~0x0C;   // Set proper polarity and phase
   DDRB |= (1<<DDB3); //miso to output
   DDRB &= ~( (1<<DDB0) | (1<<DDB2) ); //mosi to input
   cc1000_mode(CC1000_MODE_RX); //go to recv mode

   //enable the spi device
   SPCR |= 0x40;
  
   cc1000_state = cc1000_state_recv_idle;
   //mode is off by default
   cc1000_com_mode = MODE_OFF;
   SPCR &= ~0x80; //disable SPI interrupt
   mos_enable_ints(int_handle);
   */

   cc1000_default_init();
}


/** @brief Interrupt handler is called when a byte is received or sent.
 *
 * Basically implements a state machine.
 */
SIGNAL(SIG_SPI)
{  
   cc1000_state();
}

#endif
#endif
