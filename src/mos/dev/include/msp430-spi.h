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

/* File: avr-spi.h
 * Author: john ledbetter (john.ledbetter@colorado.edu)
 * Desc: Seperating out msp430/telosb specific code from spi.h 
 * Date: 5/24/2007
 */

#ifndef __MSP430_SPI_H__
#define __MSP430_SPI_H__
    
#define CC2420_CHIP_SELECT_PORT 4
#define CC2420_CHIP_SELECT_PIN 2
#define CC2420_CHIP_SELECT CC2420_CHIP_SELECT_PORT, CC2420_CHIP_SELECT_PIN

#define SPI_CLOCK_PORT 3
#define SPI_CLOCK_PIN 3
#define SPI_CLOCK SPI_CLOCK_PORT,	SPI_CLOCK_PIN

#define SPI_MOSI_PORT 3
#define SPI_MOSI_PIN 1
#define SPI_MOSI SPI_MOSI_PORT, SPI_MOSI_PIN

#define SPI_MISO_PORT 3
#define SPI_MISO_PIN 2
#define SPI_MISO SPI_MISO_PORT, SPI_MISO_PIN

#define SPI_WAIT_EOTX() while(!(U0TCTL & TXEPT))
#define SPI_WAIT_EORX() while(!(IFG1 & URXIFG0))

#define SPI_TX_BUF U0TXBUF
#define SPI_RX_BUF U0RXBUF

#define SPI_DISABLE_RX_INT() IE1 &= ~(URXIE0)
#define SPI_ENABLE_RX_INT()  IE1 |= (URXIE0)
#define SPI_DISABLE_TX_INT() IE1 &= ~(UTXIE0)
#define SPI_ENABLE_TX_INT()  IE1 |= (UTXIE0)



// initialize UART to SPI Mode, setup Pins
#define PLAT_SPI_INIT() do {				\
      U0CTL = SWRST;					\
     /* 8 bits, SPI, Master, Reset State */		\
      U0CTL |= CHAR | SYNC | MM;			\
      /* Delay Tx Half Cycle, SMCLK, 3 Pin Mode */	\
      U0TCTL = CKPH | SSEL_3 | STC | TXEPT;		\
      /* Baud Rate & Modulation Control Not Used */	\
      U0BR0  = 0x02;					\
      U0BR1  = 0x00;					\
      U0MCTL = 0x00;					\
      /* set peripheral mode for MOSI MISO UCLK */	\
      MASK_3(P3SEL, 1, 2, 3);				\
      /*UNMASK_1(P4SEL, CC2420_SFD_PIN);*/		\
      /* make sure Uart0 rx interrupt is off */		\
      IE1 &= ~(URXIE0 | UTXIE0);	                \
      /* enable UART0 SPI, send, recv */		\
      ME1 |= USPIE0 /*| UTXE0 | URXE0*/;		\
      init_spi_mutex_stuff();                           \
      /* Disable Reset State */				\
      U0CTL &= ~SWRST;					\
} while(0)


#endif
