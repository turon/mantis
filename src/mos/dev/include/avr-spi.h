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
 * Desc: Seperating out AVR specific code from spi.h 
 * Date: 5/24/2007
 */

#ifndef __AVR_SPI_H__
#define __AVR_SPI_H__



#define CC2420_CHIP_SELECT_PORT B
#define CC2420_CHIP_SELECT_PIN 0
#define CC2420_CHIP_SELECT CC2420_CHIP_SELECT_PORT, CC2420_CHIP_SELECT_PIN

#define SPI_CLOCK_PORT B
#define SPI_CLOCK_PIN 1
#define SPI_CLOCK SPI_CLOCK_PORT,	SPI_CLOCK_PIN

#define SPI_MOSI_PORT B
#define SPI_MOSI_PIN 2
#define SPI_MOSI SPI_MOSI_PORT, SPI_MOSI_PIN

#define SPI_MISO_PORT B
#define SPI_MISO_PIN 3
#define SPI_MISO SPI_MISO_PORT, SPI_MISO_PIN

/* extern int spi_int_state; */


#define SPI_DISABLE_RX_INT() SPCR &= ~(1 << SPIE)
#define SPI_ENABLE_RX_INT()  spi_int_state = SPI_INT_RECV; SPCR |= (1 << SPIE)
#define SPI_DISABLE_TX_INT() SPCR &= ~(1 << SPIE)
#define SPI_ENABLE_TX_INT()  spi_int_state = SPI_INT_SEND; SPCR |= (1 << SPIE)

#define SPI_DISABLE_INT() SPI_DISABLE_RX_INT()

#define SPI_INT_SEND 0
#define SPI_INT_RECV 1

// other
#define SPI_WAIT() { while (!(SPSR & (1 << SPIF))); }
#define SPI_WAIT_EOTX() SPI_WAIT()
#define SPI_WAIT_EORX() SPI_WAIT()

#define SPI_TX_BUF SPDR
#define SPI_RX_BUF SPDR


#define PLAT_SPI_INIT() do {			\
      init_spi_mutex_stuff();                   \
      MASK_2(SPCR, SPE, MSTR);			\
      MASK_1(SPSR, SPI2X);			\
   } while(0)


#endif
