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

#ifndef __SPI_H__
#define __SPI_H__

#ifndef PLATFORM_LINUX
/** @file spi.h
 * @brief SPI Driver for all platforms.
 */


#include "mos.h"
#include "sem.h"
#include "mutex.h"
// debugging
#include "printf.h"
#include "led.h"

#if defined(ARCH_AVR)
#include "avr-spi.h"
#elif defined(PLATFORM_TELOSB)
#include "msp430-spi.h"
#endif




//device interface to the spi
void spi_init();
uint8_t dev_mode_DEV_SPI(uint8_t new_mode);
uint16_t dev_read_DEV_SPI(void *buf, uint16_t count);
uint16_t dev_write_DEV_SPI(const void *buf, uint16_t count);
uint8_t dev_ioctl_DEV_SPI(int8_t request, ...);

enum
{
    SPI_SET_SLAVE_SELECT,
    SPI_CLEAR_SLAVE_SELECT
};

enum
{
#if defined(PLATFORM_SUPPORTS_CC2420)
    CC2420_SLAVE_SELECT,
#endif
#if defined(PLATFORM_MICROBLAZE)
    MB_SPI_ADC_SLAVE_SELECT,
#endif
    NUM_SLAVE_SELECTS
};



/* extern uint8_t* spi_buf; */
/* extern uint16_t spi_buf_cnt; */

/* extern mos_sem_t spi_sem; */
/* extern mos_mutex_t spi_mutex */;
extern mos_mutex_t spi_mutex;


static inline void init_spi_mutex_stuff()
{
/*    mos_sem_init(&spi_sem, 0); */		
//   mos_mutex_init(&spi_mutex);
   mos_mutex_init(&spi_mutex);
}

#if defined(ARCH_AVR) || defined(ARCH_MSP430)



#define ASSERT_CS(PORT_PIN) do{			\
      /*mos_mutex_lock(&spi_mutex);*/		\
      UNSET_PORT_1_(PORT_PIN);			\
   }while(0)

#define DEASSERT_CS(PORT_PIN) do{	  \
      SET_PORT_1_(PORT_PIN);		  \
      /* mos_mutex_unlock(&spi_mutex); */ \
   }while(0)

/** @brief transmit a byte (polling mode) */
static inline void spi_tx_byte(uint8_t byte)
{
   SPI_TX_BUF = byte;
   SPI_WAIT_EOTX();
}

/** @brief receive a byte (polling mode) */
static inline uint8_t spi_rx(void)
{
   SPI_TX_BUF = 0;
   SPI_WAIT_EOTX();
   SPI_WAIT_EORX();
   return SPI_RX_BUF;
}

/** @brief receive and discarde a byte (polling mode) */
//static inline void spi_rx_garbage(void)
//{
//   SPI_TX_BUF = 0;
//   SPI_WAIT_EOTX();
//   SPI_WAIT_EORX();

   // I know it's not used, it's trash. Stop warning me.
   /*__attribute__((unused))*/ //uint8_t trash = SPI_RX_BUF;
//   trash++;
//}

/** @brief receive a word (polling mode) */
static inline uint16_t spi_rx_word(void)
{
   uint16_t ret = 0;
   
   SPI_TX_BUF = 0;
   SPI_WAIT_EOTX();
   SPI_WAIT_EORX();
   ret = SPI_RX_BUF << 8;
   SPI_TX_BUF = 0;
   SPI_WAIT_EOTX();
   SPI_WAIT_EORX();
 
   ret |= SPI_RX_BUF;

/*    spi_recv((uint8_t*)&ret, sizeof(uint16_t)); */
   
   return ret;
}

static inline void spi_send(const uint8_t* buf, uint16_t size)
{
   // phased out for the collective good.
   //mos_mutex_lock(&spi_mutex);
   uint16_t i;
   
   for(i = 0; i < size; ++i)
      spi_tx_byte(buf[i]);
   
/*    spi_buf = buf; */
/*    spi_buf_cnt = size; */
/*    SPI_ENABLE_TX_INT(); */
/*    mos_sem_wait(&spi_sem); */
   
   //mos_mutex_unlock(&spi_mutex);;
}

//static inline void spi_recv(uint8_t* buf, uint16_t size)
//{
   //mos_mutex_lock(&spi_mutex);
   
 /*   spi_buf = buf; */
/*    spi_buf_cnt = size; */
/*    SPI_ENABLE_RX_INT(); */
/*    mos_sem_wait(&spi_sem); */
//   uint16_t i;
   
//   for(i = 0; i < size; ++i)
//      buf[i] = spi_rx_byte();
   //mos_mutex_unlock(&spi_mutex);
//}


#endif // defined(ARCH_AVR) || defined(ARCH_MSP430)
#endif // !defined(PLATFORM_LINUX)
#endif // !defined(__SPI_H__)
