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
 * File: spi.c
 * Author: Charles Gruenwald III
 * Desc: A unified SPI driver for various (telosb, micaz) nodes
 * Date: 05/01/2007
 */

#include "mos.h"
#include "dev.h"
#include "spi.h"
#include "bitops.h"

#include "cc2420-gpio.h"

#if defined(ARCH_AVR) || defined(ARCH_MSP430)
mos_mutex_t spi_mutex;

void spi_init()
{
    SET_DIR_2(SPI_MOSI, SPI_CLOCK);
    SET_PORT_2(SPI_MOSI, SPI_CLOCK);


#if defined(PLATFORM_SUPPORTS_CC2420)
    SET_DIR_1(CC2420_CHIP_SELECT);
    SET_PORT_1(CC2420_CHIP_SELECT);
#endif
    
    UNSET_DIR_1(SPI_MISO);

    PLAT_SPI_INIT();
}

uint8_t dev_mode_DEV_SPI(uint8_t new_mode)
{
    return DEV_UNSUPPORTED;
}

uint16_t dev_read_DEV_SPI(void *buf, uint16_t count)
{
    uint16_t i;
    for(i = 0; i < count; i++)
    {
        ((uint8_t *)buf)[i] = spi_rx();
    }
    return count;

    //rx word...
    /*
       SPI_TX_BUF = 0;
       SPI_WAIT_EOTX();
       SPI_WAIT_EORX();
       ret = SPI_RX_BUF << 8;
       SPI_TX_BUF = 0;
       SPI_WAIT_EOTX();
       SPI_WAIT_EORX();

       ret |= SPI_RX_BUF;
       */
}

uint16_t dev_write_DEV_SPI(const void *buf, uint16_t count)
{
    //reverse the bytes going out?
    //spi_tx_byte(((uint8_t *)buf)[count - i]);
    uint16_t i;
    if(count == 2)
    {
        uint16_t val = *(uint16_t *)buf;
        spi_tx_byte((uint8_t)(val>>8));
        spi_tx_byte((uint8_t)val);
    }
    else
    {
        for(i = 0; i < count; i++)
            spi_tx_byte(((uint8_t *)buf)[i]);
    }
    return count;
}

uint8_t dev_ioctl_DEV_SPI(int8_t request, ...)
{
   va_list ap;
   int arg;
   va_start (ap, request);
   
   switch (request)
   {
       case SPI_SET_SLAVE_SELECT:
       {
           arg = va_arg (ap, int);
           switch(arg)
           {
#if defined(PLATFORM_SUPPORTS_CC2420)
               case CC2420_SLAVE_SELECT:
                   ASSERT_CS(CC2420_CHIP_SELECT);
                   break;
#endif
               default:
                   va_end(ap);
                   return DEV_BAD_IOCTL;
           };
       }
       break;
       case SPI_CLEAR_SLAVE_SELECT:
       {
           DEASSERT_CS(CC2420_CHIP_SELECT);
       }
       break;
       default:
       va_end(ap);
       return DEV_BAD_IOCTL;
   }

   va_end(ap);
   return DEV_OK;
}


#ifdef CC2420_INTERRUPT_DRIVEN
static sem spi_sem;
typedef void (*spi_state_f) (void);
static Thread *spi_thread;
static uint8_t current_state;
static spi_state_f spi_state;
void spi_state_init(void);
void spi_state_idle(void);
#define SPI_STATE_INIT 0
#endif

#ifdef CC2420_INTERRUPT_DRIVEN
static uint8_t spi_init[] ARCH_PROGMEM = {
   CC2420_SXOSCON,
   CC2420_MDMCTRL0, 0x02E2 >> 8, 0x02E2 & 0xff,
   CC2420_MDMCTRL1, 0x0500 >> 8, 0x0500 & 0xff,
   CC2420_IOCFG0,   0x007F >> 8, 0x007F & 0xff,
   CC2420_SECCTRL0, 0x01C4 >> 8, 0x01C4 & 0xff,
   0
};
static uint8_t spi_i;
static uint8_t spi_data;

void spi_state_init(void)
{
   switch(current_state) {
   case SPI_STATE_INIT:
      if((spi_data = pgm_read_byte(&spi_init[spi_i++])) != 0) {
          dev_write(DEV_SPI, &spi_data, sizeof(spi_data));
//	 /*SPDR*/ SPI_TX_BUF = spi_data;
      } else {
     spi_state = spi_state_idle;
     mos_thread_resume(spi_thread);
      }
      break;
   }
}

void spi_state_idle(void)
{
   
}

SIGNAL (SIG_SPI)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   
   spi_state();
}
#endif


#endif
