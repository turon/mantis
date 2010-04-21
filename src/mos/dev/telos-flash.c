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
 * @file telos-flash.c
 * @brief driver for the ST M25P80 flash
 * @author John Ledbetter
 * @date 07/01/2005
 */
#include "mos.h"


#ifdef PLATFORM_TELOSB

#include "telos-flash.h"
#include "bitops.h"
#include "spi.h"
#include "mutex.h"
#include "dev.h"
#include "led.h"
// Command Bytes fo the M25P80
#define WREN  0x06
#define WRDI  0x04
#define RDSR  0x05
#define WRSR  0x01
#define READ  0x03
#define FREAD 0x0B
#define PP    0x02
#define SE    0xD8
#define BE    0xC7
#define DP    0xB9
#define RES   0xAB
#define DUMMY 0xAA

// Status Register Masks for the M25P80
#define WIP  0x01
#define WEL  0x02
#define BP0  0x04
#define BP1  0x08
#define BP2  0x10
#define SRWD 0x80

mos_mutex_t telos_flash_mutex;
static uint32_t st_flash_addr;
static uint8_t st_flash_mode;

#ifndef RAMFUNC
#define RAMFUNC __attribute__ ((section(".data")))
#endif

/** @brief block until write in progress is no longer set in the status register */
static inline void st_flash_block_wip()
{
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(RDSR);
   while(spi_rx() & WIP)
      ;

   SET_PORT_1(ST_FLASH_CS);
}

/** @brief enable writing to the st flash */
static inline void st_flash_write_enable()
{
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(WREN);

   SET_PORT_1(ST_FLASH_CS);
}

/** @brief disable writing to the st flash */
static inline void st_flash_write_disable()
{
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(WRDI);

   SET_PORT_1(ST_FLASH_CS);
}


/** @brief send a 24bit address over the SPI */
static inline void st_flash_tx_addr(uint32_t addr)
{
   // gah msp430 is little endian;
   ////spi_send((uint8_t*)&addr, 3);
   spi_tx_byte((addr >> 16) & 0xFF);
   spi_tx_byte((addr >>  8) & 0xFF);
   spi_tx_byte((addr >>  0) & 0xFF);
   
   
}

/** @brief read the st flash status register */
static inline uint8_t st_flash_read_sr()
{
   uint8_t sr;
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(RDSR);
   sr = spi_rx();

   SET_PORT_1(ST_FLASH_CS);

   return sr;
}

/** @brief write to the st flash status register */
static inline void st_flash_write_sr(uint8_t new_sr)
{
   st_flash_block_wip();
   st_flash_write_enable();
   
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(WRSR);
   spi_tx_byte(new_sr);

   SET_PORT_1(ST_FLASH_CS);
}

/** @brief erase the entire flash */
static inline void st_flash_bulk_erase()
{
   st_flash_block_wip();
   st_flash_write_enable();
   
   UNSET_PORT_1(ST_FLASH_CS);
   
   spi_tx_byte(BE);
   
   SET_PORT_1(ST_FLASH_CS);
}

/** @brief enter low power mode */
static inline void st_flash_deep_powerdown()
{
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(DP);

   SET_PORT_1(ST_FLASH_CS);
}

/** @brief initialize the st flash, enter deep power down mode */
void st_flash_init()
{
   mos_mutex_init(&telos_flash_mutex);
   
   SET_DIR_1(ST_FLASH_HOLD);
   SET_DIR_1(ST_FLASH_CS);
   
   SET_PORT_1(ST_FLASH_CS);
   SET_PORT_1(ST_FLASH_HOLD);
   
   SET_DIR_1(ST_FLASH_WP);
   SET_PORT_1(ST_FLASH_WP);

   SET_DIR_1(ST_FLASH_MOSI);
   SET_DIR_1(ST_FLASH_SCLK);
   UNSET_DIR_1(ST_FLASH_MISO);
   

   // this macro need only be called once,
   // either by the cc2420 init or here.
   PLAT_SPI_INIT();

   // initialize into low power mode
   st_flash_deep_powerdown();
  
   
}

/** @brief exit deep power down mode */
static inline void st_flash_wake()
{
   UNSET_PORT_1(ST_FLASH_CS);
   spi_tx_byte(RES);
   SET_PORT_1(ST_FLASH_CS);
}

/** @brief enter deep power down mode */
static inline void st_flash_sleep()
{
   st_flash_block_wip();
   st_flash_deep_powerdown();
}

/** @brief exit deep power down mode and/or read the signature */
static inline uint8_t st_flash_read_sig()
{
   UNSET_PORT_1(ST_FLASH_CS);
   spi_tx_byte(RES);
   spi_tx_byte(DUMMY);
   spi_tx_byte(DUMMY);
   spi_tx_byte(DUMMY);
   uint8_t sig = spi_rx();

   SET_PORT_1(ST_FLASH_CS);
   
   return sig;
}

/** @brief read a block of data from addr in flash */
void RAMFUNC st_flash_read(uint32_t addr, uint8_t* buffer, uint16_t count)
{   
   st_flash_block_wip();
   
   UNSET_PORT_1(ST_FLASH_CS);
   spi_tx_byte(READ);
   st_flash_tx_addr(addr);

   
 /*   spi_recv(buffer, count); */

   uint16_t i = 0;
   for(i = 0; i < count; ++i)
      buffer[i] = spi_rx();
   
   SET_PORT_1(ST_FLASH_CS);
}

/** @brief read a block of data from addr in flash, only faster! (supposedly) */
/* static inline void st_flash_fast_read(uint32_t addr, uint8_t* buffer, uint16_t count) */
/* { */
/*    st_flash_block_wip(); */
   
/*    UNSET_PORT_1(ST_FLASH_CS); */
/*    spi_tx_byte(FREAD); */
/*    st_flash_tx_addr(addr); */
/*    spi_tx_byte(DUMMY); */
   
/*    uint16_t i = 0; */
/*    for(i = 0; i < count; ++i) */
/*       buffer[i] = spi_rx(); */

/*    SET_PORT_1(ST_FLASH_CS); */
/* } */

/** @brief write to a page in flash at addr */
static void st_flash_page_program(uint32_t addr, const uint8_t* buffer, uint16_t count)
{
      
   st_flash_block_wip();
   st_flash_write_enable();
   
   UNSET_PORT_1(ST_FLASH_CS);

   spi_tx_byte(PP);
   st_flash_tx_addr(addr);
   spi_send(buffer, count);
   
   SET_PORT_1(ST_FLASH_CS);
}

/** @brief erase the sector which contains address addr */
static void RAMFUNC st_flash_erase_sector(uint32_t addr)
{
   st_flash_block_wip();
   st_flash_write_enable();
   
   UNSET_PORT_1(ST_FLASH_CS);
   
   spi_tx_byte(SE);
   st_flash_tx_addr(addr);

   SET_PORT_1(ST_FLASH_CS);
}

/** @brief put the flash into deep powerdown mode or power it up */
uint8_t RAMFUNC dev_mode_DEV_TELOS_FLASH (uint8_t newMode)
{
   switch(newMode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      st_flash_wake();
      break;

   case DEV_MODE_OFF:
      st_flash_sleep();
      break;

   default:
      return DEV_UNSUPPORTED;
      break;
   }

   st_flash_mode = newMode;
   return DEV_OK;
}

/** @brief seek or erase */
uint8_t RAMFUNC dev_ioctl_DEV_TELOS_FLASH (int8_t request, ...)
{
   va_list ap;
   
   va_start (ap, request);
   
   switch (request)
   {
   case DEV_SEEK:
      st_flash_addr = va_arg (ap, uint32_t);
      break;
   case TELOS_FLASH_BULK_ERASE:
      if(st_flash_mode == DEV_MODE_OFF)
	 st_flash_wake();
      st_flash_bulk_erase();
      if(st_flash_mode == DEV_MODE_OFF)
	 st_flash_sleep();
      break;
   case TELOS_FLASH_SECT_ERASE:
      if(st_flash_mode == DEV_MODE_OFF)
	 st_flash_wake();
      st_flash_addr = va_arg (ap, uint32_t);
      st_flash_erase_sector(st_flash_addr);
      if(st_flash_mode == DEV_MODE_OFF)
	 st_flash_sleep();
      break;
   default:
      return DEV_BAD_IOCTL;
   }

   va_end(ap);

   return DEV_OK;
   
}

/** @brief read data from flash */
uint8_t RAMFUNC dev_read_DEV_TELOS_FLASH (void *buf, uint16_t count)
{
   if(st_flash_mode == DEV_MODE_OFF)
      st_flash_wake();

   st_flash_read(st_flash_addr, (uint8_t*)buf, count);

   if(st_flash_mode == DEV_MODE_OFF)
      st_flash_sleep();
   
   return count;
}

/** @brief write up to 256 bytes of data to a page in flash */
uint8_t dev_write_DEV_TELOS_FLASH (const void *buf, uint16_t count)
{
   if (count > 256)
      count = 256;

   if(st_flash_mode == DEV_MODE_OFF)
      st_flash_wake();
   
   st_flash_page_program(st_flash_addr, (const uint8_t*)buf, count);

   if(st_flash_mode == DEV_MODE_OFF)
      st_flash_sleep();
   
   return count;
   
}




#endif
