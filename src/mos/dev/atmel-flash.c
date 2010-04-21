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

/** @file atmel-flash.c
 * @brief External 500k flash memory
 */

#include "mos.h"

#ifdef ARCH_AVR

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/signal.h>

#include "clock.h"
#include "mutex.h"
#include "atmel-flash.h"
#include "dev.h"
#include "printf.h"

mos_mutex_t atmel_flash_mutex;
static uint32_t atmel_flash_addr;

static uint8_t cur_buff;
static uint16_t cur_page;
static uint8_t dirty;			// This will never be set in unbuffered mode
static uint8_t flash_mode;

static void atmel_flash_low();
static void atmel_flash_high();
static uint8_t atmel_flash_send_byte(uint8_t spiOut);
static uint8_t atmel_flash_get_byte();

static void atmel_flash_read_memory(uint16_t page,
				     uint16_t offset, 
				     void *reqData, uint16_t len);
static uint16_t atmel_flash_crc_memory(uint16_t page,
					uint16_t offset, 
					uint32_t len);
static uint8_t atmel_flash_busy();
static uint8_t atmel_flash_write_buffer(uint8_t selected, uint16_t offset, 
					 void *reqdata, uint16_t len);
static uint8_t atmel_flash_flush_buffer(uint8_t selected, uint16_t page);
static uint8_t atmel_flash_compare_buffer(uint8_t selected, uint16_t page);
static uint8_t atmel_flash_fill_buffer(uint8_t selected, uint16_t page);
static uint8_t atmel_flash_erase_page(uint16_t page);


uint8_t dev_mode_DEV_ATMEL_FLASH (uint8_t mode)
{
   uint8_t retcode = DEV_OK;
   
   if (flash_mode == mode)
      return retcode;
   
   flash_mode = mode;
   
   switch(mode)
   {
   case ATMEL_FLASH_MODE_UNBUFFERED:
      if (dirty) {
	 while (atmel_flash_busy ())
	    ;
	 atmel_flash_flush_buffer(cur_buff, cur_page);
	 dirty = 0;
	 while (atmel_flash_busy ())
	    ;
      }
      break;
   case ATMEL_FLASH_MODE_BUFFERED:
      cur_buff = ATMEL_FLASH_BUFFER_1;
      // The first comparison in write will always fail
      cur_page = ATMEL_FLASH_MAX_PAGES; 
      dirty = 0;
      break;
   default:
      retcode = DEV_BAD_MODE;
      break;
   }
   
   return retcode;
}

uint8_t dev_ioctl_DEV_ATMEL_FLASH (int8_t request, ...)
{
   uint32_t arg;
   va_list ap;
   
   va_start (ap, request);
   
   switch (request) {
   case DEV_SEEK:
      arg = va_arg (ap, uint32_t);
      atmel_flash_addr = arg;
      break;
   case DEV_FLUSH:
      if (dirty) {
	 while (atmel_flash_busy ())
	    ;
	 atmel_flash_flush_buffer(cur_buff, cur_page);
	 dirty = 0;
	 while (atmel_flash_busy ())
	    ;
      }
      break;
   case DEV_LOCK:
      break;
   case DEV_UNLOCK:
      break;
   default:
      return DEV_BAD_IOCTL;      
   }
   
   va_end (ap);
   
   return DEV_OK;
}

/** @brief Read from the current flash address into p, for count bytes
 */
uint16_t dev_read_DEV_ATMEL_FLASH (void *p, uint16_t count)
{
   uint16_t page, offset;
   uint8_t *buf = (uint8_t *)p;
   
   page = atmel_flash_addr / ATMEL_FLASH_PAGE_SIZE;
   offset = atmel_flash_addr % ATMEL_FLASH_PAGE_SIZE;
   
   if (dirty && page <= cur_page && 
       page + (offset + count) / ATMEL_FLASH_PAGE_SIZE >= cur_page) {
      while (atmel_flash_busy ())
	 ;
      atmel_flash_flush_buffer(cur_buff, cur_page);
      dirty = 0;
   }

   // Wait for any previous actions to complete
   while (atmel_flash_busy ())
      ;
   atmel_flash_read_memory (page, offset, buf, count);
   
   return count;
}

/** @brief Write p into the current flash address, for count bytes
 */
uint16_t dev_write_DEV_ATMEL_FLASH (const void *p, uint16_t count)
{
   uint16_t page, offset, num_bytes;
   uint16_t index = 0;
   uint8_t *buf = (uint8_t *)p;
   
   page = atmel_flash_addr / ATMEL_FLASH_PAGE_SIZE;
   offset = atmel_flash_addr % ATMEL_FLASH_PAGE_SIZE;
   
   while (atmel_flash_busy ())
      ;
   while (count > 0)
   {
      if (count + offset > ATMEL_FLASH_PAGE_SIZE)
	 num_bytes = ATMEL_FLASH_PAGE_SIZE - offset;
      else
	 num_bytes = count;
      
      if (flash_mode == ATMEL_FLASH_MODE_BUFFERED)
      {
	 if (page != cur_page)
	 {
	    if (dirty) {
	       while (atmel_flash_busy ())
		  ;
	       atmel_flash_flush_buffer(cur_buff, cur_page);
	       dirty = 0;
	    }
	    cur_buff = (cur_buff==ATMEL_FLASH_BUFFER_1?
			ATMEL_FLASH_BUFFER_2 : ATMEL_FLASH_BUFFER_1);
	    cur_page = page;
	    if (num_bytes < ATMEL_FLASH_PAGE_SIZE) {
	       while (atmel_flash_busy ())
		  ;
	       atmel_flash_fill_buffer(cur_buff, page);
	    }
	    // Erase next page right now
	    while (atmel_flash_busy ())
	       ;
	    atmel_flash_erase_page (page);
	 }
		
	 atmel_flash_write_buffer(cur_buff, offset, &buf[index], num_bytes);
	 dirty = 1;
      } else {
	 atmel_flash_fill_buffer (ATMEL_FLASH_DEFAULT_BUFFER, page);
	 while (atmel_flash_busy ())
	    ;
	 atmel_flash_erase_page (page);
	 while (atmel_flash_busy ())
	    ;
	 atmel_flash_write_buffer (ATMEL_FLASH_DEFAULT_BUFFER,
				   offset, &buf[index], num_bytes);
	 atmel_flash_flush_buffer (ATMEL_FLASH_DEFAULT_BUFFER, page);
	 while (atmel_flash_busy ())
	    ;
      }
		
      index += num_bytes;
      atmel_flash_addr += num_bytes;
      count -= num_bytes;
		
      page++;
      offset = 0;
   }

   return count;
}

/* device-specific functions */

void atmel_flash_init (void)
{
   mos_mutex_init (&atmel_flash_mutex);

   uint8_t handle = mos_disable_ints ();
   
   // set the flash select pin
   DDRA |= 1 << ATMEL_FLASH_SELECT_PIN;
   // set the pin high
   ATMEL_FLASH_SELECT |= 1 << ATMEL_FLASH_SELECT_PIN;

   // clear flash clock
   ATMEL_FLASH_PORT &= ~(1 << ATMEL_FLASH_CLK);
   // set flash clock output direction
   ATMEL_FLASH_DIRE |= 1 << ATMEL_FLASH_CLK;
   // clear flash out pin
   ATMEL_FLASH_PORT &= ~(1 << ATMEL_FLASH_OUT);
   // set flash out pin direction
   ATMEL_FLASH_DIRE &= ~(1 << ATMEL_FLASH_OUT);
   // clear flash in pin
   ATMEL_FLASH_PORT |= 1 << ATMEL_FLASH_IN;
   // set flash in pin direction
   ATMEL_FLASH_DIRE |= 1 << ATMEL_FLASH_IN;

   mos_enable_ints (handle);
   
   // Unbuffered mode is default for backward compatibility
   flash_mode = ATMEL_FLASH_MODE_UNBUFFERED;
   atmel_flash_addr = 0;
   cur_buff = ATMEL_FLASH_BUFFER_1;
   cur_page = ATMEL_FLASH_MAX_PAGES;
   dirty = 0;
   
   // must wait 20 ms before device is ready to use 
   mos_udelay (20000);
}

/** @brief Compare buf to the current flash address, for count bytes
 */
uint8_t atmel_flash_compare (uint8_t *buf, uint16_t count)
{
   uint16_t page, offset, num_bytes;
   uint16_t index = 0;
   uint8_t compare = 0;

   if (dirty) {
      while (atmel_flash_busy ())
	 ;
      atmel_flash_flush_buffer(cur_buff, cur_page);
      dirty = 0;
   }
   // Wait for any previous actions to complete
   while (atmel_flash_busy ())
      ;
   while (count > 0) {
      page = atmel_flash_addr / ATMEL_FLASH_PAGE_SIZE;
      offset = atmel_flash_addr % ATMEL_FLASH_PAGE_SIZE;
      if (count + offset > ATMEL_FLASH_PAGE_SIZE)
	 num_bytes = ATMEL_FLASH_PAGE_SIZE - offset;
      else
	 num_bytes = count;

      // Are we not comparing a whole page?
      if (num_bytes < ATMEL_FLASH_PAGE_SIZE) {
	 atmel_flash_fill_buffer (ATMEL_FLASH_DEFAULT_BUFFER,
				  page);
	 while (atmel_flash_busy ())
	    ;
      }
      // Write the data we want to compare to the buffer
      atmel_flash_write_buffer (ATMEL_FLASH_DEFAULT_BUFFER,
				offset, &buf[index], num_bytes);
	
      // Compare the buffer to main memory
      if (atmel_flash_compare_buffer (ATMEL_FLASH_DEFAULT_BUFFER, page)) {
	 compare = 1;
	 break;
      }

      index += num_bytes;
      atmel_flash_addr += num_bytes;
      count -= num_bytes;
   }
   // In case we exited the loop early, act like we read the whole range
   atmel_flash_addr += count;

   return compare;
}

/** @brief Compute the crc from the current flash address, for count bytes
 */
uint16_t atmel_flash_crc(uint32_t count)
{
   uint16_t page, offset, crc;
   
   page = atmel_flash_addr / ATMEL_FLASH_PAGE_SIZE;
   offset = atmel_flash_addr % ATMEL_FLASH_PAGE_SIZE;

   if (dirty && page <= cur_page && 
       page+(offset+count)/ATMEL_FLASH_PAGE_SIZE >= cur_page)
   {
      while (atmel_flash_busy ())
	 ;
      atmel_flash_flush_buffer(cur_buff, cur_page);
      dirty = 0;
   }
   // Wait for any previous actions to complete
   while (atmel_flash_busy ())
      ;
   crc = atmel_flash_crc_memory (page, offset, count);
   
   return crc;
}

/** @brief Set the flash in low
 */
static inline void atmel_flash_low (void) 
{
   uint8_t handle = mos_disable_ints ();
   
   // clear flash clock
   ATMEL_FLASH_PORT &= ~(1 << ATMEL_FLASH_CLK);
   // clear select pin
   ATMEL_FLASH_SELECT &= ~(1 << ATMEL_FLASH_SELECT_PIN);

   mos_enable_ints (handle);
}

/** @brief Set the flash pin high
 */
static inline void atmel_flash_high (void)
{
   // set the pin high
   ATMEL_FLASH_SELECT |= 1 << ATMEL_FLASH_SELECT_PIN;
}

// 0x11010111, 3 and 5 pin, pull low FLASH_IN and FLASH_CLK  
/** @brief Init the bit macro.
 */
#define BITINIT  uint8_t clrClkAndData = PORTD & ~0x28 

// first of all, the data is shifted in in rising edge and out in falling
// The I/O address for PORTD is 18, for PIND is 16
// first set the clk to low and the input to low, then 
// check the #n bit in spiOut.  
//  if 0,    then skip the step of writing the #n bit in FLASH_IN to high 
//           else pull the flash_in to high
// then set  clk to rising edge
// then check whether the FLASH_OUT in PIND is 0
// if yes, then skip (i.e. set the spiIn's bit to 0) 
//         else set the spiIn's bit to 1
/** @brief Write one bit of data.
 */
#define WRITEBIT(n)					\
   PORTD = clrClkAndData;				\
   asm __volatile__					\
   (  "sbrc %2," #n "\n"				\
      "\tsbi 18,3\n"					\
      "\tsbi 18,5\n"					\
      : "=d" (spiIn) : "0" (spiIn), "r" (spiOut))

/** @brief Read one bit of data.
 */
#define READBIT(n)				\
   PORTD = clrClkAndData;			\
   asm __volatile__				\
   ("\tsbi 18,5\n"				\
    "\tsbic 16,2\n"				\
    "\tori %0,1<<" #n "\n"			\
    : "=d" (spiIn) : "0" (spiIn))

/** @brief Retrieve 1 byte of data from the external flash.
 */
static uint8_t atmel_flash_get_byte (void)
{
   uint8_t spiIn = 0;

   uint8_t handle = mos_disable_ints(); 

   BITINIT;
   READBIT(7);
   READBIT(6);
   READBIT(5);
   READBIT(4);
   READBIT(3);
   READBIT(2);
   READBIT(1);
   READBIT(0);
    
   mos_enable_ints(handle);
   return spiIn;
}

/** @brief Send 1 byte of data from the external flash.
 */
static uint8_t atmel_flash_send_byte (uint8_t spiOut)
{
   uint8_t spiIn = 0;
   uint8_t handle = mos_disable_ints();

   BITINIT;
   WRITEBIT(7);
   WRITEBIT(6);
   WRITEBIT(5);
   WRITEBIT(4);
   WRITEBIT(3);
   WRITEBIT(2);
   WRITEBIT(1);
   WRITEBIT(0);
    
   mos_enable_ints(handle);
   return spiIn;
}

/* Directly Read Through the Memory, doesn't change the buffer
 * It works when writing/reading the characters, but doesn't work
 * when writing the long data.  Sometimes it just doesn't work
 * need to use logic analysis figure out later. 
 */

/** @brief Directly read through the memory.
 */
static void atmel_flash_read_memory(uint16_t page,
				    uint16_t offset, 
				    void *reqData, uint16_t len)
{
   uint8_t cmd[8], *reqPtr;
   uint16_t i;

   cmd[0] = C_READ_THROUGH_MEMORY; // 8 bit of op code
   cmd[1] = (page >> 7);           // 4 bit reserve and high 4 MSB
   cmd[2] = (page << 1) | offset >> 8;// 7 bit page and 1 bit offset MSB
   cmd[3] = offset;         // low-order 8 address bits
   cmd[4] = 0x00;
   cmd[5] = 0x00;
   cmd[6] = 0x00;
   cmd[7] = 0x00;
   reqPtr = (uint8_t *)reqData;

   atmel_flash_low();
    
   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);

   for (i = 0; i < len; i++)
      reqPtr[i] = atmel_flash_get_byte();
    
   atmel_flash_high();
  
}

/** @brief Compute crc on main memory without using a buffer.
 */
static uint16_t atmel_flash_crc_memory(uint16_t page,
				       uint16_t offset, 
				       uint32_t len)
{
   uint8_t cmd[8];
   uint16_t crc;
   uint32_t i;

   cmd[0] = C_READ_THROUGH_MEMORY; // 8 bit of op code
   cmd[1] = (page >> 7);           // 4 bit reserve and high 4 MSB
   cmd[2] = (page << 1) | offset >> 8;// 7 bit page and 1 bit offset MSB
   cmd[3] = offset;         // low-order 8 address bits
   cmd[4] = 0x00;
   cmd[5] = 0x00;
   cmd[6] = 0x00;
   cmd[7] = 0x00;

   atmel_flash_low();
    
   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);

   // The following code was adapted directly from crc.c
   {
      uint8_t v, xor_flag, byte, bit;
      crc = 0xFFFF/*INITIAL_VALUE*/;
	
      for(i = 0; i < len; i++)
      {
	 // Read the current byte from flash
	 byte = atmel_flash_get_byte();
	 v = 0x80;
	 for(bit = 0; bit < 8; bit++)
	 {
	    if(crc & 0x8000)
	       xor_flag= 1;
	    else
	       xor_flag= 0;
	    crc = crc << 1;
	    if(byte & v)
	       crc= crc + 1;
	    if(xor_flag)
	       crc = crc ^ 0x1021/*POLY*/;
	    v = v >> 1;
	 }
      }
      for(bit = 0; bit < 16; bit++)
      {
	 if(crc & 0x8000)
	    xor_flag= 1;
	 else
	    xor_flag= 0;
	 crc = crc << 1;
	 if(xor_flag)
	    crc = crc ^ 0x1021/*POLY*/;
      }
   }
    
   atmel_flash_high();
   
   return crc;  
}

/** @brief Get the flash register status.
 * @return Status of external flash
 */
/*
  static uint8_t atmel_flash_get_status (void)
  {

  uint8_t status;
  atmel_flash_low();
  atmel_flash_send_byte(C_REQ_STATUS); //SEND d7h, op code for register request
  status = atmel_flash_get_byte();
  atmel_flash_high();
  return status;
  }
*/

/** @brief Check the status of flash to see whether it's busy or not.
 * 
 * A little bit slower than simply holding the clk low, But
 * easier
 * @return TRUE if busy, else return FALSE
 */
static uint8_t atmel_flash_busy (void)
{
   uint8_t status;
   uint8_t myhandle; 
   atmel_flash_low();

   myhandle = mos_disable_ints();  
   atmel_flash_send_byte(C_REQ_STATUS); //SEND d7h, op code for register request
   status = atmel_flash_get_byte();
   mos_enable_ints(myhandle);
   atmel_flash_high();
   status &= 0x80;

   if( status == 0) 
      return TRUE;
   return FALSE;
}

//first version, no length check
/** @brief Read data from the buffer.
 * @param selected Read buffer to select
 * @param offset How far the data is offset
 * @param reqdata Data requested
 * @param len Length of data
 * @return TRUE if success, else return FALSE
 */
/*
  static void atmel_flash_read_buffer(uint8_t selected, uint16_t offset, 
  void *reqdata, uint16_t len)
  {
  uint8_t cmd[5], *reqPtr;
  uint16_t i;

  if(selected == 1)
  cmd[0] = C_READ_BUFFER1; // 8 bit of op code
  else 
  cmd[0] = C_READ_BUFFER2; // 8 bit of op code

  cmd[1] = 0x00;           // 8 bit don't care code
  cmd[2] = offset>>8;           // 8 bit don't care code
  cmd[3] = offset;         // low-order 8 address bits
  cmd[4] = 0x00;            // 8 bit don't care code
  reqPtr = (uint8_t *)reqdata;

  atmel_flash_low();
    
  for (i = 0; i < sizeof (cmd); i++)
  atmel_flash_send_byte(cmd[i]);
    
  for (i = 0; i < len; i++)
  reqPtr[i] = atmel_flash_get_byte();
    
  atmel_flash_high();
  }
*/

/** @brief Write the data to the buffer.
 * @param selected Write buffer to select
 * @param offset How far the data will be offset
 * @param reqdata Data requested
 * @param len Length of data
 * @return TRUE if success, else return FALSE
 */
static uint8_t atmel_flash_write_buffer(uint8_t selected, uint16_t offset, 
					void *reqdata, uint16_t len)
{
   uint8_t cmd[4], *reqPtr;
   uint16_t i;
  
   if(selected == 1)
      cmd[0] = C_WRITE_BUFFER1; // 8 bit of op code
   else 
      cmd[0] = C_WRITE_BUFFER2; // 8 bit of op code

   cmd[1] = 0x00;           // 8 bit don't care code
   cmd[2] = offset>>8;       // 7 bit don't care code with 1 bit address
   cmd[3] = offset;         // low-order 8 address bits
   reqPtr = (uint8_t *)reqdata;
  
   // put the cs to low in order to begin writing
   atmel_flash_low();

   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);
  
   for (i = 0; i < len; i++)
      atmel_flash_send_byte(reqPtr[i]);
  
   atmel_flash_high();

   return TRUE;
}

/** @brief Dump the buffer to the memory. 
 * @param selected Flush buffer to select
 * @param page Page to flush to
 * @return TRUE if success, else return FALSE
 */
static uint8_t atmel_flash_flush_buffer(uint8_t selected, uint16_t page)
{
   uint8_t i, cmd[4];
  
   if(selected == 1)
      cmd[0] = C_FLUSH_BUFFER1; // 8 bit of op code
   else 
      cmd[0] = C_FLUSH_BUFFER2; // 8 bit of op code

   cmd[1] = page >> 7;           // 4 bit reserve and high 4 MSB
   cmd[2] = page << 1;   // 7 bit page and 1 bit offset MSB
   cmd[3] = 0x00;           // 8 bit don't care code

   // put the cs to low in order to begin writing
   atmel_flash_low();

   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);
  
   atmel_flash_high();

   return TRUE;
}

/** @brief Compare the buffer to the memory. 
 * @param selected Buffer to compare
 * @param page Page to compare to
 * @return 0 if equal, else return 0x40
 */
static uint8_t atmel_flash_compare_buffer(uint8_t selected, uint16_t page)
{
   uint8_t i, cmd[4], status;
  
   if(selected == 1)
      cmd[0] = C_COMPARE_BUFFER1; // 8 bit of op code
   else 
      cmd[0] = C_COMPARE_BUFFER2; // 8 bit of op code

   cmd[1] = page >> 7;           // 4 bit reserve and high 4 MSB
   cmd[2] = page << 1;   // 7 bit page and 1 bit offset MSB
   cmd[3] = 0x00;           // 8 bit don't care code

   // put the cs to low in order to begin writing
   atmel_flash_low();

   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);
  
   atmel_flash_high();

   do {
      atmel_flash_low();
      atmel_flash_send_byte(C_REQ_STATUS); //SEND d7h, op code for register request
      status = atmel_flash_get_byte();
      atmel_flash_high();
   } while (status & 0x80);	// wait until comparison is complete

   return status & 0x40;		// return result of comparison
}

/** @brief Read a page from flash memory and fill into the buffer. 
 * @param selected Fill buffer to select
 * @param page Page to fill from
 * @return TRUE if success, else return FALSE
 */
static uint8_t atmel_flash_fill_buffer(uint8_t selected, uint16_t page)
{
   uint8_t i, cmd[4];

   if(selected == 1)  
      cmd[0] = C_FILL_BUFFER1; // 8 bit of op code
   else
      cmd[0] = C_FILL_BUFFER2; // 8 bit of op code

   cmd[1] = page >> 7;           // 4 bit reserve and high 4 MSB
   cmd[2] = page << 1;   // 7 bit page and 1 bit don't care MSB
   cmd[3] = 0x00;           // 8 bit don't care code

   // put the cs to low in order to begin writing
   atmel_flash_low();

   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);
  
   atmel_flash_high();
    
   return TRUE;
}

/** @brief Erase a whole page in the memory. 
 * @param page Page to erase 
 * @return TRUE if success, else return FALSE
 */
static uint8_t atmel_flash_erase_page(uint16_t page)
{
   uint8_t i, cmd[4];
  
   cmd[0] = C_ERASE_PAGE; // 8 bit of op code
   cmd[1] = page >> 7;           // 4 bit reserve and high 4 MSB
   cmd[2] = page << 1;   // 7 bit page and 1 bit don't care MSB
   cmd[3] = 0x00;           // 8 bit don't care code

   // put the cs to low in order to begin writing
   atmel_flash_low();

   for (i = 0; i < sizeof (cmd); i++)
      atmel_flash_send_byte(cmd[i]);
  
   atmel_flash_high();
    
   return TRUE;  
}

#endif
