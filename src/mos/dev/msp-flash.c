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

#include "mos.h"

#ifdef PLATFORM_TELOSB


#include "dev.h"
#include "msp-flash.h"
#include "led.h"
//#include "printf.h"

#define MSP_FLASH_BLOCK_SIZE 64

uint16_t msp_flash_address;

uint8_t buffer[MSP_FLASH_BLOCK_SIZE];

//#define DEBUG_REPROG 1

#define RP_VERIFY 0x12
#define RP_WRITE  0x34

int __attribute__ ((section (".data"))) reprogram_helper(uint32_t ex_flash_addr, int type);
void __attribute__ ((section (".data"))) erase_flash(uint16_t address, int flags);


boolean mos_verify_image(uint32_t ext_flash_addr)
{
   return reprogram_helper(ext_flash_addr, RP_VERIFY);
}

void mos_reprogram(uint32_t ext_flash_addr)
{
   reprogram_helper(ext_flash_addr, RP_WRITE);
   //reprogram_helper(ext_flash_addr, 43);
   
}



int __attribute__ ((section (".data"))) reprogram_helper(uint32_t ex_flash_addr, int type)
{
  
   
   
   uint32_t addr = ex_flash_addr;
   uint16_t c_checksum;
   uint16_t a_checksum;
   boolean write = (type == RP_WRITE);

   if (write)
      mos_disable_ints();
       
   dev_open(DEV_TELOS_FLASH);
   dev_mode(DEV_TELOS_FLASH, DEV_MODE_ON);  

   if (!write)
      printf("verifying...\n");
   

   
   if (write)
      erase_flash(0x4000, MSP_FLASH_ERASE_ALL);

   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
   dev_read(DEV_TELOS_FLASH, buffer, 6);  
   
   uint16_t sect_count = *(uint16_t*)(buffer + 4);
  
   //printf("%d sectors to write\n", sect_count);
   //printf("looking for file at %x\n", (uint16_t)ex_flash_addr);
   
   uint16_t sect_size;
   uint16_t sect_addr;
   
   // skip past header
   addr += 6;
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
  
   int i;
   
   for(i = 0; i < sect_count; ++i)
   {
      // read the sector size
      LED_ON(i + 1);
      dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
 

      dev_read(DEV_TELOS_FLASH, &sect_size, 2);
      addr += 2;
      
      dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);

      // read the sector address
      dev_read(DEV_TELOS_FLASH, &sect_addr, 2);
      addr += 2;
      
      // begin computing checksum;
      c_checksum = 0;
      c_checksum ^= sect_size;
      c_checksum ^= sect_addr;
      
      //printf("[%d]  %d bytes at address %x\n", i, sect_size, sect_addr);

      uint16_t blocks = sect_size / MSP_FLASH_BLOCK_SIZE;
      uint16_t left_over = sect_size % MSP_FLASH_BLOCK_SIZE;
      
      while(blocks--)
      {
	 dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
	 dev_read(DEV_TELOS_FLASH, buffer, sizeof(buffer));
	
	 msp_flash_address = sect_addr;

	 if (write)
	    dev_write(DEV_MSP_FLASH, buffer, sizeof(buffer));
	 else
	 {
	    uint16_t* iter = (uint16_t*)buffer;
	    uint16_t* iter_end = (uint16_t*)(buffer + sizeof(buffer));
	    for( ; iter != iter_end; ++iter)
	       c_checksum ^= *iter;
	    
	    
	 }
	 
	 sect_addr += sizeof(buffer);
	 addr += sizeof(buffer);
       }
      
      
      if (left_over)
      {
	 
	 dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
	 dev_read(DEV_TELOS_FLASH, buffer, left_over);

	 
	 msp_flash_address = sect_addr;
	 if (write)
	    dev_write(DEV_MSP_FLASH, buffer, left_over);
	 else
	 {
	    uint16_t* iter = (uint16_t*)buffer;
	    uint16_t* iter_end = (uint16_t*)(buffer + left_over);
	    for( ; iter != iter_end; ++iter)
	       c_checksum ^= *iter;
	    
	 }
	 
	 addr += left_over;
      }

      dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
      dev_read(DEV_TELOS_FLASH, &a_checksum, sizeof(a_checksum));
      //printf("checksum (computed): %x\n checksum (actual): %x\n", c_checksum, a_checksum);

      if (!write)
      {
	 if (c_checksum != a_checksum)
	    return FALSE;
      }
      
      addr += 2;
     
      
      dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
      
   }

   if (write)
      asm volatile ("br #0x4000h\n");
   
   // shhh
   //dev_close(DEV_TELOS_FLASH);
   
   return TRUE;
 
   
}



void __attribute__ ((section (".data"))) erase_flash(uint16_t address, int flags)
{
   uint16_t* clr = (uint16_t*)address;
   
  while(FCTL3 & BUSY);
   
   FCTL2 = FWKEY | FSSEL1 | FN0;
   FCTL3 = FWKEY;
   FCTL1 = FWKEY |  flags;
   // FCTL1 = FWKEY | ERASE;
   // clear entire flash
   *clr = 0;
   
   while(FCTL3 & BUSY);
   
   FCTL3 = FWKEY + LOCK;
}


uint8_t  __attribute__ ((section (".data"))) dev_read_DEV_MSP_FLASH(void *buf, uint16_t count)
{
   uint8_t* d = (uint8_t*)buf;
   uint8_t* s = (uint8_t*)msp_flash_address;
   
   while(count--)
      *(d++) = *(s++);

   return DEV_OK;
}

uint8_t  __attribute__ ((section (".data"))) dev_write_DEV_MSP_FLASH(const void *buf, uint16_t count)
{
   uint16_t i,j;
   uint8_t* ptr;
   const uint8_t* data = buf;
   
   uint16_t blocks = count / MSP_FLASH_BLOCK_SIZE;
   uint16_t left_over = count % MSP_FLASH_BLOCK_SIZE;

   //ACCVIE = NMIIE = OFIE = 0;
   
   //  start of write
   ptr = (uint8_t*)msp_flash_address;

   if (msp_flash_address % MSP_FLASH_BLOCK_SIZE)
   {
      left_over = count;
      //while(FCTL3 & BUSY) ;
      blocks = 0;
   }
   
   for(i = 0; i < blocks; ++i)
   {
     
      while(FCTL3 & BUSY)
	 ;
      
      FCTL2 = FWKEY | FSSEL1 | FN0;
      FCTL3 = FWKEY;
      FCTL1 = FWKEY | BLKWRT | WRT;

     
      for(j = 0; j < MSP_FLASH_BLOCK_SIZE; ++j)
      {
	 *(ptr++) = *(data++);
	 while(!(FCTL3 & WAIT))
	    ;
      }

      FCTL1 = FWKEY;

      while(FCTL3 & BUSY);
      
      FCTL3 = FWKEY | LOCK;
      
   }
   
   for(i = 0; i < left_over; ++i)
   {
      FCTL2 = FWKEY | FSSEL1 | FN0;
      FCTL3 = FWKEY;
      FCTL1 = FWKEY | WRT;
      *(ptr++) = *(data++);
      while(FCTL3 & BUSY);
      FCTL1 = FWKEY;
      FCTL3 = FWKEY | LOCK;
   }
   
   return DEV_OK;
   
}
#endif
