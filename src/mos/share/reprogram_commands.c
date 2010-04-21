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

#ifdef ARCH_AVR

#if defined(PLATFORM_MICA2)

#include "msched.h"
#include "com.h"
#include "net.h"
#include "deluge.h"
#include "command_daemon.h"
#include "node_id.h"
#include "boot.h"
#include "mcs.h"
#include "led.h"
#include "dev.h"
#include "crc.h"
#include "printf.h"
#include "atmel-flash.h"
#include "simple_fs.h"

#include "reprogram_commands.h"

static comBuf *recvd;
static comBuf send;
static uint8_t pagebuf[PAGE_SIZE_BYTES]; // A memory buffer for flash page data

/** @brief Sends one byte of data over the serial line
 *
 * @param iface Interface to send byte on
 * @param cmd Data to send
 */
static inline void send_byte_command(uint8_t iface, uint8_t cmd)
{
   send.data[0] = cmd;
   send.size = 1;
   
   if(iface == IFACE_SERIAL)
      com_send(IFACE_SERIAL, &send);
   else if(iface == IFACE_RADIO)
      com_send(IFACE_RADIO, &send);
}

void repro_get_cb()
{
   boot_control_block_t cb;
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&cb, sizeof(cb));
   printf("Node id: %d, Byte Count: %l, Start addr: %l, Reprogram: %d\n",
	       cb.node_id, cb.byte_count, cb.start_addr, cb.reprogram);
}

void repro_set_cb()
{
   uint32_t addr;
   uint32_t size;
   uint8_t reprogram;
   
   printf ("Start address? ");
   addr = prompt_longlong ("#:");
   printf ("Image size? ");
   size = prompt_longlong ("#:");
   printf ("Reprogram? ");

   reprogram = prompt_long ("#:");
   boot_control_block_t cb;
   cb.node_id = mos_node_id_get ();
   cb.byte_count = size;
   cb.start_addr = addr;
   cb.reprogram = reprogram;
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   dev_write (DEV_AVR_EEPROM, (uint8_t *)&cb, CONTROL_BLOCK_SIZE);
}

void repro_clear_cb()
{
   boot_control_block_t cb;
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   cb.node_id = mos_node_id_get ();
   cb.byte_count = (uint32_t)0;
   cb.start_addr = (uint32_t)0;
   cb.reprogram = 0;
   dev_write (DEV_AVR_EEPROM, (uint8_t *)&cb, sizeof (cb));
}

static mos_file* reprogram_iface (uint8_t iface, uint32_t* mismatches)
{
   uint8_t j;
   uint16_t crc, crc_index;
   uint32_t addr;
   uint32_t image_size;
   uint16_t page_index;
   uint32_t image_index;

   uint8_t size = 0;
   uint8_t packet_size;

   send_byte_command (iface, ACK);
   recvd = com_recv (iface);

   if(recvd->data[0] == LOAD_IMAGE) {
      image_size = (uint32_t)*(uint16_t *)(&recvd->data[1]);
      com_free_buf (recvd);
   } else {
      com_free_buf (recvd);
      printf ("Didn't get LOAD_IMAGE\n");
      return 0;
   }
   
   // Find the next page after the last program image we wrote
   /*uint32_t start_page = (cb->start_addr + cb->byte_count + 
   		ATMEL_FLASH_PAGE_SIZE-1) / ATMEL_FLASH_PAGE_SIZE;
   addr = start_page * ATMEL_FLASH_PAGE_SIZE;
   if (!compare && addr + image_size < ATMEL_FLASH_SIZE) {
   		cb->start_addr = addr;
   } else if (!compare) {
   		// There is not enough room on flash to store this image,
   		// start again at 0
   		cb->start_addr = addr = 0;
   } else {
   		// We're only comparing, use the address already in the control block
   		addr = cb->start_addr;
   }*/
   
   mos_file* file = NULL;
   if (mismatches == NULL) {
   		file = mos_file_create("rrp", image_size);
   		if (!file) {
   			printf("Couldn't open file 'rrp'.\n");
   			return 0;
   		}
   		addr = 0;
   } else {
   		mismatches = 0;
   		file = mos_file_open("rrp");
   		// file doesn't exist
		if (!file) return NULL;
   		// TODO check for matching file size
   		addr = file->start;
   }

   image_index = 0; //image index is how many bytes written from the image
                    //image size is the total number of bytes in the image
   send_byte_command(iface, LOAD_ACK);

   while(image_index < image_size) {
      mos_led_toggle (0);
      recvd = com_recv(iface);
      // don't send address for performance
      //addr = *(uint16_t *)(&recvd->data[0]);
      //addr = image_index;
      crc = *(uint16_t *)(&recvd->data[0]);
      com_free_buf (recvd);
      // Now get the data packets

      //page index is how many bytes from the current page we've received
      page_index = 0; //page size is the size of the pages in memory.
      
      while (page_index < PAGE_SIZE_BYTES) { //loop until we have a full page
	 recvd = com_recv(iface);
         for (j = 0; j < recvd->size; j++) { //loop through the packet
	    //copy packet into page buffer [at right index]
            pagebuf[page_index+j] = recvd->data[j];
         }

	 packet_size = recvd->size;
	 com_free_buf (recvd);
         page_index += packet_size;

	 //don't send page ack for performance
	 //PAGE ACK
	 //send_byte_command(PACKET_ACK);
	 
	 // If this is the last packet then we break out
         if(packet_size < COM_DATA_SIZE) {
            size = packet_size;
            break;
         }
      }
      // actually write the page
      //dev_ioctl(DEV_ATMEL_FLASH, DEV_SEEK, addr);
      if (mismatches == NULL) {
      	mos_file_write(pagebuf, file, addr, sizeof(pagebuf));
      	//dev_write(DEV_ATMEL_FLASH, pagebuf, sizeof(pagebuf));
      } else {
      	dev_ioctl(DEV_ATMEL_FLASH, DEV_SEEK, addr);
      	// Don't write, just compare the buffer with the page already in flash
      	if (atmel_flash_compare(pagebuf, sizeof(pagebuf)))
      		(*mismatches)++;
      }
      addr += sizeof(pagebuf);
      
      // find out the proper index to pass to crc_compute
      if (packet_size < COM_DATA_SIZE)
         crc_index = (uint16_t)(image_size - image_index);
      else
         crc_index = page_index;
      
      // perform the crc calculation and send off the result

      if (crc_compute (pagebuf, crc_index) != crc)
         send_byte_command (iface, CRC_ERROR);
      else
	 send_byte_command (iface, CRC_OK);
      
      // finally update the image index
      image_index += page_index;
   }
   send_byte_command (iface, IMAGE_ACK);

	// In case the flash driver is in buffered mode, flush the buffer
    dev_ioctl(DEV_ATMEL_FLASH, DEV_FLUSH);

   return file;
}

void repro_reprogram()
{
	deluge_suspend();
   /*boot_control_block_t cb;
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&cb, sizeof(cb));*/
   
   //printf ("Receiving a code image over serial\n");
   //uint32_t image_size = reprogram_iface (IFACE_SERIAL, &cb, 0);
   mos_file* file = reprogram_iface (IFACE_SERIAL, NULL);
   mos_file_flush(file);

   boot_control_block_t cb;
   cb.node_id = mos_node_id_get ();
   cb.byte_count = file->length;
   cb.start_addr = file->start;
   cb.reprogram = 1;
   dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   dev_write (DEV_AVR_EEPROM, (uint8_t *)&cb, CONTROL_BLOCK_SIZE);
   
   deluge_resume();
}

void repro_verify()
{
   /*boot_control_block_t cb;
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, CONTROL_BLOCK_ADDR);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&cb, sizeof(cb));*/
   
   uint32_t mismatches = 0;
	mos_file* file = reprogram_iface (IFACE_SERIAL, &mismatches);
	
	if (!file) {
		printf("The code image was not found.\n");
	} else if (mismatches) {
		printf("%d pages of the flashed code image did not match.\n", mismatches);
	} else {
		printf("Verification successful.\n");
	}
}

#endif /* platform mica2 */

#endif /*PLATFORM_LINUX*/
