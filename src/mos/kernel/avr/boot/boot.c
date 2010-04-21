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
  Project Mantis
  File: boot.c
  Author: Jeff Rose
  Date: 3-15-03

  A boot loader for the atmega128.  Connects to a program over a serial
  connection to allow programming directly over the serial port.
*/

#include "mos.h"

#include <string.h>

#include "boot.h"
#include "mcs.h"
#include "led.h"
#include "com.h"
#include "boot_serial.h"
#include "crc.h"
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/boot.h>

#include "atmel-flash.h"

static comBuf in_pkt, out_pkt;
static uint16_t crc, crc_index;
static uint16_t addr;
static uint16_t image_size;
static uint16_t page_index, image_index;

#define LOOP_VAL 50

uint8_t pagebuf[PAGE_SIZE_BYTES]; // A memory buffer for flash page data
boot_control_block_t boot_cblock;

/** @brief Stub to allow compilation with i2c_eeprom.c.
 *
 * This doesn't really do anything.
 */
inline void enable_ints(uint8_t handle)
{
   SREG = handle;
}

/** @brief Stub to allow compilation with i2c_eeprom.c.
 *
 * This doesn't really do anything.
 */
inline uint8_t disable_ints()
{
   uint8_t sreg = SREG;
   cli();

   return sreg;
}

uint16_t avr_eeprom_read_polling (void *buf, uint16_t addr, uint8_t count)
{
   //set up state for interrupt state machine
   uint8_t buffer_index = 0;

   while (buffer_index < count) {
      while (EECR & (1 << EEWE)) //busy wait on EEWE
	 ;
      EEAR = addr + buffer_index; // Setup the address
      EECR |= (1 << EERE); // Set the read strobe
     
      ((uint8_t *)buf)[buffer_index++] = EEDR; // Now read the data
   }

   return 0;
}

uint16_t avr_eeprom_write_polling (void *buf, uint16_t addr, uint8_t count)
{
   //set up state for interrupt state machine
   uint8_t buffer_index = 0;

   while (buffer_index < count) {
      while (EECR & (1 << EEWE)) //busy wait on EEWE
	 ;
      while (SPMCSR & (1 << SPMEN)) // busy wait on SPMEN
	 ;
      EEAR = addr + buffer_index; // Setup the address
      EEDR = ((uint8_t *)buf)[buffer_index++];    // Now write the data
      EECR = (1 << EEMWE); //EEMWE bit of the EECR
      EECR = (1 << EEMWE) | (1 << EEWE); // Set the write strobe
   }
   return 0;
}


void mos_udelay(uint16_t usec)
{
   while(usec > 0) {
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      asm volatile("nop" ::);
      usec--;
   }
}

void atmel_flash_init ()
{
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

   // must wait 20 ms before device is ready to use
   mos_udelay (20000);
}

inline void atmel_flash_low (void) 
{
   // clear flash clock
   ATMEL_FLASH_PORT &= ~(1 << ATMEL_FLASH_CLK);
   // clear select pin
   ATMEL_FLASH_SELECT &= ~(1 << ATMEL_FLASH_SELECT_PIN);
}

inline void atmel_flash_high (void)
{
   // set the pin high
   ATMEL_FLASH_SELECT |= 1 << ATMEL_FLASH_SELECT_PIN;
}

// 0x11010111, 3 and 5 pin, pull low FLASH_IN and FLASH_CLK  
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
#define WRITEBIT(n)					\
   PORTD = clrClkAndData;				\
   asm __volatile__					\
   (  "sbrc %2," #n "\n"				\
      "\tsbi 18,3\n"					\
      "\tsbi 18,5\n"					\
      : "=d" (spiIn) : "0" (spiIn), "r" (spiOut))

#define READBIT(n)				\
   PORTD = clrClkAndData;			\
   asm __volatile__				\
   ("\tsbi 18,5\n"				\
    "\tsbic 16,2\n"				\
    "\tori %0,1<<" #n "\n"			\
    : "=d" (spiIn) : "0" (spiIn))

uint8_t atmel_flash_get_byte (void)
{
   uint8_t spiIn = 0;

   BITINIT;
   READBIT(7);
   READBIT(6);
   READBIT(5);
   READBIT(4);
   READBIT(3);
   READBIT(2);
   READBIT(1);
   READBIT(0);
    
   return spiIn;
}

uint8_t atmel_flash_send_byte (uint8_t spiOut)
{
   uint8_t spiIn = 0;

   BITINIT;
   WRITEBIT(7);
   WRITEBIT(6);
   WRITEBIT(5);
   WRITEBIT(4);
   WRITEBIT(3);
   WRITEBIT(2);
   WRITEBIT(1);
   WRITEBIT(0);
    
   return spiIn;
}

/* Directly Read Through the Memory, doesn't change the buffer
 * It works when writing/reading the characters, but doesn't work
 * when writing the long data.  Sometimes it just doesn't work
 * need to use logic analysis figure out later. 
 */
void atmel_flash_read_memory(uint16_t page,
			     uint16_t offset, 
			     void *reqData, uint32_t len)
{
   uint8_t cmd[8], *reqPtr;
   uint32_t i;

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

uint8_t send_packet(comBuf *pkt)
{
   uint8_t i;

   // Send the preamble
   for(i = 0; i < PREAMBLE_SIZE; i++)
      send_byte(PREAMBLE);

   // Now send size and data
   send_byte(pkt->size);
   for(i = 0; i < pkt->size; i++)
      send_byte(pkt->data[i]);

   return 0;
}

uint8_t recv_packet(comBuf *pkt)
{
   uint8_t i;

   for(i = 0; i < PREAMBLE_SIZE; i++)
      recv_byte();

   pkt->size = recv_byte();
   for(i = 0; i < pkt->size; i++)
      pkt->data[i] = recv_byte();

   return 0;
}

/** @brief Get an address.
 *
 * Receives two bytes, sends ADDRESS_RECVD.
 * @return Address received 
 */
void *flash_address(uint32_t addr)
{
   // Now set RAMPZ if necessary

   if(addr > 0xFFFF)
      RAMPZ = addr >> 16;
   else
      RAMPZ = 0;
   addr = addr & 0xFFFF;

   return (void *)(uint16_t)addr;
}

/** @brief Receives an image and writes it to flash.
 */
void load_image(comBuf *packet)
{
   uint8_t j;
   
   mos_led_on(2);
   
   image_size = *(uint16_t *)(&packet->data[1]);

   out_pkt.size = 1;
   out_pkt.data[0] = LOAD_ACK;
   send_packet (&out_pkt);
   
   image_index = 0; //image index is how many bytes written from the image
                    //image size is the total number of bytes in the image
   addr = 0;
   while (image_index < image_size) {
      uint8_t size = 0;
      recv_packet(&in_pkt);
      //addr = *(uint16_t *)(&in_pkt.data[0]);
      addr = image_index /*/ 2*/;
      crc = *(uint16_t *)(&in_pkt.data[0]);
      
      // Now get the data packets
      page_index = 0; //how many bytes from the current page we've received
      
      while (page_index < PAGE_SIZE_BYTES) { //loop until we have a full page
         recv_packet (&in_pkt); //get the packet from the computer
	 
	 //out_pkt.data[0] = PACKET_ACK;
	 //out_pkt.size=1;
	 //send_packet (&out_pkt);

	 //copy page into buffer
         for (j = 0; j < in_pkt.size; j++) 
            pagebuf[page_index + j] = in_pkt.data[j]; 
         page_index += in_pkt.size;

         // If this is the last packet then we break out.
         if (in_pkt.size < COM_DATA_SIZE) {
            size = in_pkt.size;
            break;
         }
      }
      
      // actually write the page 
      flash_write_page ((void *)flash_address (addr), pagebuf);

      out_pkt.size = 1;
      
      // find out the proper index to pass to crc_compute
      if (in_pkt.size < COM_DATA_SIZE)
         crc_index = image_size - image_index;
      else
         crc_index = page_index;
      
      // perform the crc calculation and send off the result 
      
      if (crc_compute (pagebuf, crc_index) != crc)
         out_pkt.data[0] = CRC_ERROR;
      else
	 out_pkt.data[0] = CRC_OK;
      
      //out_pkt.data[0] = CRC_OK;
      send_packet (&out_pkt);

      // finally update the image index 
      image_index += page_index;
   }
   out_pkt.size = 1;
   out_pkt.data[0] = IMAGE_ACK;
   send_packet (&out_pkt);

   mos_led_off(2);
}

/** @brief Sends a byte for each fuse value.
 */
void read_fuses(void)
{
   comBuf pkt;
   
   RAMPZ = 0;
   
   flash_enable_rww(); // Make sure the RWW section is enabled

   pkt.size = 4;
   pkt.data[0] = flash_read_fuse((void *)0x0000); // Low
   pkt.data[1] = flash_read_fuse((void *)0x0003); // High
   pkt.data[2] = flash_read_fuse((void *)0x0002); // Extended
   pkt.data[3] = flash_read_fuse((void *)0x0001); // Lock bits

   send_packet(&pkt);
}

/** @brief Get a page from flash.
 */
void read_image(void)
{
   comBuf packet;
   uint16_t i;
   uint16_t addr = 0;
   uint16_t word;

   flash_address(0x0000); // Set the flash flag to the beginning
   flash_enable_rww(); // Make sure the RWW section is enabled

   packet.size = 2;
   for(i = 0; i < PAGE_SIZE_BYTES; i += 2) {
      word = flash_read_word((void *)addr);
      send_byte ((uint8_t)(word >> 8));
      send_byte ((uint8_t)word);
      addr += 2;
   }
}

/** @brief Send a byte as two hex characters.
 * @param byte The byte to send
 */
void puthex(uint8_t byte)
{
   uint8_t val;
   val = byte >> 4;
   if(val >= 9)
      send_byte('0'+ val);
   else
      send_byte('A'+ (val - 10));

   val = byte & 0x0f;
   if(val >= 9)
      send_byte('0'+ val);
   else
      send_byte('A'+ (val - 10));
}

/** @brief Read the control block in at startup.
 * @param cb Control block will be read in here
 */
void read_control_block(boot_control_block_t *cb)
{
   // Read from the eeprom boot control block
   avr_eeprom_read_polling(cb, CONTROL_BLOCK_ADDR, CONTROL_BLOCK_SIZE);
}

/** @brief Write control block out to flash.
 * @param cb Control block to write
 */
void write_control_block(boot_control_block_t *cb)
{
   // Write the page into eeprom
   avr_eeprom_write_polling(cb, CONTROL_BLOCK_ADDR,
			    CONTROL_BLOCK_SIZE);
}

void clear_control_block(boot_control_block_t *cb)
{
   cb->reprogram = 0;
   write_control_block(cb);
}

uint8_t control_block_is_sane(boot_control_block_t *cb)
{
   	if (cb->start_addr + cb->byte_count > ATMEL_FLASH_SIZE)
   		return 0;
   	if (cb->byte_count > 120 * (uint32_t)1024)
   		return 0;
   	if (cb->reprogram != 0 && cb->reprogram != 1)
   		return 0;
   	return 1;
}

/** @brief Copies an image from one part of flash to another.
 * @param cb The control block
 */
void image_copy(boot_control_block_t *cb)
{
   uint32_t i = 0, size, addr;

   atmel_flash_init();
   mos_led_on(2);

   i = 0;
   addr = cb->start_addr;
   while(i < cb->byte_count) {
      uint16_t page = addr / ATMEL_FLASH_PAGE_SIZE;
      uint16_t offset = addr % ATMEL_FLASH_PAGE_SIZE;
      // read page into page buffer from eeprom
      if((cb->byte_count - i) >= PAGE_SIZE_BYTES)
	 size = PAGE_SIZE_BYTES;
      else 
	 size = cb->byte_count - i;
      atmel_flash_read_memory(page, offset, pagebuf, size);
      
      // write page into flash memory
      boot_spm_busy_wait();
      flash_write_page(flash_address(i /*/ 2*/), pagebuf);
      
      i += size;
      addr += size;
   }

   mos_led_off(2);
}

void send_byte_packet(comBuf *pkt, uint8_t byte)
{
   pkt->size = 1;
   pkt->data[0] = byte;
   send_packet(pkt);
}

void send_word_packet(comBuf *pkt, uint16_t word)
{
   pkt->size = 3;
   pkt->data[0] = WORD_PRINT;
   *((uint16_t *)&pkt->data[0]) = word;
   send_packet(pkt);
}

comBuf pkt;
int main(void)
{
   uint8_t val;
   uint16_t loop = 0;
   uint8_t loop_flag = 0;

   void (*reset_func)(void) = (void*)0x00000; // Pointer to normal RESET vector

   cli();         // Disable interrupts for the whole boot loader
   

   mos_led_init();
   mos_led_off(0);
   mos_led_on(1);
   mos_led_on(2);

   init_serial();
   
   // Send a message to let the shell know we are connected
   send_byte_packet(&pkt, LOADER_PRESENT);

   // wait for the shell to respond
   while(!(val = check_recv())) {
      if(loop > 0xf000)
	 loop_flag++;
      if(loop_flag > LOOP_VAL)
	 break;
      loop++;
   }

   mos_led_off(0);
   mos_led_on(1);
   mos_led_off(2);

   // see if we need to reprogram
   read_control_block(&boot_cblock);
   if (!control_block_is_sane(&boot_cblock)) {
   		boot_cblock.start_addr = (uint32_t)0;
   		boot_cblock.byte_count = (uint32_t)0;
   		boot_cblock.reprogram = 0;
   		write_control_block(&boot_cblock);
   } else if(boot_cblock.reprogram == 1 && val != CLEAR_CB) {
      // time to reprogram
      boot_lock_bits_set(0);
      image_copy(&boot_cblock);
      clear_control_block(&boot_cblock);
      write_control_block(&boot_cblock);
      
      while(boot_rww_busy())
	 boot_rww_enable();
      boot_spm_busy_wait();
      
      mos_led_off(0);
      mos_led_off(1);
      mos_led_off(2);
      reset_func();
   }

   if(loop_flag < LOOP_VAL) { //recvd a byte in the uart
      recv_packet(&pkt);        //get the packet
      //mos_led_on(2);
      val = pkt.data[0];        //get the value
      if(val != SHELL_PRESENT && val != CLEAR_CB) { //see if byte was shell
	 flash_enable_rww();
	 while(boot_rww_busy())
	    boot_rww_enable();
	 boot_spm_busy_wait();
	 mos_led_off(0);
	 mos_led_off(1);
	 mos_led_off(2);
	 reset_func();          //start the app if shell not present
      } else
	 ;//mos_led_on(2);
   }
   
   if(loop_flag > LOOP_VAL) {
      while(boot_rww_busy())
	 boot_rww_enable();
      boot_spm_busy_wait();
      mos_led_off(0);
      mos_led_off(1);
      mos_led_off(2);
      reset_func();
   }

   // shell must be present
   while(1) {
      recv_packet(&pkt);
      val = pkt.data[0];
      switch(val) {
      case LOAD_IMAGE: // load a new image
	 mos_led_on(0);
         load_image(&pkt);
	 break;
      case READ_FUSES:
         read_fuses();
	 break;
      case READ_FLASH:
	 read_image();
	 break;
      case CLEAR_CB:
	 read_control_block(&boot_cblock);
         clear_control_block(&boot_cblock);
	 send_byte_packet(&pkt, CLEAR_CB_PONG);
	 break;
      case START:
         flash_enable_rww (); // Make sure the RWW section is enabled
	 while(boot_rww_busy())
	    boot_rww_enable();
	 boot_spm_busy_wait();
	 send_byte_packet(&pkt, START);
	 mos_led_off(0);
	 mos_led_off(1);
	 mos_led_off(2);
	 reset_func();      //jump to the application
	 break;
      case SHELL_PRESENT:
	 mos_led_on (1);
	 send_byte_packet(&pkt, LOADER_PRESENT_PONG);
	 break;
      default:
	 send_byte_packet(&pkt, UNKNOWN);
	 break;
      }
   }
   mos_led_off(0);
   mos_led_off(1);
   mos_led_off(2);
   

   return 0;
}
