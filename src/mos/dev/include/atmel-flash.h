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

/** @file atmel-flash.h
 * @brief Interfaces with the external flash through the dev layer
 */

#ifndef _ATMEL_FLASH_H_
#define _ATMEL_FLASH_H_


#define ATMEL_FLASH_PORT PORTD
#define ATMEL_FLASH_DIRE DDRD

#define ATMEL_FLASH_SELECT PORTA
#define ATMEL_FLASH_SELECT_PIN 3

#define ATMEL_FLASH_CLK 5
#define ATMEL_FLASH_OUT 2
#define ATMEL_FLASH_IN 3

#define ATMEL_FLASH_PAGE_SIZE 264
#define ATMEL_FLASH_MAX_PAGES 2048
#define ATMEL_FLASH_SIZE 540672

#define ATMEL_FLASH_BUFFER_1 0x1
#define ATMEL_FLASH_BUFFER_2 0x2

#define ATMEL_FLASH_DEFAULT_BUFFER ATMEL_FLASH_BUFFER_1

/* These codes are for setting modes using dev_mode().
 * 
 * ATMEL_FLASH_MODE_UNBUFFERED is the original way the flash driver worked.
 * For backward compatibility, it is the default mode.  Unbuffered writing works
 * as follows:
 * 		1. Read a page from flash into the buffer on the flash chip.
 * 		2. Erase the page in flash.
 * 		3. Write the data to the buffer.
 * 		4. Write the buffer to flash.
 */
#define ATMEL_FLASH_MODE_UNBUFFERED 3

/* ATMEL_FLASH_MODE_BUFFERED is a new mode that is more efficient for writing
 * lots of sequential data that does not come in page-sized chunks (e.g. Deluge).
 * The driver defers writing a page to flash until the on-chip buffer is full.
 * Read, seek, CRC, and compare commands still function properly.  They will
 * flush the unwritten buffer if necessary.  However, unwritten data will be
 * lost if the node is reset, unless you call
 * 		dev_ioctl(DEV_ATMEL_FLASH, DEV_FLUSH);
 */
#define ATMEL_FLASH_MODE_BUFFERED 4

/* opcodes for the device */
enum {
    C_READ_BUFFER1 = 0xd4,
    C_READ_BUFFER2 = 0xd6,
    C_WRITE_BUFFER1 = 0x84,
    C_WRITE_BUFFER2 = 0x87,
    C_FILL_BUFFER1 = 0x53, 
    C_FILL_BUFFER2 = 0x55, 
    C_FLUSH_BUFFER1 = 0x83,
    C_FLUSH_BUFFER2 = 0x86,
    C_QFLUSH_BUFFER1 = 0x88,
    C_QFLUSH_BUFFER2 = 0x89,
    C_COMPARE_BUFFER1 = 0x60,
    C_COMPARE_BUFFER2 = 0x61,
    C_WRITE_THROUGH_BUFFER1 = 0x82,
    C_WRITE_THROUGH_BUFFER2 = 0x85,
    C_READ_THROUGH_MEMORY = 0xE8,
    C_REQ_STATUS = 0xd7,
    C_ERASE_PAGE = 0x81
};

/** @brief Init the external flash.
 */
void atmel_flash_init();

/** @brief Compare the buffer to flash.
 */
uint8_t atmel_flash_compare (uint8_t *p, uint16_t count);

/** @brief Compute the CRC of flash.
 * Doesn't require a buffer.
 */
uint16_t atmel_flash_crc(uint32_t count);

#endif
