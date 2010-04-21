///  This file is part of MOS, the MANTIS Operating System
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
  File: boot.h
  Author: Brian Shucker
  Date: 5-22-03

  boot loader header file
*/

/** @file boot.h
 * @brief A boot loader for the atmega128.
 *
 * Connects to a program over a serial connection to allow programming 
 * directly over the serial port.
 * @author Jeff Rose
 * @date Created: 03/15/2003
 */

#ifndef _BOOT_H_
#define _BOOT_H_

#include <inttypes.h>

/** @brief A structure to hold the boot control block in flash */
typedef struct control_block
{
   /*WARNING: struct must have even size (word aligned)*/
   /** @brief The node id */
   uint16_t node_id;
   /** @brief The starting addr of the image to flash (or 0 if none) */
   uint32_t start_addr;
   /** @brief The lengh of the image to flash (or 0 if none) */
   uint32_t byte_count;
   /** @brief Whether or not to dynamically reprogram on reboot */
   uint8_t reprogram;
   uint8_t reserved;
} boot_control_block_t;

/*control block address is in the on-chip eeprom, address 0-12 */
/** @brief location of boot control block in eeprom (currently unused) */
#define CONTROL_BLOCK_ADDR 0
#define CONTROL_BLOCK_SIZE sizeof (boot_control_block_t)
/** @brief max number of code pages in an image (8 for boot loader) */
#define MAX_PAGES 248
/** @brief location of reset vector */
#define RESET_VECTOR_ADDR 0x1E000 

void flash_write_page(void *flash_addr, uint8_t *buf);
uint8_t flash_read_fuse (void *addr);
void flash_enable_rww (void);
uint16_t flash_read_word (void *addr);

#endif /*_BOOT_H_*/
