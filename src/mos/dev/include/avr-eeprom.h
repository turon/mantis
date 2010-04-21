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

/*the basic on and off functions are available to external users,
  but the recommended use is through the dev_write call*/

/** @file avr-eeprom.h
 * @brief EEPROM driver for both the onboard 4k.
 * @author Jeff Rose
 * @author Modified: Adam Torgerson
 * @date Created: 02/17/2003
 * @date Modified: 04/06/2004
 */


#ifndef _AVR_EEPROM_H
#define _AVR_EEPROM_H

/* The following EEPROM areas are used by various components of MOS:
 * 
 * 0x000 to 0x00B		Boot control block 		(see boot.h)
 * 0x00C to 0x01F		Deluge control block	(see deluge.c)
 * 0x020 to 0x07F		Deluge page CRC's		(see deluge.c)
 * 0x080 to 0x0B3		Metadata for simple file system	(see simple_fs.c)
 * 0x0B4 to 0xFFF		Unused space
 */
#define DELUGE_INSTANCE_COUNT (4)

#define DELUGE_CONTROL_BLOCK_ADDR (0x00C) /* Right after the boot CB */
#define DELUGE_CONTROL_BLOCK_SIZE (16)

#define DELUGE_DIRECTORY_ADDR (DELUGE_CONTROL_BLOCK_ADDR +	\
			       DELUGE_INSTANCE_COUNT *		\
			       DELUGE_CONTROL_BLOCK_SIZE)

#define DELUGE_DIRECTORY_SIZE (1)

#define DELUGE_PAGE_CRC_ADDR (DELUGE_DIRECTORY_ADDR +	\
			      DELUGE_INSTANCE_COUNT *	\
			      DELUGE_DIRECTORY_SIZE)

#define SIMPLE_FS_ADDR	(DELUGE_PAGE_CRC_ADDR + 96)
#define SIMPLE_FS_MAX_FILES 10
#define SIMPLE_FS_SIZE	(2 + SIMPLE_FS_MAX_FILES * 12)

#define DELUGE_STATS_ADDR	(SIMPLE_FS_ADDR + SIMPLE_FS_SIZE)
#define DELUGE_STATS_COUNT 32
#define DELUGE_STATS_SIZE 8		// 4 16-bit fields

#define AVR_EEPROM_UNUSED_ADDR	(DELUGE_STATS_ADDR + \
				 DELUGE_STATS_COUNT * \
				 DELUGE_STATS_SIZE)

/** @brief Init the eeprom driver
 */
void avr_eeprom_init(void);

#endif
