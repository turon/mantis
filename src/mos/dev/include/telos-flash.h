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

/** @file telos-flash.h
 * @brief port and pin definitions as well as function declarations
 * @author John Ledbetter
 * @date 07/01/2005
 */
#ifdef PLATFORM_TELOSB

#ifndef __TELOS_FLASH_H__
#define __TELOS_FLASH_H__

// Port & Pin locations
#define ST_FLASH_CS_PORT 4
#define ST_FLASH_CS_PIN  4
#define ST_FLASH_CS ST_FLASH_CS_PORT, ST_FLASH_CS_PIN

#define ST_FLASH_MISO_PORT 3
#define ST_FLASH_MISO_PIN  2
#define ST_FLASH_MISO ST_FLASH_MISO_PORT, ST_FLASH_MISO_PIN

#define ST_FLASH_MOSI_PORT 3
#define ST_FLASH_MOSI_PIN  1
#define ST_FLASH_MOSI ST_FLASH_MOSI_PORT, ST_FLASH_MOSI_PIN

#define ST_FLASH_HOLD_PORT 4
#define ST_FLASH_HOLD_PIN  7
#define ST_FLASH_HOLD ST_FLASH_HOLD_PORT, ST_FLASH_HOLD_PIN

#define ST_FLASH_SCLK_PORT 3
#define ST_FLASH_SCLK_PIN  3
#define ST_FLASH_SCLK ST_FLASH_SCLK_PORT, ST_FLASH_SCLK_PIN


#define ST_FLASH_WP_PORT 1
#define ST_FLASH_WP_PIN  2
#define ST_FLASH_WP ST_FLASH_WP_PORT, ST_FLASH_WP_PIN

#define ST_FLASH_POWER_PORT 4
#define ST_FLASH_POWER_PIN 3
#define ST_FLASH_POWER ST_FLASH_POWER_PORT, ST_FLASH_POWER_PIN



void st_flash_init();
/** @addtogroup driver_telosf TELOSb External Flash
 */
/** @brief device ioctl for the telosb flash device.  This
 * IOCTL tells the driver to erase the entire flash.
 * @ingroup driver_telosf
*/
#define TELOS_FLASH_BULK_ERASE 0x11
/** @brief device ioctl for the telosb flash device. This
 * IOCTL tells the driver to erase the sectore that the associated address
 * is located in.
 * @ingroup driver_telosf
 */
#define TELOS_FLASH_SECT_ERASE 0x12


#endif

#endif
