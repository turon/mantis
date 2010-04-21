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
   File:   cc1000_csma.h
   Authors: Charles Gruenwald III & Jeff Rose
   Date:   05-13-04
  
*/

/** @file cc1000_csma.h
 * @brief Driver provides: use of cc1000 carrier sensing medium access control.
 *
 * Driver requires: AVR architecture with cc1000 connected via SPI bus. This driver
 * is mainly implemented on top of cc1000_defaults.
 *
 * Note: To activate this routing protocol, the appropriate line in main.c must
 * be uncommented along with commenting the other routing protocols.
 * @author Charles Gruenwald III, Jeff Rose
 * @date 05/13/2004
 */

#ifndef _CC1000_CSMA_H_
#define _CC1000_CSMA_H_

/* CC1000 ioctl requests */
/* set transmit power, takes uint8_t argument */
#define CC1000_TX_POWER 0
#define CC1000_RSSI     1
#define CC1000_FREQ     2

/** @brief Init function. */
void cc1000_csma_init();


/** @brief get the number of crc errors */
inline uint16_t cc1000_get_crc_errors();
/** @brief get the number of memory errors */
inline uint16_t cc1000_get_mem_errors();
/** @brief get the number of sync errors */
inline uint16_t cc1000_get_sync_errors();
/** @brief get the number of size errors */
inline uint16_t cc1000_get_size_errors();

#endif
