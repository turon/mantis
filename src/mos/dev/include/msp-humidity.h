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
/** @file msp-humidity.h
 * @brief SHT11 device driver header
 * @author John Ledbetter
 * @date 07/12/2005
 */
#ifndef __MSP_HUMIDITY_H__
#define __MSP_HUMIDITY_H__

/* Pin definitions */
#define SHT11_POWER_PORT 1
#define SHT11_POWER_PIN  7
#define SHT11_POWER SHT11_POWER_PORT, SHT11_POWER_PIN

#define SHT11_SCLK_PORT  1
#define SHT11_SCLK_PIN   6
#define SHT11_SCLK SHT11_SCLK_PORT, SHT11_SCLK_PIN

#define SHT11_DATA_PORT  1
#define SHT11_DATA_PIN   5
#define SHT11_DATA SHT11_DATA_PORT, SHT11_DATA_PIN

/* Command bytes */
#define SHT11_CMD_READ_TEMP     3 
#define SHT11_CMD_READ_HUMIDITY 5
#define SHT11_CMD_READ_SR       7
#define SHT11_CMD_WRITE_SR      6
#define SHT11_CMD_SOFT_RESET    30

/* Status register masks */
#define SHT11_SR_HEATER  (1 << 2)
#define SHT11_SR_RELOAD  (1 << 1)
#define SHT11_SR_LOW_RES (1 << 0)

/* Function declarations */
void sht11_init(void);

/* arguments for dev_ioctl_MSP_HUMIDITY/TEMPERATURE */
#define SHT11_LOW_RES_ON   0x01
#define SHT11_LOW_RES_OFF  0x02
#define SHT11_HEATER_ON    0x04
#define SHT11_HEATER_OFF   0x08
#define SHT11_RELOAD_ON    0x10
#define SHT11_RELOAD_OFF   0x20


#endif
