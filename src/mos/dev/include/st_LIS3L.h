// This file is part of MOS, the MANTIS Operating System
// See http://mantis.cs.colorado.edu/
//
// Copyright (c) 2007 STMicroelectronics and Politecnico di Torino, Italy
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following
// disclaimer in the documentation and/or other materials provided
// with the distribution.
// * Neither the name of the MANTIS Project nor the names of its
// contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file mos/dev/include/st_LIS3L.h
 * @brief Header file of STM Accelerometer Driver (st_LIS3L).
 * @author Mohammad Mostafizur Rahman Mozumdar
 * @email  mohammad.mozumdar@polito.it
 * @date Created: 12/03/2007
 */


#ifndef _ST_LIS3L_H
#define _ST_LIS3L_H


#include "msp430-i2c.h"

#ifndef LIS3L_I2C_ADDRESS
#define LIS3L_I2C_ADDRESS 0x1D
#endif

enum {
   SET_RW_REG= 0,              // Set the Read Write Register
   SET_DEFAULT_MODE,           // Initialize three control registers
   SET_THRESHOLD,              // Set the threshold
   SET_DURATION,               // Set the duration
   SET_ONOFF,                  // Set the accelerometer On/Off 
   SET_AXIS_ZYX,               // Disable/Enable the Z,Y,X axis reading
   SET_INTERRUPT_REG,          // Set the interrupt flags
   SET_CTRL_REGS               // Set the three control registers
}; 


void init_i2c();
uint8_t fixAddress(uint8_t length, uint8_t regAddress);
uint8_t  st_LIS3LreadSingleRegister(uint8_t reg, uint8_t* buffer);
uint8_t  st_LIS3LwriteSingleRegister(uint8_t reg, uint8_t* buffer);
uint8_t st_LIS3LreadMultipleRegisters(uint8_t reg,uint8_t count, uint8_t* buffer );
uint8_t st_LIS3LwriteMultipleRegisters(uint8_t reg,uint8_t count, uint8_t* buffer );
void st_LIS3LHiddenInit();
void st_LIS3LgetXYZ(uint8_t* buffer );
uint8_t st_LIS3LgetWhoAmI();
uint8_t st_LIS3LgetDuration();
void st_LIS3LsetDuration(uint8_t value);
void st_LIS3LsetOnOff(uint8_t setStatus);
void st_LIS3LsetAxis(uint8_t x,uint8_t y,uint8_t z);
uint16_t st_LIS3LgetThreshold();

void st_LIS3LsetThreshold(uint16_t threshold);
void st_LIS3LsetInterrupt(uint8_t orAnd, uint8_t latched, uint8_t xUp, uint8_t xDown,		\
 uint8_t yUp, uint8_t yDown, uint8_t zUp, uint8_t zDown);		\

void st_LIS3LsetInterruptReg(uint8_t reg_value);
uint8_t st_LIS3LgetInterruptReg();


#endif
