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

/** @file mos/dev/include/msp430-i2c.h
 * @brief Header file of I2C driver for MSP430.
 * @author Mohammad Mostafizur Rahman Mozumdar
 * @email  mohammad.mozumdar@polito.it
 * @date Created: 04/03/2007
 */


#ifndef _MSP430_I2C_H
#define _MSP430_I2C_H


#define   I2C_START    0x01
#define   I2C_STOP     0x02
#define   TIMEOUT      64

typedef struct {
  uint16_t rxdmaen: 1;  // Receive DMA enable. 0 = disabled.
  uint16_t txdmaen: 1;  // Transmit DMA enable. 0 = disabled.
  uint16_t xa: 1;       // Extended addressing. 0 = 7-bit addressing.
  uint16_t listen: 1;   // Listen. 0 = disabled.
  uint16_t i2cword : 1; // Word mode. 0 = byte mode.
  uint16_t i2crm : 1;   // Repeat mode. 0 = I2CNDAT.
  uint16_t i2cssel : 2; // Clock source select. 0=None, 1=ACLK, 2=SMCLK
  uint16_t i2cpsc : 8;  // Clock prescaler.
  uint16_t i2csclh : 8; // Shift clock high register.
  uint16_t i2cscll : 8; // Shift clock low register.
  uint16_t i2coa : 10;  // Own address register.
} msp430_i2c_config_t;


#define msp430_i2c_clearModeI2C()  U0CTL &= ~(I2C | SYNC | I2CEN)

#define msp430_i2c_isI2C() ((U0CTL & I2C) && (U0CTL & SYNC) && (U0CTL & I2CEN))
   
 // U0CTL
#define  msp430_i2c_setMasterMode()  U0CTL |= MST
#define  msp430_i2c_setSlaveMode()   U0CTL &= ~MST
  
#define  msp430_i2c_enableI2C()      U0CTL |= I2CEN
#define  msp430_i2c_disableI2C()     U0CTL &= ~I2CEN
  
  // I2CTCTL
#define  msp430_i2c_getWordMode()                (I2CTCTL & I2CWORD ) 
#define  msp430_i2c_setWordMode(mode )  I2CTCTL |= ( mode & 0x1 ) << 7
  
#define  msp430_i2c_getRepeatMode()                (I2CTCTL & I2CRM) 
#define  msp430_i2c_setRepeatMode(mode )  I2CTCTL |= ( mode & 0x1 ) << 6
  
#define  msp430_i2c_getClockSource()               ((I2CTCTL >> 4) & 0x3)
#define  msp430_i2c_setClockSource( src )  I2CTCTL = ( ( src & 0x3 ) << 4 ) | I2CTCTL
  
#define  msp430_i2c_getTransmitReceiveMode()      (I2CTCTL & I2CTRX ) 
  
#define  msp430_i2c_setTransmitMode()              I2CTCTL |= I2CTRX
#define  msp430_i2c_setReceiveMode()               I2CTCTL &= ~I2CTRX
  
#define  msp430_i2c_getStartByte()                 (I2CTCTL & I2CSTB) 
#define  msp430_i2c_setStartByte()                 I2CTCTL |= I2CSTB
  
#define  msp430_i2c_getStopBit()   (I2CTCTL & I2CSTP) 
#define  msp430_i2c_setStopBit()   (I2CTCTL |= I2CSTP)
  
#define  msp430_i2c_getStartBit()  (I2CTCTL & I2CSTT) 
#define  msp430_i2c_setStartBit()  (I2CTCTL |= I2CSTT)
  
  // I2CDR
#define  msp430_i2c_getData()     I2CDR
#define  msp430_i2c_setData(v)   (I2CDR = v)
  
  // I2CNDAT
#define  msp430_i2c_getTransferByteCount()             I2CNDAT
#define  msp430_i2c_setTransferByteCount(v )   I2CNDAT = v
  
  // I2CPSC
#define  msp430_i2c_getClockPrescaler()               I2CPSC
#define  msp430_i2c_setClockPrescaler( v)      I2CPSC = v
  

// I2COA
#define  msp430_i2c_getOwnAddress()                  I2COA
#define  msp430_i2c_setOwnAddress( addr )   I2COA = addr
  
  // I2CSA
#define  msp430_i2c_getSlaveAddress()                 I2CSA
#define  msp430_i2c_setSlaveAddress( addr )  I2CSA = addr
  
  // I2CIE
#define  msp430_i2c_disableStartDetect()    I2CIE &= ~STTIE
#define  msp430_i2c_enableStartDetect()     I2CIE |= STTIE
  
#define  msp430_i2c_disableGeneralCall()    I2CIE &= ~GCIE
#define  msp430_i2c_enableGeneralCall()     I2CIE |= GCIE
  
#define  msp430_i2c_disableTransmitReady()  I2CIE &= ~TXRDYIE
#define  msp430_i2c_enableTransmitReady()   I2CIE |= TXRDYIE
  
#define  msp430_i2c_disableReceiveReady()   I2CIE &= ~RXRDYIE
#define  msp430_i2c_enableReceiveReady()    I2CIE |= RXRDYIE
  
#define  msp430_i2c_disableAccessReady()    I2CIE &= ~ARDYIE
#define  msp430_i2c_enableAccessReady()     I2CIE |= ARDYIE
  
#define  msp430_i2c_disableOwnAddress()     I2CIE &= ~OAIE
#define  msp430_i2c_enableOwnAddress()      I2CIE |= OAIE

#define  msp430_i2c_disableNoAck()          I2CIE &= ~NACKIE
#define  msp430_i2c_enableNoAck()           I2CIE |= NACKIE
  
#define  msp430_i2c_disableArbitrationLost()   I2CIE &= ~ALIE
#define  msp430_i2c_enableArbitrationLost()    I2CIE |= ALIE
  
  // I2CIFG
#define  msp430_i2c_isStartDetectPending() (I2CIFG & STTIFG ) 
    
#define  msp430_i2c_isGeneralCallPending() ( I2CIFG & GCIFG ) 
  
#define  msp430_i2c_isTransmitReadyPending() ( I2CIFG & TXRDYIFG ) 
  
#define  msp430_i2c_isReceiveReadyPending() ( I2CIFG & RXRDYIFG ) 
  
#define  msp430_i2c_isAccessReadyPending()  ( I2CIFG & ARDYIFG ) 
  
#define  msp430_i2c_isOwnAddressPending()   ( I2CIFG & OAIFG ) 
  
#define  msp430_i2c_isNoAckPending() ( I2CIFG & NACKIFG ) 
  
#define  msp430_i2c_isArbitrationLostPending() ( I2CIFG & ALIFG ) 
  
  // I2CIV
#define  msp430_i2c_getIV() I2CIV

void msp430_i2c_setModeI2C( msp430_i2c_config_t* config );
// I2CSCLH and I2CSCLL
uint16_t msp430_i2c_getShiftClock();
void  msp430_i2c_setShiftClock( uint16_t shift );
uint8_t msp430_i2c_read (uint8_t flags, uint16_t addr, uint8_t len, uint8_t* buf );
uint8_t msp430_i2c_write(uint8_t flags, uint16_t addr, uint8_t len,uint8_t *buf );
void mps430_i2c_init(msp430_i2c_config_t* config);

 

#endif
