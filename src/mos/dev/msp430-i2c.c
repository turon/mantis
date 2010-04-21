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

/** @file mos/dev/msp430-i2c.c
 * @brief I2C BUS driver for MSP430.
 * @author Mohammad Mostafizur Rahman Mozumdar
 * @email  mohammad.mozumdar@polito.it
 * @date Created: 07/03/2007
 */




#ifdef PLATFORM_TELOSB

#include "mutex.h"
#include <msp430x16x.h>
#include "msp430-i2c.h"


static uint8_t* m_buf;
static uint8_t m_len;
static uint8_t m_pos;
static uint8_t m_flags;
static uint8_t readDone=0;
static uint8_t readBuffer=0;
static uint8_t status=0;
mos_mutex_t i2c_mutex;


void nextWrite();
void nextRead();
void processingDone( uint8_t st);

/**
  * Initialize UART0 for I2C .
  *
  * @param config contains all the configuration data (DMA access,addressing mode,Listen mode,
  *        Word mode,Clock source select .. 
  */

void msp430_i2c_setModeI2C( msp430_i2c_config_t* config ) {
   // call SIMO.makeInput();// p31
  P3DIR &= ~(0x1 << 1); 
  // call SIMO.selectModuleFunc();
  P3SEL |= (0x1 << 1); 
  // call UCLK.makeInput(); //p33
  P3DIR &= ~(0x1 << 3);
  // call UCLK.selectModuleFunc();
  P3SEL |= (0x1 << 3);
 
  U0CTL &= ~(I2C | I2CEN | SYNC);
  U0CTL = SWRST;
  U0CTL |= SYNC | I2C;
  U0CTL &= ~I2CEN;
      
  U0CTL |= ( ( config->rxdmaen << 7 ) |
             ( config->txdmaen << 6 ) |
             ( config->xa << 4 ) |
             ( config->listen << 3 ) );
  I2CTCTL = 0;
  I2CTCTL = ( ( config->i2cword << 7 ) |
	      ( config->i2crm << 6 )   |
	      ( config->i2cssel << 4 ) );
  I2CPSC = config->i2cpsc;
  I2CSCLH = config->i2csclh;
  I2CSCLL = config->i2cscll;
  I2COA = config->i2coa;
  U0CTL |= I2CEN;
}

/**
  * Get Shift clock 
  * 
  * @return shift clock which is 16 bits where most significat 8 bits are the value of I2CSCLH and 
  *         the least significant bits  are the value of I2CSCLL
  */
uint16_t msp430_i2c_getShiftClock() {
    uint16_t shift;
    shift = I2CSCLH;
    shift <<= 8;
    shift |= I2CSCLL;
    return shift;
}

/**
  * Set Shift clock 
  * 
  * @param shift clock which is 16 bits where most significat 8 bits are the value of I2CSCLH and 
  *         the least significant bits  are the value of I2CSCLL
  */
void  msp430_i2c_setShiftClock( uint16_t shift ) {
    I2CSCLH = shift >> 8;
    I2CSCLL = shift;
}

/**
  * Interrupt sub-routine tracing signal UART0TX_VECTOR 
  * 
  * The value of the I2CIV
  *    0x04  No acknowledgement
  *    0x8h  Register access ready
  *    0xAh  Receive data ready
  *    0xCh  Transmit data ready
  *
  */
 
interrupt (UART0TX_VECTOR) uart0_tx_int(void) {
    
  int i = 0;
  if (msp430_i2c_isI2C())
  switch( I2CIV ) {
      
    case 0x04:
      if ( I2CDCTL & I2CBB )
	  msp430_i2c_setStopBit();
      while( I2CDCTL & I2CBUSY );
      processingDone( 0 );
      break;
      
    case 0x08:
      while( (I2CDCTL & I2CBUSY) ) {
	if ( i++ >= TIMEOUT ) 
	  processingDone( 0 );
      }
    
      processingDone( 1 );
      break;
      
    case 0x0A:
      readBuffer= msp430_i2c_getData();
      if (!readDone) nextRead();
      break;
      
    case 0x0C:
      nextWrite();
      break;
      
    default:
      break; 
   }
}

/**
  * Writing the next byte to the I2C bus from buffer. When I2RM=1, I2CSTP must be set 
  * before the last I2CDR value is written. Othwerwise, correct STOP generation will not occur.
  * 
  */

void nextWrite() {
  if ( ( m_pos == m_len - 1 ) && ( m_flags & I2C_STOP ) ) {
     msp430_i2c_setStopBit();      // I2CSTP
  }
  else if ( m_pos == m_len ) {
     processingDone( 3 );
     return;
  }
  msp430_i2c_setData( m_buf[ m_pos++ ] ); // writing data to the I2C bus
}

/**
  * Reading received data byte which is stored at I2CDR register and when 
  * reading is done send a I2CSTP to stop the transmission 
  */
void nextRead() {
  m_buf[ m_pos++ ] = readBuffer; // Reading I2CDR
  if ( m_pos == m_len ) {
    readDone=1;
    if ( m_flags & I2C_STOP )
	{
          msp430_i2c_setStopBit(); //I2CSTP
        }
    else
	{ 
          processingDone( 2 );
        }
  }
}
/**
  * This function is called when sending/receiving is done, it clears the I2C interrupt  
  * flags(I2CIE) and unlock the BUS (i2c_metex)for others who are waiting to use the BUS.  
  *
  *@param st contains the status of the operation 0=FAILURE , 0> SUCCESS
  */
void  processingDone( uint8_t st) {
   status=st; 
   I2CIE = 0;
  //UNLOCK THE ACCESS OF THE BUS
   mos_mutex_unlock(&i2c_mutex);
}
/**
  *Reading certain amount of bytes from a Device  
  * 
  * 
  *@param flags defines whether it is a repeated start and stop mode (I2C_START|I2C_STOP) or others
  *@param addr  contains the register value from where the reading will start 
  *@param len contains the number of bytes to be read from the device 
  *@param buf is the buffer to store the receiving bytes 
  *@return the value of the processing status 0: Failure , 0> Success 
  */
uint8_t msp430_i2c_read (uint8_t flags, uint16_t addr, uint8_t len, uint8_t* buf ) {


  //LOCK THE ACCESS OF THE BUS
  mos_mutex_lock (&i2c_mutex);
  
  m_buf = buf;
  m_len = len;
  m_flags = flags;
  m_pos = 0;

  readDone=0;
  status=0;

  _DINT(); 
  msp430_i2c_setMasterMode();
  msp430_i2c_setReceiveMode();
    
  msp430_i2c_setSlaveAddress( addr );
  msp430_i2c_enableReceiveReady();
  msp430_i2c_enableAccessReady();
  msp430_i2c_enableNoAck();
  if ( flags & I2C_START )
    {
      msp430_i2c_setStartBit();
     _EINT(); 
  
   }
  else
    {
      nextRead();
    }
 
 while (I2CIE !=0);
 return status; 

}
/**
  * Writing certain amount of bytes to a Device  
  * 
  * 
  *@param flags defines whether it is a start(I2C_START) or stop(I2C_STOP) mode or both
  *@param addr  contains the address of the slave device 
  *@param len contains the number of bytes to be written 
  *@para buf buffer that stores the transmition bytes 
  *@return the value of the processing status 0: Failure , 0> Success 
  */
uint8_t msp430_i2c_write(uint8_t flags, uint16_t addr, uint8_t len,uint8_t *buf ) {
    
  //LOCK THE ACCESS OF THE BUS
  mos_mutex_lock (&i2c_mutex);

  m_buf = buf;
  m_len = len;
  m_flags = flags;
  m_pos = 0;

  status=0;
  _DINT();
  msp430_i2c_setMasterMode();

  msp430_i2c_setTransmitMode();
    
  msp430_i2c_setSlaveAddress( addr );

  msp430_i2c_enableTransmitReady();
  msp430_i2c_enableAccessReady();
  msp430_i2c_enableNoAck();
  
    
  if ( flags & I2C_START )
     {
        _EINT();
        msp430_i2c_setStartBit();
     } 
  else
    {  
       nextWrite();
       _EINT(); 
    }
 while (I2CIE !=0);
 return status; 
}
/**
  * Initialize the I2C Bus
  * 
  * 
  *@param flags contains the configuration data for the I2C bus
  */
void mps430_i2c_init(msp430_i2c_config_t* config)
{
   mos_mutex_init(&i2c_mutex); // Initialize the mutex for the access control of I2C 
   msp430_i2c_setModeI2C(config);
}

#endif
