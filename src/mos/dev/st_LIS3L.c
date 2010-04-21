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

/** @file mos/dev/st_LIS3L.c
 * @brief STM Accelerometer Driver (st_LIS3L).
 * @author Mohammad Mostafizur Rahman Mozumdar
 * @email  mohammad.mozumdar@polito.it
 * @date Created: 15/03/2007
 */



#include "mos.h"

#ifdef PLATFORM_TELOSB

#include "st_LIS3L.h"
#include "dev.h"
#include "mutex.h"

// Configuration data of the I2C bus to tune with accelerometer 
msp430_i2c_config_t msp430_i2c_default_config = { 
  rxdmaen : 0,
  txdmaen : 0,
  xa : 0,
  listen : 0,
  i2cword : 0,
  i2crm : 1,
  i2cssel : 2,
  i2cpsc : 0,
  i2csclh : 3,
  i2cscll : 3,
  i2coa : 0,
};

mos_mutex_t msp_acc_mutex;        // metex for the access control of the accelerometer
static uint8_t readWriteReg=0x1F; // default read/write register is set to 0x1F
/**
  * Initialize I2C Bus with msp430_i2c_default_config 
  *
  */
void init_i2c()
{
  mps430_i2c_init(&msp430_i2c_default_config);
  mos_mutex_init(&msp_acc_mutex); // Initialize metex
}
/**
  * Set MSB of the register when more than one byte need to be read or write   
  *
  *@param lenght number of the bytes to be read or written  
  *@param regAddress register value from where read or write will be done  
  *@return modified register value with MSB set or unchanged 
  */
uint8_t fixAddress(uint8_t length, uint8_t regAddress){
    uint16_t regAddressMod;
    regAddressMod = regAddress;
    if (length>1)
      regAddressMod |= 0x80;
    return regAddressMod;
}

/**
  * Read a single byte from a register of the accelerometer  
  *
  *@param reg register value of the accelerometer  
  *@param buffer for storing the value of the register 
  *
  *@return the status 0= Failure Others= Success
  */
uint8_t  st_LIS3LreadSingleRegister(uint8_t reg, uint8_t* buffer)
{

 init_i2c();
 if (msp430_i2c_write(I2C_START|I2C_STOP,LIS3L_I2C_ADDRESS,1,&reg)!=0)
   return msp430_i2c_read(I2C_START|I2C_STOP,LIS3L_I2C_ADDRESS,1,buffer);
 else 
   return 0; //Failure 
}
/**
  * Write a single byte to a register of the accelerometer  
  *
  *@param reg register value of the accelerometer  
  *@param buffer containing the the value of the register 
  *
  *@return the status 0= Failure Others= Success
  */
uint8_t st_LIS3LwriteSingleRegister(uint8_t reg, uint8_t* buffer)
{

  init_i2c();
  if (msp430_i2c_write(I2C_START,LIS3L_I2C_ADDRESS, 1, &reg)!=0)
    return msp430_i2c_write(I2C_STOP,LIS3L_I2C_ADDRESS, 1, buffer);
  else  
    return 0; //Failure 
}

/**
  * Read multiple bytes from the accelerometer  
  *
  *@param reg register value of the accelerometer from where the reading will get started   
  *@param count number of the bytes to be read  
  *@param buffer for storing the bytes received from the accelerometer
  *
  *@return the status 0= Failure Others= Success
  */
uint8_t st_LIS3LreadMultipleRegisters(uint8_t reg,uint8_t count, uint8_t* buffer )
{
 uint8_t modified_reg;

 init_i2c();
 modified_reg=fixAddress(count,reg);
 if (msp430_i2c_write(I2C_START|I2C_STOP,LIS3L_I2C_ADDRESS, 1,&modified_reg)!=0)
    return msp430_i2c_read(I2C_START|I2C_STOP,LIS3L_I2C_ADDRESS, count,buffer);
 else 
    return 0; //Failure 
}
/**
  * Write multiple bytes (registers) in the accelerometer  
  *
  *@param reg register value of the accelerometer from where the writing will get started   
  *@param count number of the bytes to be written  
  *@param buffer storing the bytes to be written 
  *
  *@return the status 0= Failure Others= Success
  */
uint8_t st_LIS3LwriteMultipleRegisters(uint8_t reg,uint8_t count, uint8_t* buffer )
{
 uint8_t modified_reg;

 init_i2c();
 modified_reg= fixAddress(count,reg); 
 if (msp430_i2c_write(I2C_START,LIS3L_I2C_ADDRESS, 1,&modified_reg)!=0)
   return msp430_i2c_write(I2C_STOP,LIS3L_I2C_ADDRESS, count, buffer); 
 else 
   return 0; //Failure
}
/**
  * Initialize the control registers of the accelerometer  
  * 
  * 
  * Address of the ctrl_reg1 is 0x20, mainly controls power mode, X,Y,Z axies(enable/disable)
  * Address of the ctrl_reg2 is 0x21, controls scaling of g,interrupt enable,data-ready enable ..
  * Address of the ctrl_reg3 is 0x22, mainly controls clock,high/low pass filter
  */
void st_LIS3LHiddenInit()
{
  uint8_t buffer[3];

  /********************************************************/
  buffer[0]=0x00;
  st_LIS3LwriteSingleRegister(0x1F,buffer);
  /********************************************************/
  buffer[0] = 0xC7; //40  hz
  buffer[1] = 0x04; //data ready enabled
  buffer[2] = 0x08;
  st_LIS3LwriteMultipleRegisters(0x20,3,buffer);
  /********************************************************/
}
/**
  * Get current X,Y,Z values from the acclerometer which are stroed in six registers
  * starting from 0x28 to 0x2D   
  * 
  * @param buffer will be used to store the six bytes received from the acclerometer
  * 
  */
void st_LIS3LgetXYZ(uint8_t* buffer )
{
 st_LIS3LreadMultipleRegisters (0x28,6,buffer);
}
/**
  * Get the register value of WHO_AM_I 
  *   
  * @return the value of WHO_AM_I  which is 0x3A
  * 
  */
uint8_t st_LIS3LgetWhoAmI()
{ 
  uint8_t buffer;
  st_LIS3LreadSingleRegister(0x0F,&buffer);
  
  return buffer; 
}
/**
  * Get the duration value which is stored in the register FF_WU_DURATION 
  *   
  * @return the value of FF_WU_DURATION 
  * 
  */
uint8_t st_LIS3LgetDuration()
{
  uint8_t buffer;
  st_LIS3LreadSingleRegister(0x36,&buffer);
  
  return buffer; 
}
/**
  * Set the duration value which will be stored in the register FF_WU_DURATION 
  *   
  * @param value of the duration 
  * 
  */
void st_LIS3LsetDuration(uint8_t value)
{
 st_LIS3LwriteSingleRegister(0x36,&value);
}
/**
  * Enable/Disable the accelerometer which is can be done by setting bits 6-7 of    
  * the ctrl_reg1(PD1,PD0);   
  *   PD1     PD0
  *   0        0     Power Down
  *   1        1     Normal Mode
  *   
  * @param setStatus, =1 for Normal Mode; =0 for Power Down 
  * 
  */
void st_LIS3LsetOnOff(uint8_t setStatus)
{
 uint8_t value_ctrl1;

 st_LIS3LreadSingleRegister(0x20,&value_ctrl1);
 value_ctrl1=(setStatus)? (value_ctrl1|0xC0) : (value_ctrl1 & (~0xC0));
 st_LIS3LwriteSingleRegister(0x20,&value_ctrl1);
}
/**
  * Enable/Disable the axis(X,Y,Z) reading of theaccelerometer
  * which is can be done by setting bits 0-2 of  the ctrl_reg1(Zen,Yen,Xen);   
  *   Zen/Yen/Xen
  *   0              Disable reading
  *   1              Enable reading
  *   
  * @param x value for Xen (Can be 1/0)  
  * @param y value for Yen (Can be 1/0)  
  * @param z value for Zen (Can be 1/0)  
  */
void st_LIS3LsetAxis(uint8_t x,uint8_t y,uint8_t z)
{
 uint8_t value_ctrl1;

 st_LIS3LreadSingleRegister(0x20,&value_ctrl1);
 value_ctrl1= (value_ctrl1 & ~0x07) | ((x?1:0)+(y?1<<1:0)+(z?1<<2:0));
 st_LIS3LwriteSingleRegister(0x20,&value_ctrl1);
}
/**
  * Get the threshold value which is stored in registers FF_WU_THS_L(0x34) and  
  * FF_WU_THS_H(0x35)  
  * FF_WU_THS_L= Low byte of the threshold
  * FF_WU_THS_H= High byte of the threshold
  * 
  * 
  *@return 16 bit value of the threshold
  * 
  */
uint16_t st_LIS3LgetThreshold()
{ 
 uint8_t buffer[2];
 
 st_LIS3LreadMultipleRegisters (0x34,2,buffer);  
 return buffer[0]+((uint16_t)(buffer[1])<<8); 
}
/**
  * Set the threshold value which will be stored in registers FF_WU_THS_L(0x34) and  
  * FF_WU_THS_H(0x35)  
  * FF_WU_THS_L= Low byte of the threshold
  * FF_WU_THS_H= High byte of the threshold
  * 
  * 
  *@param threshold 16 bit value of the threshold
  * 
  */
void st_LIS3LsetThreshold(uint16_t threshold)
{ 
/* uint8_t* ptr_threshold;
 uint8_t buffer[2];
 
 ptr_threshold=(uint8_t*)&threshold;
 buffer[0]=ptr_threshold[0];
 buffer[1]=ptr_threshold[1];
 st_LIS3LwriteMultipleRegisters(0x34,2,buffer);*/
 st_LIS3LwriteMultipleRegisters(0x34,2,(uint8_t*)&threshold);
}

/**
  * Set the interrupt flags register (bit by bit) which is FF_WU_CFG(0x30)   
  * 
  *@param orAnd defines And/Or combination of Interrupt events.
  *       0: OR combination of interrupt events 
  *       1: AND combination of interrupt events
  *@param latched defines latch Interrupt request into FF_WU_SRC reg 
  *       with the FF_WU_SRC reg cleared by reading FF_WU_ACK reg.
  *       0: interrupt request not latched
  *       1: interrupt request latched
  *@param zUp enables interrupt generation on Z high event
  *       0/1: disable/enable interrupt request
  *@param zDown enables interrupt generation on Z low event
  *       0/1: disable/enable interrupt request
  *@param yUp enables interrupt generation on Y high event
  *       0/1: disable/enable interrupt request
  *@param yDown enables interrupt generation on Y low event
  *       0/1: disable/enable interrupt request
  *@param xUp enables interrupt generation on X high event
  *       0/1: disable/enable interrupt request
  *@param xDown enables interrupt generation on X low event
  *       0/1: disable/enable interrupt request
  *
  * 
  */
void st_LIS3LsetInterrupt(uint8_t orAnd, uint8_t latched, uint8_t zUp, uint8_t zDown, 
 uint8_t yUp, uint8_t yDown, uint8_t xUp, uint8_t xDown){  //TRUE on; FALSE off;
 uint8_t buffer;

 buffer =
    + 0x80 * (orAnd?1:0)
    + 0x40 * (latched?1:0)
    + 0x20 * (zUp?1:0)
    + 0x10 * (zDown?1:0)
    + 0x08 * (yUp?1:0)
    + 0x04 * (yDown?1:0)
    + 0x02 * (xUp?1:0)
    + 0x01 * (xDown?1:0);
 st_LIS3LwriteSingleRegister(0x30,&buffer);
}
/**
  * Set the interrupt flags register (whole byte) which is FF_WU_CFG(0x30)   
  * 
  *@param reg_value contains 8-bit value of the interrupt flags
  *       
  * 
  */
void st_LIS3LsetInterruptReg(uint8_t reg_value){  

 st_LIS3LwriteSingleRegister(0x30,&reg_value);
}
/**
  *Get the interrupt flags register (whole byte) which is FF_WU_CFG(0x30)   
  * 
  *@return 8-bit value of the interrupt flags
  *       
  * 
  */

uint8_t st_LIS3LgetInterruptReg()
{  
  uint8_t buffer;
  st_LIS3LreadSingleRegister(0x30,&buffer);
  
  return buffer; 
}
/**
  *Read any number of bytes from the accelerometer   
  * 
  *@param buf buffer for storing received bytes
  *@param count number of bytes to be read
  *@return the status 0= Failure Others= Success
  *       
  * 
  * NOTES:
  * Before using this function use 
  * dev_ioctl(DEV_MSP_ACCELEROMETER,SET_RW_REG,readRegValue)  
  * to set the starting register (for reading) of the accelerometer
  * 
  */
uint8_t dev_read_DEV_MSP_ACCELEROMETER(void *buf, uint16_t count)
{
 if (count==1)
   return st_LIS3LreadSingleRegister(readWriteReg,(uint8_t *)buf);
 else if (count>1) 
   return st_LIS3LreadMultipleRegisters (readWriteReg,count,(uint8_t *)buf); 
 return 1;
}
/**
  *Write any number of bytes to the accelerometer   
  * 
  *@param buf buffer holding bytes to be send to accelerometer
  *@param count number of bytes to be written
  *@return the status 0= Failure Others= Success
  *       
  * 
  * NOTES:
  * Before using this function use 
  * dev_ioctl(DEV_MSP_ACCELEROMETER,SET_RW_REG,writeRegValue)  
  * to set the starting register (for writing) of the accelerometer
  * 
  */
uint8_t dev_write_DEV_MSP_ACCELEROMETER(const void *buf, uint16_t count)
{
 if (count==1)
   st_LIS3LwriteSingleRegister(readWriteReg,(uint8_t *)buf);
 else if (count>1) 
   st_LIS3LwriteMultipleRegisters (readWriteReg,count,(uint8_t *)buf); 
 return 1;
}

/**
  *Setting IO control of the accelerometer. This function takes variable registers  
  * 
  *@param request contains the value of setting
  *       SET_RW_REG                  Set the Read Write Register
  *                                   Followed by the arg containing the value of readWriteReg
  *       SET_DEFAULT_MODE            Initialize three control registers
  *                                   No more arg 
  *       SET_THRESHOLD               Set the threshold
  *                                   Followed by the arg containing the value of threshold
  *       SET_DURATION                Set the duration
  *                                   Followed by the arg containing the value of duration
  *       SET_ONOFF                   Set the accelerometer On/Off 
  *                                   Followed by the arg containing the value 1(On)/0(Off)
  *       SET_AXIS_ZYX                Disable/Enable the Z,Y,X axis reading
  *                                   Followed by the three args containing the value for      
  *                                   setting Z,Y,X(0=Disable/Enable)
  *       SET_INTERRUPT_REG           Set the interrupt flags
  *                                   Followed by the arg containing the value of interrupt flags
  *       SET_CTRL_REGS               Set the three control registers
  *                                   Followed by the three args containing the value of
  *                                   ctrl_reg1,ctrl_reg2,ctrl_reg3
  * 
  */

uint8_t dev_ioctl_DEV_MSP_ACCELEROMETER(int8_t request, ...)
{

   uint8_t input[3];
   va_list ap;
   va_start(ap, request);

   switch (request) {
   case SET_RW_REG:
      readWriteReg = (uint8_t)va_arg (ap, uint16_t);
      break;
   case SET_DEFAULT_MODE:
      st_LIS3LHiddenInit();
      break;
   case SET_THRESHOLD:
      st_LIS3LsetThreshold(va_arg (ap, uint16_t));
      break;
   case SET_DURATION:
      st_LIS3LsetDuration((uint8_t)va_arg (ap, uint16_t));
      break;
   case SET_ONOFF:
      st_LIS3LsetOnOff((uint8_t)va_arg (ap, uint16_t));
      break;
   case SET_AXIS_ZYX:
     /* input[0]=(uint8_t)va_arg (ap, uint16_t);
      input[1]=(uint8_t)va_arg (ap, uint16_t);
      input[2]=(uint8_t)va_arg (ap, uint16_t);
      st_LIS3LsetAxis(input[0], input[1],input[2]);*/
      // NOTE:**:Evaluate arguments from RIGHT to LEFT !!!:-)
      st_LIS3LsetAxis((uint8_t)va_arg (ap, uint16_t),(uint8_t)va_arg (ap, uint16_t),(uint8_t)va_arg (ap, uint16_t));
      break;
   case SET_INTERRUPT_REG:
      st_LIS3LsetInterruptReg((uint8_t)va_arg (ap, uint16_t));
      break;
   case SET_CTRL_REGS:
      input[0]=(uint8_t)va_arg (ap, uint16_t);
      input[1]=(uint8_t)va_arg (ap, uint16_t);
      input[2]=(uint8_t)va_arg (ap, uint16_t);
      st_LIS3LwriteMultipleRegisters(0x20,3,input);
      break;
   default:
      return DEV_BAD_IOCTL;
   }

   return DEV_OK;
}

/** Enable of disable the I2C Bus
  *    
  * 
  *@param newMode holds the value to set or clear the I2C bus mode 
  * 
  */
uint8_t dev_mode_DEV_MSP_ACCELEROMETER(uint8_t newMode)
{
   switch (newMode) {
   case DEV_MODE_ON:
      init_i2c();
      break;
   case DEV_MODE_OFF:
      msp430_i2c_clearModeI2C();
      break;
   default:
      return DEV_UNSUPPORTED;
   }

   return DEV_OK;
}


#endif
