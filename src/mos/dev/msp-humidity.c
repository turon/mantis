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

/** @file msp-humidity.c
 * @brief SHT11 device driver implementation
 * @author John Ledbetter
 * @date 07/12/2005
 */
#include "mos.h"

// TODO: any subsequent call to read_sr() after a write_sr() returns 0x07,
// no matter what was supposed to be written -- this means that any dev_ioctl()
// call will turn on the heater, disable OTP reloads, and set low resolution mode.
#if defined(PLATFORM_TELOSB)

#include "bitops.h"
#include "msp-humidity.h"
#include "dev.h"
#include "clock.h"

#define DATA_AS_OUT() SET_DIR_1(SHT11_DATA)
#define DATA_AS_IN()  UNSET_DIR_1(SHT11_DATA)

#define SCLK_HI()     SET_PORT_1(SHT11_SCLK)
#define SCLK_LO()     UNSET_PORT_1(SHT11_SCLK)

#define DATA_HI()     SET_PORT_1(SHT11_DATA)
#define DATA_LO()     UNSET_PORT_1(SHT11_DATA)

static uint8_t mode;

mos_mutex_t msp_temp_mutex;

// shut up about implicit declarations
uint8_t dev_mode_DEV_MSP_HUMIDITY (uint8_t newMode);
   
/** @brief transmit a byte MSB first to the SHT11 */
static inline void sht11_send_byte(uint8_t byte)
{
   int i;
   for(i = 7; i >= 0; --i)
   {
      if (byte & (1 << i))
	 DATA_HI();
      else
	 DATA_LO();
      
      SCLK_HI();
      
      SCLK_LO();
   }
}

/** @brief receives one byte of data from the SHT11*/
static uint8_t sht11_recv_byte(void)
{
   uint8_t result = 0;
   
   DATA_AS_IN();
   
   int i;
   for(i = 7; i >= 0; --i)
   {
      SCLK_HI();
         
      if (PORT_IN(SHT11_DATA_PORT) & (1 << SHT11_DATA_PIN))
	 result |= (1 << i);

      SCLK_LO();
   }

   return result;
}

/** @brief reset the comm. interface -- status register remains unchanged */  
static inline void sht11_reset_interface(void)
{
   DATA_AS_OUT();
   DATA_HI();
   SCLK_LO();
   
   int i;
   for(i = 0; i < 9; ++i)
   {
      SCLK_HI();
      SCLK_LO();
   }

   // must wait 11 ms after resetting the interface to send another command
   mos_mdelay(11);
}

/** @brief send the transmission start sequence */
static inline void sht11_transmission_start(void)
{
   DATA_AS_OUT();

   DATA_HI();
   SCLK_LO();
   SCLK_HI();
   DATA_LO(); // falling edge on DATA
   SCLK_LO(); // low pulse on SCLK
   SCLK_HI();
   DATA_HI(); // rising edge on DATA
   SCLK_LO();
}

/** @brief returns TRUE if the SHT11 pulls data low to ack. */
static inline uint8_t sht11_recv_ack(void)
{
   uint8_t i;
   DATA_AS_IN();
   DATA_HI();
   
   SCLK_HI();
   i = ((PORT_IN(SHT11_DATA_PORT) & (1 << SHT11_DATA_PIN)) ? FALSE : TRUE);
   SCLK_LO();
   
   return i;
}

/** @brief pulls data low and pulses SCLK to ack reception of a byte from the SHT11 */
static inline void sht11_send_ack(void)
{
   DATA_AS_OUT();
   DATA_LO();

   SCLK_HI();
   SCLK_LO();
}

/** @brief sends a command to the SHT11 and returns the result, or 0xFFFF on failure */
static uint16_t sht11_perform_cmd(uint8_t command)
{
   
   uint16_t result = 0;
   sht11_transmission_start();
   sht11_send_byte(command);
   if (!sht11_recv_ack())
   {
      // return failure
      return (uint16_t)-1;
   }
   

   // process the response
   switch(command)
   {
   case SHT11_CMD_READ_TEMP:
   case SHT11_CMD_READ_HUMIDITY:
      // wait for measurement to complete     
      while(PORT_IN(SHT11_DATA_PORT) & (1 << SHT11_DATA_PIN));

      result |= ((uint16_t)sht11_recv_byte()) << 8;
      sht11_send_ack();
      result |= sht11_recv_byte();
      break;
   case SHT11_CMD_READ_SR:
      result = sht11_recv_byte();
      break;
   default:
      // write sr & soft reset do not return data
      break;
   }

   return result;
   
}

/** @brief disables power to the SHT11 */
static inline void sht11_power_off(void)
{
   UNSET_PORT_1(SHT11_POWER);
}

/** @brief enables power to the SHT11 */
static inline void sht11_power_on(void)
{
   SET_PORT_1(SHT11_POWER);
}

/** @brief performs a read status register command */
static inline uint8_t sht11_read_sr(void)
{
   return (uint8_t)sht11_perform_cmd(SHT11_CMD_READ_SR);
}

/** @brief performs a write status register command */
static uint8_t sht11_write_sr(uint8_t new_sr)
{
//#warning "this function is broken!"
//   return;
   
   sht11_transmission_start();
   sht11_send_byte(SHT11_CMD_WRITE_SR);
   if (!sht11_recv_ack())
   {
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_OFF);
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_ON);
      return FALSE;
   }
   
   sht11_send_byte(new_sr);
   if (!sht11_recv_ack())
   {
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_OFF);
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_ON);
      return FALSE;
   }
   
   return TRUE;
}


/** @brief initializes the SHT11 pins with power off */ 
void sht11_init(void)
{
   mode = DEV_MODE_OFF;
   
   // do not power the sensor yet.
   SET_DIR_1(SHT11_POWER);
   sht11_power_off();
   
   SET_DIR_1(SHT11_SCLK);
   SCLK_LO();
   DATA_AS_IN();
   DATA_HI();

   mos_mutex_init(&msp_temp_mutex);
}


/** @brief returns a uint16_t in buf containing the measured humidity */
uint8_t dev_read_DEV_MSP_HUMIDITY (void *buf, uint16_t count)
{
   if (mode == DEV_MODE_OFF)
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_ON);
      
   // require at least two bytes
   if (count < 2)
      return DEV_OUT_OF_RANGE;
   
   *((uint16_t*)buf) = sht11_perform_cmd(SHT11_CMD_READ_HUMIDITY);
   return 2;
}

/** @brief returns a uint16_t in buf containing the measured temperature */
uint8_t dev_read_DEV_MSP_TEMPERATURE (void *buf, uint16_t count)
{
   if (mode == DEV_MODE_OFF)
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_ON);
   
   if (count < 2)
      return DEV_OUT_OF_RANGE;
   
   *((uint16_t*)buf) = sht11_perform_cmd(SHT11_CMD_READ_TEMP);
   return 2;
}

/** @brief unsupported: status register bits can be set via the dev_ioctl function */
uint8_t dev_write_DEV_MSP_HUMIDITY (const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

/** @brief unsupported: status register bits can be set via the dev_ioctl function */
uint8_t dev_write_DEV_MSP_TEMPERATURE (const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}


/** @brief turn the SHT11 heater on or off as well as enable or disable low resolution mode */
uint8_t dev_ioctl_DEV_MSP_HUMIDITY (int8_t request, ...)
{

   return DEV_UNSUPPORTED;
   
   if (mode == DEV_MODE_OFF)
      dev_mode_DEV_MSP_HUMIDITY(DEV_MODE_ON);
   
   uint8_t tmp = sht11_read_sr();
   
   if (request & SHT11_LOW_RES_ON)
      tmp |= SHT11_SR_LOW_RES;
   else if (request & SHT11_LOW_RES_OFF)
      tmp &= ~SHT11_SR_LOW_RES;

   if (request & SHT11_HEATER_ON)
      tmp |= SHT11_SR_HEATER;
   else if (request & SHT11_HEATER_OFF)
      tmp &= ~SHT11_SR_HEATER;

   if (request & SHT11_RELOAD_ON)
      tmp &= ~SHT11_SR_RELOAD;
   else if (request & SHT11_RELOAD_OFF)
      tmp |= SHT11_SR_RELOAD;

   tmp &= 0x07;
   sht11_write_sr(tmp);

   tmp = sht11_read_sr();
   
   
   printf("finally, sr = %x\n", (int)tmp);
   
   return DEV_OK;
}

/** @brief turn the sensor on or off */
uint8_t dev_mode_DEV_MSP_HUMIDITY (uint8_t newMode)
{
   switch(newMode) {
   case DEV_MODE_ON:
   case DEV_MODE_IDLE:
      sht11_power_on();
      sht11_reset_interface();
      if (sht11_perform_cmd(SHT11_CMD_SOFT_RESET) == (uint16_t)-1)
	 return DEV_FAILURE;
      break;

   case DEV_MODE_OFF:
      sht11_power_off();
      break;

   default:
      return DEV_UNSUPPORTED;
      break;
   }

   mode = newMode;
   return DEV_OK;
}

#endif
