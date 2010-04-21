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


/** @file dev.h
 * @brief Device Layer generic routines.
 * @author Adam Torgerson
 * @author Modified: Brian Shucker
 * @author Updating Documentation: John Ledbetter
 * @date Modified 05/29/2007
 */

#ifndef _DEV_H_
#define _DEV_H_

#include <stdarg.h>
#include <stddef.h>
#include "mutex.h"

#ifdef SCONS
#include "optsconfig.h"
#endif


#ifdef PLATFORM_TELOSB
#include "plat_dep.h"
#endif

#ifdef PLATFORM_MICROBLAZE
#include "mb-adc-parallel.h"
#include "mb-dipsw.h"
#endif

typedef enum {
   DEV_ADC = 0,
#ifdef PLATFORM_MICROBLAZE
   DEV_ADC_PARALLEL,
   DEV_DIPSW,
#endif
   DEV_SPI,
   DEV_AVR_EEPROM,
   DEV_AVR_RSSI,
   DEV_AVR_I2C,
   DEV_MICA2_LIGHT,
   DEV_MICA2_TEMP,
   DEV_MICA2_MIC,
   DEV_MICA2_SOUNDER,
   DEV_MICA2_ACCEL_X,
   DEV_MICA2_ACCEL_Y,
   DEV_MICA2_MAGNET_X,
   DEV_MICA2_MAGNET_Y,
   DEV_MICA2_BATTERY,
   DEV_MICA2_ULTRASOUND,
   DEV_HARDWARE_ID,
   DEV_ATMEL_FLASH,
   DEV_MSP_ACCELEROMETER,
   DEV_MSP_TEMPERATURE,
   DEV_MSP_HUMIDITY,
   DEV_TELOS_FLASH,
   DEV_MSP_FLASH,
   DEV_MSP_ADC,
   DEV_MICA2_GPS,
   NUM_DEVICES
} dev_id_t;

/** @defgroup dev_drivers Device Drivers
 * This group contains all device drivers for the MANTIS OS.
 */
//@{

/****************************************************************\
 * Begin ADC Driver Interface
\****************************************************************/
/** @addtogroup driver_adc ADC
 * This group contains functions associated with the ADC Driver.
 * This applies to the MICAz, MICA2, and TELOSb platforms.
 */
/** @ingroup driver_adc
 * @internal Used by the dev_open and dev_close functions.
 */
extern mos_mutex_t adc_mutex;
/** @brief read from the ADC into buf.
 * @param count the number of bytes to read.  A value of '1' will
 * read a single byte from the ADC. Any other value will read a 16-bit value.
 * @param buf the buffer to store the results in.
 * @return the number of bytes read (either 1 or 2).
 * @ingroup driver_adc
 */
uint16_t dev_read_DEV_ADC(void *buf, uint16_t count);
/** @brief not supported for the ADC.
 * @param count ignored.
 * @param buf ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_adc
 */
uint16_t dev_write_DEV_ADC(const void *buf, uint16_t count);
/** @brief change the status of the ADC.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return the mode that the ADC is now in.
 * @ingroup driver_adc
 */
uint8_t dev_mode_DEV_ADC(uint8_t md);
/** @brief send an IO Control to the ADC.
 *  Accepted parameters are:
 *    -# ADC_SET_CHANNEL, [int channel]
 * @param request The requested action.
 * @param ... any required arguments for the specified IOCTL.
 * @return DEV_OK or DEV_BAD_IOCTL.
 * @ingroup driver_adc
 */
uint8_t dev_ioctl_DEV_ADC(int8_t request, ...);
/** @brief gain an exclusive lock on the ADC.  Any other threads
 *  calling dev_open_DEV_ADC() will block until the owner calls
 *  dev_close_DEV_ADC().
 * @ingroup driver_adc
 */
#define dev_open_DEV_ADC() mos_mutex_lock(&adc_mutex)
/** @brief release exclusive lock on the ADC.  Any other thread
 *  blocking on dev_open_DEV_ADC() will now unblock.
 * @ingroup driver_adc
 */
#define dev_close_DEV_ADC() mos_mutex_unlock(&adc_mutex)


/****************************************************************\
 * Begin SPI Bus Driver Interface
\****************************************************************/

/** @addtogroup driver_spi SPI Bus
 * This group contains functions associated with the SPI Bus driver.
 * Other drivers may use the SPI Bus to communicate with hardware located
 * on it, such as the CC2420 radio.
 * In general, application programmers should not need to use this device,
 * Unless a there is an unsupported device attached to the SPI Bus.
 */
/** @ingroup driver_spi
 * @internal Used by the dev_open and dev_close functions.
 */ 
extern mos_mutex_t spi_mutex;
/** @brief read from the SPI Bus into buf.
 * @param count the number of bytes to read.  A value of '2' will
 * read a 16-bit value; any other value will read an array of bytes.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read.
 * @ingroup driver_spi
 */
uint16_t dev_read_DEV_SPI(void *buf, uint16_t count);
/** @brief write from buf to the SPI Bus.
 * @param count the number of bytes to write.  A value of '2' will
 * write a 16-bit value; any other value will write an array of bytes.
 * @param buf the buffer containing the bytes to be written.
 * @return the number of bytes written.
 * @ingroup driver_spi
 */
uint16_t dev_write_DEV_SPI(const void *buf, uint16_t count);
/** @brief not supported for the SPI Bus.
 * @param md ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_spi
 */
uint8_t dev_mode_DEV_SPI(uint8_t md);
/** @brief send an IO Control to the SPI Bus.
 *  Accepted parameters are:
 *    -# SPI_SET_SLAVE_SELECT, [slave device]
 *    -# SPI_CLEAR_SLAVE_SELECT
 * @param request The requested action.
 * @param ... any required arguments for the specified IOCTL.
 * @return DEV_OK or DEV_BAD_IOCTL.
 * @ingroup driver_spi
 */
uint8_t dev_ioctl_DEV_SPI(int8_t request, ...);
/** @brief gain an exclusive lock on the SPI Bus.  Any other threads
 *  calling dev_open_DEV_SPI() will block until the owner calls
 *  dev_close_DEV_SPI().
 * @ingroup driver_spi
 */
#define dev_open_DEV_SPI() mos_mutex_lock(&spi_mutex)
/** @brief release exclusive lock on the SPI Bus.  Any other thread
 *  blocking on dev_open_DEV_SPI() will now unblock.
 * @ingroup driver_spi
 */
#define dev_close_DEV_SPI() mos_mutex_unlock(&spi_mutex)
/****************************************************************\
 * Begin AVR EEPROM Driver Interface
\****************************************************************/
/** @addtogroup driver_avr_eeprom AVR EEPROM
 * This group contains functions associated with the AVR EEPROM Driver.
 * Applications can use this device to read or write from the AVR EEPROM.
 * This applies to all AVR-based platforms (micaz/mica2/...)
 */
/** @ingroup driver_avr_eeprom
 * @internal Used by dev_open and dev_close.
 */
extern mos_mutex_t eeprom_mutex;
/** @brief read count bytes from the current eeprom address into buf.
 * @param count the number of bytes to read.  
 * @param buf the buffer to store the result in.
 * @return the number of bytes read.
 * @ingroup driver_avr_eeprom
 */
uint16_t dev_read_DEV_AVR_EEPROM(void *buf, uint16_t count);
/** @brief write from buf to the current eeprom address.
 * @param count the number of bytes to be written.
 * @param buf the buffer containing the bytes to be written.
 * @return the number of bytes written.
 * @ingroup driver_avr_eeprom
 */
uint16_t dev_write_DEV_AVR_EEPROM(const void *buf, uint16_t count);
/** @brief not supported for the AVR EEPROM.
 * @param md ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_avr_eeprom
 */
uint8_t dev_mode_DEV_AVR_EEPROM(uint8_t md);
/** @brief send an IO Control to the EEPROM Driver.
 *  Accepted parameters are:
 *    -# DEV_SEEK, [int16_t address]
 * @param request The requested action.
 * @param ... any required arguments for the specified IOCTL.
 * @return DEV_OK or DEV_BAD_IOCTL.
 * @ingroup driver_avr_eeprom
 */
uint8_t dev_ioctl_DEV_AVR_EEPROM(int8_t request, ...);
/** @brief gain an exclusive lock on the AVR EEPROM Driver.  Any other threads
 *  calling dev_open_DEV_AVR_EEPROM() will block until the owner calls
 *  dev_close_DEV_AVR_EEPROM().
 * @ingroup driver_avr_eeprom
 */
#define dev_open_DEV_AVR_EEPROM() mos_mutex_lock(&eeprom_mutex)
/** @brief release exclusive lock on the AVR EEPROM Driver.  Any other thread
 *  blocking on dev_open_DEV_AVR_EEPROM() will now unblock.
 * @ingroup driver_avr_eeprom
 */
#define dev_close_DEV_AVR_EEPROM() mos_mutex_unlock(&eeprom_mutex)
/****************************************************************\
 * Begin AVR RSSI Driver Interface
\****************************************************************/
/** @addtogroup driver_avr_rssi RSSI
 * This group contains functions associated with the RSSI Driver.
 * Applications may use this driver to read the Receive Strength
 * Signal Indicator from the CC1000 radio.
 * This applies to the mica2 and mica2dot platforms.
 */
/** @ingroup driver_avr_rssi
 * @internal Used by the dev_open and dev_close functions.
 */
extern mos_mutex_t rssi_mutex;
/** @brief read the RSSI value from the CC1000 radio int buf.
 * @param count the number of bytes to read.  This must be either
 * '1' to read an 8-bit value, or '2' to read a 16-bit value.
 * Any other value will return no data.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read.
 * @ingroup driver_avr_rssi
 */
uint16_t dev_read_DEV_AVR_RSSI(void *buf, uint16_t count);
/** @brief not supported for the AVR RSSI Driver.
 * @param md ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_avr_rssi
 */
uint16_t dev_write_DEV_AVR_RSSI(const void *buf, uint16_t count);
/** @brief change the status of the RSSI Driver.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_avr_rssi
 */
uint8_t dev_mode_DEV_AVR_RSSI(uint8_t md);
/** @brief not supported for the AVR RSSI Driver.
 * @param request ignored.
 * @param ... ignored.
 * @return DEV_BAD_IOCTL
 * @ingroup driver_avr_rssi
 */
uint8_t dev_ioctl_DEV_AVR_RSSI(int8_t request, ...);
/** @brief gain an exclusive lock on the AVR RSSI Driver.  Any other threads
 *  calling dev_open_DEV_AVR_RSSI() will block until the owner calls
 *  dev_close_DEV_AVR_RSSI().
 * @ingroup driver_avr_rssi
 */
#define dev_open_DEV_AVR_RSSI() mos_mutex_lock(&rssi_mutex)
/** @brief release exclusive lock on the AVR RSSI Driver.  Any other thread
 *  blocking on dev_open_DEV_AVR_RSSI() will now unblock.
 * @ingroup driver_avr_rssi
 */
#define dev_close_DEV_AVR_RSSI() mos_mutex_unlock(&rssi_mutex)

/****************************************************************\
 * Begin AVR I2C Bus Driver Interface
\****************************************************************/

/** @addtogroup driver_avr_i2c AVR I2C
 * This group contains functions associated with the I2C driver for the AVR architecture.
 * Other drivers may use this driver to communicate with hardware sitting on the
 * I2C bus, such as the weatherboard gps driver.
 * In general, application programmers should not need to use this device,
 * Unless a there is an unsupported device attached to the I2C Bus.
 */

/** @brief read from the I2C Bus into buf.
 * @param count the number of bytes to read.  A value of '2' will
 * read a 16-bit value; any other value will read an array of bytes.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read.
 * @ingroup driver_avr_i2c
 */
uint16_t dev_read_DEV_AVR_I2C(void *buf, uint16_t count);
/** @brief write from buf to the I2C Bus.
 * @param count the number of bytes to write. 
 * @param buf the buffer containing the bytes to be written.
 * @return the number of bytes written.
 * @ingroup driver_avr_i2c
 */
uint16_t dev_write_DEV_AVR_I2C(const void *buf, uint16_t count);
/** @brief change the status of the AVR I2C Driver.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_avr_i2c
 */
uint8_t dev_mode_DEV_AVR_I2C(uint8_t md);
/** @brief send an IO Control to the I2C Bus.
 *  Accepted parameters are:
 *    -# I2C_ENABLE_ACK
 *    -# I2C_DISABLE_ACK
 *    -# I2C_DEST_ADDR, [uint16_t address]
 *    -# I2C_SLAVE_ADDR, [uint16_t address]
 *    -# I2C_SET_BRR, [uint16_t baudrate]
 * @param request The requested action.
 * @param ... any required arguments for the specified IOCTL.
 * @return DEV_OK or DEV_BAD_IOCTL.
 * @ingroup driver_avr_i2c
 */
uint8_t dev_ioctl_DEV_AVR_I2C(int8_t request, ...);
/****************************************************************\
 * Begin MICA2 Light Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_light MICA Light Sensor
 * This group contains functions associated with the MICA Light Sensor.
 * This applies to the MICAz and MICA2 platforms.
 */
/** @ingroup driver_mica2_light
 * @internal Used by the dev_open and dev_close functions.
 */
extern mos_mutex_t lt_mutex;
/** @brief read from the Light sensor into buf.
 * @param count the number of bytes to read.  A value of '1' will
 * read an 8-bit value; a value of '2' will read a 16-bit value.
 * any other value will return DEV_UNSUPPORTED.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read. Must be either '1' or '2'.
 * @ingroup driver_mica2_light
 */
uint16_t dev_read_DEV_MICA2_LIGHT(void *buf, uint16_t count);
/** @brief not supported for the MICA2 Light driver.
 * @param buf ignored.
 * @param count ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_light
 */
uint16_t dev_write_DEV_MICA2_LIGHT(const void *buf, uint16_t count);
/** @brief change the status of the MICA2 Light Driver.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_mica2_light
 */
uint8_t dev_mode_DEV_MICA2_LIGHT(uint8_t md);
/** @brief not supported for the MICA2 Light driver.
 * @param request ignored.
 * @param ... ignored.
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_light
 */
uint8_t dev_ioctl_DEV_MICA2_LIGHT(int8_t request, ...);
/** @brief gain an exclusive lock on the MICA2 Light Driver.  Any other threads
 *  calling dev_open_DEV_MICA2_LIGHT() will block until the owner calls
 *  dev_close_DEV_MICA2_LIGHT().
 *  Note that because the MICA2 Light and MICA2 Temperature drivers
 *  share the same ADC line, an exclusive lock on the light driver
 *  will also prevent other threads from gaining a lock on the
 *  temperature driver.
 * @ingroup driver_mica2_light
 */
#define dev_open_DEV_MICA2_LIGHT() mos_mutex_lock(&lt_mutex)
/** @brief release exclusive lock on the MICA2 Light Driver.  Any other thread
 *  blocking on dev_open_DEV_MICA2_LIGHT() will now unblock.
 *  Note that because the MICA2 Light and MICA2 Temperature drivers
 *  share the same ADC line, an exclusive lock on the light driver
 *  will also prevent other threads from gaining a lock on the
 *  temperature driver.
 * @ingroup driver_mica2_light
 */
#define dev_close_DEV_MICA2_LIGHT() mos_mutex_unlock(&lt_mutex)
/****************************************************************\
 * Begin MICA Temperature Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_temp MICA Temperature Sensor
 * This group contains functions associated with the MICA Temperature Sensor.
 * This applies to the MICAz and MICA2 platforms.
 */
/** @brief read from the Temperature sensor into buf.
 * @param count the number of bytes to read.  A value of '1' will
 * read an 8-bit value; a value of '2' will read a 16-bit value.
 * any other value will return DEV_UNSUPPORTED.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read. Must be either '1' or '2'.
 * @ingroup driver_mica2_temp
 */
uint16_t dev_read_DEV_MICA2_TEMP(void *buf, uint16_t count);
/** @brief not supported for the MICA2 Temperature driver.
 * @param buf ignored.
 * @param count ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_temp
 */
uint16_t dev_write_DEV_MICA2_TEMP(const void *buf, uint16_t count);
/** @brief change the status of the MICA2 Temperature Driver.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_mica2_temp
 */
uint8_t dev_mode_DEV_MICA2_TEMP(uint8_t md);
/** @brief not supported for the MICA2 Temperature driver.
 * @param request ignored.
 * @param ... ignored.
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_temp
 */
uint8_t dev_ioctl_DEV_MICA2_TEMP(int8_t request, ...);
/** @brief gain an exclusive lock on the MICA2 Temperature Driver.
 *  Any other threads
 *  calling dev_open_DEV_MICA2_TEMP() will block until the owner calls
 *  dev_close_DEV_MICA2_TEMP().
 *  Note that because the MICA2 Light and MICA2 Temperature drivers
 *  share the same ADC line, an exclusive lock on the temperature driver
 *  will also prevent other threads from gaining a lock on the
 *  light driver.
 * @ingroup driver_mica2_temp
 */
#define dev_open_DEV_MICA2_TEMP() mos_mutex_lock(&lt_mutex)
/** @brief release exclusive lock on the MICA2 Temperature Driver.
 *  Any other thread
 *  blocking on dev_open_DEV_MICA2_TEMP() will now unblock.
 *  Note that because the MICA2 Light and MICA2 Temperature drivers
 *  share the same ADC line, an exclusive lock on the temperature driver
 *  will also prevent other threads from gaining a lock on the
 *  light driver.
 * @ingroup driver_mica2_temp
 */
#define dev_close_DEV_MICA2_TEMP() mos_mutex_unlock(&lt_mutex)
/****************************************************************\
 * Begin MICA Microphone Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_mic MICA Microphone
 * This group contains functions associated with the MICA Microphone.
 * This applies to the MICAz and MICA2 platforms.
 */
/** @internal Used by the dev_open and dev_close functions.
 * @ingroup driver_mica2_mic */
extern mos_mutex_t mic_mutex;
/** @brief read from the Microphone into buf.
 * @param count the number of bytes to read.  A value of '1' will
 * read an 8-bit value; a value of '2' will read a 16-bit value.
 * any other value will return DEV_UNSUPPORTED.
 * @param buf the buffer to store the result in.
 * @return the number of bytes read. Must be either '1' or '2'.
 * @ingroup driver_mica2_mic
 */
uint16_t dev_read_DEV_MICA2_MIC(void *buf, uint16_t count);
/** @brief not supported for the MICA2 Microphone.
 * @param buf ignored.
 * @param count ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_mic
 */
uint16_t dev_write_DEV_MICA2_MIC(const void *buf, uint16_t count);
/** @brief change the status of the MICA2 Microphone.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_mica2_mic
 */
uint8_t dev_mode_DEV_MICA2_MIC(uint8_t md);
/** @brief not supported for the MICA2 Microphone.
 * @param request ignored.
 * @param ... ignored.
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_mic
 */
uint8_t dev_ioctl_DEV_MICA2_MIC(int8_t request, ...);
/** @brief gain an exclusive lock on the Microphone.  Any other threads
 *  calling dev_open_DEV_MICA2_MIC() will block until the owner calls
 *  dev_close_DEV_MICA2_MIC().
 * @ingroup driver_mica2_mic
 */
#define dev_open_DEV_MICA2_MIC() mos_mutex_lock(&mic_mutex)
/** @brief release exclusive lock on the Microphone.  Any other thread
 *  blocking on dev_open_DEV_MICA2_MIC() will now unblock.
 * @ingroup driver_mica2_mic
 */
#define dev_close_DEV_MICA2_MIC() mos_mutex_unlock(&mic_mutex)
/****************************************************************\
 * Begin MICA Sounder Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_sounder MICA Sounder
 * This group contains functions associated with the MICA Sounder.
 * This applies to the MICAz and MICA2 platforms.
 */
/** @internal Used by the dev_open and dev_close functions.
 * @ingroup driver_mica2_sounder */
extern mos_mutex_t sounder_mutex;
/** @brief not supported for the MICA2 Sounder.
 * @param buf ignored
 * @param count ignored.
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_sounder
 */
uint16_t dev_read_DEV_MICA2_SOUNDER(void *buf, uint16_t count);
/** @brief Cause the sounder to be turned on or off.
 * If the value of buf[0] is 0x00, the sounder is turned off.
 * Any other value will turn it on.
 * @param buf a pointer to a single byte.
 * @param count ignored.
 * @return DEV_OK
 * @ingroup driver_mica2_sounder
 */
uint16_t dev_write_DEV_MICA2_SOUNDER(const void *buf, uint16_t count);
/** @brief not supported for the MICA2 Sounder.
 * @param md ignored
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_sounder
 */
uint8_t dev_mode_DEV_MICA2_SOUNDER(uint8_t md);
/** @brief not supported for the MICA2 Sounder.
 * @param request ignored
 * @param ... ignored.
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_sounder
 */
uint8_t dev_ioctl_DEV_MICA2_SOUNDER(int8_t request, ...);
/** @brief gain an exclusive lock on the Sounder.  Any other threads
 *  calling dev_open_DEV_MICA2_SOUNDER() will block until the owner calls
 *  dev_close_DEV_MICA2_SOUNDER().
 * @ingroup driver_mica2_sounder
 */
#define dev_open_DEV_MICA2_SOUNDER() mos_mutex_lock(&sounder_mutex)
/** @brief release exclusive lock on the Sounder.  Any other thread
 *  blocking on dev_open_DEV_MICA2_SOUNDER() will now unblock.
 * @ingroup driver_mica2_sounder
 */
#define dev_close_DEV_MICA2_SOUNDER() mos_mutex_unlock(&sounder_mutex)
/****************************************************************\
 * Begin MICA Accelerometer Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_accel MICA2 Accelerometer
 * This group contains functions associated with the MICA Accelerometer.
 * This applies to the MICA2 platform.
 */
/** @internal Used by the dev_open and dev_close functions.
 * @ingroup driver_mica2_accel */
extern mos_mutex_t accel_mutex;
/** @brief read the value of the accelerometer in the X direction.
 * @param buf a pointer to the buffer to hold the data.
 * @param count either '2' to read a 16-bit value, or '1' to read an
 * 8-bit value.  Any other value will return DEV_UNSUPPORTED.
 * @return the number of bytes read (1 or 2).
 * @ingroup driver_mica2_accel
 */
uint16_t dev_read_DEV_MICA2_ACCEL_X(void *buf, uint16_t count);
/** @brief not supported for the MICA2 Accelerometer.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_accel
 */
uint16_t dev_write_DEV_MICA2_ACCEL_X(const void *buf, uint16_t count);
/** @brief turn the MICA2 Accelerometer on or off.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md the new mode
 * @return DEV_OK or DEV_UNSUPPORTED
 * @ingroup driver_mica2_accel
 */
uint8_t dev_mode_DEV_MICA2_ACCEL_X(uint8_t md);
/** @brief not supported for the MICA2 Accelerometer.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_accel
 */
uint8_t dev_ioctl_DEV_MICA2_ACCEL_X(int8_t request, ...);
/** @brief gain an exclusive lock on the Accelerometer.  Any other threads
 *  calling dev_open_DEV_MICA2_ACCEL_[X/Y]() will block until the owner calls
 *  dev_close_DEV_MICA2_ACCEL_[X/Y]().
 * @ingroup driver_mica2_accel
 */
#define dev_open_DEV_MICA2_ACCEL_X() mos_mutex_lock(&accel_mutex)
/** @brief release exclusive lock on the Accelerometer.  Any other thread
 *  blocking on dev_open_DEV_MICA2_ACCEL_[X/Y]() will now unblock.
 * @ingroup driver_mica2_accel
 */
#define dev_close_DEV_MICA2_ACCEL_X() mos_mutex_unlock(&accel_mutex)
/** @brief read the value of the accelerometer in the Y direction.
 * @param buf a pointer to the buffer to hold the data.
 * @param count either '2' to read a 16-bit value, or '1' to read an
 * 8-bit value.  Any other value will return DEV_UNSUPPORTED.
 * @return the number of bytes read (1 or 2).
 * @ingroup driver_mica2_accel
 */
uint16_t dev_read_DEV_MICA2_ACCEL_Y(void *buf, uint16_t count);
/** @brief not supported for the MICA2 Accelerometer.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_accel
 */
uint16_t dev_write_DEV_MICA2_ACCEL_Y(const void *buf, uint16_t count);
/** @brief turn the MICA2 Accelerometer on or off.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md the new mode
 * @return DEV_OK or DEV_UNSUPPORTED
 * @ingroup driver_mica2_accel
 */
uint8_t dev_mode_DEV_MICA2_ACCEL_Y(uint8_t md);
/** @brief not supported for the MICA2 Accelerometer.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_accel
 */
uint8_t dev_ioctl_DEV_MICA2_ACCEL_Y(int8_t request, ...);
/** @brief gain an exclusive lock on the Accelerometer.  Any other threads
 *  calling dev_open_DEV_MICA2_ACCEL_[X/Y]() will block until the owner calls
 *  dev_close_DEV_MICA2_ACCEL_[X/Y]().
 * @ingroup driver_mica2_accel
 */
#define dev_open_DEV_MICA2_ACCEL_Y() mos_mutex_lock(&accel_mutex)
/** @brief release exclusive lock on the Accelerometer.  Any other thread
 *  blocking on dev_open_DEV_MICA2_ACCEL_[X/Y]() will now unblock.
 * @ingroup driver_mica2_accel
 */
#define dev_close_DEV_MICA2_ACCEL_Y() mos_mutex_unlock(&accel_mutex)

/****************************************************************\
 * Begin MICA Magnet Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_mag MICA2 Magnet
 * This group contains functions associated with the MICA Magnet
 * This applies to the MICAz/MICA2 platform.
 */
/** @internal Used by the dev_open and dev_close functions.
 * @ingroup driver_mica2_mag */
extern mos_mutex_t magnet_x_mutex;
/** @brief read data from the Magnet (X-direction) into buf.
 * @param count either '2' to read a 16-bit value, or '1' to read an
 * 8-bit value.  Any other value will return DEV_UNSUPPORTED.
 * @param buf the buffer to hold the data.
 * @return number of bytes read ('1' or '2')
 * @ingroup driver_mica2_mag
 */
uint16_t dev_read_DEV_MICA2_MAGNET_X(void *buf, uint16_t count);
/** @brief unsupported for the MICA2 Magnet Driver.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_mag
 */
uint16_t dev_write_DEV_MICA2_MAGNET_X(const void *buf, uint16_t count);
/** @brief turn the MICA2 Magnet on or off.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md the new mode
 * @return DEV_OK or DEV_UNSUPPORTED
 * @ingroup driver_mica2_mag
 */
uint8_t dev_mode_DEV_MICA2_MAGNET_X(uint8_t md);
/** @brief unsupported for the MICA2 Magnet Driver.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_mag
 */
uint8_t dev_ioctl_DEV_MICA2_MAGNET_X(int8_t request, ...);
/** @brief gain an exclusive lock on the Magnet.  Any other threads
 *  calling dev_open_DEV_MICA2_MAGNET_[X/Y]() will block until the owner calls
 *  dev_close_DEV_MICA2_MAGNET_[X/Y]().
 * @ingroup driver_mica2_mag
 */
#define dev_open_DEV_MICA2_MAGNET_X() mos_mutex_lock(&magnet_x_mutex)
/** @brief release exclusive lock on the Magnet.  Any other thread
 *  blocking on dev_open_DEV_MICA2_MAGNET_[X/Y]() will now unblock.
 * @ingroup driver_mica2_mag
 */
#define dev_close_DEV_MICA2_MAGNET_X() mos_mutex_unlock(&magnet_x_mutex)

/** @internal Used by the dev_open and dev_close functions.
 * @ingroup driver_mica2_mag */
extern mos_mutex_t magnet_y_mutex;
/** @brief read data from the Magnet (Y-direction) into buf.
 * @param count either '2' to read a 16-bit value, or '1' to read an
 * 8-bit value.  Any other value will return DEV_UNSUPPORTED.
 * @param buf the buffer to hold the data.
 * @return number of bytes read ('1' or '2')
 * @ingroup driver_mica2_mag
 */
uint16_t dev_read_DEV_MICA2_MAGNET_Y(void *buf, uint16_t count);
/** @brief unsupported for the MICA2 Magnet Driver.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED
 * @ingroup driver_mica2_mag
 */
uint16_t dev_write_DEV_MICA2_MAGNET_Y(const void *buf, uint16_t count);
/** @brief turn the MICA2 Magnet on or off.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md the new mode
 * @return DEV_OK or DEV_UNSUPPORTED
 * @ingroup driver_mica2_mag
 */
uint8_t dev_mode_DEV_MICA2_MAGNET_Y(uint8_t md);
/** @brief unsupported for the MICA2 Magnet Driver.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 * @ingroup driver_mica2_mag
 */
uint8_t dev_ioctl_DEV_MICA2_MAGNET_Y(int8_t request, ...);
/** @brief gain an exclusive lock on the Magnet.  Any other threads
 *  calling dev_open_DEV_MICA2_MAGNET_[X/Y]() will block until the owner calls
 *  dev_close_DEV_MICA2_MAGNET_[X/Y]().
 * @ingroup driver_mica2_mag
 */
#define dev_open_DEV_MICA2_MAGNET_Y() mos_mutex_lock(&magnet_y_mutex)
/** @brief release exclusive lock on the Magnet.  Any other thread
 *  blocking on dev_open_DEV_MICA2_MAGNET_[X/Y]() will now unblock.
 * @ingroup driver_mica2_mag
 */
#define dev_close_DEV_MICA2_MAGNET_Y() mos_mutex_unlock(&magnet_y_mutex)
/****************************************************************\
 * Begin MICA Battery Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_battery MICA2 Battery Driver
 * This group contains functions associated with reading the MICA2
 * Battery status.
 */
//@{
/** @internal Used by the dev_open and dev_close */
extern mos_mutex_t battery_mutex;

/** @brief read the current battery value.
 * @param buf the buffer to read the battery value into.
 * @param count the number of bytes to read.  This must be either
 * '1' to read an 8-bit value, or '2' to read a 16-bit value.
 * @return the number of bytes read.
 */
uint16_t dev_read_DEV_MICA2_BATTERY(void *buf, uint16_t count);
/** @brief not supported by the MICA2 Battery Driver.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED
 */
uint16_t dev_write_DEV_MICA2_BATTERY(const void *buf, uint16_t count);
/** @brief change the mode of the mica2 battery driver.
 * Accepted parameters are:
 *    -# DEV_MODE_ON
 *    -# DEV_MODE_OFF
 * @param md the new mode.
 * @return DEV_OK or DEV_UNSUPPORTED.
 */
uint8_t dev_mode_DEV_MICA2_BATTERY(uint8_t md);
/** @brief not supported by the MICA2 Battery Driver.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 */
uint8_t dev_ioctl_DEV_MICA2_BATTERY(int8_t request, ...);
/** @brief gain an exclusive lock on the Battery.  Any other threads
 *  calling dev_open_DEV_MICA2_BATTERY() will block until the owner calls
 *  dev_close_DEV_MICA2_BATTERY().
 */
#define dev_open_DEV_MICA2_BATTERY(void) mos_mutex_lock(&battery_mutex)
/** @brief release exclusive lock on the Battery.  Any other thread
 *  blocking on dev_open_DEV_MICA2_BATTERY() will now unblock.
 */
#define dev_close_DEV_MICA2_BATTERY(void) mos_mutex_unlock(&battery_mutex)
//@}

/****************************************************************\
 * Begin MICA Ultrasound Driver Interface
\****************************************************************/
/** @addtogroup driver_mica2_ultrasound MICA2 Ultrasound Driver
 * This group contains functions associated with the MICA2 Ultrasound
 */
//@{
/** @brief Not supported for the Ultrasound.
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED.
 */
uint16_t dev_read_DEV_MICA2_ULTRASOUND(void *buf, uint16_t count);
/** @brief Cause the ultarsound to be turned on or off.
 * If the value of buf[0] is 0x00, the sounder is turned off.
 * Any other value will turn it on.
 * @param buf a pointer to a single byte.
 * @param count ignored.
 * @return count
 */
uint16_t dev_write_DEV_MICA2_ULTRASOUND(const void *buf, uint16_t count);
/** @brief Not supported for the Ultrasound.
 * @param md ignored
 * @return DEV_UNSUPPORTED.
 */
uint8_t dev_mode_DEV_MICA2_ULTRASOUND(uint8_t md);
/** @brief Not supported for the Ultrasound.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 */
uint8_t dev_ioctl_DEV_MICA2_ULTRASOUND(int8_t request, ...);
//@}


/****************************************************************\
 * Begin DS2411 Hardware ID Interface
\****************************************************************/
/** @addtogroup driver_hardware_id Hardware ID Driver
 * This group contains functions to read the 48-bit hardware ID
 * From the avr-based nodes (mica2/micaz) as well as the TelosB.
 */
//@{
/** @internal used for dev_open and dev_close. */
extern mos_mutex_t id_mutex;
/** @brief read the 48-bit hardware ID from telosb or 
 * mica2/micaz nodes.
 * @param buf the buffer to hold the ID.
 * @param count the number of bytes to be read
 * @return the number of bytes read, 0-6.
 */
uint16_t dev_read_DEV_HARDWARE_ID(void *buf, uint16_t count);
/** @brief Not supported for the ID Driver..
 * @param buf ignored
 * @param count ignored
 * @return DEV_UNSUPPORTED.
 */
uint16_t dev_write_DEV_HARDWARE_ID(const void *buf, uint16_t count);
/** @brief Not supported for the ID Driver.
 * @param md ignored
 * @return DEV_UNSUPPORTED.
 */
uint8_t dev_mode_DEV_HARDWARE_ID(uint8_t md);
/** @brief Not supported for the ID Driver.
 * @param request ignored
 * @param ... ignored
 * @return DEV_BAD_IOCTL
 */
uint8_t dev_ioctl_DEV_HARDWARE_ID(int8_t request, ...);
#define dev_open_DEV_HARDWARE_ID(void) mos_mutex_lock(&id_mutex)
#define dev_close_DEV_HARDWARE_ID(void) mos_mutex_unlock(&id_mutex)
//@}

//@{
extern mos_mutex_t atmel_flash_mutex;
uint16_t dev_read_DEV_ATMEL_FLASH(void *buf, uint16_t count);
uint16_t dev_write_DEV_ATMEL_FLASH(const void *buf, uint16_t count);
uint8_t dev_mode_DEV_ATMEL_FLASH(uint8_t md);
uint8_t dev_ioctl_DEV_ATMEL_FLASH(int8_t request, ...);
#define dev_open_DEV_ATMEL_FLASH(void) mos_mutex_lock(&atmel_flash_mutex)
#define dev_close_DEV_ATMEL_FLASH(void) mos_mutex_unlock(&atmel_flash_mutex)
//@}

//@{
extern mos_mutex_t loopback_mutex;
uint16_t dev_read_DEV_LOOPBACK(void *buf, uint16_t count);
uint16_t dev_write_DEV_LOOPBACK(const void *buf, uint16_t count);
uint8_t dev_mode_DEV_LOOPBACK(uint8_t md);
uint8_t dev_ioctl_DEV_LOOPBACK(int8_t request, ...);
#define dev_open_DEV_LOOPBACK(void) mos_mutex_lock(&loopback_mutex)
#define dev_close_DEV_LOOPBACK(void) mos_mutex_unlock(&loopback_mutex)
//@}

//@{
extern mos_mutex_t msp_temp_mutex;
uint8_t dev_read_DEV_MSP_HUMIDITY(void *buf, uint16_t count);
uint8_t dev_write_DEV_MSP_HUMIDITY(const void *buf, uint16_t count);
uint8_t dev_mode_DEV_MSP_HUMIDITY(uint8_t md);
uint8_t dev_ioctl_DEV_MSP_HUMIDITY(int8_t request, ...);
#define dev_open_DEV_MSP_HUMIDITY(void) mos_mutex_lock(&msp_temp_mutex)
#define dev_close_DEV_MSP_HUMIDITY(void) mos_mutex_unlock(&msp_temp_mutex)
//@}

//@{
uint8_t dev_read_DEV_MSP_TEMPERATURE(void *buf, uint16_t count);
uint8_t dev_write_DEV_MSP_TEMPERATURE(const void *buf, uint16_t count);
#define dev_mode_DEV_MSP_TEMPERTURE(newMode)   dev_mode_DEV_MSP_HUMIDITY(newMode)
#define dev_ioctl_DEV_MSP_TEMPERATURE(request) dev_ioctl_DEV_MSP_HUMIDITY(request)
#define dev_open_DEV_MSP_TEMPERATURE(void) mos_mutex_lock(&msp_temp_mutex)
#define dev_close_DEV_MSP_TEMPERATURE(void) mos_mutex_unlock(&msp_temp_mutex)
//@}

//@{
extern mos_mutex_t msp_acc_mutex;
uint8_t dev_read_DEV_MSP_ACCELEROMETER(void *buf, uint16_t count);
uint8_t dev_write_DEV_MSP_ACCELEROMETER(const void *buf, uint16_t count);
uint8_t dev_ioctl_DEV_MSP_ACCELEROMETER(int8_t request, ...);
uint8_t dev_mode_DEV_MSP_ACCELEROMETER(uint8_t newMode);
#define dev_open_DEV_MSP_ACCELEROMETER(void) mos_mutex_lock(&msp_acc_mutex)
#define dev_close_DEV_MSP_ACCELEROMETER(void) mos_mutex_unlock(&msp_acc_mutex)
//@}

/** @addtogroup driver_telosf TELOSb External Flash
 * This group contains functions associated with the TELOSb External Flash
 * (ST M25P80)
 */
/** @internal used by the dev_open and dev_close functions.
 * @ingroup driver_telosf
 */
extern mos_mutex_t telos_flash_mutex;

/** @brief this attribute causes the function to be mirrored in RAM,
* allowing the function to be called even if the program flash has been erased.
* @ingroup driver_telosf
 */
#define TELOSB_RAMFUNC __attribute__((section (".data")))

/** @brief Read count number of bytes from the flash and store it in buf.
 * Note: This function is mirrored in RAM and can be called even after the
 * program flash has been erased.
 * @param count the number of bytes to read.
 * @param buf the buffer used to store the read data.
 * @return the number of bytes read.
 * @ingroup driver_telosf
 */
uint8_t TELOSB_RAMFUNC dev_read_DEV_TELOS_FLASH(void *buf, uint16_t count);
/** @brief Write count bytes from buf to the external flash.
 * @param count the number of bytes to write. The maximum number of bytes
 * That can be written at once is 256.
 * @param buf a pointer to the data to be written.
 * @return the number of bytes written. 
 * @ingroup driver_telosf
 */
uint8_t dev_write_DEV_TELOS_FLASH(const void *buf, uint16_t count);
/** @brief turn the flash on or off.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md the new mode
 * @return DEV_OK or DEV_UNSUPPORTED
 * @ingroup driver_telosf
 */
uint8_t dev_mode_DEV_TELOS_FLASH(uint8_t md);
/** @brief send an IO Control command to the flash driver.
 *  Accepted parameters are:
 *    -# DEV_SEEK, [uint32_t address]
 *    -# TELOS_FLASH_BULK_ERASE
 *    -# TELOS_FLASH_SECT_ERASE, [uint32_t address]
 *  Note: This function is mirrored in RAM and can be called even after the
 * program flash has been erased.
 * @param request the specified ioctl.
 * @param ... any data associated with the requested ioctl.
 * @return DEV_OK or DEV_BAD_IOCTL
 * @ingroup driver_telosf
 */
uint8_t TELOSB_RAMFUNC dev_ioctl_DEV_TELOS_FLASH(int8_t request, ...);
/** @brief gain an exclusive lock on the External Flash.  Any other threads
 *  calling dev_open_DEV_TELOS_FLASH() will block until the owner calls
 *  dev_close_DEV_TELOS_FLASH().
 * @ingroup driver_telosf
 */
#define dev_open_DEV_TELOS_FLASH(void) mos_mutex_lock(&telos_flash_mutex)
/** @brief release exclusive lock on the External Flash.  Any other thread
 *  blocking on dev_open_DEV_TELOS_FLASH() will now unblock.
 * @ingroup driver_telosf
 */
#define dev_close_DEV_TELOS_FLASH(void) mos_mutex_unlock(&telos_flash_mutex)

//@{
uint8_t  dev_read_DEV_MSP_FLASH(void *buf, uint16_t count);
uint8_t  TELOSB_RAMFUNC dev_write_DEV_MSP_FLASH(const void *buf, uint16_t count);
uint8_t  TELOSB_RAMFUNC dev_ioctl_DEV_MSP_FLASH(int8_t request, ...);
#define dev_open_DEV_MSP_FLASH(void) 
#define dev_close_DEV_MSP_FLASH(void)
//@}

/****************************************************************\
 * Begin GPS Driver Interface
\****************************************************************/

/** @addtogroup driver_gps Weatherboard GPS
 * This group contains functions associated with the GPS Driver.
 * The Leadtek GPS is found on the Mica Weatherboard, and can be used
 * With either the MICA2 or MICAZ.
 */
/** @ingroup driver_gps
 * @internal Used by dev_open and dev_close.
 */
extern mos_mutex_t mica2_gps_mutex;
/** @brief read data from the GPS into buf.
 * This data can be in either string form or as a gps_gga_t structure.
 * This is configurable via dev_ioctl_DEV_MICA2_GPS().
 * @param buf the buffer to read data into.
 * @param count the size of the buffer.
 * @return the number of bytes read.
 * @ingroup driver_gps
 */
uint16_t dev_read_DEV_MICA2_GPS(void *buf, uint16_t count);
/** @brief write from buf to the Leadtek GPS device.
 * @param count the number of bytes to write.
 * @param buf the buffer containing the bytes to be written.
 * @return DEV_OK
 * @ingroup driver_gps
 */
uint16_t dev_write_DEV_MICA2_GPS(const void *buf, uint16_t count);
/** @brief change the status of the GPS.
 *  Accepted parameters are:
 *    -# DEV_MODE_OFF: Turn the device off.
 *    -# DEV_MODE_ON:  Turn the device on.
 * @param md The mode you wish to change to.
 * @return DEV_OK or DEV_UNSUPPORTED.
 * @ingroup driver_gps
 */
uint8_t dev_mode_DEV_MICA2_GPS(uint8_t md);
/** @brief send an IO Control to the GPS Driver.
 *  Accepted parameters are:
 *    -# MICA2_GPS_READ_GGA
 *    -# MICA2_GPS_READ_STRING
 * @param request The requested action.
 * @param ... any required arguments for the specified IOCTL.
 * @return DEV_OK or DEV_BAD_IOCTL.
 * @ingroup driver_gps
 */
uint8_t dev_ioctl_DEV_MICA2_GPS(int8_t request, ...);
/** @brief gain an exclusive lock on the GPS device.  Any other threads
 *  calling dev_open_DEV_GPS() will block until the owner calls
 *  dev_close_DEV_GPS().
 * @ingroup driver_gps
 */
#define dev_open_DEV_MICA2_GPS(void) mos_mutex_lock(&mica2_gps_mutex)
/** @brief release exclusive lock on the GPS device.  Any other thread
 *  blocking on dev_open_DEV_MICA_GPS() will now unblock.
 * @ingroup driver_gps
 */
#define dev_close_DEV_MICA2_GPS(void) mos_mutex_unlock(&mica2_gps_mutex)
/****************************************************************/




/** @name Device Driver Macros
 * These macros alias functions of the type dev_foo(DEVICE, ...)
 * to dev_foo_DEVICE(...), allowing code to be written either way. */
//@{
/** @brief Reads count bytes from a device and stores the result in the 
 * provided buffer.  */
#define dev_read(dev,buf,count) dev_read_##dev(buf, count)

/** @brief Writes count bytes to a device from the provided buffer. */
#define dev_write(dev,buf,count) dev_write_##dev(buf, count)

/** @brief Set the mode of a device. */
#define dev_mode(dev,mode) dev_mode_##dev(mode)

/** @brief A pass-through to send arbitrary commands
 * to a device (meaning is defined by device driver).
 * This macro means that dev_ioctl(DEVICE_NAME, request, ...) is equivalent
 * to dev_ioctl_DEVICE_NAME(request, ...), so either format may be used.
*/
#define dev_ioctl(dev,request,args...) dev_ioctl_##dev(request, ##args)

/** @brief Gain an exclusive lock on a device driver. */
#define dev_open(dev) dev_open_##dev()
/** @brief Release an exclusive lock on a device driver. */
#define dev_close(dev) dev_close_##dev()
//@}


/** @name Device Mode Constants
 * These constants can be used as paramenters to any dev_mode function.
 */
//@{
/** @brief Power down the device. */
#define DEV_MODE_OFF     0 
/** @brief Put the device in low power mode, if applicable. */
#define DEV_MODE_IDLE    1 
/** @brief Turn on the device. */
#define DEV_MODE_ON      2 
//@}

/** @name Device IOCTL Constants
 * These constants can be used as parameters to some dev_ioctl functions,
 * depending on the specific device.  For information on what IOCTLS a
 * driver will accept, check the documentation for that driver. */
//@{
/** @brief Seek the underlying device pointer. */ 
#define DEV_SEEK 1
/** @brief Flush any buffered data to the device. */
#define DEV_FLUSH 2
/** @brief Lock the device. */
#define DEV_LOCK 3
/** @brief Unlock the device. */
#define DEV_UNLOCK 4
//@}

/** @name Device Return Constants
 * These constants may be returned from various dev_ functions to indicate
 * The result of an operation.
 */
//@{
/** @brief operation succeeded. */
#define DEV_OK 0
/** @brief operation is not supported. */
#define DEV_UNSUPPORTED 100
/** @brief device driver not registered (this constant is no longer used) */
#define DEV_NOT_REGISTERED 101
/** @brief an argument was not in the expected range. */
#define DEV_OUT_OF_RANGE 102
/** @brief device does not support the specified IOCTL. */
#define DEV_BAD_IOCTL 103
/** @brief device does not support the specified mode. */
#define DEV_BAD_MODE 104
/** @brief operation failed to complete. */
#define DEV_FAILURE 105
//@}

//@}
#endif
