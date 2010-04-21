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

#include "mos.h"

#include "msched.h"
#include "dev.h"
#include "clock.h"
#include "led.h"
#include "printf.h"
#include "mutex.h"

#if defined(HARDWARE_ID) || !defined(SCONS)

#if defined(ARCH_AVR)
/** @brief One wire directional port. */
#define OW_DIR_PORT DDRA
/** @brief One wire port. */
#define OW_PORT     PORTA
/** @brief One wire pin mask */
#define OW_PIN_MASK 0x10
/** @brief One wire pin */
#define OW_PIN      PINA

#elif defined(PLATFORM_TELOSB)

#define OW_DIR_PORT P2DIR
#define OW_PORT     P2OUT
#define OW_PIN_MASK (1 << 4)
#define OW_PIN      P2IN

#endif

/**@brief These are the delays defined in the appnotes */
#define DELAY_A 6
#define DELAY_B 64
#define DELAY_C 60
#define DELAY_D 10
#define DELAY_E 9
#define DELAY_F 55
#define DELAY_H 480
#define DELAY_I 70
#define DELAY_J 410

/** @brief Sets the one wire port to input */
#define ow_input() (OW_DIR_PORT &= ~OW_PIN_MASK)
/** @brief Sets the one wire port to output */
#define ow_output() (OW_DIR_PORT |= OW_PIN_MASK)
/** @brief Clears a bit on the one wire port */
#define ow_low() (OW_PORT &= ~OW_PIN_MASK)
/** @brief Sets a bit on the one wire port */
#define ow_high() (OW_PORT |= OW_PIN_MASK)

mos_mutex_t id_mutex;

/** @brief Read a bit from the one wire interface */
#define ow_read() (OW_PIN & OW_PIN_MASK)

/** @brief Drive the one wire interface low */
#define ow_drive() do {				\
      ow_output();				\
      ow_low();					\
   } while (0)

/** @brief Command function for input on the mica2. */
#define ow_release() do {			\
      ow_high();				\
      ow_input();				\
   } while (0)

/** @brief The read command. */
#define OW_READ_ROM 0x33
/** @brief 48 bit ID. */
#define OW_ID_LEN 6

static uint8_t id[OW_ID_LEN];

static uint8_t id_read(void);
static uint8_t reset(void);
static inline uint8_t read_byte(void);
static inline void write_byte(uint8_t byte);

void hardware_id_init(void)
{
   mos_mutex_init(&id_mutex);
   
   // Turn on the internal pullup to supply current
   ow_input();
   ow_high();

   // Get the id once and sotre in a buffer
   id_read();
   
   // Tri-state the pin by default
   ow_input();
   ow_low();
}

/** @brief Read the mica2 id.
 * @param buf id buffer
 * @param count id character count
 */
uint16_t dev_read_DEV_HARDWARE_ID(void *buf, uint16_t count)
{
   uint8_t i;
   uint8_t *ptr = (uint8_t *)buf;

   for (i = 0; i < OW_ID_LEN && i < count; i++)
      ptr[i] = id[i];

   return i;
}

uint16_t dev_write_DEV_HARDWARE_ID(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_mode_DEV_HARDWARE_ID(uint8_t md)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_HARDWARE_ID(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}

/*** Functions private to this file. ***/

static uint8_t id_read(void)
{
   uint8_t i = 0;
   uint8_t int_handle;

   int_handle = mos_disable_ints();

   ow_input();
   ow_high();
   
   if(reset()) {
      // reset condition didn't occur properly
      return 0;
   }
   
   mos_udelay(DELAY_H);
   write_byte(OW_READ_ROM);

   read_byte();  // Family code
   
   for(i = 0; i < OW_ID_LEN; i++)
      id[i] = read_byte();

   read_byte();  // CRC byte

   ow_input();
   ow_low();
   
   mos_enable_ints(int_handle);

   return OW_ID_LEN;
}

static uint8_t reset(void)
{
   uint8_t result;

   // would normally wait DELAY_G here but it is 0, so...
   ow_drive();
   mos_udelay(DELAY_H);
   ow_release();
   mos_udelay(DELAY_I);
   result = ow_read();
   mos_udelay(DELAY_J);
   
   return result;
}

static uint8_t read_bit(void)
{
   uint8_t result;

   ow_drive();
   mos_udelay(DELAY_A);
   ow_release();
   mos_udelay(DELAY_E);
   
   result = ow_read();

   mos_udelay(DELAY_F);
   return result;
}

static uint8_t read_byte(void)
{
   uint8_t i;
   uint8_t val;

   val = 0;
   
   for(i = 0; i < 8; i ++) {
      val >>= 1;
      if(read_bit())
         val |= 0x80;
   }
   
   return val;
}

static inline void write_bit(uint8_t bit)
{
   if(bit) {
      ow_drive();
      mos_udelay(DELAY_A);
      ow_release();
      mos_udelay(DELAY_B);
   } else {
      ow_drive();
      mos_udelay(DELAY_C);
      ow_release();
      mos_udelay(DELAY_D);
   }
}

static inline void write_byte(uint8_t byte)
{
   uint8_t i;

   for(i = 0; i < 8; i ++) {
      write_bit(byte & 0x01);
      byte >>= 1;
   }
}


#endif
