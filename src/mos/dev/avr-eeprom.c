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
  File: eeprom.c
  Author: Jeff Rose
  Date: 2-17-03

  Edited 04-06-2004 by Adam Torgerson
  -Added device layer support

  EEPROM driver for both the onboard 4k.
*/

#include "mos.h"

#ifdef ARCH_AVR

#include "avr-eeprom.h"
#include "sem.h"
#include "dev.h"
#include "mutex.h"

/* Define some states so we know what to do in the interrupt handlers. */
/** @brief Define for EEPROM write state
 */
#define EEPROM_WRITE   0
/** @brief Define for EEPROM read state
 */
#define EEPROM_READ    1

static mos_sem_t eeprom_sem;
mos_mutex_t eeprom_mutex;

static uint8_t eeprom_state;
static uint8_t *eeprom_data;
static uint16_t eeprom_index;
static uint16_t eeprom_size;
static uint16_t eeprom_addr;

static uint16_t dev_eeprom_addr;

/** @brief Read from the current eeprom address into buf for count bytes
 */
uint16_t dev_read_DEV_AVR_EEPROM(void *buf, uint16_t count)
{
   //set up state for interrupt state machine
   eeprom_state = EEPROM_READ;
   eeprom_data = (uint8_t *)buf;
   eeprom_addr = dev_eeprom_addr;
   eeprom_size = count;
   eeprom_index = 0;

   EECR |= (1 << EERIE); // Enable the eeprom ready interrupt

   //wait until state machine posts semaphore
   mos_sem_wait(&eeprom_sem);

   //increment address
   dev_eeprom_addr += count;

   return count;
}

/** @brief Write from buf to the current eeprom address for count bytes
 */
uint16_t dev_write_DEV_AVR_EEPROM(const void *buf, uint16_t count)
{
   //set up state for interrput state machine
   eeprom_state = EEPROM_WRITE;
   eeprom_data = (uint8_t *)buf;
   eeprom_addr = dev_eeprom_addr;
   eeprom_size = count;
   eeprom_index = 0;

   // Enable the eeprom ready interrupt
   EECR |= (1 << EERIE);

   //wait until state machine posts semaphore
   mos_sem_wait(&eeprom_sem);

   //increment address
   dev_eeprom_addr += count;

   return count;
}

uint8_t dev_ioctl_DEV_AVR_EEPROM(int8_t request, ...)
{
   int16_t i;
   va_list ap;
   
   va_start (ap, request);
   
   switch (request) {
   case DEV_SEEK:
      i = va_arg (ap, int16_t);
      dev_eeprom_addr = i;
      break;
   default:
      return DEV_BAD_IOCTL;
      break;
   }

   va_end (ap);
   
   return DEV_OK;
}

uint8_t dev_mode_DEV_AVR_EEPROM(uint8_t md)
{
   return DEV_UNSUPPORTED;
}

void avr_eeprom_init(void)
{
   //init state vars
   mos_sem_init(&eeprom_sem, 0);
   mos_mutex_init (&eeprom_mutex);
}

/** @brief Interrupt handler for the on-chip eeprom operations.
 */
SIGNAL(SIG_EEPROM_READY)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   if(eeprom_state == EEPROM_WRITE) {
      if(eeprom_index < eeprom_size) {
	 EEAR = eeprom_addr + eeprom_index;  // Set the address register
	 EEDR = eeprom_data[eeprom_index++]; // Fill the data register
	 
	 // Now execute the write sequence
	 EECR = (1 << EEMWE) | (1 << EERIE);
	 EECR = (1 << EEMWE) | (1 << EERIE) | (1 << EEWE);
      } else {
	 // Disable this interrupt
	 EECR &= ~(1 << EERIE); 
	 mos_sem_post(&eeprom_sem); // Wake the thread that called write
      }
   } else { // EEPROM_READ
      if(eeprom_index < eeprom_size) {
	 EEAR = eeprom_addr + eeprom_index; // Setup the address
	 EECR |= (1 << EERE); // Set the read strobe
	 eeprom_data[eeprom_index++] = EEDR; // Now read the data
      } else {
	 EECR &= ~(1 << EERIE); // Disable this interrupt
	 // Wake the thread that called read
	 mos_sem_post(&eeprom_sem);
      }
   }
}

#endif
