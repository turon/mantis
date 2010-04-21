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

/**************************************************************************/
/* File:    node_id.c                                                     */
/* Author      Charles Gruenwald III   :  gruenwal@colorado.edu           */
/*   Date:  04/14/2004                                                    */
/*                                                                        */
/* Description: this uses the eeprom to get and set the node id.          */
/**************************************************************************/

#include "mos.h"

#include "dev.h"
#if defined(ARCH_AVR)
#include "avr-eeprom.h"
#elif defined(ARCH_MSP430)
#include "telos-flash.h"
#endif

static uint16_t global_node_id = -1;

void mos_node_id_init(void)
{
#ifdef ARCH_AVR
   dev_open(DEV_AVR_EEPROM);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 0);
   dev_read(DEV_AVR_EEPROM, (uint8_t *)&global_node_id, sizeof(global_node_id));
   dev_close(DEV_AVR_EEPROM);
#elif defined(ARCH_MSP430)
   
   dev_open(DEV_TELOS_FLASH);
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, (uint32_t)0);
   dev_read(DEV_TELOS_FLASH, (uint8_t *)&global_node_id, sizeof(global_node_id));
   dev_close(DEV_TELOS_FLASH);
   
#else
   global_node_id = -1;
#endif 
}

uint16_t mos_node_id_get(void)
{
   return global_node_id;
}

void mos_node_id_set (uint16_t new_node_id)
{
   
#ifdef ARCH_AVR
   dev_open(DEV_AVR_EEPROM);
   dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, 0);
   dev_write(DEV_AVR_EEPROM, (uint8_t *)&new_node_id, sizeof(new_node_id));
   dev_close(DEV_AVR_EEPROM);
#elif defined(ARCH_MSP430)
   dev_open(DEV_TELOS_FLASH);

   dev_mode(DEV_TELOS_FLASH, DEV_MODE_ON);
   
   dev_ioctl(DEV_TELOS_FLASH, TELOS_FLASH_SECT_ERASE, (uint32_t)0);
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, (uint32_t)0);
   dev_write(DEV_TELOS_FLASH, (uint8_t *)&new_node_id, sizeof(new_node_id));
   dev_close(DEV_TELOS_FLASH);

   dev_mode(DEV_TELOS_FLASH, DEV_MODE_OFF);


   
#endif
   global_node_id = new_node_id;
}

