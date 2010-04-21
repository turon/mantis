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

/** @file xmos-flash.c
 * @brief External 500k flash memory
 */
 
/* This device exists to provide the same interface in XMOS that atmel-flash.c
 * provides on the node.  It does not attempt to exactly simulate
 * the behavior of the atmel flash driver.
 */

#include "mos.h"

/* This stuff only works in XMOS. */
#ifdef PLATFORM_LINUX

#include <stdio.h>
#include <unistd.h>

#include "mutex.h"
#include "xmos-flash.h"
#include "dev.h"

#define DEFAULT_FLASH_SIZE	(512*1024)

static mos_mutex_t xmos_flash_mutex;
static uint32_t xmos_flash_addr;

static FILE* flash_file = NULL;
static char* flash_file_name = NULL;
static uint32_t flash_size;

// storage for default file name
static char namebuf[11] = "0000.flash";

static void lazyInit();

uint8_t dev_mode_DEV_ATMEL_FLASH (uint8_t mode)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_ATMEL_FLASH (int8_t request, ...)
{
   if (!flash_file_name) return DEV_NOT_REGISTERED;
   if (!flash_file) lazyInit();
	
   uint32_t arg;
   va_list ap;

   mos_mutex_lock (&xmos_flash_mutex);
   
   va_start (ap, request);
   switch (request) {
   case DEV_SEEK:
      arg = va_arg (ap, uint32_t);
      xmos_flash_addr = arg;
      //printf("xmos_flash_addr = %i;\n", xmos_flash_addr);
      break;
   default:
      return DEV_BAD_IOCTL;      
   }
   va_end (ap);

   mos_mutex_unlock (&xmos_flash_mutex);
   
   return DEV_OK;
}

/** @brief Read from the current flash address into p, for count bytes
 */
uint16_t dev_read_DEV_ATMEL_FLASH (void *p, uint16_t count)
{
   if (!flash_file_name) return DEV_NOT_REGISTERED;
   if (xmos_flash_addr+count > flash_size) return DEV_OUT_OF_RANGE;
   if (!flash_file) lazyInit();

   mos_mutex_lock (&xmos_flash_mutex);
	
   fseek(flash_file, xmos_flash_addr, SEEK_SET);
   uint16_t len = fread(p, 1, count, flash_file);
   xmos_flash_addr += len;
	
   mos_mutex_unlock (&xmos_flash_mutex);
   
   return len;
}

/** @brief Write p into the current flash address, for count bytes
 */
uint16_t dev_write_DEV_ATMEL_FLASH (const void *p, uint16_t count)
{
   if (!flash_file_name) return DEV_NOT_REGISTERED;
   if (xmos_flash_addr+count > flash_size) return DEV_OUT_OF_RANGE;
   if (!flash_file) lazyInit();
	
   mos_mutex_lock (&xmos_flash_mutex);
	
   fseek(flash_file, xmos_flash_addr, SEEK_SET);
   uint16_t len = fwrite(p, 1, count, flash_file);
   fflush(flash_file);
   xmos_flash_addr += len;
	
   mos_mutex_unlock (&xmos_flash_mutex);
   
   return len;
}

/* device-specific functions */

void xmos_flash_init (void)
{
   flash_file = NULL;
   flash_file_name = NULL;
   flash_size = DEFAULT_FLASH_SIZE;
   mos_mutex_init (&xmos_flash_mutex);

   mos_mutex_lock (&xmos_flash_mutex);
	
   if (!mos_arg_check("-flashfile", &flash_file_name)) {
      // default name is hex of node id
      sprintf(namebuf, "%04hx.flash", mos_node_id_get());
      flash_file_name = namebuf;
   }
	
   char* fsize;
   if (mos_arg_check("-flashsize", &fsize)) {
      flash_size = atol(fsize);
   }
	
   mos_mutex_unlock (&xmos_flash_mutex);
}

/* Many XMOS apps may call xmos_flash_init() without ever using the
 * XMOS flash device.  We prevent the creation of many unused large files
 * by creating the flash emulation file only once it is actually used.
 */
static void lazyInit()
{
   mos_mutex_lock (&xmos_flash_mutex);
   // check for race condition
   if (flash_file) return;
	
   // See if the flash file exists
   if (access(flash_file_name, F_OK)) {
      // write a bunch of zeros to the new file
      flash_file = fopen(flash_file_name, "w");
      fseek(flash_file, 0, SEEK_SET);
      void* buf = calloc(1, flash_size);
      fwrite(buf, 1, flash_size, flash_file);
      fflush(flash_file);
      free(buf);
      fclose(flash_file);
   }
	
   flash_file = fopen(flash_file_name, "r+");
   if (!flash_file) {
      perror("xmos_flash_init: fopen");
      mos_mutex_unlock (&xmos_flash_mutex);
      return;
   }
	
   mos_mutex_unlock (&xmos_flash_mutex);
}

#endif /* PLATFORM_LINUX */
