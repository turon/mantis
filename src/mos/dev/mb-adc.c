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

/* File mb_adc.c
 * Author: Charles Gruenwald III
 * Date: 03/17/2007
 */

#include "mos.h"
#include "dev.h"

#ifdef PLATFORM_MICROBLAZE

#include "mb-adc.h"
#include "mb-spi.h"
#include "xilinx_help.h"

void adc_init()
{
}


uint8_t dev_mode_DEV_ADC(uint8_t new_mode)
{
    return DEV_UNSUPPORTED;
}

uint16_t dev_read_DEV_ADC(void *buf, uint16_t count)
{
    Xuint16 ret;

    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, MB_SPI_ADC_SLAVE_SELECT);
    dev_read(DEV_SPI, &ret, 2);
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, SPI_CLEAR_SLAVE_SELECT);

    //Mask out null bytes and the like
    ret &= 0x1FFF;
    ret >>= 3;

    *(uint16_t *)buf = ret;
    return count;
}

uint16_t dev_write_DEV_ADC(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_ADC(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}


#endif
