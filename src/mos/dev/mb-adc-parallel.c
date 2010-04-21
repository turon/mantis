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

/* File mb-adc-parallel.c
 * Author: Charles Gruenwald III
 * Date: 03/17/2007
 */

#include "mos.h"

#ifdef PLATFORM_MICROBLAZE

#include "dev.h"
#include "mb-adc-parallel.h"


#define MB_ADC_PARALLEL_CONVST (1 << 0)
#define MB_ADC_PARALLEL_BUSY   (1 << 1)
#define MB_ADC_PARALLEL_RD     (1 << 2)


static XGpio ADCData;    /* The driver instance for GPIO Device configured as I/P */
static XGpio ADCControl;


void init_mb_adc_parallel()
{
    //Initialize ADC Data GPIO
    XGpio_Initialize(&ADCData, XPAR_OPB_GPIO_0_DEVICE_ID);

    //Set lower 8 bits to input
    XGpio_SetDataDirection(&ADCData, 1, 0xFF);

    //Initialize ADC Control GPIO
    XGpio_Initialize(&ADCControl, XPAR_OPB_GPIO_1_DEVICE_ID);

    //Set direction, only input is busy signal
    XGpio_SetDataDirection(&ADCControl, 1, 0x2);

    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_RD);
}

uint8_t dev_mode_DEV_ADC_PARALLEL(uint8_t new_mode)
{
    return DEV_UNSUPPORTED;
}

uint16_t dev_read_DEV_ADC_PARALLEL(void *buf, uint16_t count)
{
    Xuint16 data;
    Xuint16 data_lower;

    //Pulse the conversion
    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_CONVST | MB_ADC_PARALLEL_RD); 
    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_RD); 
    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_CONVST | MB_ADC_PARALLEL_RD); 


    //Wait for it to finish
    Xuint32 status;
    Xuint16 bcount = 0;
    do
    {
        status = XGpio_DiscreteRead(&ADCControl, 1);
        bcount++;
    }
    while(status & MB_ADC_PARALLEL_BUSY);

    //Make ADC Use the bus - rd line low
    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_CONVST);	 

    // Higher 8 bits
    data = XGpio_DiscreteRead(&ADCData, 1);

    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_RD | MB_ADC_PARALLEL_CONVST);

    XGpio_DiscreteWrite(&ADCControl, 1, 0x0);	
    data_lower = XGpio_DiscreteRead(&ADCData, 1);
    XGpio_DiscreteWrite(&ADCControl, 1, MB_ADC_PARALLEL_RD);

    data <<= 2;
    data |= (data_lower & 0x03);
    *(uint16_t *)buf = data;
    return count;
}

uint16_t dev_write_DEV_ADC_PARALLEL(const void *buf, uint16_t count)
{
   return DEV_UNSUPPORTED;
}

uint8_t dev_ioctl_DEV_ADC_PARALLEL(int8_t request, ...)
{
   return DEV_BAD_IOCTL;
}


#endif
