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
/* File mb-cc2420-gpio.c
 * Author: Charles Gruenwald III
 * Date: 04/20/2007
 */

#include "mos.h"
#include "cc2420-gpio.h"
#include "printf.h"
#include "mutex.h"

#if defined(PLATFORM_MICROBLAZE)

mos_mutex_t cc2420_gpio_mutex;

#define MB_CC2420_GPIO_FIFO  (1 << 0)
#define MB_CC2420_GPIO_CCA   (1 << 2)
#define MB_CC2420_GPIO_SFD   (1 << 3)
#define MB_CC2420_GPIO_RESETN (1 << 4)
#define MB_CC2420_GPIO_VREG_EN (1 << 5)

//on different gpio
#define MB_CC2420_GPIO_FIFOP (1 << 0)

#define CC2420_GPIO_INTERRUPT_ID XPAR_OPB_INTC_0_CC2420_GPIO_INT_IP2INTC_IRPT_INTR

static XGpio cc2420gpio;
static XGpio cc2420gpio_int;
static uint8_t output_val;

XInterruptHandler cc2420_fifop_interrupt(void *CallbackRef)
{
    XGpio *GpioPtr = (XGpio *)CallbackRef;
    printf("cc2420int\n");
    mos_cc2420_fifop_interrupt_handler();
    XGpio_InterruptClear(GpioPtr, XGPIO_IR_CH1_MASK);
    return NULL;
}


static inline void cc2420_gpio_set_output(uint8_t vals_to_set)
{
    output_val |= vals_to_set;
    XGpio_DiscreteWrite(&cc2420gpio, 1, output_val);
}

static inline void cc2420_gpio_clear_output(uint8_t vals_to_clear)
{
    output_val &= ~(vals_to_clear);
    XGpio_DiscreteWrite(&cc2420gpio, 1, output_val);
}

boolean cc2420_get_fifo()
{
    return (XGpio_DiscreteRead(&cc2420gpio, 1) & MB_CC2420_GPIO_FIFO);
}

boolean cc2420_get_fifop()
{
    return (XGpio_DiscreteRead(&cc2420gpio_int, 1) & MB_CC2420_GPIO_FIFOP);
}

boolean cc2420_get_cca()
{
    return (XGpio_DiscreteRead(&cc2420gpio, 1) & MB_CC2420_GPIO_CCA);
}

boolean cc2420_get_sfd()
{
    return (XGpio_DiscreteRead(&cc2420gpio, 1) & MB_CC2420_GPIO_SFD);
}

void cc2420_gpio_init()
{
    //Initialize ADC Control GPIO
    XGpio_Initialize(&cc2420gpio, XPAR_CC2420_GPIO_DEVICE_ID);

    //Set direction, only outputs are vreg ang resetn
    uint8_t output_val = 0;
    XGpio_SetDataDirection(&cc2420gpio, 1, MB_CC2420_GPIO_CCA | MB_CC2420_GPIO_SFD | MB_CC2420_GPIO_FIFO);

    cc2420_gpio_set_output(MB_CC2420_GPIO_RESETN);
    cc2420_gpio_set_output(MB_CC2420_GPIO_VREG_EN);
    mos_udelay(4000);
    cc2420_gpio_clear_output(MB_CC2420_GPIO_RESETN);
    mos_udelay(4);
    cc2420_gpio_set_output(MB_CC2420_GPIO_RESETN);
    mos_udelay(20);

    XGpio_DiscreteWrite(&cc2420gpio, 1, MB_CC2420_GPIO_RESETN);
    XGpio_DiscreteWrite(&cc2420gpio, 1, MB_CC2420_GPIO_RESETN | MB_CC2420_GPIO_VREG_EN);

    XGpio_DiscreteWrite(&cc2420gpio, 1, MB_CC2420_GPIO_VREG_EN | MB_CC2420_GPIO_RESETN);
    mos_udelay(4000);
    XGpio_DiscreteWrite(&cc2420gpio, 1, MB_CC2420_GPIO_VREG_EN);
    mos_udelay(4);
    XGpio_DiscreteWrite(&cc2420gpio, 1, MB_CC2420_GPIO_VREG_EN | MB_CC2420_GPIO_RESETN);
    mos_udelay(20);

    XGpio_Initialize(&cc2420gpio_int, XPAR_CC2420_GPIO_INT_DEVICE_ID);
    XGpio_SetDataDirection(&cc2420gpio_int, 1, MB_CC2420_GPIO_FIFOP);
    XGpio_InterruptEnable(&cc2420gpio_int, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalEnable(&cc2420gpio_int);

    mb_register_interrupt(CC2420_GPIO_INTERRUPT_ID,
            (XInterruptHandler)cc2420_fifop_interrupt,
            (void *)&cc2420gpio_int);
}

void cc2420_enable_fifop_interrupt()
{
    mb_enable_interrupt(CC2420_GPIO_INTERRUPT_ID);
}
void cc2420_disable_fifop_interrupt()
{
    mb_disable_interrupt(CC2420_GPIO_INTERRUPT_ID);
}

#endif
