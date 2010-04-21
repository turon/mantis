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
 * Date: 05/01/2007
 */

#include "mos.h"
#include "bitops.h"
#include "cc2420-gpio.h"
#include "mutex.h"
#include "clock.h"

#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)


mos_mutex_t cc2420_gpio_mutex;

boolean cc2420_get_fifo()
{
    return (PORT_IN(CC2420_FIFO_PORT) & (1 << CC2420_FIFO_PIN)) ? true : false;
}

boolean cc2420_get_fifop()
{
    return (PORT_IN(CC2420_FIFOP_PORT) & (1 << CC2420_FIFOP_PIN)) ? true : false;
}
boolean cc2420_get_cca()
{
    return (PORT_IN(CC2420_CCA_PORT) & (1 << CC2420_CCA_PIN)) ? true : false;
}
boolean cc2420_get_sfd()
{
    return (PORT_IN(CC2420_SFD_PORT) & (1 << CC2420_SFD_PIN)) ? true : false;
}

void cc2420_gpio_init()
{
    SET_DIR_2(CC2420_RESET, CC2420_VREG);
    SET_PORT_1(CC2420_RESET);

    // set the voltage regulator on, reset pin inactive
    SET_PORT_1(CC2420_VREG);
    mos_udelay(4000);
    UNSET_PORT_1(CC2420_RESET);
    mos_udelay(4);
    SET_PORT_1(CC2420_RESET);
    mos_udelay(20);

    UNSET_DIR_1(CC2420_CCA);
    UNSET_DIR_1(CC2420_SFD);
    UNSET_PORT_1(CC2420_SFD);

    UNSET_DIR_1(CC2420_FIFO);

    UNSET_DIR_1(CC2420_FIFOP);
    UNSET_PORT_1(CC2420_FIFOP);

    PLAT_ENABLE_FIFOP_INT();

    UNSET_DIR_1(CC2420_FIFOP);
    UNSET_PORT_1(CC2420_FIFOP);
}

void cc2420_enable_fifop_interrupt()
{
    //enable FIFOP (receive) interrupt
    MASK_1(EIMSK, CC2420_FIFOP_PIN);
}

void cc2420_disable_fifop_interrupt()
{
    //disable FIFOP (receive) interrupt
    UNMASK_1(EIMSK, CC2420_FIFOP_PIN);
}

#endif

