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
#include "dev.h"

#if defined(PLATFORM_IMOTE2)

inline void plat_init()
{
    /* clock enable Mem controler, internal mem, i2c, os timer */
    CKEN = (CKEN22_MEMC | CKEN20_IMEM | CKEN15_PMI2C | CKEN9_OST);
    /* oscillator on */
    OSCC = (OSCC_OON);
    /* wait for oscillator */
    while ((OSCC & OSCC_OOK) == 0);
    DVFS_SwitchCoreFreq(13, 13);
    /* initialize memory controller */
    SA1110 = SA1110_SXSTACK(1);
    MSC0 = MSC0 | (1<<3) | (1<<15) | 2 ;
    MSC1 = MSC1 | (1<<3);
    MSC2 = MSC2 | (1<<3);
    MECR =0; /* no PC Card is present and 1 card slot */
    MDCNFG = 0x0B002BCD; /* enable SDRAM. */
    /* init MMU */
    initMMU();
    enableICache();
    initSyncFlash();
    enableDCache();
}



#endif
