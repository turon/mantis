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

#ifdef PLATFORM_MICAZ

#define CC2420_RESET_PORT A
#define CC2420_RESET_PIN 6
#define CC2420_RESET CC2420_RESET_PORT, CC2420_RESET_PIN

#define CC2420_VREG_PORT A
#define CC2420_VREG_PIN 5
#define CC2420_VREG CC2420_VREG_PORT, CC2420_VREG_PIN

#define CC2420_FIFO_PORT B
#define CC2420_FIFO_PIN 7
#define CC2420_FIFO CC2420_FIFO_PORT, CC2420_FIFO_PIN

#define CC2420_SFD_PORT D
#define CC2420_SFD_PIN 4
#define CC2420_SFD CC2420_SFD_PORT, CC2420_SFD_PIN

#define CC2420_CCA_PORT D
#define CC2420_CCA_PIN 6
#define CC2420_CCA CC2420_CCA_PORT, CC2420_CCA_PIN

#define CC2420_FIFOP_PORT E
#define CC2420_FIFOP_PIN 6
#define CC2420_FIFOP CC2420_FIFOP_PORT, CC2420_FIFOP_PIN

#define CC2420_FIFOP_INTERRUPT() SIGNAL(SIG_INTERRUPT6)

#define PLAT_ENABLE_FIFOP_INT() do {			\
EICRB |= (0x3 << (CC2420_FIFOP_PIN - 4) * 2);	\
MASK_1(EIFR, CC2420_FIFOP_PIN);			\
} while(0)


#endif

