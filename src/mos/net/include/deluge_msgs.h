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

#include "deluge_impl.h"
#include "plat_dep.h"
#include <avr/pgmspace.h>

/** @file deluge_msgs.h
 * @brief Definitions for deluge message to be put in program memory
 */

#ifdef DELUGE_PRINT_PACKETS
extern const char sPktPortType[] ARCH_PROGMEM;
extern const char sSendNList[] ARCH_PROGMEM;
extern const char sSendSumm[] ARCH_PROGMEM;
extern const char sSendProf[] ARCH_PROGMEM;
extern const char sSendReq[] ARCH_PROGMEM;
extern const char sSendData[] ARCH_PROGMEM;
extern const char sSendComm[] ARCH_PROGMEM;
extern const char sRecvNList[] ARCH_PROGMEM;
extern const char sRecvSumm[] ARCH_PROGMEM;
extern const char sRecvProf[] ARCH_PROGMEM;
extern const char sRecvReq[] ARCH_PROGMEM;
extern const char sRecvData[] ARCH_PROGMEM;
extern const char sRecvComm[] ARCH_PROGMEM;
#endif

#ifdef DELUGE_PRINT_EVENT
extern const char sInitDeluge[] ARCH_PROGMEM;
extern const char sDisplayTime[] ARCH_PROGMEM;	// display time
extern const char sReboot[] ARCH_PROGMEM;
extern const char sNewVer[] ARCH_PROGMEM;
extern const char sPageDone[] ARCH_PROGMEM;
extern const char sClearStats[] ARCH_PROGMEM;
extern const char sStartRec[] ARCH_PROGMEM;
extern const char sStopRec[] ARCH_PROGMEM;
extern const char sChangeVer[] ARCH_PROGMEM;
extern const char sClearTimer[] ARCH_PROGMEM;		// clear timer
extern const char sWipe[] ARCH_PROGMEM;
extern const char sDone[] ARCH_PROGMEM;
extern const char sChangeCache[] ARCH_PROGMEM;
extern const char sChangeTx[] ARCH_PROGMEM;
extern const char sHello[] ARCH_PROGMEM;
/*extern const char s[] ARCH_PROGMEM;
extern const char s[] ARCH_PROGMEM;
extern const char s[] ARCH_PROGMEM;
extern const char s[] ARCH_PROGMEM;
extern const char s[] ARCH_PROGMEM;
extern const char s[] ARCH_PROGMEM;*/
#endif

extern const char sProgErr[] ARCH_PROGMEM;
extern const char sPageErr[] ARCH_PROGMEM;
extern const char sFileErr[] ARCH_PROGMEM;
