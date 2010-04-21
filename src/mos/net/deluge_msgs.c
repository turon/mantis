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
#ifdef ARCH_AVR
#include <avr/pgmspace.h>
#endif

#ifdef ARCH_AVR

#ifdef DELUGE_PRINT_PACKETS
const char sPktPortType[] ARCH_PROGMEM = "%C<<%d %C ";
const char sSendNList[] ARCH_PROGMEM =   "\e[30;43m[%3d %3d %3d %3d %3d] ";
const char sSendSumm[] ARCH_PROGMEM =    "\e[30;43m%3C %3C %3C\e[0m\n";
const char sSendProf[] ARCH_PROGMEM =    "\e[30;46m%10l %5d %3C %3C\e[0m\n";
const char sSendReq[] ARCH_PROGMEM =     "\e[30;42m%016b %016b %016b %5d %3C %3C %3C\e[0m\n";
const char sSendData[] ARCH_PROGMEM =    "\e[30;41m%3C %3C %3C\e[0m\n";
const char sSendComm[] ARCH_PROGMEM =    "\e[30;45m%3C %3C %5d\e[0m\n";
const char sRecvNList[] ARCH_PROGMEM =   "\e[40;33m[%3d %3d %3d %3d %3d] ";
const char sRecvSumm[] ARCH_PROGMEM =    "\e[40;33m%3C %3C %3C\e[0m\n";
const char sRecvProf[] ARCH_PROGMEM =    "\e[40;36m%10l %5d %3C %3C\e[0m\n";
const char sRecvReq[] ARCH_PROGMEM =     "\e[40;32m%016b %016b %016b %5d %3C %3C %3C\e[0m\n";
const char sRecvData[] ARCH_PROGMEM =    "\e[40;31m%3C %3C %3C\e[0m\n";
const char sRecvComm[] ARCH_PROGMEM =    "\e[40;35m%3C %3C %5d\e[0m\n";
#endif

#ifdef DELUGE_PRINT_EVENT
const char sInitDeluge[] ARCH_PROGMEM = "Initializing Deluge\n";
const char sDisplayTime[] ARCH_PROGMEM = "\xFF\xFF\6";	// display time
const char sReboot[] ARCH_PROGMEM = "Rebooting now\n";
const char sNewVer[] ARCH_PROGMEM = "New version  \n";
const char sPageDone[] ARCH_PROGMEM = "Completed page %C\n";
const char sClearStats[] ARCH_PROGMEM = "Clearing stats\n";
const char sStartRec[] ARCH_PROGMEM = "Start recording stats\n";
const char sStopRec[] ARCH_PROGMEM = "Stop recording stats\n";
const char sChangeVer[] ARCH_PROGMEM = "Changing version to %C\n";
const char sClearTimer[] ARCH_PROGMEM = "\xFF\xFF\3";		// clear timer
const char sWipe[] ARCH_PROGMEM = "Wiping...";
const char sDone[] ARCH_PROGMEM = "done.\n";
const char sChangeCache[] ARCH_PROGMEM = "Changing cache size to %C\n";
const char sChangeTx[] ARCH_PROGMEM = "Changing transmit power to %C\n";
const char sHello[] ARCH_PROGMEM = "Hello!\n";
#endif

const char sProgErr[] ARCH_PROGMEM = "# program CRC error %h != %h\n";
const char sPageErr[] ARCH_PROGMEM = "# page CRC error %h != %h\n";
const char sFileErr[] ARCH_PROGMEM = "# Could not open file '%s'.\n";

#endif
