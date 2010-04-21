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

// build configuration options
// to undefine/redefine a device, use each app's optsconfig.h file

// devices
#define AVR_RSSI
#define MICA2_ACCEL            // accelerometer
#define MICA2_BATTERY          // battery voltage
#define MICA2_HUMIDITY         // humidity sensor
#define HARDWARE_ID            // node ID
#define MICA2_LIGHT_TEMP       // light and temperature sensors
#define MICA2_MAGNET           // magnetometer
#define MICA2_MIC              // microphone
#define MICA2_SOUNDER          // external speaker
#define MICA2_ULTRASOUND       // ultrasound
#define MICA2_GPS              // GPS
#define LOOPBACK               // loopback network device

// radios
#define CC1000                 // mica2 and mica2dot


/* only 1 of these 4 can be defined at a time, redefine in
   optsconfig.h if necessary */

//#define CC1000_BMAC       
#define CC1000_CSMA
//#define CC1000_RAW        
//#define CC1000_TDMA       


#define CC2420                 // micaz
#define MAXSTREAM              // dev board
#define XMOS_RADIO             // linux

// interfaces
#define SERIAL              
#define UART                
#define UDP                 
#define TERMINAL            

// Enable signal strength measurement
#define GET_RSSI             
//#define TS_PACKET

