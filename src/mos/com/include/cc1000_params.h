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

/** @file cc1000_params.h
 * @brief Parameters used to initialize the CC1000 radio
 */

#ifndef _CC1000_PARAM_H_
#define _CC1000_PARAM_H_

#include "com.h"
#include "plat_dep.h"

/** @brief register settings for cc1000 */
#define MAIN_PARAM       0x11 
#define FREQ2A_PARAM     0x7C // 914.9988 Mhz
#define FREQ1A_PARAM     0x20
#define FREQ0A_PARAM     0x00
#define FREQ2B_PARAM     0x5D
#define FREQ1B_PARAM     0x0B
#define FREQ0B_PARAM     0x43
//from here to FS_DELAY_PARAM gets written sequentially

#define FSEP1_PARAM      0x03
#define FSEP0_PARAM      0xE3

#define RX_CURRENT_PARAM 0x8C
#define FRONT_END_PARAM  0x30
#define PA_POW_PARAM     0xFF
#define PLL_RX_PARAM     0x70
#define LOCK_PARAM       0x10
#define CAL_PARAM        0x26
#define MODEM2_PARAM     0x90
#define MODEM1_PARAM     0x6F

#define CC1000_BAUD_9600
//force 9600 baud when using FEC
#if defined (RADIO_USE_FEC) || defined (CC1000_BAUD_9600)
#define MODEM0_PARAM     0x45 //9600 bps
#else
#define MODEM0_PARAM     0x55  //19.2kbps
#endif

#define MATCH_PARAM      0x20
#define FSCTRL_PARAM     0x01
#define FSHAPE7_PARAM    0x00
#define FSHAPE6_PARAM    0x00
#define FSHAPE5_PARAM    0x00
#define FSHAPE4_PARAM    0x00
#define FSHAPE3_PARAM    0x00
#define FSHAPE2_PARAM    0x00
#define FSHAPE1_PARAM    0x00
#define FSDELAY_PARAM    0x00
#define PRESCALER_PARAM  0x00
#define TEST6_PARAM      0x00
#define TEST5_PARAM      0x00
#define TEST4_PARAM      0x3F
#define TEST3_PARAM      0x00
#define TEST2_PARAM      0x00
#define TEST1_PARAM      0x00
#define TEST0_PARAM      0x00
#define TX_CURRENT_PARAM 0xF3
#define PLL_TX_PARAM     0x70


uint8_t cc1000_params[FREQS_NUM][SETTING_NUM] ARCH_PROGMEM =
{
   {  // 902.265, 19.2 kBoudrate      0x00
      0xD6,0x00,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03			
      0xD6,0x07,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  // 902.791, 19.2 kBoudrate      0x01
      0xD6,0x20,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03      
      0xD6,0x27,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  // 903.318, 19.2 kBoudrate      0x02
      0xD6,0x40,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03 
      0xD6,0x47,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  // 903.845, 19.2 kBoudrate        0x03  
      0xD6,0x60,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD6,0x67,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  // 904.371, 19.2 kBoudrate       0x04
      0xD6,0x80,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03      
      0xD6,0x87,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06		
   },
   {  // 904.898, 19.2 kBoudrate       0x05
      0xD6,0xa0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03   
      0xD6,0xa7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  // 905.43f, 19.2 kBoudrate       0x06  
      0xD6,0xc0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD6,0xc7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_905_951_MHZ        0x07  
      0xD6,0xE0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD6,0xE7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_906_478_MHZ	0x08  
      0xD7,0x00,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0x07,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_907_004_MHZ        0x0a
      0xD7,0x20,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0x27,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_907_531_MHZ        0x0a
      0xD7,0x40,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03				
      0xD7,0x47,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_908_058_MHZ        0x0b
      0xD7,0x60,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03	
      0xD7,0x67,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_908_584_MHZ	0x0c
      0xD7,0x80,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0x87,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_909_111_MHZ	0x0d  
      0xD7,0xA0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0xA7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_909_638_MHZ        0x0e
      0xD7,0xc0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0xc7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_910_164_MHZ        0x0f
      0xD7,0xE0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD7,0xE7,0x2B// FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_910_691_MHZ	0x10
      0xD8,0x00,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0x07,0x2B// FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_911_217_MHZ	0x11
      0xD8,0x20,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0x27,0x2B// FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_911_744_MHZ        0x12
      0xD8,0x40,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03				
      0xD8,0x47,0x2B// FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_912_271_MHZ        0x13
      0xD8,0x60,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0x67,0x2B// FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_912_797_MHZ	0x14
      0xD8,0x80,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03	
      0xD8,0x87,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_913_324_MHZ	0x15
      0xD8,0xA0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0xA7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_913_851_MHZ        0x16
      0xD8,0xc0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0xc7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_914_377_MHZ        0x17
      0xD8,0xE0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD8,0xE7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {   //#define FREQ_914_907_MHZ        0x18
      0xD9,0x00,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD9,0x07,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_915_430_MHZ        0x19
      0xD9,0x20,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD9,0x27,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_915_957_MHZ        0x1a
      0xD9,0x40,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD9,0x47,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_916_484_MHZ        0x1b
      0xD9,0x60,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD9,0x67,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_917_010_MHZ        0x1c
      0xD9,0x80,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03
      0xD9,0x87,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   }, 
   {  //#define FREQ_917_537_MHZ        0x1d
      0xD9,0xA0,0x00,// FREQ2A,FREQ1A,FREQ0A 01 - 03       
      0xD9,0xA7,0x2B // FREQ2B,FREQ1B,FREQ0B 04 - 06
   } 
}; 

#endif
