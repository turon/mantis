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

#if defined(ARCH_AVR)

inline void plat_init()
{
    adc_init();
    spi_init();

#if defined(platform_mica2) || defined(platform_mica2dot)
#if defined(avr_rssi) || !defined(scons)
    avr_rssi_init();
#endif
#endif
    avr_eeprom_init();
    avr_i2c_init();

    WDTCR = (1 << WDCE) | (1 << WDE); /*disable watchdog timer*/
    WDTCR = 0x00;
    MCUCSR |= (1 << JTD); /*disable on-chip debugging*/

#if defined(PLATFORM_MICA_ANY)
#if defined(MICA2_LIGHT_TEMP) || !defined(SCONS)
    mica2_light_temp_init();
#endif

#if defined(MICA2_MIC) || !defined(SCONS)
    mica2_mic_init();
#endif

#if defined(MICA2_ACCEL) || !defined(SCONS)
    mica2_accel_init();
#endif

#if defined(MICA2_BATTERY) || !defined(SCONS)
    mica2_battery_init();
#endif

#if defined(MICA2_GPS) || !defined(SCONS)
    mica2_gps_init();
#endif

#endif /*if platform mica2, mica2dot, micaz*/


#if defined(PLATFORM_MICA2) || defined (PLATFORM_MICAZ)
    //mica2_magnet_init();
    //mica2_ultrasound_init();

    atmel_flash_init();

#if defined(HARDWARE_ID) || !defined(SCONS)
    hardware_id_init();   
#endif

#if defined(MICA2_SOUNDER) || !defined(SCONS)
    mica2_sounder_init();
#endif

#endif

    // MAC layer
    //which MAC to use is defined in com.h
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#ifdef CC1000_RAW
    cc1000_raw_init();
#endif

#ifdef CC1000_CSMA
    cc1000_csma_init();
#endif

#ifdef CC1000_CSMA_ACK
    cc1000_csma_ack_init();
#endif

#ifdef CC1000_BMAC
    cc1000_bmac_init();
#endif

#ifdef CC1000_TDMA
    cc1000_tdma_init(mos_node_id_get());
#endif

#endif /* PLATFORM_MICA2 / PLATFORM_MICA2DOT */

#ifdef PLATFORM_MICA_ANY
    //FIXME: light and temp won't right work unless we read from another ADC
    //driver first
    uint8_t val;
#warning ADC Hack
    dev_read(DEV_MICA2_ACCEL_Y, &val, sizeof(val));
#endif

#if defined(PLATFORM_MICAZ)
#if defined(CC2420) || !defined(SCONS)
    cc2420_init();
#endif
#endif

#ifdef PLATFORM_AVRDEV
    maxstream_init();
#endif
}


#endif
