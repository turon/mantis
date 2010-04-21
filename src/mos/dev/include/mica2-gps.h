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

#ifndef __MICA2_GPS_H__
#define __MICA2_GPS_H__

// pin 7 on switch 1
#define GPS_POWER_ADDR (1 << 6)
// pin 8 on switch 1
#define GPS_ENABLE_ADDR (1 << 7)

// pin 2 on switch 2
#define GPS_TX_ADDR (1 << 1)
// pin 1 on switch 2
#define GPS_RX_ADDR (1 << 0)


#define GPS_TX_RX  ( GPS_TX_ADDR | GPS_RX_ADDR )


// these two ADG715BRU switches on the i2c line
// are used to connect peripherals on the weatherboard to
// power, the uart, etc.

// switch 1 contains the enable switch for the GPS
#define SWITCH_1 (uint16_t)0x48 /*72*/
// switch 2 contains the uart tx/rx switches for the GPS
#define SWITCH_2 (uint16_t)0x49 /*73*/

/** @addtogroup driver_gps
 * <b>Notes:</b> See /src/apps/gps_test/ for an example application which uses this GPS Driver.
 */

/** @brief GPS Fix quality constants.
 * These are simply helper enumerations
 * for usage with the gps_gga_t structure.
 * @ingroup driver_gps */
enum GPS_FIX_QUALITY
{
   /** @brief an invalid fix */
   FIX_INVALID = 0,
   /** @brief a GPS fix */
   FIX_GPS,
   /** @brief a DGPS fix */
   FIX_DGPS,
   /** @brief a PPS fix */
   FIX_PPS,
   /** @brief a Real Time Kinematic fix */
   FIX_RTK,
   /** @brief a Float Real Time Kinematic fix */
   FIX_FRTK,
   /** @brief a Dead Reckoning fix */
   FIX_ESTIMATED,
   /** @brief Manual input mode */
   FIX_MANUAL,
   /** @brief Simulation mode */
   FIX_SIMULATION
};

/** @brief use with dev_ioctl to specify what kind of structure dev_read should expect.
 * @ingroup driver_gps */
enum MICA2_GPS_READ_TYPE
{
   /** @brief read a single gps packet */
   MICA2_GPS_READ_STRING,
   /** @brief wait for the next gga packet and read a struct gps_gga_t. */
   MICA2_GPS_READ_GGA
};

   

/** @brief simple time structure.
 * This is a helper structure
 * for usage with the gps_gga_t structure.
 * @ingroup driver_gps */
typedef struct timestamp_struct
{
   /** @brief current hour (0-23) */
   uint8_t hours;
   /** @brief current minute */
   uint8_t minutes;
   /** @brief current second */
   uint8_t seconds;
   
} timestamp_t;

/** @brief represents a measurement.
 * This is a helper structure
 * for usage with the gps_gga_t structure.
 * @ingroup driver_gps */
typedef struct meter_struct
{
   /** @brief the whole part of the measurement */
   uint16_t whole;
   /** @brief the decimal portion */
   uint16_t decimal;

   /** @brief a character representing the units of the measurement. */
   char units;
} measurement_t;

/** @brief represents a latitude or longitude value
 * This is a helper structure
 * for usage with the gps_gga_t structure.
 * @ingroup driver_gps */
typedef struct latitude_struct
{
   /** @brief the degrees of the value */
   uint16_t degrees;
   /** @brief the minutes of the value */
   measurement_t minutes;
   /** @brief a character representing the direction - 'N','W', etc. */
   char direction;
} latlong_t;
   

/** @brief represents a GPS Fix Data Packet from the GPS.
 * This structure contains all the information available in a
 * GGA Packet from the GPS in parsed form.
 * @ingroup driver_gps */
typedef struct gga_struct
{
   /** @brief the UTC time the measurement was taken */
   timestamp_t utc;
   /** @brief the current latitude */
   latlong_t latitude;
   /** @brief the current longitude */
   latlong_t longitude;
   /** @brief a GPS_FIX_QUALITY representing the current fix quality */
   uint8_t fix_quality;
   /** @brief the number of satellites the GPS is receiving data from */
   uint8_t satellite_count;
   /** @brief the horizontal dilution of the current position */
   measurement_t horiz_dilution;
   /** @brief altitude above sea level */
   measurement_t altitude;
   /** @brief height of geoid above WGS84 ellipsoid. duh. */
   measurement_t geoid_height;
   /** @brief seconds since the last update */
   uint16_t since_last_update;
   /** @brief the DGPS station ID (if fix quality = DGPS) */
   uint16_t dgps_id;
} gps_gga_t;

/** @brief prints a GGA packet to standard out.
 * @param gga a pointer to the GGA structure to print.
 * @ingroup driver_gps */
void mica2_gps_print_gga(const gps_gga_t* gga);


/** @brief initializes the GPS.
    called from main().
*/
void mica2_gps_init(void);

#endif // __MICA2_GPS_H__
