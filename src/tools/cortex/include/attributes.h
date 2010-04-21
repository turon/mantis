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

/**************************************************************************/
/* File:    attributes.h                                                  */
/* Author     Jeff Rose   :  rosejn@colorado.edu                          */
/*   Date: 03/23/04                                                       */
/*                                                                        */
/* These are the supported attributes that are built into cortex.         */
/**************************************************************************/

#ifndef _ATTRIBUTES_H_
#define _ATTRIBUTES_H_

#include <glib.h>

#include "snet.h"

#define ATTR_HISTORY_SIZE 600 // Max number of data points to store

#define ATTR_LOCATION           1000
#define ATTR_LIGHT_SENSOR       1001
#define ATTR_TEMP_SENSOR        1002
#define ATTR_BEDREST            1003

typedef struct _location_attr
{
   gint x;
   gint y;
}location_attr_t;

typedef struct _bedrest_addr
{
   gint battery;
   gint light;
   gint temp;
   gint accelx;
   gint accely;
   gint txpower;
   gint rssi;
   GTimeVal time;
}bedrest_attr_t;

typedef struct _sensor_data
{
   gint value;
   GTimeVal time;
}sensor_data_t;

typedef struct _sensor_attr
{
   GQueue *data_points;
}sensor_attr_t;


#endif
