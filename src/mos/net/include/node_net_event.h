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

/** @file net/include/node_net_event.h
 * @brief net event layer
 * @author Charles Gruenwald III
 * @date 04/14/2004
 *
 * This is an event layer which was used for the BioNET demo, it has not been
 * thoroughly tested on 0.9 yet, use at your own risk.
 */

#ifndef NODE_NET_EVENT_H
#define NODE_NET_EVENT_H

#include <inttypes.h>

#define NET_EVENT_PORT 1

#define LEDS                  1
#define RF_LEDS_BLINK         2
#define CLICK                 4
#define BATTERY               5
#define TOGGLE_LEDS           6
#define LIGHT_AND_TEMP        7
#define MST_TEST_PACKET       9
#define THERMAL              10
#define LIGHT                11

#define APP_PRESENT   200

//radio commands
#define RF_POWER_SET 300
#define RF_POWER_GET 301
#define RF_POWER_VALUE 302

#define SET_NODE_ID 303

#define SENSE_DELAY_SET 304
#define SENSE_DELAY_GET 305
#define SENSE_DELAY_VALUE 306

#define COMM_DELAY_SET 307
#define COMM_DELAY_GET 308
#define COMM_DELAY_VALUE 309

void mos_net_daemon_init();

/** @brief Start the net daemon. */
void mos_net_daemon();
/** @brief Register an event function.
 * @param command_to_reg Command to register
 * @param func_pointer Function pointer
 * @return FALSE if too many or no events, else TRUE
 */
int8_t mos_register_rf_function(uint16_t command_to_reg,
				void (*func_pointer)(void *));
uint16_t mos_get_event_source(void *p);

/** @brief Send event with an arguement. 
 * @param destination Destination
 * @param event Event
 * @param arg Arguement
 */
void send_event_arg(uint16_t destination, uint16_t event, uint16_t arg);
/** @brief Send event with an 8-bit arguement.
 * @param destination Destination
 * @param event Event
 * @param arg Arguement
 */
void send_event_arg8(uint16_t destination, uint16_t event, uint8_t arg);
/** @brief Send an event buffer.
 * @param destination Destination
 * @param event Event
 * @param buffer Buffer
 * @param size Buffer size
 */
void send_event_buf(uint16_t destination, uint16_t event, uint8_t *buffer,
		    uint8_t size);
/** @brief Send a generic event.
 * @param destination Destination
 * @param event Event
 */
void send_event(uint16_t destination, uint16_t event);

#define NET_EVENT_HDR_SIZE 6

/** @brief Net event structure. */
typedef struct {
  /** @brief Who the packet is from */
   uint16_t from;
  /** @brief Who the packet is to */
   uint16_t to;
  /** @brief Event */
   uint16_t event;
  /** @brief Buffer used */
   uint8_t *buffer;
} net_event_t;

/** @brief Light & Temp structure. */
typedef struct {
  /** @brief Light data */
   uint8_t light;
  /** @brief Temp data */
   uint8_t temp;
} light_temp_t;


#endif
