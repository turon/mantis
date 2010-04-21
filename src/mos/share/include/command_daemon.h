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

/** @file command_daemon.h
 * @brief Mantis Commander
 *
 * This is an application which will allow interfacing with a node from
 *   a computer over the serial line, it provides arbitrary functions.
 * @author Charles Gruenwald III
 * @date Created: 04/14/2004
 */

#ifndef _command_daemon_h_
#define _command_daemon_h_

#include "com.h"
#include <stdbool.h>

#define MOS_COMMANDER_STACK_SIZE 164

/** @brief Send the given string for a prompt and return an int response. 
 *
 * @param string String to be sent
 * @return Integer response
 */
uint8_t  prompt_uint8(char * string);

/** @brief  Send the given string for a prompt and return a long response. 
 *
 * @param string String to be sent
 * @return Multiple byte response
 */
uint16_t prompt_long(char *string);

/** @brief  Send the given string return a 4-byte integer response. 
 *
 * @param string String to be sent
 * @return Multiple byte response
 */
uint32_t prompt_longlong(char *string);

/** @brief Send the given string for a prompt and return a one byte response. 
 *
 * @param string String to be sent
 * @return Single byte response
 */
char     prompt_char(char *string);

/** @brief Parses incoming packets.
 *
 * This is a system function which parses an incoming packet,
 * and calls a function
 * if it finds a match. 
 * @param recvd Packet received  
 */
bool mos_command_parse (comBuf *recvd);

/** @brief Registers a function to be called on string input.
 * 
 * Takes the string to be typed in and the function to be called
 * and returns the 
 * success or failure of registration.
 * @param name String to use
 * @param func_pointer Function to register
 * @return Success or failure of registration
 */
bool mos_register_function(char * name, void (*func_pointer)(void));

/** @brief Initialize the command daemon.
 * 
 * The mos_command_daemon thread used to initialize the command list.  This is 
 * a problem for applications that want to register their own commands, since
 * they don't know how soon the daemon thread will finish initializing the list.
 * Applications which want to add their own commands should call
 * mos_command_daemon_init first.
 * 
 * For backward compatibility mos_command_daemon still calls 
 * mos_command_daemon_init, and mos_command_daemon_init can be called more 
 * than once.
 */
void mos_command_daemon_init();

/** @brief Thead which listens for input over the serial line.
 *
 * This is the actual thread that accepts commands over serial, parses
 * them, and executes commands. 
 */
void mos_command_daemon (void);

/** @brief Sends the command to tell the shell to reset it's timer
 *
 * The shell has some timing functions built in so that the timing on the node
 * may be tested. This command tells the shell to reset it's internal timer
 * to zero.
 */
void mos_clear_ext_timer();

/** @brief Sends the command telling the shell to show it's timer
 *
 * This tells the shell to show the elapsed time on it's internal timer.
 */
void mos_show_ext_timer();

/** @brief Sends the command telling the shell to show then clear it's timer
 *
 * This tells the shell to show the elapsed time on it's internal timer, then
 * clear the current running value.
 */
void mos_show_and_clear_ext_timer();


#ifdef PLATFORM_TELOSB
#define IFACE_STDIO IFACE_SERIAL2
#else
#define IFACE_STDIO IFACE_SERIAL
#endif




#endif
