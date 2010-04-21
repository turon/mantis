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


/** @file command_list.h
 * @brief This is an application which will allow interfacing with a nymph from
 * a computer over the serial line, it provides arbitrary functions
 * @author Charles Gruenwald III 
 * @author Modified: Jeff Rose
 * @date Created: 02/12/2004
 * @date Modified 03/10/2004
 */

#ifndef _command_list_h_
#define _command_list_h_

#include <stdbool.h>
#include "com.h"

typedef struct command_list_s {
  char * input;
  void (*func_pointer)(void);
} command_list_t;

char prompt_char(char * string);

/** @brief Register a command.
 * @param in_list Command list data structure
 * @param name Name of function
 * @param func_pointer Location of function
 * @param max_command_count Maximum number of commands
 * @return TRUE if success, else return FALSE
 */
bool register_function(command_list_t in_list [],
		       char * name, void (*func_pointer)(void),
		       uint16_t max_command_count);

/** @brief Command parse function. 
 * @param in_list Command list data structure
 * @param command_string String for command
 * @param max_command_count Maximum number of commands
 * @return TRUE if the command string exists, else return FALSE
 */
bool command_parse(command_list_t in_list [],
		   char *command_string,
		   uint16_t max_command_count);
/** @brief Print the list
 * @param in_list Command list data structure
 * @param max_command_count Maximum number of commands
 */
int16_t help_list(command_list_t in_list [],
		  uint16_t max_command_count);

#endif
