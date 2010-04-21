///  This file is part of MOS, the MANTIS Operating System
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

#ifndef __SHELL_H_
#define __SHELL_H_

#include <inttypes.h>


typedef void (*update_callback)(int current_page, int total_pages);
typedef void (*find_node_callback)(int retries);

int8_t shell_load_image(char *arg, update_callback update_func);
int8_t shell_start_execution(char *arg);
int8_t shell_read_flash(char *arg);
int    shell_find_node (find_node_callback find_callback_func);
int    shell_find_loader (find_node_callback find_callback_func);
void   shell_send_byte (uint8_t byte_to_send);
void   shell_send_restart();
void   shell_send_to_node (char * string_to_send);
void   shell_load_bootloader();

enum {
   NODE_NOT_FOUND,
   BOOTLOADER_MODE,
   APP_MODE,
   SREC_READ_ERROR,
   COMMAND_MATCH,
   COMMAND_MISS,
   NODE_RESTARTED,
   LOAD_COMMAND_FAILURE,
   PAGE_PACKET_FAILURE,
   SUCCESS
};

#endif
