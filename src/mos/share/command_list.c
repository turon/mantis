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
/* Mantis Commander:                                                      */
/* This is an application which will allow interfacing with a nymph from  */
/* a computer over the serial line, it provides arbitrary functions.      */
/**************************************************************************/

#include "mos.h"

#ifndef SCONS
#include <config.h>
#endif

#include "command_list.h"
#include <string.h>
#ifdef PLATFORM_LINUX
#include <stdio.h>
#else
#include "printf.h"
#endif

mos_mutex_t command_mutex;

bool command_parse(command_list_t in_list[], char *command_string,
		   uint16_t max_command_count)
{
   uint16_t i;

   mos_mutex_lock(&command_mutex);

   for(i = 0; i < max_command_count; i++) {
      if(in_list[i].input) {
	 if(!strcmp(in_list[i].input, command_string)) {
	    in_list[i].func_pointer();
	    mos_mutex_unlock(&command_mutex);
	    return true;
	 }
      }
   }

   mos_mutex_unlock(&command_mutex);
   return false;
}

bool register_function(command_list_t in_list[], char * name,
		       void (*func_pointer)(void), uint16_t max_command_count)
{
   uint16_t i;
   bool ret;

   mos_mutex_lock(&command_mutex);
   
   for(i = 0; i < max_command_count && in_list[i].input != NULL; i++)
      ;

   if(i == max_command_count) {
      ret = false;
   }
   
   if(i < max_command_count) {
      in_list[i].input = name;
      in_list[i].func_pointer = func_pointer;
      in_list[++i].input = NULL;
      ret = true;
   } else {
      ret = false; //must have been past max command count
   }

   mos_mutex_unlock(&command_mutex);
   
   return ret;
}

int16_t help_list(command_list_t in_list[], uint16_t max_command_count)
{
   uint16_t i;
   uint8_t wrap = 1;

   printf("cmds: \n");
   for(i = 0; i < max_command_count && in_list[i].input != NULL; i++) {
      if(wrap++ % 6 == 0)
	 printf("\n");
      printf("[%s] ", in_list[i].input);
   }
   printf(".\n");

   return 0;
}


