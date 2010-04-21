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
/* File:    node_id.c                                                     */
/* Author   Jeff Rose    rosejn@colorado.edu                              */
/*   Date:  05/19/2004                                                    */
/*                                                                        */
/* Description: Gets a random ID initially and lets the user set one.     */
/**************************************************************************/
#include "mos.h"
#include "arg.h"

#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

boolean retrieved = FALSE;

static uint16_t node_id;

void mos_node_id_init ()
{
   int r;
   pid_t pid = getpid ();
   uint8_t *id;
   
   if (mos_arg_check ("-id", &id)) {
      node_id = atoi (id);
      printf ("Got id: %d\n", node_id);
   } else {
      srand ((unsigned int)pid);
      r = rand ();

      node_id = r % UINT16_MAX;
   }
}

inline uint16_t mos_node_id_get ()
{
   return node_id;
}

void mos_node_id_set (uint16_t new_node_id)
{
   node_id = new_node_id;
}
