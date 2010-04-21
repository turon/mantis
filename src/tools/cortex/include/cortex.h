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
/*   File: cortex.h                                                       */
/* Author: Jeff Rose   :     rosejn@colorado.edu                          */
/*   Date: 03/17/04                                                       */
/*                                                                        */
/* The primary header for the whole cortex framework.                     */
/**************************************************************************/

#ifndef _CORTEX_H_
#define _CORTEX_H_

#include <inttypes.h>

#include <glib/gprintf.h>
#include <string.h>
#include <stdbool.h>

/* This should be defined if the cortex is to be built for a PDA */
//#define MINI_GUI

/* The filename and root node of the glade interface data.*/
#ifndef MINI_GUI
#define GLADE_FILE "glade/cortex.glade"
#else
#define GLADE_FILE "glade/cortex-mini.glade"
#endif

#define FN __FUNCTION__
#define LN __LINE__
#define FL __FILE__
 
/* simple debug output */
#define DEBUG

#ifdef DEBUG
//#define debug(msg,args...) g_printerr("[%s]:%s(%d): \"", FL, FN, LN);
#define STRIP_SLASHES(msg) strrchr(msg, '/') + 1
#define debug(msg,args...) do {				\
   printf("[%s:%d]%s(): '", STRIP_SLASHES(FL), LN, FN); \
   printf(msg, ## args);				\
   printf("'\n");					\
  } while(0)
#else
#define debug(msg,args...)
#endif

#endif

