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

#include <gtk/gtk.h>
#include "command_list.h"

#define LOCAL_COMMAND_COUNT_MAX 30

command_list_t local_commands[LOCAL_COMMAND_COUNT_MAX];
void load_bootloader_callback (GtkMenuItem *menu_item, gpointer data);
void quit_callback (GtkMenuItem *menu_item, gpointer data);
void clear_callback (GtkMenuItem *menu_item, gpointer data);
void about_callback (GtkMenuItem *menu_item, gpointer data);
void load_srec_file(gchar *filename);
void clear_output_buffer();
void load();
void reload();
void reload_srec_callback (GtkMenuItem *menu_item, gpointer data);
void load_srec_callback (GtkMenuItem *menu_item, gpointer data);

gboolean
txtInput_enter_notify_event            (GtkWidget       *widget,
                                        GdkEventKey     *event,
                                        gpointer         user_data);

void scroll_output(gchar *input, GtkTextView *output);
