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

#include <glib.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <stdbool.h>

typedef struct {
   gchar *hab_type;
   gchar *hab_id;
   gchar *node_id;
   gchar *resource;
} constraint_t;

typedef struct {
   gchar *name;
   GSList *constraint_list;
} filter_list_t;


typedef void (*filter_model_add_filter_f)(filter_list_t *filter_list);
typedef void (*filter_model_remove_filter_f)(filter_list_t *filter_list);

gboolean filter_model_init();
GSList *get_filter_list();
void filter_model_show_list();
void filter_model_update_filter_list(char *name, GSList *constraint_list);
void filter_model_delete_filter_list(char *name);
filter_list_t *filter_model_find_filter_list(char *name);
void filter_model_save_filter_list();

void filter_model_add_filter_register_func(filter_model_add_filter_f func);
void filter_model_remove_filter_register_func(filter_model_remove_filter_f func);
