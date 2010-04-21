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
/*   File: gui_gtk.h                                                      */
/* Author: Jeff Rose   :     rosejn@colorado.edu                          */
/*   Date: 03/25/04                                                       */
/*                                                                        */
/**************************************************************************/

#ifndef _GUI_GTK_H_
#define _GUI_GTK_H_

#include <gtk/gtk.h>
#include <glade/glade.h>

#include "bionet.h"
#include "gui_topo.h"

#define GUI_UPDATE_INTERVAL 1000 // Milliseconds between realtime graph updates
#define GUI_SELECT_NONE     NULL
#define GUI_TEMP_MAX        120.0
#define GUI_TEMP_MIN        40.0
#define GUI_LIGHT_MAX       256.0
#define GUI_LIGHT_MIN       0.0

#define CORTEX_TREEVIEW_MIN_Y 120
#define CORTEX_TREEVIEW_MAX_Y 600

extern GtkWidget *window_data;

#define MSG_WIN(msg) {							\
      GtkWidget *dialog;						\
      dialog = gtk_dialog_new_with_buttons("Message", GTK_WINDOW(window_data), \
					   GTK_DIALOG_DESTROY_WITH_PARENT, \
					   GTK_STOCK_OK, GTK_RESPONSE_NONE, \
					   NULL);			\
      GtkWidget *label = gtk_label_new(msg);				\
      gtk_window_set_modal(GTK_WINDOW(dialog), TRUE);			\
      gtk_container_add(GTK_CONTAINER(GTK_DIALOG(dialog)->vbox), label); \
      gtk_widget_show_all(dialog);;					\
      gtk_dialog_run(GTK_DIALOG(dialog));				\
      gtk_widget_destroy(dialog);					\
   }

void quit_handler(GtkWidget *w, gpointer p);

/* Start running the gtk gui. */
void gui_gtk_run(int *argcp, const char ***argvp);

/* Set the currently selected node. */
void gui_gtk_set_current_node(topo_node_t *node, gboolean isTopo);

/* Set the content of the statusbar. */
// NOTE: You must allocate the passed string, it will be freed with
//       the status changes...
void gui_gtk_set_status(gchar *status);

void gui_add_hab(bionet_hab_t *hab);
void gui_add_node(bionet_node_t *node);
void gui_clear_nodes();
void gui_update_resource(bionet_resource_t *resource);

GtkWidget *get_widget(GladeXML *xml, char *name);

extern GladeXML *xml;

#endif
