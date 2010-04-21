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
/* File:    gui_tree.h                                                    */
/* Author     Jeff Rose   :  rosejn@colorado.edu                          */
/*   Date: 03/25/04                                                       */
/*                                                                        */
/* A gnome canvas topology display module.                                */
/**************************************************************************/

#ifndef _GUI_TOPO_H_
#define _GUI_TOPO_H_

#include <gtk/gtk.h>

#ifndef MINI_GUI
#include <libgnomecanvas/libgnomecanvas.h>
#endif

#include "bionet.h"

//#define TOPO_SIZE 400 // Size of the topology widget in pixels (and canvas region)
#define TOPO_NUM_NODES 10
#define TOPO_NODE_SIZE 20
#define TOPO_RADIUS    150
#define TOPO_LABEL_SIZE 256
#define TOPO_SPACING 20
#define TOPO_SPACING_OFFSET 5
#define TOPO_UNCONF_ROWS 3

#define TOPO_NODE_BASE_STATION 0
#define TOPO_NODE_REGULAR 1
#define TOPO_NODE_INACTIVE 2
#define TOPO_NODE_UNCONFIGURED 3

#define TOPO_BG_FILENAME "glade/ecenter.gif"
#define TOPO_BG_OFFSET 80
#define TOPO_BG_LINE_WIDTH 3
#define TOPO_BG_LINE_COLOR "black"

typedef struct _gui_topo
{
   GtkWidget *widget;
   GHashTable *nodes;
   GHashTable *unconf_nodes;
#ifndef MINI_GUI
   GnomeCanvasItem *bg;
#endif
   struct _topo_node *selected;
   gint bg_w;
   gint bg_h;
}gui_topo_t;

typedef struct _topo_node
{
   gui_topo_t *gui_topo;
#ifndef MINI_GUI
   GnomeCanvasItem *group;
   GnomeCanvasItem *node;
   GnomeCanvasItem *edge;
   GnomeCanvasPoints *points;
   GnomeCanvasItem *label;
#endif
   gchar *text;
   gchar *hab_type;
   gchar *hab_id;
   gchar *node_id;
   gchar *id;
   gint type;
   gdouble x, y, z;
}topo_node_t;

/* initialize the topology */
void gui_topo_init();

/* Create a new gtk topo view widget. */
gui_topo_t *gui_topo_new(void);

/* Add a new node to a topo view. */
void gui_topo_node_new (topo_node_t *node, gint light, gint temp);
void gui_topo_node_new_pos (topo_node_t *node, gdouble x, gdouble y, gdouble z);

/* Update a node in a topology. */
void gui_topo_node_update (topo_node_t *node, gint light, gint temp);
void gui_topo_node_update_pos (topo_node_t *node, gdouble x, gdouble y, gdouble z);

/* Highlight the currently selected node. */
void gui_topo_set_current_node (topo_node_t *node);

/* return a topo node from an id */
topo_node_t *topo_node_from_id(gchar *id);


#endif
