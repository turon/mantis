//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <math.h>
#include <glib/gprintf.h>

#include "cortex.h"
#include "gui_gtk.h"
#include "gui_topo.h"

#include "model_gtk.h"

extern gui_topo_t *topo;
extern gint num_nodes;

gint max_x, max_y;

static gint temp_get_color (uint8_t temp);

#ifndef MINI_GUI
gint item_event_handler(GnomeCanvasItem *item, GdkEvent *event, topo_node_t *node);
#endif

void topo_canvas_size_allocate_handler(GtkWidget *widget, GtkAllocation *alloc, gpointer data);


static void gui_topo_add_node(bionet_node_t *node);
static void gui_topo_select_node(bionet_node_t *node);

void gui_topo_init()
{
   model_add_node_register_func(gui_topo_add_node);
   model_select_node_register_func(gui_topo_select_node);
}

gchar *node_name(bionet_node_t *node)
{
   gchar *name = (gchar *)g_malloc(256);
   g_sprintf(name, "%s.%s.%s",
	     node->hab_type,
	     node->hab_id,
	     node->id);
   return name;
}

static void gui_topo_add_node(bionet_node_t *node)
{
   gchar *id = node_name(node);
   if(topo_node_from_id(id))
   {
      debug("node already in topology");
      g_free(id);
      return;
   }

#ifndef MINI_GUI
   GnomeCanvas *canvas;
#endif
   
   gdouble x, y;
   int int_x, int_y;
   topo_node_t *new_node = g_new(topo_node_t, 1);

   debug ("blasting a new topo node");
   new_node->hab_type = g_strdup(node->hab_type);
   new_node->hab_id = g_strdup(node->hab_id);
   new_node->node_id = g_strdup(node->id);
   new_node->id = id;
   
   new_node->gui_topo = topo;

   gdouble node_radius = (gdouble)TOPO_NODE_SIZE / 2.0;
#ifndef MINI_GUI
   gtk_widget_get_size_request (topo->widget, &int_x, &int_y);
#endif
   x = int_x;
   y = int_y;
   gint type;

   if (strcmp(node->id, "0") == 0)
      type = TOPO_NODE_BASE_STATION;
   else
      type = TOPO_NODE_UNCONFIGURED;

#ifndef MINI_GUI
   canvas = GNOME_CANVAS(topo->widget);
#endif

   if (type == TOPO_NODE_UNCONFIGURED || type == TOPO_NODE_BASE_STATION) {
      g_hash_table_insert(topo->unconf_nodes, new_node->id, new_node);
      gint pos = g_hash_table_size (topo->unconf_nodes) - 1;
      x = (pos / TOPO_UNCONF_ROWS) * TOPO_SPACING + TOPO_SPACING_OFFSET;
      y = (pos % TOPO_UNCONF_ROWS) * TOPO_SPACING + TOPO_SPACING_OFFSET;
   }

   /* Create a new group for all the objects associated with this node. */
#ifndef MINI_GUI
   new_node->group = gnome_canvas_item_new(gnome_canvas_root(canvas),
					   gnome_canvas_group_get_type(),
					   "x", x,
					   "y", y,
					   NULL);

   new_node->x = x;
   new_node->y = y;
   new_node->z = 0;
   
   g_signal_connect(G_OBJECT(new_node->group), "event",
		      (GtkSignalFunc)item_event_handler,
		      new_node);

   if(type == TOPO_NODE_BASE_STATION)
      new_node->node = gnome_canvas_item_new(GNOME_CANVAS_GROUP(new_node->group),
					     gnome_canvas_ellipse_get_type(),
					     "x1", 0.0,
					     "y1", 0.0,
					     "x2", (gdouble)TOPO_NODE_SIZE,
					     "y2", (gdouble)TOPO_NODE_SIZE,
					     "fill-color-rgba", 0x0000ffff,
					     "outline-color", "black",
					     NULL);
   else
   {
      new_node->node = gnome_canvas_item_new(GNOME_CANVAS_GROUP(new_node->group),
					     gnome_canvas_ellipse_get_type(),
					     "x1", 0.0,
					     "y1", 0.0,
					     "x2", (gdouble)TOPO_NODE_SIZE,
					     "y2", (gdouble)TOPO_NODE_SIZE,
					     "fill-color","blue",
					     "outline-color", "black",
					     NULL);
   }

   new_node->type = type;
   new_node->text = g_malloc(TOPO_LABEL_SIZE);
   g_sprintf(new_node->text, "%s", new_node->node_id);
   
   new_node->label = gnome_canvas_item_new(GNOME_CANVAS_GROUP(new_node->group),
					   gnome_canvas_text_get_type(),
					   "x", node_radius,
					   "y", node_radius,
					   "anchor", GTK_ANCHOR_CENTER,
					   "font", "fixed",
					   "fill-color", "red",
					   "text", new_node->text,
					   NULL);
   
#endif

   new_node->x = x;
   new_node->y = y;
   new_node->z = 0;

   new_node->type = type;
   new_node->text = g_malloc(TOPO_LABEL_SIZE);
   g_sprintf(new_node->text, "%s", new_node->node_id);  
}

gui_topo_t *gui_topo_new(void)
{
#ifndef MINI_GUI
   GnomeCanvas *canvas;
#endif
   
   gui_topo_t *topo = g_new(gui_topo_t, 1);
   GdkPixbuf *bg_pixbuf;

   topo->selected = NULL;
   topo->nodes = g_hash_table_new(g_str_hash, g_direct_equal);
   topo->unconf_nodes = g_hash_table_new (g_str_hash, g_direct_equal);

#ifndef MINI_GUI
   topo->widget = gnome_canvas_new_aa();
   //topo->widget = get_widget(xml, "topo_canvas");
   canvas = GNOME_CANVAS(topo->widget);

   gnome_canvas_set_pixels_per_unit(canvas, 1.0);
   
   bg_pixbuf = gdk_pixbuf_new_from_file (TOPO_BG_FILENAME, NULL);
   if (bg_pixbuf == NULL) {
      printf ("Could not load image '%s'\n", TOPO_BG_FILENAME);
      exit (-1);
   }
   max_x = gdk_pixbuf_get_width (bg_pixbuf);
   max_y = gdk_pixbuf_get_height (bg_pixbuf);
   topo->bg_w = max_x;
   topo->bg_h = max_y + TOPO_BG_OFFSET;
   
   g_signal_connect(topo->widget, "size_allocate", G_CALLBACK(topo_canvas_size_allocate_handler),
		    topo);
   
   debug("max_x: %d, max_y: %d", max_x, max_y);
   
   // Create the background by drawing a white square
   gnome_canvas_item_new(gnome_canvas_root(canvas),
			 gnome_canvas_rect_get_type(),
			 "x1", 0.0,
			 "y1", 0.0,
			 "x2", (gdouble)max_x,
			 "y2", (gdouble)max_y + TOPO_BG_OFFSET,
			 "fill-color", "white",
			 NULL);

   topo->bg = gnome_canvas_item_new (gnome_canvas_root (canvas),
				     gnome_canvas_pixbuf_get_type (),
				     "x", 0.0,
				     "y", (gdouble)TOPO_BG_OFFSET,
				     "height", (gdouble)max_y,
				     "width", (gdouble)max_x,
				     "height-set", TRUE,
				     "width-set", TRUE,
				     "pixbuf", bg_pixbuf, NULL);
   
   // set where can the canvas scroll (our usable area)
   gnome_canvas_set_scroll_region(canvas, 0.0, 0.0, (gdouble)max_x,
				  (gdouble)max_y + TOPO_BG_OFFSET);
   //gnome_canvas_set_center_scroll_region(canvas, FALSE);
   
   GnomeCanvasPoints *points = gnome_canvas_points_new (2);
   points->coords[0] = 5.0;
   points->coords[1] = (gdouble)TOPO_BG_OFFSET - 5.0;
   points->coords[2] = (gdouble)max_x - 5.0;
   points->coords[3] = (gdouble)TOPO_BG_OFFSET - 5.0;
   gnome_canvas_item_new (gnome_canvas_root (canvas),
			  gnome_canvas_line_get_type (),
			  "width-pixels", TOPO_BG_LINE_WIDTH,
			  "fill-color", TOPO_BG_LINE_COLOR,
			  "points", points, NULL);
   gnome_canvas_points_unref (points);

   gtk_widget_show(topo->widget);

#endif
   
   return topo;
}

void topo_canvas_size_allocate_handler(GtkWidget *widget, GtkAllocation *alloc, gpointer data)
{
   //gui_topo_t *topo = (gui_topo_t *)data;
   //gdouble width_ratio = (gdouble)alloc->width / (gdouble)topo->bg_w;
   //gdouble height_ratio = (gdouble)alloc->height / (gdouble)topo->bg_h;
   //gdouble pixel_ratio = width_ratio;
   gdouble pixel_ratio = 0.0;
   debug("new size: %d, %d, %f", alloc->width, alloc->height, pixel_ratio);
   
   //gnome_canvas_set_pixels_per_unit(GNOME_CANVAS(widget), pixel_ratio);
   //gnome_canvas_update_now(GNOME_CANVAS(widget));
}

static gint temp_get_color (uint8_t temp)
{
   gint color = 0;
   color |= temp;
   color = color << 8;
   color |= temp;
   color = color << 8;
   color |= temp;
   color = color << 8;
   color |= 0xff;

   return color;
}

void gui_topo_node_new_pos (topo_node_t *node, gdouble x, gdouble y, gdouble z)
{
#ifndef MINI_GUI
   GnomeCanvas *canvas;
#endif
   
   topo_node_t *new_node = g_new(topo_node_t, 1);
   gdouble node_radius = (gdouble)TOPO_NODE_SIZE / 2.0;

   new_node->gui_topo = topo;
   new_node->x = x;
   new_node->y = y;
   new_node->z = z;
   new_node->id = g_strdup(node->id);
   if (strcmp(new_node->id, "0") == 0)
      new_node->type = TOPO_NODE_BASE_STATION;
   else
      new_node->type = TOPO_NODE_INACTIVE;

#ifndef MINI_GUI
   canvas = GNOME_CANVAS(topo->widget);

   new_node->group = gnome_canvas_item_new(gnome_canvas_root(canvas),
					   gnome_canvas_group_get_type(),
					   "x", x,
					   "y", y,
					   NULL);

   gtk_signal_connect(GTK_OBJECT(new_node->group), "event",
		      (GtkSignalFunc)item_event_handler,
		      new_node);

   new_node->node = gnome_canvas_item_new(GNOME_CANVAS_GROUP(new_node->group),
					  gnome_canvas_ellipse_get_type(),
					  "x1", 0.0,
					  "y1", 0.0,
					  "x2", (gdouble)TOPO_NODE_SIZE,
					  "y2", (gdouble)TOPO_NODE_SIZE,
					  "fill-color", "blue",
					  "outline-color", "black",
					  NULL);

   new_node->text = g_malloc(TOPO_LABEL_SIZE);
   g_sprintf(new_node->text, "%s", new_node->id);  
   new_node->label = gnome_canvas_item_new(GNOME_CANVAS_GROUP(new_node->group),
					   gnome_canvas_text_get_type(),
					   "x", node_radius,
					   "y", node_radius,
					   "anchor", GTK_ANCHOR_CENTER,
					   "font", "fixed",
					   "fill-color-rgba", 0xff00ffff,
					   "text", new_node->text,
					   NULL);
#endif
   
   /* Add the topo_node to the hash table so we can edit it later. */
   g_hash_table_insert(topo->nodes, new_node->id, new_node);
   g_hash_table_remove(topo->unconf_nodes, new_node->id);
}

static gboolean compare_func(gpointer key, gpointer value, gpointer user_data)
{
   gchar *name = (gchar *)key;
   gchar *id_to_check = (gchar *)user_data;

   if(strcmp(name, id_to_check) == 0)
      return true;
   
   return false;
}


topo_node_t *topo_node_from_id(gchar *id)
{
   topo_node_t *node = g_hash_table_find (topo->nodes, compare_func, id);
   if (node == NULL)
      node = g_hash_table_find (topo->unconf_nodes, compare_func, id);

   return node;
}

/* Update a node in a topology. */
void gui_topo_node_update(topo_node_t *node, gint light, gint temp)
{
   
   gint color;
   color = temp_get_color (light);
   if (node->type != TOPO_NODE_BASE_STATION &&
       (topo_node_t *)topo->selected != node) {
      node->type = TOPO_NODE_REGULAR;
#ifndef MINI_GUI
      gnome_canvas_item_set (node->node, "fill-color-rgba", color, NULL);
#endif
   }
}

void gui_topo_node_update_pos (topo_node_t *node, gdouble x, gdouble y, gdouble z)
{

   if (node != NULL) {
#ifndef MINI_GUI
      gnome_canvas_item_move (node->group, x - node->x, y - node->y);
#endif
      node->x = x;
      node->y = y;

      // move the node to the correct hash table
      g_hash_table_foreach_remove(topo->unconf_nodes, compare_func,  node->id);
      g_hash_table_insert(topo->nodes, node->id, node);
   }

}

/* wrapper function to get the topo_node from the bionet_node */
static void gui_topo_select_node(bionet_node_t *node)
{
   if(node == NULL)
   {
      gui_topo_set_current_node(NULL);
   } else {
      gchar *id = node_name(node);
      topo_node_t *topo_node = topo_node_from_id(id);
      g_free(id);
      gui_topo_set_current_node(topo_node);
   }
}

/* Highlight the currently selected node. */
void gui_topo_set_current_node(topo_node_t *node)
{
   if(topo->selected != NULL)
   {
      g_sprintf(topo->selected->text, "%s", topo->selected->node_id);
#ifndef MINI_GUI
      gnome_canvas_item_set(topo->selected->label, "text", topo->selected->text, NULL);
      gnome_canvas_item_set(topo->selected->node, "fill-color", "blue", NULL);
#endif
   }
      
   /* If this is selecting a new node set the new color. */
   if(node)
   {
      g_sprintf(node->text, "%s", node->id);
#ifndef MINI_GUI
      gnome_canvas_item_set(node->label, "text", node->text, NULL);
      gnome_canvas_item_set(node->node, "fill-color", "green", NULL);
#endif
      topo->selected = node;
   } else {
      topo->selected = NULL;
   }
}


/*** Functions private to this file. ***/
#ifndef MINI_GUI
gint item_event_handler(GnomeCanvasItem *item, GdkEvent *event, topo_node_t *node)
{
   static gboolean dragging = FALSE; // Is mouse button being held down for dragging
   static gdouble x, y;
   gdouble new_x, new_y;
   gdouble item_x, item_y, mouse_x, mouse_y;
   GdkCursor *fleur;
   
   /* We need to convert to item coordinates because some transormation
      could have happened on this item... */
   mouse_x = event->button.x;
   mouse_y = event->button.y;
   g_object_get (item, "x", &item_x, "y", &item_y, NULL);
   gnome_canvas_item_w2i(item->parent, &item_x, &item_y);
   
   switch (event->type)
   {
   case GDK_BUTTON_PRESS:
      /* We deselect if the control key is down. */
      if(event->button.state & GDK_CONTROL_MASK)
      {
	 gui_topo_set_current_node(node);
	 gui_gtk_set_current_node(GUI_SELECT_NONE, TRUE);
      }
      else
      {
	 gui_topo_set_current_node(node);
	 gui_gtk_set_current_node(node, TRUE);
      }
      /* We don't want to move the basestation... If we do we will
	 have to move all the lines to other nodes too. */
      //if(node->id == 0)
      // break;
      
      x = item_x;
      y = item_y;

      fleur = gdk_cursor_new(GDK_FLEUR);
      gnome_canvas_item_grab(item,
			     GDK_POINTER_MOTION_MASK | 
			     GDK_BUTTON_RELEASE_MASK,
			     fleur,
			     event->button.time);
      gdk_cursor_destroy(fleur);
      dragging = TRUE;

      gui_gtk_set_status(g_strdup_printf("Node %s -- x: %5.0f  y: %5.0f",
					 node->id, x, y));
      
      break;

   case GDK_MOTION_NOTIFY:
      if(dragging && (event->motion.state & GDK_BUTTON1_MASK)) 
      {
	 gdouble node_radius = (gdouble)TOPO_NODE_SIZE / 2.0;
	 new_x = mouse_x - node_radius;
	 new_y = mouse_y - node_radius;
	 /* force the dragging to within the image area */
	 if (new_x < 0)
	    new_x = 0;
	 if (new_y < TOPO_BG_OFFSET)
	    new_y = TOPO_BG_OFFSET;
	 if (new_x > max_x - TOPO_NODE_SIZE)
	    new_x = max_x - TOPO_NODE_SIZE;
	 if (new_y > max_y + TOPO_BG_OFFSET - TOPO_NODE_SIZE)
	    new_y = max_y + TOPO_BG_OFFSET - TOPO_NODE_SIZE;

	 if (strcmp(node->node_id, "0") == 0)
	    node->type = TOPO_NODE_BASE_STATION;
	 if (node->type != TOPO_NODE_INACTIVE)
	    node->type = TOPO_NODE_REGULAR;

	 g_hash_table_remove (topo->unconf_nodes, node->id);
	 g_hash_table_insert (topo->nodes, node->id, node);
	 
	 /* First move the node's group. */
	 gnome_canvas_item_move(item, new_x - x, new_y - y);
	 node->x = new_x;
	 node->y = new_y;


	 /* Now redraw the line too. */
	 /*
	 node->points->coords[2] = node->points->coords[2] + new_x - x;
	 node->points->coords[3] = node->points->coords[3] + new_y - y;
	 gnome_canvas_item_set(node->edge,
			       "points", node->points,
			       NULL);
	 */
	 x = new_x;
	 y = new_y;
	 gui_gtk_set_status(g_strdup_printf("Node %s -- x: %5.0f  y: %5.0f",
					    node->id, x, y));
      }
      break;

   case GDK_BUTTON_RELEASE:
      gnome_canvas_item_ungrab(item, event->button.time);
      dragging = FALSE;

      gui_gtk_set_status(g_strdup_printf("Selected Node: %s.%s.%s",
					 node->hab_type,
					 node->hab_id,
					 node->node_id));
      break;

   case GDK_2BUTTON_PRESS:
      debug("\n\ndouble click\n\n");
      break;
			    

   default:
      break;
   }

   return FALSE;
}
#endif //MINI_GUI
