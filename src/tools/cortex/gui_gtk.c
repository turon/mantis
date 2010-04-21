//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <time.h>
#include <stdbool.h>

#include "cortex.h"
#include "bionet-interface.h"
#include "gui_gtk.h"
#include "gui_topo.h"
#include "gui_xml.h"
#include "model_gtk.h"


//for managing filter lists
#include "filter_model.h"
#include "filter_wizard.h"

//for widgets/windows
#include "generic_view.h"
#include "nodelist_view.h"
#include "hablist_view.h"
#include "commanding_view.h"
#include "filterlist_view.h"

gchar *global_filename = NULL;

#ifndef MINI_GUI
GtkWidget *system_status_label;
#endif

gui_topo_t *topo;

GtkWidget *statusbar;
GtkWidget *window_data;
GtkWidget *window_filter_wizard;
GtkWidget *window_commanding_interface;

GtkWidget *dialog_about;
//GtkWidget *dialog_quit;


gint num_nodes;
gchar *current_node;

GladeXML *xml;

//glade signals won't connect automatically on arm
extern GtkWidget *commanding_hablist;
extern GtkWidget *hab_treeview;
void commanding_hablist_changed(GtkComboBox *widget, gpointer user_data);
void hab_selection_handler(GtkTreeView *treeview,
			   GtkTreePath *arg1,
			   GtkTreeViewColumn *arg2,
			   gpointer user_data);



void dialog_quit_handler(GtkWidget *w, gpointer p)
{
   //gtk_widget_show(dialog_quit);
   gtk_main_quit();
   gtk_exit(0);
}

void rate_button_handler(GtkButton *b, gpointer p)
{
   MSG_WIN("Rate setting not yet implemented");
}

void build_filter_handler(GtkWidget *w, gpointer p)
{
   debug("Got build filter callback");
}

void gui_gtk_new_node_handler(gchar *id)
{
   if(g_hash_table_lookup(topo->unconf_nodes, id))
      return;
   if(g_hash_table_lookup(topo->nodes, id))
      return;
   
   // We just create a new node with zero values here, and then
   //  when the update event comes it will get the correct data
   
   debug("need to add node to something");
   num_nodes++;
}

GtkWidget *get_widget(GladeXML *xml, char *name)
{
   GtkWidget *ret = glade_xml_get_widget(xml, name);
   
   if(!ret) {
      debug("Couldn't get widget %s", name);
      exit(-1);
   }

   return ret;
}

/* Start running the gtk gui. */
void gui_gtk_run(int *argcp, const char ***argvp)
{
   current_node = GUI_SELECT_NONE;
   num_nodes = 0;
   
   // Load the gui from a glade file
   xml = glade_xml_new(GLADE_FILE, NULL, NULL);
   if(!xml) {
      debug("An error occurred while creating the interface.");
      return;
   }
   
   // define our own quit event handler to properly exit
   // TODO: do we really want quit confirmation?? (i don't)
   //dialog_quit = get_widget(xml, "dialog_quit");
   //gtk_widget_hide (dialog_quit);

#ifndef MINI_GUI
   system_status_label = get_widget(xml, "system_status_label");
#endif
   
   window_data = get_widget(xml, "window_data");   
   window_filter_wizard = get_widget(xml, "window_filter_wizard");   
   window_commanding_interface = get_widget(xml, "window_commanding_interface");

   dialog_about = get_widget(xml, "dialog_about");
   gtk_widget_hide(dialog_about);

   statusbar = get_widget(xml, "statusbar_visual");

   //Get the parent widget from glade and set the scroll bars to show up
   //  automatically if necessary. */
   // Now create the topology object
   topo = gui_topo_new();
#ifndef MINI_GUI
   GtkWidget *scroll = get_widget(xml, "topo_scrolled");
   gtk_container_add(GTK_CONTAINER(scroll), topo->widget);

//now that we know the size of the background we can properly size the main window
   gint window_w = topo->bg_w + 530;
   if(window_w > 1024)
      window_w = 1024;
   gint window_h = topo->bg_h + 160;
   if(window_h > 800)
      window_h = 800;
   gtk_window_resize(GTK_WINDOW(window_data), window_w, window_h);
   GtkWidget *pane = get_widget(xml, "topo_pane");
   gtk_paned_set_position(GTK_PANED(pane), topo->bg_w / 2);
#endif

   nodelist_view_init();
   
   hablist_view_init();

   //initialize the generic view, model, and callbacks
   generic_view_init();
   //initialize the commanding view window, models and callbacks
   commanding_view_init();

   gui_topo_init();
   
   gtk_widget_show_all(window_data);

   filter_model_init();
   filterlist_view_init();

   filter_wizard_init();

   #ifndef MINI_GUI
   // connect all the glade signals now that we have created our widgets
   glade_xml_signal_autoconnect(xml);
#else
   glade_xml_signal_autoconnect(xml);


   g_signal_connect(G_OBJECT(commanding_hablist), "changed",
                    G_CALLBACK(commanding_hablist_changed), NULL);
   g_signal_connect(G_OBJECT(hab_treeview), "cursor_changed",
                    G_CALLBACK(hab_selection_handler), NULL);
#endif
   
}

static gboolean gui_node_selection_task(gpointer data)
{
   if(data == NULL)
   {
      debug("null NODE");
      return FALSE;
   }
   topo_node_t *topo_node = (topo_node_t *)data;
   bionet_hab_t *hab = bionet_cache_lookup_hab(topo_node->hab_type,
					       topo_node->hab_id);

   if(hab != NULL)
   {
      model_select_hab(hab);
	 
   } else
   {
      debug("the hab (cache) this node belongs to is NULL");
   }

   bionet_node_t *node = bionet_cache_lookup_node(topo_node->hab_type,
						  topo_node->hab_id,
						  topo_node->node_id);

   if(node != NULL)
   {
      model_select_node(node);
   } else {
      debug("the node (cache) this node belongs to is NULL");
   }

   return FALSE;
}

/* Set the currently selected node. */
void gui_gtk_set_current_node(topo_node_t *node, gboolean isTopo)
{
   gchar *str;

   if(node == NULL)
   {
      printf("\n\n");
      debug("de-selecting node");
   } else {
      printf("\n\n");
   }
   
   
   // Since the tree emits this signal again we can just call it
   //  and then run when it calls us back
   // TODO: look into a cleaner mechanism for this stuff.
   if(isTopo)
   {
      if(node == NULL)
	 gui_topo_set_current_node(NULL);

      gui_node_selection_task(node);
      return;
   }

   gui_topo_set_current_node(current_node);

   str = g_strdup_printf("Selected Node: %s", node->id);
   gui_gtk_set_status(str);
   g_free(str);

   debug("Selected a new node: %s", node->id);
   
}

/* Set the content of the statusbar. */
// NOTE: You must allocate the passed string, it will be freed with
//       the status changes...
void gui_gtk_set_status(gchar *status)
{
   static guint context_id = 0;
   static gchar *current = NULL;
   
   // If there is already a status message we need to cleanup first
   if(context_id && current) {
      gtk_statusbar_pop(GTK_STATUSBAR(statusbar), context_id);
      g_free(current);
   }
   
   // Now get this random context id and then push the status
   context_id = gtk_statusbar_get_context_id(GTK_STATUSBAR(statusbar),
					     "b.s. context");
   gtk_statusbar_push(GTK_STATUSBAR(statusbar), context_id, status);
   current = status;
}
