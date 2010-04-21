//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <stdbool.h>

#include "cortex.h"
#include "gui_gtk.h" //provides gtk headers and get_widget
#include "bionet.h"
#include "bionet-interface.h"
#include "model_gtk.h"
#include "hablist_view.h"
#include "nodelist_view.h"

GtkTreeStore *hab_model;
GtkWidget *hab_treeview;

static gchar *selected_hab = NULL;

static void hablist_setup_treeview();
static void hablist_add_hab(bionet_hab_t *hab);

void hablist_view_init(void)
{
   hab_treeview = get_widget(xml, "treeview_hablist");
   hablist_setup_treeview();
   model_add_hab_register_func(hablist_add_hab);
   model_select_hab_register_func(hablist_select_hab);
}

static void hablist_setup_treeview(void)
{
   GtkTreeViewColumn *column;
   GtkCellRenderer *renderer;

   // Setup the HAB treeview
   hab_model = gtk_tree_store_new(MAX_HAB_INDEX, G_TYPE_STRING);

   gtk_tree_view_set_model(GTK_TREE_VIEW(hab_treeview),
			   GTK_TREE_MODEL(hab_model));

   g_object_unref(hab_model);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("HAB Name", renderer,
						     "text", HAB_NAME, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(hab_treeview), column);

   //removing (ALL) temporarily
//   GtkTreeIter iter;
//   gtk_tree_store_append(hab_model, &iter, NULL);
//   gtk_tree_store_set(hab_model, &iter, HAB_NAME, "(ALL)", -1);

}

void hablist_add_hab(bionet_hab_t *hab)
{
   GtkTreeIter iter;
   gchar hab_buf[256];
   gchar *item;
   
   g_sprintf(hab_buf, "%s.%s", hab->type, hab->id);

   if(gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(hab_model), &iter, "0")) {
      do {
         gtk_tree_model_get(GTK_TREE_MODEL(hab_model), &iter, HAB_NAME, &item, -1);
         
         if(strcmp(hab_buf, item) == 0) {
            g_free(item);
            debug("Not adding HAB: %s, already in view", hab_buf);
            return;
         }
         g_free(item);
      } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(hab_model), &iter));
   }
   
   gtk_tree_store_append(hab_model, &iter, NULL);
   gtk_tree_store_set(hab_model, &iter, HAB_NAME, hab_buf, -1);

   debug("Got a new HAB: %s", hab_buf);
}

void hablist_select_hab(bionet_hab_t *hab)
{

   debug("hablist selecting a hab");
   
   GtkTreeIter iter;
   gchar *item;

   gchar hab_name[256];
   g_sprintf(hab_name, "%s.%s", hab->type, hab->id);
   
   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(hab_model), &iter, "0"))
      return;

   do {
      gtk_tree_model_get(GTK_TREE_MODEL(hab_model), &iter, HAB_NAME, &item, -1);

      if(strcmp(hab_name, item) == 0)
      {
	 GtkTreePath *path = gtk_tree_model_get_path(GTK_TREE_MODEL(hab_model),
                                                     &iter);

	 gtk_tree_view_set_cursor(GTK_TREE_VIEW(hab_treeview), path, NULL, false);

	 debug("found this hab: %s", hab_name);
         g_free(item);
	 break;
      }
      g_free(item);
   } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(hab_model), &iter));
			 
}



//SIGNAL HANDLERS
gboolean hab_selection_task(gpointer data)
{
   
   gchar *hab_name = (gchar *)data;

   if(selected_hab == NULL)
   {
      selected_hab = hab_name;
   } else {
   
      if(strcmp(hab_name, selected_hab) == 0)
	 return FALSE;
   
      g_free(selected_hab);
      selected_hab = hab_name;
   }
   

   debug("%s", hab_name);

   gchar **hab_tok;
   hab_tok = g_strsplit(hab_name, ".", 0);

   nodelist_view_clear_nodes();

   bionet_hab_t *hab;
   hab = bionet_cache_lookup_hab(hab_tok[0], hab_tok[1]);
   if(hab != NULL)
   {
      model_select_hab_skip(hab, hablist_select_hab);
   } else {
      debug("NULL hab: %s, %s", hab_tok[0], hab_tok[1]);
      model_select_hab(NULL);
   }
   
   g_strfreev(hab_tok);
   
   return FALSE;
}

void hab_selection_handler_helper(GtkTreeModel *model, GtkTreePath *path,
				  GtkTreeIter *iter, gpointer data)
{
   gchar *hab_name;
   
   gtk_tree_model_get(model, iter, HAB_NAME, &hab_name, -1);
   debug("%s", hab_name);

   gchar *arg = g_strdup(hab_name);
   hab_selection_task(arg);
}

void hab_selection_handler(GtkTreeView *treeview,
			   GtkTreePath *arg1,
			   GtkTreeViewColumn *arg2,
			   gpointer user_data)
{
   GtkTreeSelection *selected_list;
   selected_list = gtk_tree_view_get_selection(treeview);
   
   gtk_tree_selection_selected_foreach(selected_list, hab_selection_handler_helper,
				       NULL);  
}



