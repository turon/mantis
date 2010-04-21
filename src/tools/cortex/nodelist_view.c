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
#include "nodelist_view.h"


// all the hab and node data is manipulated with these gtk tree stores
static GtkTreeStore *nodelist_model;
static GtkWidget *nodelist_treeview;

// these 'gumballs' are the status display images... they follow stoplight
// coloring for out of range values
static GdkPixbuf *red_gumball;
static GdkPixbuf *green_gumball;
static GdkPixbuf *yellow_gumball;

static gchar selected_hab [256];

static void nodelist_update_resource(bionet_resource_t *resource);
static void nodelist_setup_treeview();
static void nodelist_add_node(bionet_node_t *node);
static void nodelist_remove_node(bionet_node_t *node);
static void nodelist_select_hab(bionet_hab_t *hab);

void nodelist_view_init(void)
{
   nodelist_treeview = get_widget(xml, "treeview_nodelist");
   nodelist_setup_treeview();
   model_add_node_register_func(nodelist_add_node);
   model_remove_node_register_func(nodelist_remove_node);
   model_update_resource_register_func(nodelist_update_resource);
   model_select_hab_register_func(nodelist_select_hab);
   model_select_node_register_func(nodelist_select_node);
}

void nodelist_setup_treeview(void)
{
   GtkTreeViewColumn *column;
   GtkCellRenderer *renderer;
   GtkTreeViewColumn *pixbuf_column;
   GtkCellRenderer *pixbuf_renderer;

   
   red_gumball = gdk_pixbuf_new_from_file("glade/red_gumball.png", NULL);
   green_gumball = gdk_pixbuf_new_from_file("glade/green_gumball.png", NULL);
   yellow_gumball = gdk_pixbuf_new_from_file("glade/yellow_gumball.png", NULL);   

   
   nodelist_model = gtk_tree_store_new(MAX_NODE_INDEX, G_TYPE_STRING, GDK_TYPE_PIXBUF,
				   G_TYPE_STRING);
   
   gtk_tree_view_set_model(GTK_TREE_VIEW(nodelist_treeview),
			   GTK_TREE_MODEL(nodelist_model));
   g_object_unref(nodelist_model);
   
   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("Node Name", renderer,
						     "text", NODE_NAME, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(nodelist_treeview), column);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("Value", renderer,
						     "text", NODE_VALUE, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(nodelist_treeview), column);

   pixbuf_renderer = gtk_cell_renderer_pixbuf_new();
   pixbuf_column = gtk_tree_view_column_new_with_attributes("Gumball",
							    pixbuf_renderer,
							    "pixbuf",
							    NODE_GUMBALL, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(nodelist_treeview), pixbuf_column);   

}

gboolean node_hab_select_task(gpointer data)
{
   bionet_hab_t *hab = (bionet_hab_t *)data;

   //Iterate list of nodes in hab adding each one
   GSList *p = hab->nodes;
   if(p == NULL) {
      debug("list of nodes is NULL");
   }

   for(; p != NULL; p = p->next) {
      nodelist_add_node((bionet_node_t *)p->data);
   }
   
   return FALSE;
}


void nodelist_select_node(bionet_node_t *node)
{
   GtkTreeIter iter;
   gchar *item;

   gchar node_name [256];
   g_sprintf(node_name, "%s.%s.%s", node->hab_type, node->hab_id, node->id);
   
   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(nodelist_model), &iter, "0"))
      return;

   do {
      gtk_tree_model_get(GTK_TREE_MODEL(nodelist_model), &iter, NODE_NAME, &item, -1);

      if(strcmp(node_name, item) == 0)
      {
	 GtkTreePath *path = gtk_tree_model_get_path(GTK_TREE_MODEL(nodelist_model),
						     &iter);
	 gtk_tree_view_set_cursor(nodelist_treeview, path, NULL, false);
	 debug("found this node: %s", node_name);
	 break;
      }
      g_free(item);
   } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(nodelist_model), &iter));
			 
}


static void nodelist_select_hab(bionet_hab_t *hab)
{
   debug("nodelist select hab");
   //handle deselecting the current node list
   if(hab == NULL) {
      debug("hab was null");
      gtk_tree_store_clear(nodelist_model);
      memset(selected_hab, 0, sizeof(selected_hab));
      return;
   }
   
   //check to see if it's already selected
   gchar new_hab [256];
   g_sprintf(new_hab, "%s.%s", hab->type, hab->id);

   if(strcmp(new_hab, selected_hab) == 0) {
      debug("already selected");
   }
   
   g_sprintf(selected_hab, "%s.%s", hab->type, hab->id);
   node_hab_select_task(hab);
}

static void nodelist_remove_node(bionet_node_t *node)
{

   gchar hab_buf [256];
   g_sprintf(hab_buf, "%s.%s", node->hab_type, node->hab_id);

   if(selected_hab == NULL)
      return;
   
   if(strcmp(hab_buf, selected_hab) != 0 &&
      strcmp("*.*", selected_hab) != 0)
      return;

   //get the string version
   gchar node_id [256];
   g_sprintf(node_id, "%s.%s.%s",
	     node->hab_type,
	     node->hab_id,
	     node->id);

 
   GtkTreeIter iter;

   if(gtk_tree_model_get_iter_first(GTK_TREE_MODEL(nodelist_model), &iter)) {
      gchar *node_name;
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(nodelist_model), &iter,
			    NODE_NAME, &node_name, -1);
	 if(strcmp(node_id, node_name) == 0)
	 {
	    gtk_tree_store_set(nodelist_model, &iter, NODE_GUMBALL, green_gumball, -1);
	    return;
	 }
	 g_free(node_name);
	 
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(nodelist_model), &iter));
   }
   
}

static void nodelist_update_resource(bionet_resource_t *resource)
{

   gchar hab_buf [256];
   g_sprintf(hab_buf, "%s.%s", resource->hab_type, resource->hab_id);

   if(strcmp(hab_buf, selected_hab) != 0 &&
      strcmp("*.*", selected_hab) != 0)
      return;
   
   gchar node_id [256];
   gchar this_resource [256];
   g_sprintf(node_id, "%s.%s.%s", resource->hab_type,
	     resource->hab_id,
	     resource->node_id);

   g_sprintf(this_resource, "%s %s %s", 
	     bionet_resource_data_type_to_string(resource->data_type),
	     bionet_resource_flavor_to_string(resource->flavor),
	     resource->id);
   
   GtkTreeIter iter;
   GtkTreeIter child;
   gchar *item;
   gchar *resource_id;
   gboolean resource_found = false;
   gboolean node_found = false;
   
   //update node model
   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(nodelist_model), &iter, "0"))
      return;

   do {
      gtk_tree_model_get(GTK_TREE_MODEL(nodelist_model), &iter, NODE_NAME, &item, -1);
      //FIXME: why does this need to be set here??? (it's a gumball until mouseover
      //otherwise)
      //gtk_tree_store_set(nodelist_model, &iter, NODE_VALUE, "", -1);
      
      if(strcmp(node_id, item) == 0) {
	 gtk_tree_store_set(nodelist_model, &iter, NODE_GUMBALL, green_gumball, -1);

	 node_found = true;
	 if (gtk_tree_model_iter_children(GTK_TREE_MODEL(nodelist_model), &child, &iter)) {
	    do {
	       gtk_tree_model_get(GTK_TREE_MODEL(nodelist_model), &child, NODE_NAME,
				  &resource_id, -1);

	       if(resource_id == NULL || this_resource == NULL)
	       {
		  debug("resource id was null?");
		  break;
	       }
	       
	       if(strcmp(resource_id, this_resource) == 0) {
		  gtk_tree_store_set(nodelist_model, &child, NODE_VALUE,
				     bionet_resource_value_to_string(resource), -1);
		  resource_found = true;
		  g_free(resource_id);
		  break;
	       }
	       
	       g_free(resource_id);
	    } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(nodelist_model), &child));
	 }
	 g_free(item);
	 break;
      }
	
      g_free(item);
   } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(nodelist_model), &iter));

   if(node_found == false) {
      debug("ERROR: couldn't find a node to add a resource to");
      return;
   }
   
   if(resource_found == false && node_found == true) {
      gtk_tree_store_append (nodelist_model, &child, &iter);
      gtk_tree_store_set(nodelist_model, &child,
			 NODE_NAME, this_resource,
			 NODE_VALUE, bionet_resource_value_to_string(resource),
			 NODE_GUMBALL, yellow_gumball,
			 -1);
   }
}


static void nodelist_add_node(bionet_node_t *node)
{


   gchar hab_buf [256];
   g_sprintf(hab_buf, "%s.%s", node->hab_type, node->hab_id);

   if(strcmp(hab_buf, selected_hab) != 0 &&
      strcmp("*.*", selected_hab) != 0)
      return;
   
   GtkTreeIter iter;
   gchar node_id [256];
   
   g_sprintf(node_id, "%s.%s.%s", node->hab_type, node->hab_id, node->id);

   gboolean node_found = false;
   if(gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(nodelist_model), &iter, "0")) {
      gchar *node_name;
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(nodelist_model), &iter,
			    NODE_NAME, &node_name, -1);
	 if(strcmp(node_id, node_name) == 0)
	 {
	    
	    gtk_tree_store_set(nodelist_model, &iter, NODE_GUMBALL, green_gumball, -1);
	    node_found = true;
	    g_free(node_name);
	    break;
	 }
	 g_free(node_name);
	 
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(nodelist_model), &iter));
   }

   if(!node_found)
   {
      gtk_tree_store_append(nodelist_model, &iter, NULL);
      gtk_tree_store_set(nodelist_model, &iter, NODE_NAME, node_id, NODE_GUMBALL,
			 green_gumball, NODE_VALUE, "", -1);
   }
   
   //update the resources
   GSList *p = node->resources;
   for(; p != NULL; p = p->next)
      nodelist_update_resource((bionet_resource_t *)p->data);
      
}


//SIGNAL HANDLERS

static gboolean node_selection_task(gpointer data)
{
   gchar *node_name = (gchar *)data;

   gchar **node_tok;
   node_tok = g_strsplit(node_name, ".", 0);

   bionet_node_t *node = bionet_cache_lookup_node(node_tok[0],
						  node_tok[1],
						  node_tok[2]);
   model_select_node_skip(node, nodelist_select_node);
   
   g_free(node_name);

   g_strfreev(node_tok);

   return FALSE;
}

void node_selection_handler_helper(GtkTreeModel *model, GtkTreePath *path,
				  GtkTreeIter *iter, gpointer data)
{
   gchar *path_str = gtk_tree_path_to_string(path);
   
   gchar *node_name;
   
   gtk_tree_model_get(model, iter, NODE_NAME, &node_name, -1);
   debug("%s: %s", node_name, path_str);

   if(gtk_tree_store_iter_depth(GTK_TREE_STORE(model), iter) > 0) {
      g_free(node_name);
      return;
   }
   node_selection_task(node_name);
}

void node_selection_handler(GtkTreeView *treeview,
			    GtkTreePath *arg1,
			    GtkTreeViewColumn *arg2,
			    gpointer user_data)
{
   GtkTreeSelection *selected_list;
   selected_list = gtk_tree_view_get_selection(treeview);
   
   gtk_tree_selection_selected_foreach(selected_list, node_selection_handler_helper,
				       NULL);  
}

void nodelist_view_clear_nodes()
{
   
   gtk_tree_store_clear(nodelist_model);
}

gboolean nodelist_event_handler(GtkTextTag *texttag,
				GObject *arg1,
				GdkEvent *event,
				GtkTextIter *arg2,
				gpointer user_data)
{
   return false;
   if(event)
   {
      printf("\n\nevent\n\n");
      if(event->type == GDK_2BUTTON_PRESS)
      {
	 printf("event\n");
      }
   }
   return false;
}
