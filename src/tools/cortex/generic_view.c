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
#include "model_gtk.h"
#include "generic_view.h"

static GtkTreeStore *generic_model;
static GtkWidget *generic_treeview;

static void generic_update_resource(bionet_resource_t *resource);
static void generic_view_add_node(bionet_node_t *node);
static void generic_setup_treeview(void);
static void generic_view_add_hab(bionet_hab_t *hab);

void generic_view_init(void)
{
   generic_treeview = get_widget(xml, "treeview_generic");
   generic_setup_treeview();
   model_add_hab_register_func(generic_view_add_hab);
   model_add_node_register_func(generic_view_add_node);
   model_update_resource_register_func(generic_update_resource);
}

static void generic_setup_treeview(void)
{
   GtkTreeViewColumn *column;
   GtkCellRenderer *renderer;

   //Setup the Generic treeview
   generic_model = gtk_tree_store_new(MAX_GENERIC_INDEX, G_TYPE_STRING, G_TYPE_STRING);
   
   gtk_tree_view_set_model(GTK_TREE_VIEW(generic_treeview), GTK_TREE_MODEL(generic_model));
   g_object_unref(generic_model);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("HAB Name", renderer, "text", 
						     GENERIC_NODE_NAME, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(generic_treeview), column);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("Value", renderer,
						     "text", GENERIC_NODE_VALUE, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(generic_treeview), column);

}

static void generic_view_add_hab(bionet_hab_t *hab)
{
   GtkTreeIter iter;
   gchar hab_buf [256];
   
   g_sprintf(hab_buf, "%s.%s", hab->type, hab->id);
   
   gtk_tree_store_append(generic_model, &iter, NULL);
   gtk_tree_store_set(generic_model, &iter, GENERIC_NODE_NAME, hab_buf, -1);
}

/*
void generic_view_close_window_button_clicked(GtkButton *button, gpointer data)
{
   gtk_widget_hide();
}
*/

static void generic_update_resource(bionet_resource_t *resource)
{
   gchar node_buf [256];
   gchar hab_buf [256];
   gboolean hab_found = false;
   gchar this_resource [256];

   g_sprintf(hab_buf, "%s.%s", resource->hab_type,
	     resource->hab_id);

   g_sprintf(node_buf, "%s.%s.%s", resource->hab_type,
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


   //update generic model
   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(generic_model), &iter, "0"))
      return;

   //locate the hab
   do {
      gchar *hab_name;
      gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &iter, GENERIC_NODE_NAME, &hab_name, -1);
      if(strcmp(hab_buf, hab_name) == 0) {
	 hab_found = true;
	 g_free(hab_name);
	 break;
      }
      g_free(hab_name);
   } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &iter));
   
   if(hab_found == false) {
      debug("Update resource for unknown hab: %s", node_buf);
      return;
   }
     

   //locate the node
   gboolean node_found = false;
   gchar *node_id;
   if(gtk_tree_model_iter_children(GTK_TREE_MODEL(generic_model), &child, &iter)) {
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &child, GENERIC_NODE_NAME,
			    &node_id, -1);
	 if(strcmp(node_id, node_buf) == 0){	
	    node_found = true;
	    break;
	 } 
	 g_free(node_id);
      } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &child));
       
   }
   else {
      debug("Update resource, hab has no children.");
      return;
   }

   if(node_found == false) {
      debug("Update resource, node not found.");
      return;
   }

   memcpy((void *)&iter, (void *)&child, sizeof(child));

   do {
      gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &iter, GENERIC_NODE_NAME, &item, -1);
      //FIXME: why does this need to be set here??? (it's a gumball until mouseover
      //otherwise)
      //gtk_tree_store_set(generic_model, &iter, GENERIC_NODE_VALUE, "", -1);
      if(strcmp(node_id, item) == 0)
	{
	  if(!gtk_tree_model_iter_children(GTK_TREE_MODEL(generic_model), &child, &iter))
	    {
	      //node has no children
	      g_free(item);
	      break;
	    }
	  do {
	    gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &child, GENERIC_NODE_NAME,
			       &resource_id, -1);
	    if(strcmp(resource_id, this_resource) == 0){
	      gtk_tree_store_set(generic_model, &child, GENERIC_NODE_VALUE, 
				 bionet_resource_value_to_string(resource), -1);
	      resource_found = true;
	      g_free(resource_id);
	      break;
	    }
	    g_free(resource_id);
	  } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &child));
	  
	  g_free(item);
	  break;
	}
      g_free(item);
   } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &iter));
   
   if(resource_found == false) {
      gtk_tree_store_append (generic_model, &child, &iter);
      gtk_tree_store_set(generic_model, &child,
			 GENERIC_NODE_NAME, this_resource,
			 GENERIC_NODE_VALUE, bionet_resource_value_to_string(resource),
			 -1);
   }

}



static void generic_view_add_node(bionet_node_t *node)
{
   GtkTreeIter iter;
   GtkTreeIter child;
   gchar node_buf [256];
   gchar hab_buf [256];
   gboolean hab_found = false;

   debug("%s.%s.%s", node->hab_type, node->hab_id, node->id);
   g_sprintf(hab_buf, "%s.%s", node->hab_type, node->hab_id);
   g_sprintf(node_buf, "%s.%s.%s", node->hab_type, node->hab_id, node->id);

   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(generic_model), &iter, "0")) {
      debug("generic view is unpopulated!");
      return;
   }

   gchar *hab_name;
   do {
      gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &iter, GENERIC_NODE_NAME, &hab_name,
			 -1);
      if(strcmp(hab_buf, hab_name) == 0) {
	 //node is already in tree
	 hab_found = true;
	 break;
      }
      g_free(hab_name);     
   } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &iter));
   

   if(hab_found == false) {
      debug("New node for unknown hab: %s %s", hab_buf, node_buf);
      return;
   }
   
   gchar *node_id;

   //iterate hab's children to see if node exists
   if(gtk_tree_model_iter_children(GTK_TREE_MODEL(generic_model), &child, &iter)) {
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(generic_model), &child, GENERIC_NODE_NAME,
			    &node_id, -1);
	 if(strcmp(node_id, node_buf) == 0){	
	    debug("New node already exists!");
	    return;
	 } 
	 g_free(node_id);
      } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(generic_model), &child));
      
      debug("appending to already there child");
      gtk_tree_store_append (generic_model, &child, &iter);
      gtk_tree_store_set(generic_model, &child, GENERIC_NODE_NAME, node_buf, -1);
   }
   else {
      debug("appending new child");
      gtk_tree_store_append (generic_model, &child, &iter);
      gtk_tree_store_set(generic_model, &child, GENERIC_NODE_NAME, node_buf, -1);
   }
}
