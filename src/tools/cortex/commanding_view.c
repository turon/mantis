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

GtkWidget *commanding_hablist;
static GtkTreeStore *commanding_hablist_model;

static GtkWidget *commanding_nodelist;
static GtkTreeStore *commanding_nodelist_model;

static GtkWidget *commanding_resourcelist;
static GtkTreeStore *commanding_resourcelist_model;

static GtkWidget *resource_value_entry;

static gchar selected_hab [256];
static gchar selected_node [256];
static gchar selected_resource [256];

extern GSList *bionet_habs;

static void commanding_setup_model();
static void commanding_add_hab(bionet_hab_t *hab);
static void commanding_add_node(bionet_node_t *node);
static void commanding_select_hab(bionet_hab_t *hab);
static void commanding_select_node(bionet_node_t *node);

void commanding_view_init(void)
{
   commanding_hablist = get_widget(xml, "commanding_hablist");
   commanding_nodelist = get_widget(xml, "commanding_nodelist");
   commanding_resourcelist = get_widget(xml, "commanding_resourcelist");
   commanding_setup_model();

   resource_value_entry = get_widget(xml, "resource_value_entry");

   model_add_hab_register_func(commanding_add_hab);
   model_select_hab_register_func(commanding_select_hab);
   model_add_node_register_func(commanding_add_node);
   model_select_node_register_func(commanding_select_node);
  
}

static void commanding_setup_model()
{
   //hab list
   commanding_hablist_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_combo_box_set_model(GTK_COMBO_BOX(commanding_hablist),
			   GTK_TREE_MODEL(commanding_hablist_model));
   g_object_unref(commanding_hablist_model);

   //node list
   commanding_nodelist_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_combo_box_set_model(GTK_COMBO_BOX(commanding_nodelist),
			   GTK_TREE_MODEL(commanding_nodelist_model));
   g_object_unref(commanding_nodelist_model);

   //resource list
   commanding_resourcelist_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_combo_box_set_model(GTK_COMBO_BOX(commanding_resourcelist),
			   GTK_TREE_MODEL(commanding_resourcelist_model));
   g_object_unref(commanding_resourcelist_model);
}

static void commanding_add_hab(bionet_hab_t *hab)
{
   GtkTreeIter iter;
   gchar hab_buf [256];
   
   g_sprintf(hab_buf, "%s.%s", hab->type, hab->id);
   
   gtk_tree_store_append(commanding_hablist_model, &iter, NULL);
   gtk_tree_store_set(commanding_hablist_model, &iter, 0, hab_buf, -1);

}

static void commanding_add_node(bionet_node_t *node)
{
   gchar hab_buf [256];
   g_sprintf(hab_buf, "%s.%s", node->hab_type, node->hab_id);
   
   if(strcmp(hab_buf, selected_hab) != 0)
      return;

   GtkTreeIter iter;
   gchar node_id [256];

   g_sprintf(node_id, "%s.%s.%s", node->hab_type, node->hab_id, node->id);

   gboolean node_found = false;
   if(gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(commanding_nodelist_model),
					  &iter, "0"))
   {
      gchar *node_name;
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(commanding_nodelist_model), &iter,
			    0, &node_name, -1);
	 if(strcmp(node_id, node_name) == 0)
	 {
	    node_found = true;
	    break;
	 }
	 g_free(node_name);
	 
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(commanding_nodelist_model), &iter));
   }

   if(!node_found)
   {
      gtk_tree_store_append(commanding_nodelist_model, &iter, NULL);
      gtk_tree_store_set(commanding_nodelist_model, &iter, 0, node_id,
			 -1);
   }
}

static gboolean select_hab_task(gpointer data)
{
   bionet_hab_t *hab = (bionet_hab_t *)data;
   
   GSList *node_list = hab->nodes;
   
   for(; node_list != NULL; node_list = node_list -> next) {
      bionet_node_t *node = node_list->data;
      commanding_add_node(node);
   }

   return FALSE;
}

static void commanding_select_hab(bionet_hab_t *hab)
{
   //handle deselecting
   if(hab == NULL) {
      gtk_tree_store_clear(commanding_nodelist_model);
      memset(selected_hab, 0, sizeof(selected_hab));
      return;
   }

   gchar new_hab [256];
   g_sprintf(new_hab, "%s.%s", hab->type, hab->id);

   debug("%s.%s", hab->type, hab->id);   

   if(strcmp(new_hab, selected_hab) == 0)
      return;

   g_sprintf(selected_hab, "%s.%s", hab->type, hab->id);

   //select the hab in the combo box
   GtkTreeIter iter;
   if(gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(commanding_hablist_model),
					  &iter,
					  "0"))
   {
      gchar *current_hab;
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(commanding_hablist_model),
			    &iter,
			    0, &current_hab,
			    -1);
	 if(strcmp(current_hab, selected_hab) == 0) {
	    gtk_combo_box_set_active_iter(GTK_COMBO_BOX(commanding_hablist),
					  &iter);
	 }
	 
	 g_free(current_hab);
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(commanding_hablist_model),
						      &iter));
   } else {
      debug("Error iterating hablist!\n");
   }
   //populate the nodelist model
   gtk_tree_store_clear(commanding_nodelist_model);
   select_hab_task(hab);
}

static gboolean select_node_task(gpointer p)
{
   bionet_node_t *node = (bionet_node_t *)p;

   GSList *resource_list = node->resources;
   for(; resource_list != NULL; resource_list = resource_list->next)
   {
      bionet_resource_t *resource = resource_list->data;

      //check to see if it is an actuator or parameter
      if(strcmp("Actuator", bionet_resource_flavor_to_string(resource->flavor)) == 0 ||
	 strcmp("Parameter", bionet_resource_flavor_to_string(resource->flavor)) == 0)
      {
	 GtkTreeIter iter;

	 gtk_tree_store_append(commanding_resourcelist_model, &iter, NULL);
	 gtk_tree_store_set(commanding_resourcelist_model, &iter, 0, resource->id, -1);
	 debug("Resource: %s %s", bionet_resource_data_type_to_string(resource->data_type),
	       bionet_resource_flavor_to_string(resource->flavor));
      }
   }

   return FALSE;
}

static void commanding_select_node(bionet_node_t *node)
{
   gchar new_node [256];
   g_sprintf(new_node, "%s.%s.%s", node->hab_type, node->hab_id, node->id);

   if(strcmp(new_node, selected_node) == 0)
      return;

   g_sprintf(selected_node, "%s.%s.%s", node->hab_type, node->hab_id, node->id);

   //select the node in the combo box
   GtkTreeIter iter;
   if(gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(commanding_nodelist_model),
					  &iter, "0"))
   {
      gchar *current_node;
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(commanding_nodelist_model),
			    &iter,
			    0, &current_node,
			    -1);
	 if(strcmp(current_node, selected_node) == 0)
	    gtk_combo_box_set_active_iter(GTK_COMBO_BOX(commanding_nodelist),
					  &iter);
	 g_free(current_node);
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(commanding_nodelist_model),
				       &iter));
   } else {
      debug("Error iterating nodelist!");
   }
   
   //populate the resourcelist model
   gtk_tree_store_clear(commanding_resourcelist_model);
   select_node_task(node);
}

//SIGNAL HANDLERS
gboolean hablist_changed_task(gpointer data)
{
   gchar *hab_name = (gchar *)data;
   
   gchar **hab_tok = g_strsplit(hab_name, ".", 0);
   bionet_hab_t *hab = bionet_cache_lookup_hab(hab_tok[0], hab_tok[1]);

   if(hab == NULL) {
      debug("cache lookup returned NULL");
   } else {
      model_select_hab(hab);
   }
   
   g_free(hab_name);
   g_strfreev(hab_tok);
      
   return FALSE;
}

void commanding_hablist_changed(GtkComboBox *widget, gpointer user_data)
{
   gchar *hab_name;
   GtkTreeIter iter;
   GtkTreeModel *model;

   model = gtk_combo_box_get_model(GTK_COMBO_BOX(widget));
   if(gtk_combo_box_get_active_iter(GTK_COMBO_BOX(widget), &iter))
   {
      gtk_tree_model_get (model, &iter, 0, &hab_name, -1);
      hablist_changed_task(hab_name);
   }
}

static gboolean nodelist_changed_helper(gpointer data)
{
   gchar *node_name = (gchar *)data;
   GSList *nodes;

   GSList *hab_p = bionet_habs;
   for(; hab_p != NULL; hab_p = hab_p -> next)
   {
      bionet_hab_t *hab = (bionet_hab_t *)hab_p->data;
      GSList *node_p = hab->nodes;
      for(; node_p != NULL; node_p = node_p -> next)
      {
	 bionet_node_t *node = (bionet_node_t *)node_p->data;
	 if(bionet_node_matches_id(node, node_name))
	    model_select_node(node);
      }
   }
   g_free(node_name);

   return FALSE;
}

void commanding_nodelist_changed(GtkComboBox *widget,
				 gpointer user_data)

{
   gchar *node_name;
   GtkTreeIter iter;
   GtkTreeModel *model;

   model = gtk_combo_box_get_model(GTK_COMBO_BOX(widget));
   
   if(gtk_combo_box_get_active_iter(GTK_COMBO_BOX(widget), &iter))
   {
      gtk_tree_model_get(model, &iter, 0, &node_name, -1);
      nodelist_changed_helper(node_name);
   }
}

void commanding_resourcelist_changed(GtkComboBox *widget,
				     gpointer user_data)
{
   printf("commanding resourcelist changed\n");
   GtkTreeIter iter;
   GtkTreeModel *model;

   gchar *new_resource;

   model = gtk_combo_box_get_model(GTK_COMBO_BOX(widget));
   if(gtk_combo_box_get_active_iter(GTK_COMBO_BOX(widget), &iter))
   {
      gtk_tree_model_get (model, &iter, 0, &new_resource, -1);

      printf("selecting resource: %s\n", new_resource);
      strncpy(selected_resource, new_resource, sizeof(selected_resource));
      
      g_free(new_resource);
   }
}

void commanding_send_setting_clicked(GtkButton *button)
{
   gchar resource_id [256];
   g_sprintf(resource_id, "%s:%s", selected_node, selected_resource);

   bionet_set_resource_by_name_pattern(resource_id,
				       gtk_entry_get_text(GTK_ENTRY(resource_value_entry)));
}
