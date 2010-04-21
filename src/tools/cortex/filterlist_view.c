
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

#include "bionet-hab.h"

#include "bionet-interface.h"
#include "filterlist_view.h"
#include "model_gtk.h"
#include "filter_model.h"
#include "filter_wizard.h"

static void setup_filterlist_view_model();

static GtkWidget *filterlist_combobox;
static GtkTreeStore *filterlist_combobox_model;

static GtkWidget *filterlist_view_hablist;
static GtkTreeStore *filterlist_view_hablist_model;

static GtkWidget *filterlist_view_nodelist;
static GtkTreeStore *filterlist_view_nodelist_model;

static GtkWidget *window_filter_wizard;

static GtkWidget *edit_button;

static GSList *constraints;

extern GSList *bionet_habs;

// these 'gumballs' are the status display images... they follow stoplight
// coloring for out of range values
GdkPixbuf *red_gumball;
GdkPixbuf *green_gumball;
GdkPixbuf *yellow_gumball;

static gchar selected_hab [256];
static gchar selected_node [256];

static void filterlist_view_update_nodes_from_hab(bionet_hab_t *hab);

//for updating the list of filters
static void filterlist_view_add_filter(filter_list_t *filter_list);
static void filterlist_view_remove_filter(filter_list_t *filter_list);

static void filterlist_view_add_hab(bionet_hab_t *hab);
static void filterlist_view_add_node(bionet_node_t *node);
static void filterlist_view_update_resource(bionet_resource_t *resource);
static void filterlist_view_select_hab(bionet_hab_t *hab);
static void filterlist_view_select_node(bionet_node_t *node);

static gboolean selected = false;

//for checking constraints
static gboolean constraints_check_hab(bionet_hab_t *hab);
static gboolean constraints_check_node(bionet_node_t *node);
static gboolean constraints_check_resource(bionet_resource_t *resource);
   
void filterlist_view_init(void)
{
   
   edit_button = get_widget(xml, "btn_edit_filterlist");
   
   window_filter_wizard = get_widget(xml, "window_filter_wizard");
  
   filterlist_combobox = get_widget(xml, "filterlist_combobox");

   filterlist_view_hablist = get_widget(xml, "filterlist_view_hablist");
   filterlist_view_nodelist = get_widget(xml, "filterlist_view_nodelist");

   setup_filterlist_view_model();

   filter_model_add_filter_register_func(filterlist_view_add_filter);
   filter_model_remove_filter_register_func(filterlist_view_remove_filter);

   model_update_resource_register_func(filterlist_view_update_resource);
   model_add_node_register_func(filterlist_view_add_node);
   model_add_hab_register_func(filterlist_view_add_hab);

   model_select_hab_register_func(filterlist_view_select_hab);
   model_select_node_register_func(filterlist_view_select_node);
   
}

static void setup_filterlist_view_model()
{
   //create the model for the list of filters
   filterlist_combobox_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_combo_box_set_model(GTK_COMBO_BOX(filterlist_combobox),
			   GTK_TREE_MODEL(filterlist_combobox_model));
   g_object_unref(filterlist_combobox_model);

   //populate the model
   GtkTreeIter iter;
   GSList *p = get_filter_list();

   gboolean select_first = false;
   if(p != NULL)
      select_first = true;  
   
   while(p != NULL)
   {
      filter_list_t *filter_list = (filter_list_t *)p->data;

      gtk_tree_store_append(filterlist_combobox_model, &iter, NULL);
      gtk_tree_store_set(filterlist_combobox_model, &iter,
			 0, filter_list->name,
			 -1);
      p = p -> next;
   }

   //setup the hablist view
   GtkTreeViewColumn *column;
   GtkCellRenderer *renderer;

   filterlist_view_hablist_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_tree_view_set_model(GTK_TREE_VIEW(filterlist_view_hablist),
			   GTK_TREE_MODEL(filterlist_view_hablist_model));
   g_object_unref(filterlist_view_hablist_model);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("HAB Name", renderer,
						     "text", 0, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(filterlist_view_hablist), column);

   //setup the nodelist view
   GtkTreeViewColumn *pixbuf_column;
   GtkCellRenderer *pixbuf_renderer;

   //load the images
   red_gumball = gdk_pixbuf_new_from_file("glade/red_gumball.png", NULL);
   green_gumball = gdk_pixbuf_new_from_file("glade/green_gumball.png", NULL);
   yellow_gumball = gdk_pixbuf_new_from_file("glade/yellow_gumball.png", NULL);   

   filterlist_view_nodelist_model = gtk_tree_store_new(FILTERLIST_MAX_NODE_INDEX,
						       G_TYPE_STRING,
						       GDK_TYPE_PIXBUF,
						       G_TYPE_STRING);

   gtk_tree_view_set_model(GTK_TREE_VIEW(filterlist_view_nodelist),
			   GTK_TREE_MODEL(filterlist_view_nodelist_model));
   g_object_unref(filterlist_view_nodelist_model);

   //add the columns
   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("Node Name", renderer,
						     "text", FILTERLIST_NODE_NAME, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(filterlist_view_nodelist), column);

   renderer = gtk_cell_renderer_text_new();
   column = gtk_tree_view_column_new_with_attributes("Value", renderer,
						     "text", FILTERLIST_NODE_VALUE, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(filterlist_view_nodelist), column);

   pixbuf_renderer = gtk_cell_renderer_pixbuf_new();
   pixbuf_column = gtk_tree_view_column_new_with_attributes("Gumball",
							    pixbuf_renderer,
							    "pixbuf",
							    FILTERLIST_NODE_GUMBALL, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(filterlist_view_nodelist), pixbuf_column);   

   if(select_first)
      gtk_combo_box_set_active(GTK_COMBO_BOX(filterlist_combobox), 0);
}

static void filterlist_view_add_filter(filter_list_t *filter_list)
{
   if(filter_list == NULL)
      return;
   GtkTreeIter iter;
   gtk_tree_store_append(filterlist_combobox_model, &iter, NULL);
   gtk_tree_store_set(filterlist_combobox_model, &iter,
		      0, filter_list->name, -1);
   
   if(selected == false)
   {
      gtk_combo_box_set_active(GTK_COMBO_BOX(filterlist_combobox), 0);
      selected = true;
   }
}
static void filterlist_view_remove_filter(filter_list_t *filter_list)
{
   
}

static void filterlist_view_add_hab(bionet_hab_t *hab)
{
   debug("GOT add hab");
   if(!constraints_check_hab(hab))
      return;
   
   GtkTreeIter iter;
   gchar hab_buf [256];
   g_sprintf(hab_buf, "%s.%s", hab->type, hab->id);
   
   gchar *hab_id;
   
   gboolean hab_found = false;
   if(gtk_tree_model_get_iter_first(GTK_TREE_MODEL(filterlist_view_hablist_model), &iter))
   {
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(filterlist_view_hablist_model), &iter,
			    0, &hab_id, -1);
	 
	 if(hab_id == NULL)
	    return;

	 if(strcmp(hab_id,hab_buf) == 0)
	 {
	    hab_found = true;
	    debug("Hab already in tree");
	    
	    break;
	 }
	 
	 g_free(hab_id);
	 
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(filterlist_view_hablist_model),
				       &iter));
   }
   if(hab_found == false)
   {
      debug("adding hab to filtered list");
      gtk_tree_store_append(filterlist_view_hablist_model, &iter, NULL);
      gtk_tree_store_set(filterlist_view_hablist_model, &iter, 0, hab_buf, -1);
      
   }

   filterlist_view_update_nodes_from_hab(hab);

}

static void filterlist_view_add_node(bionet_node_t *node)
{


   gchar hab_id [256];
   g_sprintf(hab_id, "%s.%s", node->hab_type, node->hab_id);

   if(selected_hab == NULL)
      return;
   
   if(strcmp(hab_id, selected_hab) != 0)
      return;

   if(!constraints_check_node(node))
      return;

   debug("past beginning");
   
   GtkTreeIter iter;
   gchar node_buf [256];
   g_sprintf(node_buf, "%s.%s.%s",
	     node->hab_type,
	     node->hab_id,
	     node->id);
   
   gchar *node_id;
   
   gboolean node_found = false;
   
   if(gtk_tree_model_get_iter_first(GTK_TREE_MODEL(filterlist_view_nodelist_model), &iter))
   {
      do {
	 gtk_tree_model_get(GTK_TREE_MODEL(filterlist_view_nodelist_model), &iter,
			    0, &node_id, -1);
	 
	 
	 if(strcmp(node_id,node_buf) == 0)
	 {
	    node_found = true;
	    debug("Node already in tree");
	    
	    break;
	 }
	 
	 g_free(node_id);
	 
      } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(filterlist_view_nodelist_model),
				       &iter));
   }
   if(node_found == false)
   {
      debug("adding node to filtered list");
      gtk_tree_store_append(filterlist_view_nodelist_model, &iter, NULL);
      gtk_tree_store_set(filterlist_view_nodelist_model, &iter,
			 FILTERLIST_NODE_NAME, node_buf,
			 FILTERLIST_NODE_VALUE, "",
			 FILTERLIST_NODE_GUMBALL, green_gumball,
			 -1);
   }
   
   //update resources
   GSList *p = node->resources;
   for(; p != NULL; p = p -> next)
      filterlist_view_update_resource((bionet_resource_t *)p->data);
}

static void filterlist_view_update_resource(bionet_resource_t *resource)
{

   if(selected_hab == NULL || resource == NULL)
      return;
   
   gchar hab_id [256];
   g_sprintf(hab_id, "%s.%s", resource->hab_type, resource->hab_id);
   if(strcmp(hab_id, selected_hab) != 0)
      return;

   if(!constraints_check_resource(resource))
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
   if(!gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(filterlist_view_nodelist_model),
					   &iter, "0"))
      return;

   do {
      gtk_tree_model_get(GTK_TREE_MODEL(filterlist_view_nodelist_model), &iter,
			 FILTERLIST_NODE_NAME, &item, -1);
      //FIXME: why does this need to be set here??? (it's a gumball until mouseover
      //otherwise)
      //gtk_tree_store_set(filterlist_view_nodelist_model, &iter, FILTERLIST_NODE_VALUE, "", -1);
      
      if(strcmp(node_id, item) == 0) {
	 node_found = true;
	 if (gtk_tree_model_iter_children(GTK_TREE_MODEL(filterlist_view_nodelist_model),
					  &child, &iter)) {
	    do {
	       gtk_tree_model_get(GTK_TREE_MODEL(filterlist_view_nodelist_model), &child,
				  FILTERLIST_NODE_NAME, &resource_id, -1);
	       if(resource_id == NULL)
	       {
		  debug("null resource id?");
		  continue;
	       }
	       if(strcmp(resource_id, this_resource) == 0){
		  gtk_tree_store_set(filterlist_view_nodelist_model, &child, FILTERLIST_NODE_VALUE,
				     bionet_resource_value_to_string(resource), -1);
		  resource_found = true;
		  g_free(resource_id);
		  break;
	       }
	       
	       g_free(resource_id);
	    } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(filterlist_view_nodelist_model),
					      &child));
	 }
	 g_free(item);
	 break;
      }
	
      g_free(item);
   } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(filterlist_view_nodelist_model), &iter));

   if(node_found == false) {
      debug("ERROR: couldn't find a node to add a resource to");
      return;
   }
   
   if(resource_found == false && node_found == true) {
      gtk_tree_store_append (filterlist_view_nodelist_model, &child, &iter);
      gtk_tree_store_set(filterlist_view_nodelist_model, &child,
			 FILTERLIST_NODE_NAME, this_resource,
			 FILTERLIST_NODE_VALUE, bionet_resource_value_to_string(resource),
			 FILTERLIST_NODE_GUMBALL, yellow_gumball,
			 -1);
   }
					       
}

static void filterlist_view_update_nodes_from_hab(bionet_hab_t *hab)
{

   GSList *p;
   for(p = hab->nodes; p != NULL; p = p -> next)
   {
      debug("found a node in this list");
      filterlist_view_add_node((bionet_node_t *)p->data);
   }

   
}


static gboolean filterlist_node_selection_task(gpointer data)
{
   gchar *node_name = (gchar *)data;

   gchar **node_tok;
   node_tok = g_strsplit(node_name, ".", 0);
   
   bionet_node_t *node;
   node = bionet_cache_lookup_node(node_tok[0],
				   node_tok[1],
				   node_tok[2]);

   model_select_node_skip(node,
			  filterlist_view_select_node);

   g_strfreev(node_tok);
   
   g_free(node_name);

   return FALSE;
}


void filterlist_node_selection_handler_helper(GtkTreeModel *model, GtkTreePath *path,
				  GtkTreeIter *iter, gpointer data)
{
   gchar *path_str = gtk_tree_path_to_string(path);
   
   gchar *node_name;
   
   gtk_tree_model_get(model, iter, FILTERLIST_NODE_NAME, &node_name, -1);
   debug("%s: %s", node_name, path_str);

   if(gtk_tree_store_iter_depth(filterlist_view_nodelist_model, iter) > 0) {
      g_free(node_name);
      return;
   }
   filterlist_node_selection_task(node_name);
}

void filterlist_node_selection_handler(GtkTreeView *treeview,
			    GtkTreePath *arg1,
			    GtkTreeViewColumn *arg2,
			    gpointer user_data)
{
   GtkTreeSelection *selected_list;
   selected_list = gtk_tree_view_get_selection(treeview);
   
   gtk_tree_selection_selected_foreach(selected_list, filterlist_node_selection_handler_helper,
				       NULL);  
}


static void filterlist_view_select_hab(bionet_hab_t *hab)
{
   debug("filterlist selecting a hab");
   
   GtkTreeIter iter;
   gchar *item;

   gchar hab_name [256];
   g_sprintf(hab_name, "%s.%s", hab->type, hab->id);

   if(strcmp(hab_name, selected_hab) == 0)
      return;

   g_sprintf(selected_hab, "%s.%s", hab->type, hab->id);

   if(!gtk_tree_model_get_iter_from_string(
	 GTK_TREE_MODEL(filterlist_view_hablist_model), &iter, "0"))
      return;

   do {
      gtk_tree_model_get(
	 GTK_TREE_MODEL(filterlist_view_hablist_model),
	 &iter, FILTERLIST_HAB_NAME, &item, -1);

      if(strcmp(hab_name, item) == 0)
      {
	 GtkTreePath *path = gtk_tree_model_get_path(
	    GTK_TREE_MODEL(filterlist_view_hablist_model),
	    &iter);

	 gtk_tree_view_set_cursor(GTK_TREE_VIEW(filterlist_view_hablist),
				  path, NULL, false);

	 debug("found this hab: %s", hab_name);
	 break;
      }
      g_free(item);
   } while (gtk_tree_model_iter_next(
	       GTK_TREE_MODEL(filterlist_view_hablist_model), &iter));
}

static void filterlist_view_select_node(bionet_node_t *node)
{
   debug("filterlist selecting a node");
   
   GtkTreeIter iter;
   gchar *item;

   gchar node_name [256];
   g_sprintf(node_name, "%s.%s.%s",
	     node->hab_type,
	     node->hab_id,
	     node->id);

   if(strcmp(node_name, selected_node) == 0)
      return;

   g_sprintf(selected_node, "%s.%s.%s",
	     node->hab_type,
	     node->hab_id,
	     node->id);
   
   if(!gtk_tree_model_get_iter_from_string(
	 GTK_TREE_MODEL(filterlist_view_nodelist_model),
	 &iter, "0"))
      return;

   do {
      gtk_tree_model_get(
	 GTK_TREE_MODEL(filterlist_view_nodelist_model),
	 &iter, FILTERLIST_NODE_NAME, &item, -1);

      if(strcmp(node_name, item) == 0)
      {
	 GtkTreePath *path = gtk_tree_model_get_path(
	    GTK_TREE_MODEL(filterlist_view_nodelist_model),
	    &iter);
	 gtk_tree_view_set_cursor(GTK_TREE_VIEW(filterlist_view_nodelist),
				  path, NULL, false);
	 debug("found this node: %s", node_name);
	 break;
      }
      g_free(item);
   } while (gtk_tree_model_iter_next(
	       GTK_TREE_MODEL(filterlist_view_nodelist_model), &iter));
   
}

static gboolean constraints_check_hab(bionet_hab_t *hab)
{
   GSList *p = constraints;
   for(;p != NULL; p=p->next)
   {
      constraint_t *constraint = (constraint_t *)p->data;
      
      if((strcmp("*", constraint->hab_type) == 0 ||
	  strcmp(constraint->hab_type, hab->type) == 0) &&
	 (strcmp("*", constraint->hab_id) == 0 ||
	  strcmp(constraint->hab_id, hab->id) == 0))
	 return true;
   }
   return false;
}

static gboolean constraints_check_node(bionet_node_t *node)
{
   GSList *p = constraints;
   for(;p != NULL; p = p->next)
   {
      constraint_t *constraint = (constraint_t *)p->data;
      if(constraint)
	if((strcmp("*", constraint->hab_type) == 0 ||
	    strcmp(constraint->hab_type, node->hab_type) == 0) &&
	   (strcmp("*", constraint->hab_id) == 0 ||
	    strcmp(constraint->hab_id, node->hab_id) == 0) &&
	   (strcmp("*", constraint->node_id) == 0 ||
	    strcmp(constraint->node_id, node->id) == 0))
	  return true;
   } 
   return false;
}

static gboolean constraints_check_resource(bionet_resource_t *resource)
{
   GSList *p = constraints;
   for(;p != NULL; p = p->next)
   {
      constraint_t *constraint = (constraint_t *)p->data;
      if(constraint == NULL) {
	 return false;
      }
      
      if((strcmp("*", constraint->hab_type) == 0 ||
	  strcmp(constraint->hab_type, resource->hab_type) == 0) &&
	 (strcmp("*", constraint->hab_id) == 0 ||
	  strcmp(constraint->hab_id, resource->hab_id) == 0) &&
	 (strcmp("*", constraint->node_id) == 0 ||
	  strcmp(constraint->node_id, resource->id) == 0) &&
	 (strcmp("*", constraint->resource) == 0 ||
	  strcmp(constraint->resource, resource->id) == 0)) {
	 return true;
      }
      
   }
   return false;
}

//SIGNAL HANDLERS
gboolean combobox_selection_task(gpointer data)
{
   gchar hab_name [256];

   gtk_tree_store_clear(filterlist_view_hablist_model);

   GSList *cur = constraints;
   while(cur != NULL)
   {
      constraint_t *constraint = (constraint_t *)cur->data;

      GSList *p = bionet_habs;
      for(; p != NULL; p = p -> next)
	 if(bionet_hab_matches_type_and_id((bionet_hab_t *)p->data,
					   constraint->hab_type,
					   constraint->hab_id))
	    filterlist_view_add_hab((bionet_hab_t *)p->data);

      cur = cur->next;
   }
   return FALSE;
}

void filterlist_combobox_handler(GtkComboBox *box, gpointer data)
{

   //determine the constraints
   gchar *filter_name;
   GtkTreeIter iter;

   gtk_combo_box_get_active_iter(GTK_COMBO_BOX(filterlist_combobox), &iter);
   gtk_tree_model_get (GTK_TREE_MODEL(filterlist_combobox_model), &iter, 0, &filter_name, -1);

   if(strcmp(filter_name, "None") == 0)
   {
      gtk_widget_set_sensitive(edit_button, FALSE);
   } else {
      gtk_widget_set_sensitive(edit_button, TRUE);
   }

   filter_list_t *flist = filter_model_find_filter_list(filter_name);
   
   g_free(filter_name);

   if(flist == NULL)
   {
      constraints = NULL;
      return;
   } else {
      constraints = flist->constraint_list;
   }

   gtk_tree_store_clear(filterlist_view_nodelist_model);
   //gtk_tree_store_clear(filterlist_view_hablist_model);
   combobox_selection_task(NULL);
}



void edit_filterlist_callback(GtkButton *button)
{
   gchar *filter_name;
   GtkTreeIter iter;

   gtk_combo_box_get_active_iter(GTK_COMBO_BOX(filterlist_combobox), &iter);
   gtk_tree_model_get (GTK_TREE_MODEL(filterlist_combobox_model), &iter, 0, &filter_name, -1);


   filter_list_t *flist = filter_model_find_filter_list(filter_name);

   if(flist != NULL)
   {
      filter_wizard_edit_filterlist(filter_name);
      gtk_widget_show(window_filter_wizard);
   }
   
   g_free(filter_name);
   
}

gboolean filterlist_hab_selection_task(gpointer data)
{
   gchar *hab_name = (gchar *)data;

   debug("%s", hab_name);

   g_sprintf(selected_hab, "%s", hab_name);


   gchar **hab_tok;
   hab_tok = g_strsplit(hab_name, ".", 0);

   bionet_hab_t *hab = bionet_cache_lookup_hab(hab_tok[0], hab_tok[1]);
   if(hab != NULL)
   {
      gtk_tree_store_clear(filterlist_view_nodelist_model);
      filterlist_view_update_nodes_from_hab(hab);
      model_select_hab_skip(hab, filterlist_view_select_hab);
   } else {
      gtk_tree_store_clear(filterlist_view_nodelist_model);
      debug("NULL hab: %s, %s", hab_tok[0], hab_tok[1]);
      model_select_hab_skip(NULL, filterlist_view_select_hab);
   }
   
   g_free(hab_name);
   g_strfreev(hab_tok);
   
   return FALSE;
}

void filterlist_view_hab_selection_handler_helper(GtkTreeModel *model, GtkTreePath *path,
						  GtkTreeIter *iter, gpointer data)
{
   gchar *hab_name;
   
   gtk_tree_model_get(model, iter, FILTERLIST_HAB_NAME, &hab_name, -1);
   debug("%s", hab_name);

   gchar *arg = g_strdup(hab_name);
   filterlist_hab_selection_task(arg);

}

void filterlist_view_hab_selection_handler(GtkTreeView *treeview,
					   gpointer user_data)
{
   GtkTreeSelection *selected_list;
   selected_list = gtk_tree_view_get_selection(treeview);
   
   gtk_tree_selection_selected_foreach(selected_list, filterlist_view_hab_selection_handler_helper,
				       NULL);  

}
