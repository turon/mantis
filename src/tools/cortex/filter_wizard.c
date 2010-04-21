//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)


#include "cortex.h"
#include "filter_wizard.h"
#include "filter_model.h"
#include "gui_gtk.h"

static GtkWidget *combo_box_filter_list_name;
static GtkTreeStore *filter_list_name_model;

static GtkWidget *treeview_constraints;
static GtkTreeStore *constraint_model;

static GtkWidget *radiobutton_existing_filter_list;
static GtkWidget *radiobutton_new_filter_list;

static GtkWidget *entry_hab_type;
static GtkWidget *entry_hab_id;
static GtkWidget *entry_node_id;
static GtkWidget *entry_resource;

static GtkWidget *entry_new_name;

static GtkWidget *filter_wizard_notebook;

static gchar *filter_name;

static void filter_list_init();
static void show_constraints();
static void save_constraints();
static void filter_list_setup_model();

static void filter_wizard_view_add_filter(filter_list_t *filter_list);
static void filter_wizard_view_remove_filter(filter_list_t *filter_list);

static gboolean load_filter_from_name(gchar *name);

void filter_wizard_init()
{

   filter_wizard_notebook = get_widget(xml, "build_filter_notebook");
   
   combo_box_filter_list_name = get_widget(xml, "combobox_filter_wizard_filter_list");
   treeview_constraints = get_widget(xml, "treeview_constraints");
   radiobutton_existing_filter_list = get_widget(xml, "radiobutton_existing_filter_list");
   radiobutton_new_filter_list = get_widget(xml, "radiobutton_new_filter_list");

   entry_hab_type = get_widget(xml, "entry_hab_type");
   entry_hab_id = get_widget(xml, "entry_hab_id");
   entry_node_id = get_widget(xml, "entry_node_id");
   entry_resource = get_widget(xml, "entry_resource");

   entry_new_name = get_widget(xml, "entry_new_name");
   
   filter_list_setup_model();
   
   filter_list_init();

   filter_model_add_filter_register_func(filter_wizard_view_add_filter);
   
}

static void filter_list_setup_model()
{

   
   //setup the existing filter list model
   filter_list_name_model = gtk_tree_store_new(1, G_TYPE_STRING);
   gtk_combo_box_set_model(GTK_COMBO_BOX(combo_box_filter_list_name),
			   GTK_TREE_MODEL(filter_list_name_model));
   g_object_unref(filter_list_name_model);


   //setup the constraint tree view model
   constraint_model = gtk_tree_store_new(MAX_CONSTRAINT_INDEX,
				       G_TYPE_STRING,
				       G_TYPE_STRING,
				       G_TYPE_STRING,
				       G_TYPE_STRING);
   gtk_tree_view_set_model(GTK_TREE_VIEW(treeview_constraints),
			   GTK_TREE_MODEL(constraint_model));

   GtkCellRenderer *renderer;
   GtkTreeViewColumn *column;
   
   renderer = gtk_cell_renderer_text_new();

   //hab type
   column = gtk_tree_view_column_new_with_attributes("HAB Type", renderer,
						     "text", HAB_TYPE, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(treeview_constraints),
			       column);

   //hab id
   column = gtk_tree_view_column_new_with_attributes("HAB Id", renderer,
						     "text", HAB_ID, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(treeview_constraints),
			       column);

   //node id
   column = gtk_tree_view_column_new_with_attributes("Node Id", renderer,
						     "text", NODE_ID, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(treeview_constraints),
			       column);

   //resource
   column = gtk_tree_view_column_new_with_attributes("Resource", renderer,
						     "text", RESOURCE, NULL);
   gtk_tree_view_append_column(GTK_TREE_VIEW(treeview_constraints),
			       column);


}

static void filter_list_init()
{

   GtkTreeIter iter;

   
   //populate the model
   GSList *cur_flist = get_filter_list();
   while(cur_flist != NULL)
   {
      filter_list_t *filter_list = (filter_list_t *)cur_flist->data;

      if(strcmp(filter_list -> name, "None") == 0)
      {
	 cur_flist = cur_flist->next;
	 continue;
      }
      
      //add this filter's name to the model
      gtk_tree_store_append(filter_list_name_model, &iter, NULL);
      gtk_tree_store_set(filter_list_name_model, &iter,
			 0, filter_list->name,
			 -1);

      cur_flist = cur_flist -> next;
   }

   gtk_combo_box_set_active(GTK_COMBO_BOX(combo_box_filter_list_name), 0);
   
}

static void filter_wizard_view_add_filter(filter_list_t *filter_list)
{
   if(filter_list == NULL || strcmp(filter_list->name, "None") == 0)
      return;
   GtkTreeIter iter;
   gtk_tree_store_append(filter_list_name_model, &iter, NULL);
   gtk_tree_store_set(filter_list_name_model, &iter,
		      0, filter_list->name, -1);
}
static void filter_wizard_view_remove_filter(filter_list_t *filter_list)
{
   
}

void filter_list_switch_page(GtkNotebook *notebook,
			     GtkNotebookPage *page,
			     guint page_num,
			     gpointer user_data)
{
   if(page_num == 2)
   {
      show_constraints();
   }

   if (page_num == 3)
   {
      
      save_constraints();
   }
}

static void save_constraints()
{
   GtkTreeIter iter;
   if( !gtk_tree_model_get_iter_first(GTK_TREE_MODEL(constraint_model),
				      &iter))
   {
      debug("Empty constraint list, exiting!");
      return;
   }

   GSList *constraint_list = NULL;

   do {
      constraint_t *new_constraint = malloc(sizeof(constraint_t));
      gtk_tree_model_get(GTK_TREE_MODEL(constraint_model), &iter,
			 HAB_TYPE, &(new_constraint->hab_type),
			 HAB_ID, &(new_constraint->hab_id),
			 NODE_ID, &(new_constraint->node_id),
			 RESOURCE, &(new_constraint->resource),
			 -1);
      constraint_list = g_slist_append(constraint_list,
				       new_constraint);
   } while(gtk_tree_model_iter_next(GTK_TREE_MODEL(constraint_model),
				    &iter));

   filter_model_show_list();

   filter_model_update_filter_list(filter_name, constraint_list);
   filter_model_show_list();

   filter_model_save_filter_list();
}

static void show_constraints()
{
   if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton_existing_filter_list)))
   {
      GtkTreeIter iter;
      if(!gtk_combo_box_get_active_iter(GTK_COMBO_BOX(combo_box_filter_list_name), &iter))
      {
	 debug("Invalid existing filter list.");
	 return;
      }
      gtk_tree_model_get (GTK_TREE_MODEL(filter_list_name_model), &iter, 0, &filter_name, -1);
      
      if(!load_filter_from_name(filter_name))
	 debug("Existing filter list not found!\n");
      
   } else if (gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton_new_filter_list)))
   {
      if(filter_name != NULL)
	 g_free(filter_name);
	 
      const char *new_name;
      new_name = gtk_entry_get_text(GTK_ENTRY(entry_new_name));
      filter_name = g_strdup(new_name);
      debug("\nshowing for new filter list constraints: %s.", new_name);
   } else {
      debug("Error showing constraints in filter list.");
   }
	  
}

void filter_wizard_edit_filterlist(gchar *_filter_name)
{

  gtk_notebook_set_current_page(GTK_NOTEBOOK(filter_wizard_notebook),2);
  filter_name = g_strdup(_filter_name);
  if(!load_filter_from_name(filter_name))
    debug("Existing filter list not found!\n");

}


gboolean load_filter_from_name(gchar *filter_name)
{
   
   filter_list_t *flist=filter_model_find_filter_list((char *)filter_name);

   if(flist == NULL)
      return false;
   
   //clear any existing entry
   gtk_tree_store_clear(constraint_model);

   GtkTreeIter iter;

   gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(filter_list_name_model),
				       &iter, "0");

   GSList *p;
   p = flist->constraint_list;
   while(p != NULL)
   {
      constraint_t *constraint = (constraint_t *)p->data;
      gtk_tree_store_append(constraint_model, &iter, NULL);
      gtk_tree_store_set(constraint_model, &iter,
			 HAB_TYPE, constraint->hab_type,
			 HAB_ID, constraint->hab_id,
			 NODE_ID, constraint->node_id,
			 RESOURCE, constraint->resource,
			 -1);
      p = p -> next;
   }
   return true;

}

void selected_row_helper(GtkTreeModel *model,
			 GtkTreePath *path,
			 GtkTreeIter *iter,
			 gpointer data)
{
   gchar *hab_type;
   gchar *hab_id;
   gchar *node_id;
   gchar *resource;
   gtk_tree_model_get(model, iter,
		      HAB_TYPE, &hab_type,
		      HAB_ID, &hab_id,
		      NODE_ID, &node_id,
		      RESOURCE, &resource,
		      -1);


   gtk_entry_set_text(GTK_ENTRY(entry_hab_type), hab_type);
   gtk_entry_set_text(GTK_ENTRY(entry_hab_id), hab_id);
   gtk_entry_set_text(GTK_ENTRY(entry_node_id), node_id);
   gtk_entry_set_text(GTK_ENTRY(entry_resource), resource);
   g_free(hab_type);
   g_free(hab_id);
   g_free(node_id);
   g_free(resource);
}

void constraints_row_activated(GtkTreeView *tree_view,
			       gpointer user_data)
{

   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(tree_view);
   gtk_tree_selection_selected_foreach(selected_rows, selected_row_helper,
				       NULL);  
}

void add_constraint_handler(GtkButton *button)
{
   GtkTreeIter iter;
   gtk_tree_store_append(constraint_model, &iter, NULL);
   gtk_tree_store_set(constraint_model, &iter,
		      HAB_TYPE, gtk_entry_get_text(GTK_ENTRY(entry_hab_type)),
		      HAB_ID, gtk_entry_get_text(GTK_ENTRY(entry_hab_id)),
		      NODE_ID, gtk_entry_get_text(GTK_ENTRY(entry_node_id)),
		      RESOURCE, gtk_entry_get_text(GTK_ENTRY(entry_resource)),
		      -1);
}

void update_constraint_helper(GtkTreeModel *model,
			 GtkTreePath *path,
			 GtkTreeIter *iter,
			 gpointer data)
{
     gtk_tree_store_set(constraint_model, iter,
		      HAB_TYPE, gtk_entry_get_text(GTK_ENTRY(entry_hab_type)),
		      HAB_ID, gtk_entry_get_text(GTK_ENTRY(entry_hab_id)),
		      NODE_ID, gtk_entry_get_text(GTK_ENTRY(entry_node_id)),
		      RESOURCE, gtk_entry_get_text(GTK_ENTRY(entry_resource)),
		      -1); 
}

void update_constraint_handler(GtkButton *button)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, update_constraint_helper,
				       NULL);
   
}

void delete_constraint_helper(GtkTreeModel *model,
			 GtkTreePath *path,
			 GtkTreeIter *iter,
			 gpointer data)
{
   gtk_tree_store_remove(GTK_TREE_STORE(model), iter);
}
void delete_constraint_handler(GtkButton *button)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, delete_constraint_helper,
				       NULL);  
   
}

void clear_constraints_handler(GtkButton *button)
{
   gtk_tree_store_clear(GTK_TREE_STORE(constraint_model));
}

void resource_entry_changed(GtkEditable *editable, gpointer data)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, update_constraint_helper,
				       NULL);
}

void hab_id_entry_changed(GtkEditable *editable, gpointer data)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, update_constraint_helper,
				       NULL);
}

void hab_type_entry_changed(GtkEditable *editable, gpointer data)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, update_constraint_helper,
				       NULL);
}

void node_id_entry_changed(GtkEditable *editable, gpointer data)
{
   GtkTreeSelection *selected_rows;
   selected_rows = gtk_tree_view_get_selection(GTK_TREE_VIEW(treeview_constraints));
   gtk_tree_selection_selected_foreach(selected_rows, update_constraint_helper,
				       NULL);
}

void build_filter_finish_handler(GtkButton *b, gpointer *data)
{
   gtk_notebook_set_current_page(filter_wizard_notebook,0);
   gtk_entry_set_text(GTK_ENTRY(entry_new_name),"");
   debug("Add filter to filter list");
}


