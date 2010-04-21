//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)
#include <gtk/gtk.h>
#include "cortex.h"
#include "gui_xml.h"
#include "gui_topo.h"

extern gchar *global_filename;
extern GtkWidget *dialog_about;
extern gui_topo_t *topo;
extern GtkWidget *window_data;
extern GtkWidget *window_filter_wizard;
extern GtkWidget *window_commanding_interface;
void quit_handler(GtkWidget *w, gpointer p)
{
   gtk_main_quit();
   gtk_exit(0);
}

void dialog_about_handler(GtkWidget *w, gpointer p)
{
   gtk_widget_show(dialog_about);
}

void disconnect_handler(GtkWidget *w, gpointer p)
{
   debug("Got disconnect callback");
}

void new_connection_handler(GtkWidget *w, gpointer p)
{
   debug("Got new connection callback");
}

void save_xml_topology_as_file_helper(GtkDialog *chooser,
				      gint arg1, gpointer data)
{
   gchar *filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER (chooser));

   if(arg1 == GTK_RESPONSE_CANCEL) {
      gtk_widget_destroy (GTK_WIDGET (chooser));
      return;
   }

   if(filename == NULL) {
      debug("tried to write a null filename");
      return;
   }
   debug("writing to '%s'", filename);
   global_filename = filename;
   
   gui_xml_write_file(topo, filename);

   gtk_widget_destroy(GTK_WIDGET (chooser));
}

void save_xml_topology_as_handler(GtkMenuItem *menu_item, gpointer data)
{
   GtkWidget *dialog = gtk_file_chooser_dialog_new("Save XML Topology",
						   GTK_WINDOW (window_data),
						   GTK_FILE_CHOOSER_ACTION_SAVE,
						   GTK_STOCK_CANCEL,
						   GTK_RESPONSE_CANCEL,
						   GTK_STOCK_OPEN,
						   GTK_RESPONSE_ACCEPT,
						   NULL);

   gtk_window_set_modal(GTK_WINDOW(dialog), TRUE);

   g_signal_connect(G_OBJECT (dialog), "response",
		    G_CALLBACK (save_xml_topology_as_file_helper), NULL);
   
   gtk_widget_show(dialog);
}

void save_xml_topology_handler(GtkMenuItem *menu_item, gpointer data)
{
   if(global_filename != NULL) {
      debug("writing to '%s'", global_filename);
      gui_xml_write_file(topo, global_filename);
   } else
      save_xml_topology_as_handler(menu_item, data);
   
}

void open_xml_topology_file_helper(GtkDialog *chooser,
				   gint arg1, gpointer data)
{
   gchar *filename = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER (chooser));

   if(arg1 == GTK_RESPONSE_CANCEL) {
      gtk_widget_destroy(GTK_WIDGET (chooser));
      return;
   }

   if(filename == NULL) {
      debug ("tried to open a null filename");
      return;
   }

   global_filename = filename;
   gui_xml_open_file(topo, filename);

   gtk_widget_destroy(GTK_WIDGET(chooser));
}

void open_xml_topology_handler(GtkMenuItem *menu_item, gpointer data)
{
   GtkWidget *dialog = gtk_file_chooser_dialog_new("Open XML Topology",
						   GTK_WINDOW (window_data),
						   GTK_FILE_CHOOSER_ACTION_OPEN,
						   GTK_STOCK_CANCEL,
						   GTK_RESPONSE_CANCEL,
						   GTK_STOCK_OPEN,
						   GTK_RESPONSE_ACCEPT,
						   NULL);
   gtk_window_set_modal(GTK_WINDOW(dialog), TRUE);

   g_signal_connect(G_OBJECT(dialog), "response",
		    G_CALLBACK(open_xml_topology_file_helper), NULL);

   gtk_widget_show(dialog);
}


void menu_build_filter_wizard_handler(GtkMenuItem *menu_item, gpointer data)
{
  gtk_widget_show(window_filter_wizard);
}

void menu_issue_command_handler(GtkMenuItem *menu_item, gpointer data)
{
  gtk_widget_show(window_commanding_interface);
}
