//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>

#include <gdk/gdkkeysyms.h>
#include <gtk/gtk.h>

#include "callbacks.h"
#include "interface.h"
#include "support.h"

#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

GtkWidget*
create_gtkMosShell (void)
{
  GtkWidget *gtkMosShell;
  GtkWidget *vbox1;
  GtkWidget *menubar1;
  GtkWidget *menuitem1;
  GtkWidget *menuitem1_menu;
  GtkWidget *quit1;
  GtkWidget *load1;
  GtkWidget *load1_menu;
  GtkWidget *load_an_srec1;
  GtkWidget *menuitem4;
  GtkWidget *menuitem4_menu;
  GtkWidget *about1;
  GtkWidget *scrolledwindow1;
  GtkWidget *txtOutput;
  GtkWidget *txtInput;
  GtkAccelGroup *accel_group;

  accel_group = gtk_accel_group_new ();

  gtkMosShell = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (gtkMosShell), _("Mos Shell v. 1.4"));
  gtk_window_set_position (GTK_WINDOW (gtkMosShell), GTK_WIN_POS_CENTER);
  gtk_window_set_default_size (GTK_WINDOW (gtkMosShell), 515, 362);

  vbox1 = gtk_vbox_new (FALSE, 0);
  gtk_widget_show (vbox1);
  gtk_container_add (GTK_CONTAINER (gtkMosShell), vbox1);

  menubar1 = gtk_menu_bar_new ();
  gtk_widget_show (menubar1);
  gtk_box_pack_start (GTK_BOX (vbox1), menubar1, FALSE, FALSE, 0);

  menuitem1 = gtk_menu_item_new_with_mnemonic (_("_File"));
  gtk_widget_show (menuitem1);
  gtk_container_add (GTK_CONTAINER (menubar1), menuitem1);

  menuitem1_menu = gtk_menu_new ();
  gtk_menu_item_set_submenu (GTK_MENU_ITEM (menuitem1), menuitem1_menu);

  quit1 = gtk_image_menu_item_new_from_stock ("gtk-quit", accel_group);
  gtk_widget_show (quit1);
  gtk_container_add (GTK_CONTAINER (menuitem1_menu), quit1);

  load1 = gtk_menu_item_new_with_mnemonic (_("_Load"));
  gtk_widget_show (load1);
  gtk_container_add (GTK_CONTAINER (menubar1), load1);

  load1_menu = gtk_menu_new ();
  gtk_menu_item_set_submenu (GTK_MENU_ITEM (load1), load1_menu);

  load_an_srec1 = gtk_menu_item_new_with_mnemonic (_("Load an _Srec..."));
  gtk_widget_show (load_an_srec1);
  gtk_container_add (GTK_CONTAINER (load1_menu), load_an_srec1);
  gtk_widget_add_accelerator (load_an_srec1, "activate", accel_group,
                              GDK_L, GDK_CONTROL_MASK,
                              GTK_ACCEL_VISIBLE);

  menuitem4 = gtk_menu_item_new_with_mnemonic (_("_Help"));
  gtk_widget_show (menuitem4);
  gtk_container_add (GTK_CONTAINER (menubar1), menuitem4);

  menuitem4_menu = gtk_menu_new ();
  gtk_menu_item_set_submenu (GTK_MENU_ITEM (menuitem4), menuitem4_menu);

  about1 = gtk_menu_item_new_with_mnemonic (_("_About"));
  gtk_widget_show (about1);
  gtk_container_add (GTK_CONTAINER (menuitem4_menu), about1);

  scrolledwindow1 = gtk_scrolled_window_new (NULL, NULL);
  gtk_widget_show (scrolledwindow1);
  gtk_box_pack_start (GTK_BOX (vbox1), scrolledwindow1, TRUE, TRUE, 0);

  txtOutput = gtk_text_view_new ();
  gtk_widget_show (txtOutput);
  gtk_container_add (GTK_CONTAINER (scrolledwindow1), txtOutput);
  gtk_text_view_set_editable (GTK_TEXT_VIEW (txtOutput), FALSE);
  gtk_text_view_set_cursor_visible (GTK_TEXT_VIEW (txtOutput), FALSE);

  txtInput = gtk_entry_new ();
  gtk_widget_show (txtInput);
  gtk_box_pack_start (GTK_BOX (vbox1), txtInput, FALSE, FALSE, 0);

  g_signal_connect ((gpointer) quit1, "activate",
                    G_CALLBACK (on_quit1_activate),
                    NULL);
  g_signal_connect ((gpointer) load_an_srec1, "activate",
                    G_CALLBACK (on_load_an_srec1_activate),
                    NULL);
  g_signal_connect ((gpointer) about1, "activate",
                    G_CALLBACK (on_about1_activate),
                    NULL);
  g_signal_connect ((gpointer) txtInput, "key_press_event",
                    G_CALLBACK (txtInput_enter_notify_event),
                    NULL);

  /* Store pointers to all widgets, for use by lookup_widget(). */
  GLADE_HOOKUP_OBJECT_NO_REF (gtkMosShell, gtkMosShell, "gtkMosShell");
  GLADE_HOOKUP_OBJECT (gtkMosShell, vbox1, "vbox1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, menubar1, "menubar1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, menuitem1, "menuitem1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, menuitem1_menu, "menuitem1_menu");
  GLADE_HOOKUP_OBJECT (gtkMosShell, quit1, "quit1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, load1, "load1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, load1_menu, "load1_menu");
  GLADE_HOOKUP_OBJECT (gtkMosShell, load_an_srec1, "load_an_srec1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, menuitem4, "menuitem4");
  GLADE_HOOKUP_OBJECT (gtkMosShell, menuitem4_menu, "menuitem4_menu");
  GLADE_HOOKUP_OBJECT (gtkMosShell, about1, "about1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, scrolledwindow1, "scrolledwindow1");
  GLADE_HOOKUP_OBJECT (gtkMosShell, txtOutput, "txtOutput");
  GLADE_HOOKUP_OBJECT (gtkMosShell, txtInput, "txtInput");

  gtk_window_add_accel_group (GTK_WINDOW (gtkMosShell), accel_group);

  return gtkMosShell;
}

