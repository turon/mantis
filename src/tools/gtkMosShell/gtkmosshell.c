//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <config.h>

#include <gtk/gtk.h>
#include <glade/glade.h>
#include <string.h>

#include <ctype.h>
#include "interface.h"
#include "support.h"
#include "callbacks.h"
#include "mos.h"
#include "msched.h"
#include "com.h"
#include "serial.h"
#include "arg.h"
#include "shell.h"
#include "command_list.h"

#define GLADE_FILE "/usr/local/share/gmosshell/glade/gtkmosshell.glade"

GtkWidget *main_win;
GtkWidget *txt_input;
GtkWidget *txt_output;
GtkWidget *progress_win;
GtkWidget *progress_bar;
GtkWidget *progress_label;
GladeXML *xml;

extern char *srec;
extern int program;
extern GMutex *output_mutex;
GMutex *serial_mutex;


void check_args (int argc, char *argv[]); //parse command line args

void quit()
{
   gtk_main_quit ();
   _exit (0);
}

boolean node_command_parse(comBuf *input)
{
   if(input->size == 1) {
      if(input->data[0]==200) { //START command
	 printf("RESTART\n");
	 return true;
      }
   }
   return false;
}

void serial_to_textbox()
{
   // create a holder for the input string
   gchar input_string [256] = "";
   gchar null_char = '\0';
   IF_SET set;
   uint8_t retval;
   comBuf *recvd;

   com_mode(IFACE_SERIAL, IF_LISTEN);
   while(1)
   {
      IF_ZERO (&set);
      IF_SET (IFACE_SERIAL, &set);
      g_mutex_lock (serial_mutex);
      retval = com_select (&set, COM_NOBLOCK);
      if (IF_ISSET (IFACE_SERIAL, &set)) {
	 recvd = com_recv(IFACE_SERIAL);    //retrieve data from serial
	 g_mutex_unlock (serial_mutex);
      } else {
	 g_mutex_unlock (serial_mutex);
	 usleep(10000);
	 continue;
      }
      input_string[0] = '\0'; //re-init the storage string
      if(!node_command_parse(recvd)) {
	 // add the user input to the input string
         strcat(input_string, recvd->data);
	 strcat(input_string, &null_char);
	 scroll_output(input_string, GTK_TEXT_VIEW(txt_output));
      }
      com_free_buf(recvd);

   }
}

void set_mos()
{
   g_mutex_lock(serial_mutex);
   fprintf(stderr,"Loading the bootloader.\n");
   shell_load_bootloader();
   g_mutex_unlock(serial_mutex);
}

void serial_off(){
   fprintf(stderr,"turning off serial\n");
   com_mode(IFACE_SERIAL,IF_OFF);
}

void serial_on(){
   fprintf(stderr,"turning on serial\n");
   com_mode(IFACE_SERIAL, IF_LISTEN);
}
void start (void)
{
   int argc = mos_arg_argc ();
   char **argv = mos_arg_argv ();

   local_commands[0].input=NULL;
   register_function(local_commands,"quit",
		     quit, LOCAL_COMMAND_COUNT_MAX);
   register_function(local_commands,"clear",
		     clear_output_buffer, LOCAL_COMMAND_COUNT_MAX);

   register_function(local_commands,"load",
		     load, LOCAL_COMMAND_COUNT_MAX);
   register_function(local_commands,"reload",
		     reload, LOCAL_COMMAND_COUNT_MAX);
   register_function(local_commands,"set_mos",set_mos,
		     LOCAL_COMMAND_COUNT_MAX);
   register_function(local_commands,"serial off",serial_off,
		     LOCAL_COMMAND_COUNT_MAX);
   register_function(local_commands,"serial on",serial_on,
		     LOCAL_COMMAND_COUNT_MAX);
#ifdef ENABLE_NLS
   bindtextdomain (GETTEXT_PACKAGE, PACKAGE_LOCALE_DIR);
   bind_textdomain_codeset (GETTEXT_PACKAGE, "UTF-8");
   textdomain (GETTEXT_PACKAGE);
#endif

   check_args (argc, argv);
   
   gtk_set_locale ();
   gtk_init (&argc, &argv);
   glade_init ();
   // add_pixmap_directory (PACKAGE_DATA_DIR "/" PACKAGE "/pixmaps");

   g_thread_init (NULL);
   serial_mutex = g_mutex_new ();
   output_mutex = g_mutex_new ();
   xml = glade_xml_new (GLADE_FILE, NULL, NULL);
   if (!xml) {
      g_warning ("Couldn't open glade xml file");
      return;
   }
   
   main_win = glade_xml_get_widget (xml, "main_window");
   txt_output = glade_xml_get_widget (xml, "txtOutput");
   txt_input = glade_xml_get_widget (xml, "txtInput");

   progress_win = glade_xml_get_widget (xml, "progress_window");
   progress_bar = glade_xml_get_widget (xml, "load_progress");
   progress_label = glade_xml_get_widget (xml, "progress_label");

   glade_xml_signal_autoconnect (xml);


   
   if(program)
      load_srec_file(srec);

   gtk_widget_show (main_win);
   
   mos_thread_new (serial_to_textbox, 128, PRIORITY_NORMAL);
  
   gtk_main ();
   return;
}



void check_args (int argc, char *argv[])
{
   int c;
   
   while ((c = getopt (argc, argv, "Bnd:p:r:")) != -1) {
      switch (c) {
      case 'p':
	 program = 1;

      case 'r':
	 srec = g_malloc(strlen(optarg) + 1);
	 strcpy(srec,optarg);
	 break;

      case '?':
	 if(isprint(optopt))
	    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
	 else
	    fprintf(stderr,"Unknown option character `\\x%x'.\n",optopt);
	 break;
	 
      default:
	 printf("Default...\n");
      }      
   }
}
