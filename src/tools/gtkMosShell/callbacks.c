//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <gtk/gtk.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "callbacks.h"
#include "interface.h"
#include "support.h"
#include "com.h"
#include "msched.h"
#include "shell.h"
#include "command_list.h"

extern GtkWidget *main_win;
extern GtkWidget *progress_win;
extern GtkWidget *progress_bar;
extern GtkWidget *txt_output;
extern GtkWidget *txt_input;
extern GtkWidget *progress_label;
extern GMutex *serial_mutex;
extern char *srec;

GMutex *output_mutex;

void serial_to_textbox (void);
void find_node();
void find_node_callback_func(int retries);
void load_srec_file(gchar *filename);
void clear_output_buffer();
void scroll_output(gchar *input, GtkTextView *output);
void load();


void load_bootloader_callback(GtkMenuItem *menu_item, gpointer data){
   g_mutex_lock(serial_mutex);
   fprintf(stderr,"Loading the bootloader.\n");
   shell_load_bootloader();
   g_mutex_unlock(serial_mutex);
}

void about_callback (GtkMenuItem *menu_item, gpointer data)
{
   g_warning ("About unimplemented");
}

void clear_callback (GtkMenuItem *menu_item, gpointer data)
{
   clear_output_buffer();
}

void clear_output_buffer()
{
   GtkTextBuffer * scroll_buff =
      gtk_text_view_get_buffer(GTK_TEXT_VIEW(txt_output));
   gtk_text_buffer_set_text(scroll_buff,"",-1);
   gtk_text_view_set_buffer(GTK_TEXT_VIEW(txt_output),scroll_buff);
}

void quit_callback (GtkMenuItem *menu_item, gpointer data)
{
   printf ("Exiting\n");

   //while (gtk_events_pending ())
   //  gtk_main_iteration ();

   gtk_main_quit ();
   _exit (0);
}

void progress_update (int current, int total)
{
   gdouble frac = (gdouble)current / (gdouble)total;
   gchar *str = g_strdup_printf ("%d/%d", current, total);
   if (frac > 1.0)
      frac = 1.0;
   
   gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progress_bar), frac);
   gtk_progress_bar_set_text (GTK_PROGRESS_BAR (progress_bar), str);
			      
   while (gtk_events_pending ())
      gtk_main_iteration ();

   g_free (str);
}

void load_srec_callback_helper (GtkDialog *chooser, gint arg1, gpointer data)
{
   gchar *filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (chooser));
   
   if (arg1 == GTK_RESPONSE_CANCEL) {
      gtk_widget_destroy (GTK_WIDGET (chooser));
      return;
   }
      
   gtk_widget_destroy (GTK_WIDGET (chooser));

   load_srec_file(filename);
   srec = filename;

}

void find_node_callback_func(int retries)
{
   gchar *buffer;
   buffer = g_strdup_printf("Waiting for a sensor node. [%d]",retries);
   gtk_label_set_text(GTK_LABEL(progress_label),buffer);
   gtk_widget_show_all (progress_win);
   while (gtk_events_pending ())
      gtk_main_iteration ();
   
   g_free(buffer);
}
void reload_srec_callback (GtkMenuItem *menu_item, gpointer data)
{
   reload();
}

void reload()
{
      if(srec == NULL){
      GtkWidget *dialog = gtk_file_chooser_dialog_new ("Load SREC File",
						       GTK_WINDOW (main_win),
						       GTK_FILE_CHOOSER_ACTION_OPEN,
						       GTK_STOCK_CANCEL,
						       GTK_RESPONSE_CANCEL,
						       GTK_STOCK_OPEN,
						       GTK_RESPONSE_ACCEPT,
						       NULL);
      gtk_window_set_modal (GTK_WINDOW (dialog), TRUE);
      g_signal_connect (G_OBJECT (dialog), "response",
			G_CALLBACK (load_srec_callback_helper),
			NULL);

      gtk_widget_show (dialog);
   }
   else
      load_srec_file(srec);
}
void load_srec_callback (GtkMenuItem *menu_item, gpointer data)
{
   load();
}
void load()
{
   GtkWidget *dialog = gtk_file_chooser_dialog_new ("Load SREC File",
						    GTK_WINDOW (main_win),
						    GTK_FILE_CHOOSER_ACTION_OPEN,
						    GTK_STOCK_CANCEL,
						    GTK_RESPONSE_CANCEL,
						    GTK_STOCK_OPEN,
						    GTK_RESPONSE_ACCEPT,
						    NULL);
   gtk_window_set_modal (GTK_WINDOW (dialog), TRUE);
   g_signal_connect (G_OBJECT (dialog), "response",
		     G_CALLBACK (load_srec_callback_helper),
		     NULL);
   
   gtk_widget_show (dialog);
}
gboolean
txtInput_enter_notify_event            (GtkWidget       *widget,
                                        GdkEventKey     *event,
                                        gpointer         user_data)
{
   comBuf packet;
   gchar *cursor;
   packet.size=0;

   //for scrolling...
   GtkTextIter start, end;
   GtkTextMark* mark;
   
   if(event->string[0] != 13)
      return FALSE;

   /* retrieve input and output widgets */
      
   /* get the buffer that the output text_view is using */
   GtkTextBuffer * scroll_buff =
      gtk_text_view_get_buffer(GTK_TEXT_VIEW(txt_output));
      
   /* create a holder for the input string */
   gchar input_string[256] = "";
   gchar null_string [] = "";
      
   /* add the user input to the input string */
   strcat(input_string, gtk_entry_get_text(GTK_ENTRY(txt_input)));
   
   if(command_parse(local_commands,input_string,
		    LOCAL_COMMAND_COUNT_MAX))
   {
      //local command executed, clear the text box.
      gtk_entry_set_text(GTK_ENTRY(txt_input), null_string);
      return FALSE;
   }
   
   cursor=input_string;
   // copy the string into a packet
//   while (*cursor && packet.size < COM_DATA_SIZE - 1) {
//      if(*cursor != '\n') //make sure we don't sent newline
//	 packet.data[packet.size++] = *cursor++; //add char, inc size
//      else
//	 *cursor++;
//   }
      
//   packet.data[packet.size++]='\0'; //null terminate string
//   com_send(IFACE_SERIAL, &packet); // send it over the uart

   shell_send_to_node(cursor);
   strcat(input_string, "\n");

   gtk_text_buffer_get_bounds (scroll_buff, &start, &end);
   gtk_text_buffer_insert  (scroll_buff, &end, input_string, -1);
   gtk_text_buffer_get_bounds (scroll_buff, &start, &end);
	 
	 
   mark = gtk_text_buffer_create_mark (scroll_buff, NULL, &end, 1);
   gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(txt_output), mark,
				0.0, 0, 0.0, 1.0);
   gtk_text_buffer_delete_mark (scroll_buff, mark);

      
   /* clear the text entry (input) */
   gtk_entry_set_text(GTK_ENTRY(txt_input), null_string);
   
   return FALSE;
}
void load_srec_file(gchar *filename)
{
   gchar *loading_buff;
   int ret_val;
   if(filename == NULL)
   {
      g_warning ("Tried to open a NULL filename\n");
      return;
   }
   loading_buff = g_strdup_printf("Loading: [%s]",filename);
   
   gtk_progress_bar_set_fraction (GTK_PROGRESS_BAR (progress_bar), 0.0);
   
   gtk_widget_show_all (progress_win);
   
   while (gtk_events_pending ())
      gtk_main_iteration ();

   // lock serial mutex during serial use
   g_mutex_lock (serial_mutex);

   
   gtk_label_set_text (GTK_LABEL (progress_label), loading_buff);
   
   find_node();
   
   ret_val = shell_load_image (filename, progress_update);
   
   while(ret_val != SUCCESS)
   {
      switch(ret_val){
      case SREC_READ_ERROR:
	 g_warning("Error reading SREC file!\n");
	 return;
	 break;
      case LOAD_COMMAND_FAILURE:
	 g_warning("Node didn't accept \"LOAD\" command\n");
	 break;
      case PAGE_PACKET_FAILURE:
	 g_warning("Page packet failed. \n");
	 break;
      default:
	 g_warning("Unknown return code\n");
	 break;
      }
      find_node();
      gtk_label_set_text (GTK_LABEL (progress_label), loading_buff);
      ret_val = shell_load_image (filename, progress_update);
   }
   
   
   shell_start_execution (NULL);

   //unlock serial mutex, we're done with serial
   g_mutex_unlock (serial_mutex);
   
   //mos_thread_new (serial_to_textbox, 128, PRIORITY_NORMAL);
   gtk_widget_hide (progress_win);

   g_free(loading_buff);
}

void find_node(){
   gchar *waiting_buff;
   waiting_buff = g_strdup_printf("Waiting for a sensor node.");
   gtk_label_set_text (GTK_LABEL (progress_label), waiting_buff);
   shell_find_loader(find_node_callback_func);
   g_free(waiting_buff);
}

void scroll_output(gchar *input, GtkTextView *output)
{

   g_mutex_lock (output_mutex);
   /* get the buffer that the output text_view is using */
   GtkTextBuffer * scroll_buff =
      gtk_text_view_get_buffer(GTK_TEXT_VIEW(txt_output));

   //for scrolling...
   GtkTextIter start, end;
   GtkTextMark* mark; 

   gtk_text_buffer_get_bounds (scroll_buff, &start, &end);
   //gtk_text_buffer_insert_interactive  (scroll_buff, &end, input, -1, false);
   gtk_text_buffer_insert  (scroll_buff, &end, input, -1);
   gtk_text_buffer_get_bounds (scroll_buff, &start, &end);
   
   mark = gtk_text_buffer_create_mark (scroll_buff, NULL, &end, 1);
   gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(output), mark, 0.0, 0, 0.0, 1.0);
   gtk_text_buffer_delete_mark (scroll_buff, mark);

   g_mutex_unlock (output_mutex);
   while (gtk_events_pending ())
      gtk_main_iteration ();
}
