//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "common.h"

#include "cortex/include/net_event.h"
#include "cortex.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#define ENTRIES_PER_FILE 400

static void ob_conn_func (GConn* conn, GConnEvent* event, gpointer user_data);

static gboolean read_eof = FALSE;
static char *out_dir;
static char current_file[256];
static FILE * output_file;

/* see if path is a valid directory */
static void check_dir (char *path)
{
   struct stat fstat;
   
   if (stat (path, &fstat)) {
      printf ("Problem opening '%s'\n", path);
      exit (-1);
   }

   if (!S_ISDIR (fstat.st_mode)) {
      printf ("Arg passed, '%s', is not a directory, exiting\n", path);
      exit (-1);
   }
}

/* function to call on exit () */
void flush_raw (void)
{
}

static void init_raw (void)
{
}

/* maintain a rolling log file */
static void update_filename (void)
{
   static int first_run = 1;
   static int current_number = -1;
   static int nodes = ENTRIES_PER_FILE -1;

   nodes++;
   if (nodes == ENTRIES_PER_FILE) {
      nodes = 0;
      current_number++;
      snprintf (current_file, sizeof (current_file), "%s/log-%.10d.log",
		out_dir, current_number);
      printf ("Setting current file to '%s'\n", current_file);
      if (!first_run) {
	 flush_raw ();
	 init_raw ();
      } else
	 first_run = 0;
   }  
}

int main (int argc, char** argv)
{
   gchar* hostname;
   gint port;
   GMainLoop* main_loop;
   GConn* conn;

   gnet_init ();

   /* Parse args */
   if (argc != 4)
   {
      g_print ("usage: %s <server> <port> <output-directory>\n", argv[0]);
      exit(EXIT_FAILURE);
   }
   hostname = argv[1];
   port = atoi(argv[2]);
   out_dir = argv[3];
   check_dir (out_dir);
   update_filename ();

   /* Create the main loop */
   main_loop = g_main_new(FALSE);
  
   /* Create connection object */
   conn = gnet_conn_new (hostname, port, ob_conn_func, NULL);
   g_assert (conn);

   /* Connect */
   gnet_conn_connect (conn);
   gnet_conn_set_watch_error (conn, TRUE);
   gnet_conn_timeout (conn, 30000);  /* 30 second timeout */

   //open the xml file
   init_raw ();
   atexit (flush_raw);
   
   /* Start the main loop */   
   g_main_run (main_loop);

   exit (EXIT_SUCCESS);
   return 0;
}


/* write event data to xml file */
void raw_write_node (gint node_id, gint light, gint temp, gint time)
{
   printf("%d\t%d\t%d\t%d\n", node_id, light, temp, time);
   update_filename ();
}

/* Handle GConn events */
static void
ob_conn_func (GConn* conn, GConnEvent* event, gpointer user_data)
{
   switch (event->type)
   {
   case GNET_CONN_CONNECT:
   {
      gnet_conn_timeout (conn, 0);	/* reset timeout */

      printf ("Connected to server\n");
      
      gnet_conn_read (conn);
      break;
   }

   case GNET_CONN_READ:
   {
      /* Write line out */
      net_event_header_t hdr;
      light_temp_event_t ev;

      gnet_unpack ("!iiliii", event->buffer, sizeof (hdr) + sizeof (ev),
		   &hdr.size, &hdr.event_type, &hdr.time_seconds,
		   &ev.node_id, &ev.light, &ev.temp);
      
      printf ("Got an event: %d %d %ld %d %d %d\n", hdr.size, hdr.event_type,
	      hdr.time_seconds, ev.node_id, ev.light, ev.temp);
      xml_write_node (ev.node_id, ev.light, ev.temp, hdr.time_seconds);

      /* Check if done */
      if (read_eof) {
	 exit (EXIT_SUCCESS);
      }

      /* Read another line */
      gnet_conn_read (conn);
      
      break;
   }

   case GNET_CONN_WRITE:
   {
      /* do nothing */
      break;
   }

   case GNET_CONN_CLOSE:
   {
      gnet_conn_delete (conn);
      exit (EXIT_SUCCESS);
      break;
   }

   case GNET_CONN_TIMEOUT:
   {
      gnet_conn_delete (conn);
      fprintf (stderr, "Connection timeout\n");
      exit (EXIT_FAILURE);
      break;
   }

   case GNET_CONN_ERROR:
   {
      gnet_conn_delete (conn);
      fprintf (stderr, "Connection failure\n");
      exit (EXIT_FAILURE);
      break;
   }

   default:
      g_assert_not_reached ();
   }
}
