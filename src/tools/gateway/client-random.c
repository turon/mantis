//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    cortex.c                                                      */
/* Author     Adam Torgerson    adam.torgerson@colorado.edu               */
/*   Date: 03/30/04                                                       */
/*                                                                        */
/* The gateway software, to be connected to a relay node and forward      */
/*   packets on to connect visualization clients.                         */
/**************************************************************************/

#include "common.h"

#include "cortex.h"
#include "cortex/include/net_event.h"

static void conn_func (GConn* conn, GConnEvent* event, gpointer user_data);

int main(int argc, char** argv)
{
   gchar* hostname;
   gint port;
   gint interval = 1000;
   GMainLoop* main_loop;
   GConn* conn;

   gnet_init ();

   /* Parse args */
   if (argc == 3) {
      hostname = argv[1];
      port = atoi (argv[2]);
   } else if (argc == 4) {
      hostname = argv[1];
      port = atoi (argv[2]);
      interval = atoi (argv[3]);
   } else {
      printf ("Defaulting connection to localhost:4200\n");
      hostname = "localhost";
      port = 4200;
   }

   /* Create the main loop */
   main_loop = g_main_new(FALSE);
  
   /* Create connection object */
   conn = gnet_conn_new (hostname, port, conn_func, &interval);
   g_assert (conn);

   /* Connect */
   gnet_conn_connect (conn);
   gnet_conn_set_watch_error (conn, TRUE);
   gnet_conn_timeout (conn, 30000);  /* 30 second timeout */

   /* Start the main loop */
   g_main_run (main_loop);

   exit (EXIT_SUCCESS);
   return 0;
}

gboolean random_func (gpointer data)
{
   gint size;
   gchar *buf;
   net_event_header_t hdr;
   light_temp_event_t ev;
   GConn* conn = (GConn*) data;
   static int first = 0;

   hdr.size = sizeof (ev);
   hdr.event_type = NET_EVENT_LIGHT_TEMP_RANDOM;
   hdr.time_seconds = time (NULL);

   if (first < 61) {
      ev.node_id = first;
      first++;
   } else
      ev.node_id = g_random_int_range (0, 61);
   ev.light = g_random_int_range (0, 254);
   ev.temp = g_random_int_range (40, 90);
   printf ("node: id: %d, light: %d, temp: %d, time: %ld\n", ev.node_id,
	   ev.light, ev.temp, hdr.time_seconds);
   
   size = gnet_pack_strdup ("!iiliii", &buf,
			    hdr.size, hdr.event_type, hdr.time_seconds,
			    ev.node_id, ev.light, ev.temp);
   
   gnet_conn_write (conn, buf, size);
   g_free (buf);

   return TRUE;
}

/* Handle GConn events */
static void conn_func (GConn* conn, GConnEvent* event, gpointer user_data)
{
   gint interval = *(gint *)user_data;
   
   switch (event->type)
   {
   case GNET_CONN_CONNECT:
   {
      gnet_conn_timeout (conn, 0);	/* reset timeout */
      
      printf ("Connected to server\n");
      
      /* add the random function to be called periodically */
      g_timeout_add (interval, random_func, conn);
      
      gnet_conn_read (conn);
      break;
   }

   /* read complete */
   case GNET_CONN_READ:
   {
      /* Write line out */
      net_event_header_t hdr;
      light_temp_event_t ev;

      gnet_unpack ("!iiliii", event->buffer, sizeof (hdr) + sizeof (ev),
		   &hdr.size, &hdr.event_type, &hdr.time_seconds,
		   &ev.node_id, &ev.light, &ev.temp);
      
      printf ("Got an event: %d %d %ld %d %d %d\n", hdr.size,
	      hdr.event_type, hdr.time_seconds, ev.node_id, ev.light, ev.temp);

      /* Read another line */
      gnet_conn_read (conn);
      
      break;
   }

   /* write complete */
   case GNET_CONN_WRITE:
   {
      /* do nothing */
      break;
   }

   case GNET_CONN_CLOSE:
   {
      gnet_conn_delete (conn);
      printf ("Connection closed.\n");
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
