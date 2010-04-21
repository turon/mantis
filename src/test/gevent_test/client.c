//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    client.c                                                      */
/* Author   Jeff Rose   :    rosejn@colorado.edu                          */
/*   Date: 06/09/04                                                       */
/*                                                                        */
/* A test app for the client side library of the gevent system.           */
/**************************************************************************/

#include "gevent.h"

#define TEST_PORT  8888
#define TEST_ADDR  "localhost"

#define TEST_EVENT_ID 234

gboolean send_func(gpointer data)
{
   gevent_client *client = (gevent_client *)data;
   gint event_id = TEST_EVENT_ID;
   gchar buf[256] = "This is a test event.";
   
   g_print("Sending event: %d\n", event_id);
   
   if(gevent_client_send(client, event_id, buf, 20) == -1)
   {
      g_print("Server has disconnected... Exiting program!\n");
      exit(-1);
   }
   
   return true;
}


int main(int argc, char** argv)
{
   gchar* hostname;
   gint port;
   gint interval = 2000;
   GMainLoop* main_loop;
   gevent_client *client;

   hostname = TEST_ADDR;
   port = TEST_PORT;
   
   /* Create the main loop */
   main_loop = g_main_new(FALSE);
  
   /* Create connection object */
   client = gevent_client_new(hostname, port, NULL);

   /* add the random function to be called periodically */
   g_timeout_add (interval, send_func, client);
   
   /* Start the main loop */
   g_main_run (main_loop);

   return 0;
}
