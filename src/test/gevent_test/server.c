//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:   server.c                                                       */
/* Author  Jeff Rose   :  rosejn@colorado.edu                             */
/*   Date: 03/28/05                                                       */
/*                                                                        */
/* A simple test server that utilizes the gevent library.                 */
/**************************************************************************/

#include "gevent.h"

#define TEST_PORT  8888
#define TEST_ADDR  localhost

#define TEST_EVENT_ID 234

gchar read_val[256];


gint test_handler(gint id, gpointer pkt, gint size, gpointer data)
{
   gchar *ptr = (gchar *)pkt;
   ptr[size] = '\0';
   g_print("Got event: %d with data: %s\n", id, pkt);

   return 0;
}


int main(int argc, const char **argv)
{
   GMainLoop *main_loop;
   gevent_server *server;

   /* We need to get a main loop from glib since this isn't a full gtk app. */
   main_loop = g_main_new(FALSE);

   /* Now setup an event server and register handlers. */
   server = gevent_server_new(TEST_PORT, NULL);
   
   gevent_server_register(server, TEST_EVENT_ID, test_handler);
   
   g_main_run(main_loop);
   
   return 0;
}
