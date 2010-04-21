//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <stdlib.h>
#include <stdio.h>

#include "cortex.h"
#include "gui_gtk.h"
#include "gui_xml.h"

#include "bionet.h"
#include "bionet-interface.h"

gchar read_val[256];

static void null_log_handler(const gchar *log_domain, GLogLevelFlags log_level,
			     const gchar *message, gpointer user_data)
{
   
}

int main(int argc, const char **argv)
{
   //bionet sure has a lot of debug messages... silence them
   //TODO: write bionet debugs to a file??
   //g_log_set_default_handler(null_log_handler, NULL);

   // standard gtk+ init, followed by a glade init
   // cast the argvp because gtk_init doesn't want a const
   // and everything else does */
   gtk_init(&argc, &argv);
   glade_init();

   //once the ui has been inited, the cond is broadcast and we are ready to run
   gui_gtk_run(&argc, &argv);
   
   char *hostname = NULL;
   if(argc > 1) {
      hostname = argv[1];
      printf("Using host: %s\n", hostname);
   }
   
   //prime the bionet connection, wait on a cond
   bionet_interface_main(hostname);


   gtk_main();
   
   return 0;
}
