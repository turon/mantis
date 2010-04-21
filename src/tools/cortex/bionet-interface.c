
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>

#include "cortex.h"
#include "bionet-interface.h"
#include "gui_gtk.h"
#include "model_gtk.h"
#include "bionet.h"

#ifndef MINI_GUI
extern GtkWidget *system_status_label;
#endif

int nag_fd;

bool bionet_connect(gpointer data);
bool bionet_iterate(gpointer data);

void new_hab_callback(bionet_hab_t *hab);
void lost_hab_callback(bionet_hab_t *hab);

void new_node_callback(bionet_node_t *node);
void lost_node_callback(bionet_node_t *node);

void resource_value_callback(bionet_resource_t *resource);

void register_callbacks()
{
   bionet_register_callback_new_hab(new_hab_callback);
   bionet_register_callback_lost_hab(lost_hab_callback);

   bionet_register_callback_new_node(new_node_callback);
   bionet_register_callback_lost_node(lost_node_callback);
   
   bionet_register_callback_resource_value(resource_value_callback);
}

void new_node_callback(bionet_node_t *node)
{
   model_add_node(node);
}

void lost_node_callback(bionet_node_t *node)
{
   model_remove_node(node);
}

void lost_hab_callback(bionet_hab_t *hab)
{
   model_remove_hab(hab);
}

void new_hab_callback(bionet_hab_t *hab)
{
   model_add_hab(hab);
}

void resource_value_callback(bionet_resource_t *resource)
{
   model_update_resource(resource);
}

bool connect_to_nag(gchar *host)
{
   bionet_set_nag_hostname(host);
   nag_fd = bionet_connect_to_nag();
   if (nag_fd < 0) {
      debug("nag connection failed!");
#ifndef MINI_GUI
      gtk_label_set_text(GTK_LABEL(system_status_label), "Connection Failed!");
#endif
      return false;
   } else /*if (bionet_is_connected())*/ {
      debug("connected to nag, subscribing to habs, nodes and resources");
      if(bionet_subscribe_hab_list_by_name_pattern("*.*")) {
	 debug("hab subscription failed!\n");
#ifndef MINI_GUI
         gtk_label_set_text(GTK_LABEL(system_status_label), "Hab subscription Failed!");
#endif
	 return false;
      }
      if(bionet_subscribe_node_list_by_name_pattern("*.*.*")) {
	 debug("node subscription failed!\n");
#ifndef MINI_GUI
	 gtk_label_set_text(GTK_LABEL(system_status_label), "Node subscription Failed!");
#endif
	 return false;
      }
      if(bionet_subscribe_resource_by_name_pattern("*.*.*:*")) {
	 debug("resource subscription failed!\n");
#ifndef MINI_GUI
	 gtk_label_set_text(GTK_LABEL(system_status_label), "Resource subscription Failed!");
#endif
	 return false;
      }
   } /*else {
      debug("Problem connecting to nag");
      }*/
	      
#ifndef MINI_GUI
   gtk_label_set_text(GTK_LABEL(system_status_label), "Connected!");
#endif
   
   return true;
}

gpointer bionet_interface_main(gpointer arg)
{
   register_callbacks();

   g_idle_add(bionet_connect, arg);

   return NULL;
}

bool bionet_connect(gpointer data)
{
   //subscribes to habs, nodes and resources
   char *hostname = "bionet-base.colorado.edu";
   if(data != NULL) {
      hostname = (char *)data;
   }
   bool ret = connect_to_nag(hostname);

   if(ret == FALSE)
   {
#ifndef MINI_GUI
      gtk_label_set_text(GTK_LABEL(system_status_label), "Error connecting nag.");
#endif
      debug("Retrying NAG connection");
      sleep(1);
      return true;
   }

#ifndef MINI_GUI
   gtk_label_set_text(GTK_LABEL(system_status_label), "Successfully connecting nag.");
#endif
   
   //grabs new-hab and new-node messages after subscription
   bionet_handle_queued_nag_messages();
   
   g_idle_add(bionet_iterate, NULL);

   return false;
}

bool bionet_iterate(gpointer data)
{
   int r;
   fd_set readers;
   struct timeval tv;
   tv.tv_sec = 0;
   tv.tv_usec = 50000;

   FD_ZERO(&readers);
   FD_SET(nag_fd, &readers);

   r = select(nag_fd + 1, &readers, NULL, NULL, &tv);
   
   if (r < 0) {

#ifndef MINI_GUI
      gtk_label_set_text(GTK_LABEL(system_status_label), "Error reading from nag.");
#endif
      
      debug("Error selecting...");
      perror("select(): ");
      return true;
   } else if (r == 0) {
      //debug("Shouldn't happen");
      
   } else {
      bionet_read_from_nag();
      bionet_handle_queued_nag_messages();
   }

   return true;
}
