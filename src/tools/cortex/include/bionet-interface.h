
#include <glib.h>
#include "bionet.h"


void register_callbacks();
bool connect_to_nag();
bool bionet_iterate();
gpointer bionet_interface_main(gpointer p);
void bionet_update_node_list(gchar *pattern, gint to_lock);
