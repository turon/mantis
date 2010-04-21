
#include <glib.h>

#include "bionet-util.h"




int bionet_hab_add_node(bionet_hab_t *hab, bionet_node_t *node) {
    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_add_node(): NULL HAB passed in");
        return -1;
    }

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_add_node(): NULL Node passed in");
        return -1;
    }


    // FIXME: make sure the node's not in the list yet!


    // ok, add the node to the hab's node-list
    hab->nodes = g_slist_append(hab->nodes, node);
    return 0;
}


