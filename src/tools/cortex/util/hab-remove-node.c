

#include <string.h>

#include <glib.h>

#include "bionet-util.h"




int bionet_hab_remove_all_nodes(bionet_hab_t *hab) {
    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_remove_all_nodes(): NULL HAB passed in");
        return -1;
    }


    // remove all of this HAB's Nodes
    do {
        bionet_node_t *node;

        node = g_slist_nth_data(hab->nodes, 0);
        if (node == NULL) break;  // done

        hab->nodes = g_slist_remove(hab->nodes, node);

        bionet_node_free(node);
    } while(1);

    return 0;
}




int bionet_hab_remove_node_by_id(bionet_hab_t *hab, const char *node_id) {
    bionet_node_t *node;


    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_remove_node_by_id(): NULL HAB passed in");
        return -1;
    }

    if (node_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_remove_node_by_id(): NULL node_id passed in");
        return -1;
    }


    node = bionet_hab_get_node_by_id(hab, node_id);
    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_remove_node_by_id(): Node '%s' not found", node_id);
        return -1;
    }

    hab->nodes = g_slist_remove(hab->nodes, node);

    return 0;
}


