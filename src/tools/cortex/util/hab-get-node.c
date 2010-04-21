

#include <string.h>

#include "bionet-util.h"




bionet_node_t *bionet_hab_get_node_by_id(bionet_hab_t *hab, const char *node_id) {
    int i;


    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_get_node_by_id(): NULL HAB passed in");
        return NULL;
    }

    if (node_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_hab_get_node_by_id(): NULL Node-ID passed in");
        return NULL;
    }

    for (i = 0; i < g_slist_length(hab->nodes); i ++) {
        bionet_node_t *node;

        node = g_slist_nth_data(hab->nodes, i);
        if (strcmp(node->id, node_id) != 0) continue;

        return node;
    }

    return NULL;
}


