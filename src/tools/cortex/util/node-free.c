

#include <stdlib.h>

#include <glib.h>

#include "bionet-util.h"




void bionet_node_free(bionet_node_t *node) {
    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NULL Node passed to bionet_node_free()!");
        return;
    }

    if (node->private != NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_free_node(): Node '%s' has non-NULL private pointer, ignoring", node->id);
    }

    if (node->id != NULL) {
        free(node->id);
    }

    if (node->hab_type != NULL) {
        free(node->hab_type);
    }

    if (node->hab_id != NULL) {
        free(node->hab_id);
    }


    // free all the resources
    while (node->resources != NULL) {
        bionet_resource_t *resource = node->resources->data;

        node->resources = g_slist_remove(node->resources, resource);
        bionet_resource_free(resource);
    }


    free(node);
}


