
#include <string.h>

#include <glib.h>

#include "bionet-util.h"


bionet_resource_t *bionet_node_get_resource_by_id(bionet_node_t *node, const char *resource_id) {
    GSList *i;


    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_get_resource_by_id(): NULL Node passed in");
        return NULL;
    }

    if (resource_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_get_resource_by_id(): NULL Resource-ID passed in");
        return NULL;
    }


    for (i = node->resources; i != NULL; i = i->next) {
        bionet_resource_t *resource = i->data;

        if (strcmp(resource->id, resource_id) == 0) {
            return resource;
        }
    }

    return NULL;
}

