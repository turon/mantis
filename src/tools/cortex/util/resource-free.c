

#include <stdlib.h>

#include <glib.h>

#include "bionet-util.h"


void bionet_resource_free(bionet_resource_t *resource) {
    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_free(): NULL Resource passed in");
        return;
    }

    if (resource->hab_type != NULL) {
        free(resource->hab_type);
    }

    if (resource->hab_id != NULL) {
        free(resource->hab_id);
    }

    if (resource->node_id != NULL) {
        free(resource->node_id);
    }

    if (resource->id != NULL) {
        free(resource->id);
    }

    free(resource);
}

