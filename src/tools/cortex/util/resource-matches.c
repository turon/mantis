
#include <string.h>

#include "bionet-util.h"


int bionet_resource_matches_id(const bionet_resource_t *resource, const char *id) {
    if ((strcmp(id, "*") == 0) || (strcmp(id, resource->id) == 0)) {
        return 1;
    } 

    return 0;
}


int bionet_resource_matches_habtype_habid_nodeid_resourceid(
    const bionet_resource_t *resource,
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *resource_id
) {
    if (
        (
            (strcmp(hab_type, "*") == 0) ||
            (strcmp(hab_type, resource->hab_type) == 0) 
        ) &&
        (
            (strcmp(hab_id, "*") == 0) ||
            (strcmp(hab_id, resource->hab_id) == 0) 
        ) &&
        (
            (strcmp(node_id, "*") == 0) ||
            (strcmp(node_id, resource->node_id) == 0)
        ) &&
        (
            (strcmp(resource_id, "*") == 0) ||
            (strcmp(resource_id, resource->id) == 0)
        )
    ) {
        return 1;
    } 

    return 0;
}

