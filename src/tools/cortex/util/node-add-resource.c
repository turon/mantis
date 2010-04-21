

#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"





int bionet_node_add_resource_with_valuestr_timestr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const char *time_str
) {
    bionet_resource_t *resource;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_resource(): NULL Node passed in");
        return -1;
    }

    resource = bionet_resource_new_with_valuestr_timestr(
        node->hab_type,
        node->hab_id,
        node->id,
        data_type_str,
        flavor_str,
        resource_id,
        value_str,
        time_str
    );

    if (resource == NULL) {
        // the bionet_resource_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->resources = g_slist_append(node->resources, resource);

    return 0;
}




int bionet_node_add_resource_with_valuestr_timevalptr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const struct timeval *tv
) {
    bionet_resource_t *resource;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_resource(): NULL Node passed in");
        return -1;
    }

    resource = bionet_resource_new_with_valuestr_timevalptr(
        node->hab_type,
        node->hab_id,
        node->id,
        data_type_str,
        flavor_str,
        resource_id,
        value_str,
        tv
    );

    if (resource == NULL) {
        // the bionet_resource_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->resources = g_slist_append(node->resources, resource);

    return 0;
}



int bionet_node_add_resource_with_valueptr_timestr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const char *time_str
) {
    bionet_resource_t *resource;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_resource_with_value(): NULL Node passed in");
        return -1;
    }

    resource = bionet_resource_new_with_valueptr_timestr(
        node->hab_type,
        node->hab_id,
        node->id,
        data_type_str,
        flavor_str,
        resource_id,
        valueptr,
        time_str
    );

    if (resource == NULL) {
        // the bionet_resource_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->resources = g_slist_append(node->resources, resource);

    return 0;
}




int bionet_node_add_resource_with_valueptr_timevalptr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const struct timeval *tv
) {
    bionet_resource_t *resource;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_resource_with_value(): NULL Node passed in");
        return -1;
    }

    resource = bionet_resource_new_with_valueptr_timevalptr(
        node->hab_type,
        node->hab_id,
        node->id,
        data_type_str,
        flavor_str,
        resource_id,
        valueptr,
        tv
    );

    if (resource == NULL) {
        // the bionet_resource_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->resources = g_slist_append(node->resources, resource);

    return 0;
}


