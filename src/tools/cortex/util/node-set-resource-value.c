
#include <glib.h>

#include "bionet-util.h"


int bionet_node_set_resource_value(
    bionet_node_t *node,
    const char *resource_id,
    const void *value,
    const struct timeval *timestamp
) {
    bionet_resource_t *resource;
    int r;


    //
    // sanity checking
    //

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value(): NULL Node passed in");
        return -1;
    }

    if (resource_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value(): NULL Resource-ID passed in");
        return -1;
    }

    if (value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value(): NULL value passed in");
        return -1;
    }


    resource = bionet_node_get_resource_by_id(node, resource_id);
    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "couldnt find Resource '%s' of Node '%s'", resource_id, node->id);
        return -1;
    }

    r = bionet_resource_value_from_pointer(value, resource);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error updating resource value!");
        return -1;
    }

    r = bionet_resource_time_from_timeval(timestamp, resource);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error updating resource time!");
        return -1;
    }

    return 0;
}


int bionet_node_set_resource_value_with_valuestr(
    bionet_node_t *node,
    const char *resource_id,
    const char *valuestr,
    const struct timeval *timestamp
) {
    bionet_resource_t *resource;
    int r;


    //
    // sanity checking
    //

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value_with_valuestr(): NULL Node passed in");
        return -1;
    }

    if (resource_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value_with_valuestr(): NULL Resource-ID passed in");
        return -1;
    }

    if (valuestr == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_set_resource_value_with_valuestr(): NULL value passed in");
        return -1;
    }


    resource = bionet_node_get_resource_by_id(node, resource_id);
    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "couldnt find Resource '%s' of Node '%s'", resource_id, node->id);
        return -1;
    }

    r = bionet_resource_value_from_string(valuestr, resource);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error updating resource value!");
        return -1;
    }

    r = bionet_resource_time_from_timeval(timestamp, resource);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error updating resource time!");
        return -1;
    }

    return 0;
}

