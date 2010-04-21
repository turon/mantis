

#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"




bionet_resource_t *bionet_resource_new_without_value(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id
) {
    bionet_resource_t *resource;


    if (resource_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-ID passed in");
        return NULL;
    }

    if (flavor_str == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Flavor passed in");
        return NULL;
    }

    if (data_type_str == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Data-Type passed in");
        return NULL;
    }


    resource = (bionet_resource_t *)calloc(1, sizeof(bionet_resource_t));
    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        goto cleanup;
    }


    if (hab_type != NULL) {
        resource->hab_type = strdup(hab_type);
        if (resource->hab_type == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            goto cleanup;
        }
    }

    if (hab_id != NULL) {
        resource->hab_id = strdup(hab_id);
        if (resource->hab_id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            goto cleanup;
        }
    }

    if (node_id != NULL) {
        resource->node_id = strdup(node_id);
        if (resource->node_id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            goto cleanup;
        }
    }


    resource->id = strdup(resource_id);
    if (resource->id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        goto cleanup;
    }


    resource->data_type = bionet_resource_data_type_from_string(data_type_str);
    if (resource->data_type == BIONET_RESOURCE_DATA_TYPE_INVALID) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): error parsing data type from '%s'", data_type_str);
        goto cleanup;
    }

    resource->flavor = bionet_resource_flavor_from_string(flavor_str);
    if (resource->flavor == BIONET_RESOURCE_FLAVOR_INVALID) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): error parsing flavor from '%s'", flavor_str);
        goto cleanup;
    }

    return resource;


cleanup:
    if (resource != NULL) {
        bionet_resource_free(resource);
    }
    return NULL;
}




bionet_resource_t *bionet_resource_new_with_valuestr_timestr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const char *time_str
) {
    int r;
    bionet_resource_t *resource;


    if (value_str == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Value passed in");
        return NULL;
    }


    resource = bionet_resource_new_without_value(hab_type, hab_id, node_id, data_type_str, flavor_str, resource_id);
    if (resource == NULL) {
        return NULL;
    }


    r = bionet_resource_value_from_string(value_str, resource);
    if (r < 0) {
        // the set_resource_value function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    r = bionet_resource_time_from_string(time_str, resource);
    if (r < 0) {
        // the set_resource_time function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    return resource;


cleanup:
    if (resource != NULL) {
        bionet_resource_free(resource);
    }
    return NULL;
}




bionet_resource_t *bionet_resource_new_with_valuestr_timevalptr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const struct timeval *tv
) {
    int r;
    bionet_resource_t *resource;


    if (value_str == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Value passed in");
        return NULL;
    }


    resource = bionet_resource_new_without_value(hab_type, hab_id, node_id, data_type_str, flavor_str, resource_id);
    if (resource == NULL) {
        return NULL;
    }


    r = bionet_resource_value_from_string(value_str, resource);
    if (r < 0) {
        // the set_resource_value function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    r = bionet_resource_time_from_timeval(tv, resource);
    if (r < 0) {
        // the set_resource_time function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    return resource;


cleanup:
    if (resource != NULL) {
        bionet_resource_free(resource);
    }
    return NULL;
}




bionet_resource_t *bionet_resource_new_with_valueptr_timestr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const char *time_str
) {
    int r;
    bionet_resource_t *resource;


    if (valueptr == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Value passed in");
        return NULL;
    }


    resource = bionet_resource_new_without_value(hab_type, hab_id, node_id, data_type_str, flavor_str, resource_id);
    if (resource == NULL) {
        return NULL;
    }


    r = bionet_resource_value_from_pointer(valueptr, resource);
    if (r < 0) {
        // the set_resource_value function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    r = bionet_resource_time_from_string(time_str, resource);
    if (r < 0) {
        // the set_resource_time function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    return resource;


cleanup:
    if (resource != NULL) {
        bionet_resource_free(resource);
    }
    return NULL;
}




bionet_resource_t *bionet_resource_new_with_valueptr_timevalptr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const struct timeval *tv
) {
    int r;
    bionet_resource_t *resource;


    if (valueptr == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_new(): NULL Resource-Value passed in");
        return NULL;
    }


    resource = bionet_resource_new_without_value(hab_type, hab_id, node_id, data_type_str, flavor_str, resource_id);
    if (resource == NULL) {
        return NULL;
    }


    r = bionet_resource_value_from_pointer(valueptr, resource);
    if (r < 0) {
        // the set_resource_value function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    r = bionet_resource_time_from_timeval(tv, resource);
    if (r < 0) {
        // the set_resource_time function will have logged an error message if it failed, so we dont have to
        goto cleanup;
    }

    return resource;


cleanup:
    if (resource != NULL) {
        bionet_resource_free(resource);
    }
    return NULL;
}


