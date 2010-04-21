
#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"


int bionet_subscribe_resource_by_habtype_habid_nodeid_resourceid(const char *hab_type,  const char *hab_id, const char *node_id, const char *resource_id) {
    int r;


    if (hab_type     == NULL) hab_type = "*";
    if (hab_id       == NULL) hab_id   = "*";
    if (node_id      == NULL) node_id  = "*";
    if (resource_id  == NULL) resource_id  = "*";

    // FIXME: here check the strings for validity ([-a-zA-Z0-9])


    if (bionet_connect_to_nag() < 0) {
        return -1;
    }


    r = bionet_nxio_send(
        libbionet_nag_nxio,
        "<subscribe-resource hab-type='%s' hab-id='%s' node-id='%s' resource-id='%s'/>",
        hab_type,
        hab_id,
        node_id,
        resource_id
    );
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource(): error sending request to NAG");
        return -1;
    }

    r = libbionet_read_ok_from_nag();
    if (r != 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource(): error reading reply from NAG");
        libbionet_kill_nag_connection();
        return -1;
    }

    return 0;
}




int bionet_subscribe_resource_by_name_pattern(const char *resource_name_pattern) {
    char hab_type[256];
    char hab_id[256];
    char node_id[256];
    char resource_id[256];

    const char *p;
    int size;
    char *separator;


    if (resource_name_pattern == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): NULL resource_name_pattern passed in");
        return -1;
    }

    p = resource_name_pattern;


    // get the HAB-Type
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): error parsing pattern '%s'", resource_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_type) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): HAB-Type of Resource Name pattern '%s' is too long (max %d)", resource_name_pattern, (sizeof(hab_type) - 1));
        return -1;
    }
    memcpy(hab_type, p, size);
    hab_type[size] = (char)NULL;


    // get the HAB-ID
    p = separator + 1;
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): error parsing pattern '%s'", resource_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): HAB-ID of Resource Name pattern '%s' is too long (max %d)", resource_name_pattern, (sizeof(hab_id) - 1));
        return -1;
    }
    memcpy(hab_id, p, size);
    hab_id[size] = (char)NULL;


    // get the Node-ID
    p = separator + 1;
    separator = strchr(p, ':');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): error parsing pattern '%s'", resource_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(node_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): Node-ID of Resource Name pattern '%s' is too long (max %d)", resource_name_pattern, (sizeof(node_id) - 1));
        return -1;
    }
    memcpy(node_id, p, size);
    node_id[size] = (char)NULL;


    // get the Resource-ID
    p = separator + 1;
    if (strlen(p) > (sizeof(resource_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_resource_by_name_pattern(): Resource-ID of Resource Name pattern '%s' is too long (max %d)", resource_name_pattern, (sizeof(resource_id) - 1));
        return -1;
    }
    strcpy(resource_id, p);


    return bionet_subscribe_resource_by_habtype_habid_nodeid_resourceid(hab_type, hab_id, node_id, resource_id);
}

