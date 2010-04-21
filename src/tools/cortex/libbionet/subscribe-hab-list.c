
#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"


int bionet_subscribe_hab_list_by_habtype_habid(const char *hab_type,  const char *hab_id) {
    int r;


    if (hab_type == NULL) hab_type = "*";
    if (hab_id   == NULL) hab_id   = "*";

    // FIXME: here check the strings for validity ([-a-zA-Z0-9])


    if (bionet_connect_to_nag() < 0) {
        return -1;
    }


    r = bionet_nxio_send(libbionet_nag_nxio, "<subscribe-hab-list type='%s' id='%s'/>", hab_type, hab_id);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list(): error sending request to NAG");
        return -1;
    }

    r = libbionet_read_ok_from_nag();
    if (r != 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list(): error reading reply from NAG");
        libbionet_kill_nag_connection();
        return -1;
    }

    return 0;
}




int bionet_subscribe_hab_list_by_name_pattern(const char *hab_name_pattern) {
    char hab_type[256];
    char hab_id[256];

    const char *p;
    int size;
    char *separator;


    if (hab_name_pattern == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list_by_name_pattern(): NULL hab_name_pattern passed in");
        return -1;
    }

    p = hab_name_pattern;


    // get the HAB-Type
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list_by_name_pattern(): error parsing pattern '%s'", hab_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_type) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list_by_name_pattern(): HAB-Type of '%s' is too long (max %d)", hab_name_pattern, (sizeof(hab_type) - 1));
        return -1;
    }
    memcpy(hab_type, p, size);
    hab_type[size] = (char)NULL;


    // get the HAB-ID
    p = separator + 1;
    if (strlen(p) > (sizeof(hab_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_subscribe_hab_list_by_name_pattern(): HAB-ID of '%s' is too long (max %d)", hab_name_pattern, (sizeof(hab_id) - 1));
        return -1;
    }
    strcpy(hab_id, p);

    return bionet_subscribe_hab_list_by_habtype_habid(hab_type, hab_id);
}

