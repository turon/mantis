

#define _GNU_SOURCE // for strndup(3)


#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "libbionet-internal.h"
#include "bionet.h"




// shortcut for lazy programmers
int bionet_list_all_habs(GSList **habs) {
    return bionet_list_habs_by_type_and_id(habs, "*", "*");
}




int bionet_list_habs_by_name_pattern(GSList **habs, const char *hab_name_pattern) {
    char hab_type[256];
    char hab_id[256];

    const char *p;
    int size;
    char *separator;

    if (hab_name_pattern == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs_by_name_pattern(): NULL hab_name_pattern passed in");
        return -1;
    }

    p = hab_name_pattern;


    // get the HAB-Type
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs_by_name_pattern(): error parsing pattern '%s'", hab_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_type) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs_by_name_pattern(): HAB-Type of HAB Name pattern '%s' is too long (max %d)", hab_name_pattern, (sizeof(hab_type) - 1));
        return -1;
    }
    memcpy(hab_type, p, size);
    hab_type[size] = (char)NULL;


    // get the HAB-ID
    p = separator + 1;
    if (strlen(p) > (sizeof(hab_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs_by_name_pattern(): HAB-ID of HAB Name pattern '%s' is too long (max %d)", hab_name_pattern, (sizeof(hab_id) - 1));
        return -1;
    }
    strcpy(hab_id, p);


    return bionet_list_habs_by_type_and_id(habs, hab_type, hab_id);
}


    

int bionet_list_habs_by_type_and_id(GSList **habs, const char *hab_type, const char *hab_id) {
    int r;

    xmlDoc *xml;
    xmlNode *reply;
    xmlNode *hab_node;


    *habs = (GSList *)NULL;

    if (hab_type == NULL) hab_type = "*";
    if (hab_id == NULL) hab_id = "*";


    // FIXME: here check the bytes in hab_type and hab_id for validity ([-a-zA-Z0-9])


    if (bionet_connect_to_nag() < 0) {
        return -1;
    }


    r = bionet_nxio_send(libbionet_nag_nxio, "<list-habs type='%s' id='%s'/>", hab_type, hab_id);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs(): error sending request to NAG");
        return -1;
    }


    {
        const char *accept[] = { "habs", "error", NULL };
        struct timeval timeout;

        timeout.tv_sec = libbionet_nag_timeout;
        timeout.tv_usec = 0;

        r = bionet_nxio_read_acceptable(libbionet_nag_nxio, &timeout, &xml, accept, &libbionet_queued_messages_from_nag);
        if (r < 0) {
            libbionet_kill_nag_connection();
            return -1;
        }
    }


    if (xml == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_habs(): timeout reading reply from NAG");
        libbionet_kill_nag_connection();
        return -1;
    }

    reply = xmlDocGetRootElement(xml);

    if (strcmp((char *)reply->name, "error") == 0) {
        libbionet_handle_error_message(xml);
        return -1;
    }



    //
    // look at all those HABs!
    //

    for (hab_node = reply->children; hab_node; hab_node = hab_node->next) {
        xmlChar *type, *id;
        bionet_hab_t *hab;


        if (hab_node->type != XML_ELEMENT_NODE) {
            continue;
        }


        hab = (bionet_hab_t *)calloc(1, sizeof(bionet_hab_t));
        if (hab == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "bionet_list_habs(): out of memory reading hab list!");
            xmlFreeDoc(xml);
            return -1;
        }


        type = xmlGetProp(hab_node, (xmlChar *)"type");
        if (type == NULL) {
            // FIXME: clean up
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "prop 'type' not found!");
            xmlFreeDoc(xml);
            return -1;
        }

        id = xmlGetProp(hab_node, (xmlChar *)"id");
        if (id == NULL) {
            // FIXME: clean up
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "prop 'id' not found!");
            xmlFreeDoc(xml);
            return -1;
        }

        hab->type = strdup((char *)type);
        hab->id = strdup((char *)id);

        xmlFree(type);
        xmlFree(id);

        *habs = g_slist_append(*habs, hab);
    }

    xmlFreeDoc(xml);

    return 0;
}


