

#define _GNU_SOURCE // for strndup(3)


#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "libbionet-internal.h"
#include "bionet.h"




// for lazy application coders
int bionet_list_all_nodes(GSList **nodes) {
    return bionet_list_nodes_by_habtype_habid_nodeid(nodes, "*", "*", "*");
}




int bionet_list_nodes_by_habtype_habid_nodeid(GSList **nodes, const char *hab_type, const char *hab_id, const char *node_id) {
    int r;

    xmlDoc *xml;
    xmlNode *reply;
    xmlNode *node_node;


    *nodes = (GSList *)NULL;

    if (hab_type == NULL) hab_type = "*";
    if (hab_id   == NULL) hab_id   = "*";
    if (node_id  == NULL) node_id  = "*";

    // FIXME: here check the strings for validity ([-a-zA-Z0-9])


    if (bionet_connect_to_nag() < 0) {
        return -1;
    }


    r = bionet_nxio_send(libbionet_nag_nxio, "<list-nodes hab-type='%s' hab-id='%s' node-id='%s'/>", hab_type, hab_id, node_id);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_habtype_habid_nodeid(): error sending request to NAG");
        return -1;
    }


    {
        const char *accept[] = { "nodes", "error", NULL };
        struct timeval timeout;

        timeout.tv_sec = libbionet_nag_timeout;
        timeout.tv_usec = 0;

        r = bionet_nxio_read_acceptable(libbionet_nag_nxio, &timeout, &xml, accept, &libbionet_queued_messages_from_nag);
        if (r < 0) {
            libbionet_kill_nag_connection();
            return -1;
        }
    }


    reply = xmlDocGetRootElement(xml);

    if (strcmp((char *)reply->name, "error") == 0) {
        libbionet_handle_error_message(xml);
        return -1;
    }


    //
    // look at all those Nodes!
    //

    for (node_node = reply->children; node_node != NULL; node_node = node_node->next) {
        bionet_node_t *node;

        // FIXME: If we get "<nodes></nodes>", the "<nodes>" has non-NULL children with non-XML_ELEMENT_NODE types...
        //    This makes it run fine, but I dont fully understand libxml2's structures.
        if (node_node->type != XML_ELEMENT_NODE) {
            continue;
        }

        node = libbionet_parse_node_from_xml(node_node);
        if (node == NULL) {
            // the libbionet_parse_node_from_xml() function will have logged an error message, so we dont have to
            xmlFreeDoc(xml);
            return -1;
        }

        *nodes = g_slist_append(*nodes, node);
    }


    xmlFreeDoc(xml);
    return 0;
}




int bionet_list_nodes_by_name_pattern(GSList **nodes, const char *node_name_pattern) {
    char hab_type[256];
    char hab_id[256];
    char node_id[256];

    const char *p;
    int size;
    char *separator;


    if (node_name_pattern == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): NULL node_name_pattern passed in");
        return -1;
    }

    p = node_name_pattern;


    // get the HAB-Type
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): error parsing pattern '%s'", node_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_type) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): HAB-Type of '%s' is too long (max %d)", node_name_pattern, (sizeof(hab_type) - 1));
        return -1;
    }
    memcpy(hab_type, p, size);
    hab_type[size] = (char)NULL;


    // get the HAB-ID
    p = separator + 1;
    separator = strchr(p, '.');
    if (separator == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): error parsing pattern '%s'", node_name_pattern);
        return -1;
    }
    size = separator - p;
    if (size >= (sizeof(hab_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): HAB-ID of '%s' is too long (max %d)", node_name_pattern, (sizeof(hab_id) - 1));
        return -1;
    }
    memcpy(hab_id, p, size);
    hab_id[size] = (char)NULL;


    // get the Node-ID
    p = separator + 1;
    if (strlen(p) > (sizeof(node_id) - 1)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_list_nodes_by_name_pattern(): Node-ID of '%s' is too long (max %d)", node_name_pattern, (sizeof(node_id) - 1));
        return -1;
    }
    strcpy(node_id, p);


    return bionet_list_nodes_by_habtype_habid_nodeid(nodes, hab_type, hab_id, node_id);
}


