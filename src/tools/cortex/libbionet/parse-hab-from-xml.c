

#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"
#include "bionet-util.h"


bionet_hab_t *libbionet_parse_hab_from_xml(xmlNode *hab_node) {
    xmlChar *hab_type, *hab_id;
    bionet_hab_t *hab;


    if (hab_node->type != XML_ELEMENT_NODE) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "invalid XML element type passed to libbionet_parse_hab_from_xml()");
        return NULL;
    }

    if (strcmp((char *)hab_node->name, "hab") != 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "non-<hab> XML element passed to libbionet_parse_hab_from_xml()");
        return NULL;
    }


    hab = (bionet_hab_t *)calloc(1, sizeof(bionet_hab_t));
    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        return NULL;
    }


    //
    // handle hab-type
    //

    hab_type = xmlGetProp(hab_node, (xmlChar *)"type");
    if (hab_type == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing <hab> message, prop 'type' not found!");
        free(hab);
        return NULL;
    }

    hab->type = strdup((char *)hab_type);
    xmlFree(hab_type);

    if (hab->type == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        free(hab);
        return NULL;
    }


    //
    // handle hab-id
    //

    hab_id = xmlGetProp(hab_node, (xmlChar *)"id");
    if (hab_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing <hab> message, prop 'id' not found!");
        free(hab->type);
        free(hab);
        return NULL;
    }

    hab->id = strdup((char *)hab_id);
    xmlFree(hab_id);

    if (hab->id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        free(hab->type);
        free(hab);
        return NULL;
    }

    return hab;
}


