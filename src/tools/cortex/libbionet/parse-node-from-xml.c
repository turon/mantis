

#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"
#include "bionet-util.h"


int libbionet_parse_node_resource_from_xml(bionet_node_t *node, xmlNode *resource_node) {
    xmlChar *id, *flavor, *type, *value, *time;
    int r;

    id = xmlGetProp(resource_node, (xmlChar *)"id");
    flavor = xmlGetProp(resource_node, (xmlChar *)"flavor");
    type = xmlGetProp(resource_node, (xmlChar *)"type");
    value = xmlGetProp(resource_node, (xmlChar *)"value");
    time = xmlGetProp(resource_node, (xmlChar *)"time");

    if (!bionet_is_valid_name_component((char *)id)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NAG published Node %s.%s.%s with invalid Resource-ID '%s'", node->hab_type, node->hab_id, node->id, id);
        xmlFree(id);
        xmlFree(flavor);
        xmlFree(type);
        xmlFree(value);
        xmlFree(time);
        return -1;
    }

    r = bionet_node_add_resource_with_valuestr_timestr(node, (char *)type, (char *)flavor, (char *)id, (char *)value, (char *)time);

    xmlFree(id);
    xmlFree(flavor);
    xmlFree(type);
    xmlFree(value);
    xmlFree(time);

    if (r < 0) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "Cannot parse Resource for published Node %s.%s.%s: type='%s', flavor='%s', id='%s', value='%s', time='%s'",
            node->hab_type,
            node->hab_id,
            node->id,
            type,
            flavor,
            id,
            value,
            time
        );
        return -1;
    }

    return 0;
}


int libbionet_parse_node_stream_from_xml(bionet_node_t *node, xmlNode *stream_node) {
    xmlChar *id, *direction, *type, *host, *port;
    int r;

    id = xmlGetProp(stream_node, (xmlChar *)"id");
    direction = xmlGetProp(stream_node, (xmlChar *)"direction");
    type = xmlGetProp(stream_node, (xmlChar *)"type");
    host = xmlGetProp(stream_node, (xmlChar *)"host");
    port = xmlGetProp(stream_node, (xmlChar *)"port");

    if (!bionet_is_valid_name_component((char *)id)) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NAG published Node %s.%s.%s with invalid Stream-ID '%s'", node->hab_type, node->hab_id, node->id, id);
        xmlFree(id);
        xmlFree(direction);
        xmlFree(type);
        xmlFree(host);
        xmlFree(port);
        return -1;
    }

    r = bionet_node_add_stream_from_strings(node, (char *)id, (char *)direction, (char *)type, (char *)host, (char *)port);

    xmlFree(id);
    xmlFree(direction);
    xmlFree(type);
    xmlFree(host);
    xmlFree(port);

    if (r < 0) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "Cannot parse Stream for published Node %s.%s.%s: id='%s', direction='%s', type='%s', host='%s', port='%s'",
            node->hab_type,
            node->hab_id,
            node->id,
            id,
            direction,
            type,
            host,
            port
        );
        return -1;
    }

    return 0;
}


bionet_node_t *libbionet_parse_node_from_xml(xmlNode *node_node) {
    xmlChar *hab_type, *hab_id, *node_id;
    bionet_node_t *node;


    if (node_node->type != XML_ELEMENT_NODE) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "invalid XML element type passed to libbionet_parse_node_from_xml()");
        return NULL;
    }

    if (strcmp((char *)node_node->name, "node") != 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "non-<node> XML element passed to libbionet_parse_node_from_xml()");
        return NULL;
    }


    node = (bionet_node_t *)calloc(1, sizeof(bionet_node_t));
    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        return NULL;
    }


    //
    // handle node-id
    //

    node_id = xmlGetProp(node_node, (xmlChar *)"node-id");
    if (node_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing <node> message, prop 'node-id' not found!");
        free(node);
        return NULL;
    }

    node->id = strdup((char *)node_id);
    xmlFree(node_id);

    if (node->id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        free(node);
        return NULL;
    }


    //
    // handle hab-type
    //

    hab_type = xmlGetProp(node_node, (xmlChar *)"hab-type");
    if (hab_type == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing <node> message, prop 'hab-type' not found!");
        free(node->id);
        free(node);
        return NULL;
    }

    node->hab_type = strdup((char *)hab_type);
    xmlFree(hab_type);

    if (node->hab_type == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        free(node->id);
        free(node);
        return NULL;
    }


    //
    // handle hab-id
    //

    hab_id = xmlGetProp(node_node, (xmlChar *)"hab-id");
    if (hab_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing <node> message, prop 'hab-id' not found!");
        free(node->id);
        free(node->hab_type);
        free(node);
        return NULL;
    }

    node->hab_id = strdup((char *)hab_id);
    xmlFree(hab_id);

    if (node->hab_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
        free(node->id);
        free(node->hab_type);
        free(node);
        return NULL;
    }


    // look at all those features!
    {
        xmlNode *feature_node;

        for (feature_node = node_node->children; feature_node != NULL; feature_node = feature_node->next) {
            int r;

            if (feature_node->type != XML_ELEMENT_NODE) {
                continue;
            }

            if (strcmp((char *)feature_node->name, "resource") == 0) {
                r = libbionet_parse_node_resource_from_xml(node, feature_node);

            } else if (strcmp((char *)feature_node->name, "stream") == 0) {
                r = libbionet_parse_node_stream_from_xml(node, feature_node);

            } else {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NAG published Node %s.%s.%s with invalid feature '%s', skipping", node->hab_type, node->hab_id, node->id, (char *)feature_node->name);
                r = 0;
            }

            if (r < 0) {
                bionet_node_free(node);
                return NULL;
            }
        }
    }

    return node;
}


