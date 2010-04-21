
#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"
#include "bionet-util.h"


bionet_resource_t *libbionet_parse_resource(xmlNode *x) {
    // these are optional
    xmlChar *hab_type=NULL, *hab_id=NULL, *node_id=NULL;

    // these are required
    xmlChar *resource_id=NULL, *flavor=NULL, *type=NULL, *value=NULL, *time=NULL;

    bionet_resource_t *resource = NULL;


    if (x->type != XML_ELEMENT_NODE) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "invalid XML element type passed to libbionet_parse_resource()");
        return NULL;
    }

    if (strcmp((char *)x->name, "resource") != 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "non-<resource> XML element passed to libbionet_parse_resource()");
        return NULL;
    }


    // these can be NULL so we dont have to check them
    hab_type = xmlGetProp(x, (xmlChar *)"hab-type");
    hab_id   = xmlGetProp(x, (xmlChar *)"hab-id");
    node_id  = xmlGetProp(x, (xmlChar *)"node-id");

    resource_id = xmlGetProp(x, (xmlChar *)"id");
    if (resource_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <resource> with no Resource-ID");
        goto cleanup;
    }

    type = xmlGetProp(x, (xmlChar *)"type");
    if (type == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <resource> with no Type");
        goto cleanup;
    }

    flavor = xmlGetProp(x, (xmlChar *)"flavor");
    if (flavor == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <resource> with no Flavor");
        goto cleanup;
    }

    value = xmlGetProp(x, (xmlChar *)"value");
    if (value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <resource> with no Value");
        goto cleanup;
    }

    time = xmlGetProp(x, (xmlChar *)"time");
    if (time == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <resource> with no Time");
        goto cleanup;
    }


    resource = bionet_resource_new_with_valuestr_timestr(
        (char *)hab_type,
        (char *)hab_id,
        (char *)node_id,
        (char *)type,
        (char *)flavor,
        (char *)resource_id,
        (char *)value,
        (char *)time
    );


cleanup:
    if (hab_type != NULL)    xmlFree(hab_type);
    if (hab_id != NULL)      xmlFree(hab_id);
    if (node_id != NULL)     xmlFree(node_id);
    if (resource_id != NULL) xmlFree(resource_id);
    if (flavor != NULL)      xmlFree(flavor);
    if (type != NULL)        xmlFree(type);
    if (value != NULL)       xmlFree(value);
    if (time != NULL)        xmlFree(time);

    return resource;
}

