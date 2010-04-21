
#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"



int bionet_nag_message_queue_is_empty(void) {
    if (libbionet_queued_messages_from_nag == NULL) {
        return 1;
    }

    return 0;
}




void bionet_handle_one_queued_nag_message(void) {
    xmlDoc *xml;
    xmlNode *root;


    xml = g_slist_nth_data(libbionet_queued_messages_from_nag, 0);
    if (xml == NULL) return;

    libbionet_queued_messages_from_nag = g_slist_remove(libbionet_queued_messages_from_nag, xml);


    root = xmlDocGetRootElement(xml);

    if (strcmp((char *)root->name, "hab") == 0) {
        bionet_hab_t *hab;

        hab = libbionet_parse_hab_from_xml(root);
        xmlFreeDoc(xml);
        if (hab == NULL) {
            return;
        }
        libbionet_cache_add_hab(hab);

        if (libbionet_callback_new_hab != NULL) {
            libbionet_callback_new_hab(hab);
        }

    } else if (strcmp((char *)root->name, "lost-hab") == 0) {
        xmlChar *type, *id;
        bionet_hab_t *hab;

        type = xmlGetProp(root, (xmlChar *)"type");
        if (type == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-hab> with no type");
            xmlFreeDoc(xml);
            return;
        }

        id = xmlGetProp(root, (xmlChar *)"id");
        if (id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-hab> with no id");
            xmlFree(type);
            xmlFreeDoc(xml);
            return;
        }

        hab = bionet_cache_lookup_hab((char *)type, (char *)id);
        xmlFree(type);
        xmlFree(id);
        xmlFreeDoc(xml);
        if (hab == NULL) {
            return;
        }

        if (libbionet_callback_lost_hab != NULL) {
            libbionet_callback_lost_hab(hab);
        }

        libbionet_cache_remove_hab(hab);

    } else if (strcmp((char *)root->name, "node") == 0) {
        bionet_node_t *node;

        if (libbionet_callback_new_node == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_INFO, "dropping <node> request from NAG: no callback registered");
            xmlFreeDoc(xml);
            return;
        }

        node = libbionet_parse_node_from_xml(root);

        xmlFreeDoc(xml);

        if (node == NULL) {
            return;
        }

        libbionet_cache_add_node(node);

        libbionet_callback_new_node(node);

    } else if (strcmp((char *)root->name, "lost-node") == 0) {
        xmlChar *hab_type, *hab_id, *node_id;
        bionet_node_t *node;

        hab_type = xmlGetProp(root, (xmlChar *)"hab-type");
        if (hab_type == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-node> with no hab-type");
            xmlFreeDoc(xml);
            return;
        }

        hab_id = xmlGetProp(root, (xmlChar *)"hab-id");
        if (hab_id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-node> with no hab-id");
            xmlFree(hab_type);
            xmlFreeDoc(xml);
            return;
        }

        node_id = xmlGetProp(root, (xmlChar *)"node-id");
        if (node_id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-node> with no node-id");
            xmlFree(hab_type);
            xmlFree(hab_id);
            xmlFreeDoc(xml);
            return;
        }

        node = bionet_cache_lookup_node((char *)hab_type, (char *)hab_id, (char *)node_id);

        xmlFree(hab_type);
        xmlFree(hab_id);
        xmlFree(node_id);
        xmlFreeDoc(xml);

        if (node == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got <lost-node> for an unknown Node (%s.%s.%s)", hab_type, hab_id, node_id);
            return;
        }

        if (libbionet_callback_lost_node != NULL) {
            libbionet_callback_lost_node(node);
        }

        libbionet_cache_remove_node(node);

    } else if (strcmp((char *)root->name, "resource") == 0) {
        bionet_resource_t *resource;
        bionet_resource_t *cached_resource;

        resource = libbionet_parse_resource(root);

        xmlFreeDoc(xml);

        if (resource == NULL) {
            return;
        }

        cached_resource = bionet_cache_lookup_resource(resource->hab_type, resource->hab_id, resource->node_id, resource->id);
        if (cached_resource == NULL) {
            libbionet_cache_add_resource(resource);
            cached_resource = resource;
        } else {
            bionet_resource_value_copy(&resource->value, &cached_resource->value);
            memcpy(&cached_resource->time, &resource->time, sizeof(struct timeval));
            bionet_resource_free(resource);
        }

        if (libbionet_callback_resource_value != NULL) {
            libbionet_callback_resource_value(cached_resource);
        }

    } else {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "unhandled message from NAG: %s", root->name);
        xmlFreeDoc(xml);
    }
}


void bionet_handle_queued_nag_messages(void) {
    while (! bionet_nag_message_queue_is_empty()) {
        bionet_handle_one_queued_nag_message();
    }
}


