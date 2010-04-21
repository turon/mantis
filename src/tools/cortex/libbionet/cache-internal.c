
#include "libbionet-internal.h"
#include "bionet.h"




void libbionet_cache_add_hab(bionet_hab_t *hab) {
    bionet_habs = g_slist_prepend(bionet_habs, hab);
}


void libbionet_cache_remove_hab(bionet_hab_t *hab) {
    bionet_habs = g_slist_remove(bionet_habs, hab);
}


void libbionet_cache_add_node(bionet_node_t *node) {
    bionet_hab_t *hab;

    hab = bionet_cache_lookup_hab(node->hab_type, node->hab_id);
    if (hab == NULL) {
        hab = bionet_hab_new(node->hab_type, node->hab_id);
        libbionet_cache_add_hab(hab);
    }

    hab->nodes = g_slist_prepend(hab->nodes, node);
}


void libbionet_cache_remove_node(bionet_node_t *node) {
    bionet_hab_t *hab;

    hab = bionet_cache_lookup_hab(node->hab_type, node->hab_id);
    if (hab == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cache tried to remove a node for a non-existent HAB (%s.%s.%s)", node->hab_type, node->hab_id, node->id);
        return;
    }

    hab->nodes = g_slist_remove(hab->nodes, node);
}


void libbionet_cache_add_resource(bionet_resource_t *resource) {
    bionet_node_t *node;

    node = bionet_cache_lookup_node(resource->hab_type, resource->hab_id, resource->node_id);
    if (node == NULL) {
        node = bionet_node_new(resource->hab_type, resource->hab_id, resource->node_id);
        libbionet_cache_add_node(node);
    }

    node->resources = g_slist_prepend(node->resources, resource);
}


