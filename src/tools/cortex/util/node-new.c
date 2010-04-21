#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"



bionet_node_t* bionet_node_new(
	const char* hab_type,
	const char* hab_id,
	const char* node_id
) {
    bionet_node_t* node;

    if (node_id == NULL) {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "node-new(): NULL node id passed in");
	return NULL;
    }


    node = calloc(1, sizeof(bionet_node_t));
    if (node == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
	return NULL;
    }

    if (hab_type != NULL) {
        node->hab_type = strdup(hab_type);
        if (node->hab_type == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            free(node);
            return NULL;
        }
    }

    if (hab_id != NULL) {
        node->hab_id = strdup(hab_id);
        if (node->hab_id == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            free(node->hab_type);
            free(node);
            return NULL;
        }
    }

    node->id = strdup(node_id);
    if (node->id == NULL) {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
            free(node->hab_type);
            free(node->hab_id);
            free(node);
	return NULL;
    }

    return node;
}

