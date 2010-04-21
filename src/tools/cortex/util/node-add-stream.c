

#include "bionet-util.h"




int bionet_node_add_stream(bionet_node_t *node, bionet_stream_t *stream) {
    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_stream(): NULL Node passed in");
        return -1;
    }

    if (stream == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_stream(): NULL Stream passed in");
        return -1;
    }

    node->streams = g_slist_append(node->streams, stream);

    return 0;
}




int bionet_node_add_stream_native(
    bionet_node_t *node,
    const char *id,
    bionet_stream_direction_t direction,
    const char *type,
    const char *host,
    uint16_t port
) {
    bionet_stream_t *stream;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_stream(): NULL Node passed in");
        return -1;
    }

    stream = bionet_stream_new(id, direction, type, host, port);
    if (stream == NULL) {
        // the bionet_stream_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->streams = g_slist_append(node->streams, stream);

    return 0;
}




int bionet_node_add_stream_from_strings(
    bionet_node_t *node,
    const char *id,
    const char *direction_str,
    const char *type,
    const char *host,
    const char *port_str
) {
    bionet_stream_t *stream;

    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_stream_from_strings(): NULL Node passed in");
        return -1;
    }

    stream = bionet_stream_new_from_strings(id, direction_str, type, host, port_str);
    if (stream == NULL) {
        // the bionet_stream_new() function will have printed an error message so we dont have to
        return -1;
    }

    node->streams = g_slist_append(node->streams, stream);

    return 0;
}


