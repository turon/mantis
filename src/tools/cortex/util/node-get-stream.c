
#include <string.h>

#include <glib.h>

#include "bionet-util.h"


bionet_stream_t *bionet_node_get_stream_by_id(bionet_node_t *node, const char *stream_id) {
    GSList *i;


    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_get_stream_by_id(): NULL Node passed in");
        return NULL;
    }

    if (stream_id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_get_stream_by_id(): NULL Stream-ID passed in");
        return NULL;
    }


    for (i = node->streams; i != NULL; i = i->next) {
        bionet_stream_t *stream = i->data;

        if (strcmp(stream->id, stream_id) == 0) {
            return stream;
        }
    }

    return NULL;
}

