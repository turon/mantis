
#include <sys/types.h>
#include <sys/socket.h>

#include "bionet-util.h"


int bionet_stream_accept(bionet_stream_t *stream) {
    if (stream == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_stream_accept(): NULL stream passed in!");
        return -1;
    }

    return accept(stream->socket, NULL, 0);
}

