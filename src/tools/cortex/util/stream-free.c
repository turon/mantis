
#include <stdlib.h>

#include "bionet-stream.h"


void bionet_stream_free(bionet_stream_t *stream) {
    if (stream == NULL) {
        return;
    }

    if (stream->id != NULL) {
        free(stream->id);
    }

    if (stream->type != NULL) {
        free(stream->type);
    }

    if (stream->host != NULL) {
        free(stream->host);
    }

    free(stream);
}

