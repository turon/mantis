
#include <string.h>

#include "bionet-util.h"


const char *bionet_stream_direction_to_string(bionet_stream_direction_t direction) {
    static char *stream_direction_to_string_array[] = {
        "(invalid)",
        "Producer",
        "Consumer"
    };

    // FIXME: validate stream_direction

    return stream_direction_to_string_array[direction];
}


bionet_stream_direction_t bionet_stream_direction_from_string(const char *direction_string) {

    if (strcasecmp(direction_string, "producer") == 0) {
        return BIONET_STREAM_DIRECTION_PRODUCER;

    } else if (strcasecmp(direction_string, "consumer") == 0) {
        return BIONET_STREAM_DIRECTION_CONSUMER;
    }

    return BIONET_RESOURCE_DATA_TYPE_INVALID;
}

