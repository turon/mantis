
#include <string.h>

#include "bionet-util.h"


const char *bionet_resource_flavor_to_string(bionet_resource_flavor_t flavor) {
    static char *resource_flavor_to_string_array[] = {
        "(invalid)",
        "Sensor",
        "Actuator",
        "Parameter"
    };

    // FIXME: validate flavor

    return resource_flavor_to_string_array[flavor];
}


bionet_resource_flavor_t bionet_resource_flavor_from_string(const char *flavor_string) {

    if (strcasecmp(flavor_string, "sensor") == 0) {
        return BIONET_RESOURCE_FLAVOR_SENSOR;

    } else if (strcasecmp(flavor_string, "actuator") == 0) {
        return BIONET_RESOURCE_FLAVOR_ACTUATOR;

    } else if (strcasecmp(flavor_string, "parameter") == 0) {
        return BIONET_RESOURCE_FLAVOR_PARAMETER;

    }

    return BIONET_RESOURCE_FLAVOR_INVALID;
}




