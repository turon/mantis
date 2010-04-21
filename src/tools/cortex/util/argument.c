

#include <stdlib.h>

#include "bionet-util.h"




bionet_command_argument_t *bionet_command_argument_new(
    const char *id, 
    bionet_resource_data_type_t data_type,
    const bionet_resource_value_t *min,
    const bionet_resource_value_t *max,
    const bionet_resource_value_t *default_value
) {
    bionet_command_argument_t *arg;

    if (id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): NULL id passed in!");
        return NULL;
    }

    if (default_value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): NULL default_value passed in!");
        return NULL;
    }

    // min and max may be NULL, to indicate no limits (other than the limits imposed by the data type)

    arg = (bionet_command_argument_t *)calloc(1, sizeof(bionet_command_argument_t));
    if (arg == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): out of memory");
        return NULL;
    }

    arg->data_type = data_type;

    if (min != NULL) {
        arg->min = bionet_resource_value_dup(min);
        if (arg->min == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): out of memory");
            return NULL;
        }
    }

    if (max != NULL) {
        arg->max = bionet_resource_value_dup(max);
        if (arg->max == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): out of memory");
            return NULL;
        }
    }

    arg->default_value = bionet_resource_value_dup(default_value);
    if (arg->default_value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): out of memory");
        return NULL;
    }

    arg->value = bionet_resource_value_dup(default_value);
    if (arg->value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_new(): out of memory");
        return NULL;
    }

    return arg;
}




void bionet_command_argument_free(bionet_command_argument_t *arg) {
    if (arg == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_argument_free(): NULL arg passed in");
        return;
    }

    if (arg->id != NULL) {
        free(arg->id);
    }

    if (arg->min != NULL) {
        free(arg->min);
    }

    if (arg->max != NULL) {
        free(arg->max);
    }

    if (arg->default_value != NULL) {
        free(arg->default_value);
    }

    if (arg->value != NULL) {
        free(arg->value);
    }

    free(arg);
}

