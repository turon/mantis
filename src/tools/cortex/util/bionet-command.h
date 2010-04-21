

#ifndef __BIONET_COMMAND_H
#define __BIONET_COMMAND_H


#include <glib.h>

#include "bionet-util.h"


typedef struct {
    char *id;
    bionet_resource_data_type_t data_type;
    bionet_resource_value_t *min;
    bionet_resource_value_t *max;
    bionet_resource_value_t *default_value;
    bionet_resource_value_t *value;
} bionet_command_argument_t;


typedef struct {
    char *id;
    GSList *arguments;  // each is a bionet_command_argument_t*
} bionet_command_t;




bionet_command_argument_t *bionet_command_argument_new(
    const char *id, 
    bionet_resource_data_type_t data_type,
    const bionet_resource_value_t *min,
    const bionet_resource_value_t *max,
    const bionet_resource_value_t *default_value
);

void bionet_command_argument_free(bionet_command_argument_t *arg);




bionet_command_t *bionet_command_new(const char *id);

void bionet_command_free(bionet_command_t *cmd);

void bionet_command_add_argument(bionet_command_t *cmd, bionet_command_argument_t *arg);




#endif //  __BIONET_COMMAND_H

