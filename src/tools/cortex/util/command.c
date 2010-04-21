

#include <stdlib.h>
#include <string.h>

#include "bionet-util.h"




bionet_command_t *bionet_command_new(const char *id) {
    bionet_command_t *cmd;

    if (id == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_new(): NULL id passed in");
        return NULL;
    }

    cmd = (bionet_command_t *)calloc(1, sizeof(bionet_command_t));
    if (cmd == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_new(): out of memory");
        return NULL;
    }

    cmd->id = strdup(id);
    if (cmd->id == NULL) {
        free(cmd);
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_new(): out of memory");
        return NULL;
    }

    return cmd;
}




void bionet_command_free(bionet_command_t *cmd) {
    if (cmd == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_free(): NULL command passed in");
        return;
    }

    if (cmd->id != NULL) {
        free(cmd->id);
    }

    while (cmd->arguments != NULL) {
        bionet_command_argument_t *arg;

        arg = cmd->arguments->data;
        cmd->arguments = g_slist_remove(cmd->arguments, arg);

        bionet_command_argument_free(arg);
    }

    free(cmd);
}




void bionet_command_add_argument(bionet_command_t *cmd, bionet_command_argument_t *arg) {
    if (cmd == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_add_argument(): NULL command passed in");
        return;
    }

    if (arg == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_command_add_argument(): NULL argument passed in");
        return;
    }

    cmd->arguments = g_slist_append(cmd->arguments, arg);
}


