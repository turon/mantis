
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"


int bionet_node_add_command(bionet_node_t *node, bionet_command_t *cmd) {
    if (node == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_command(): NULL Node passed in");
        return -1;
    }

    if (cmd == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_node_add_command(): NULL Command passed in");
        return -1;
    }

    node->commands = g_slist_append(node->commands, cmd);

    return 0;
}

