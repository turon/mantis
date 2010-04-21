

#include <ctype.h>
#include <stdio.h>

#include <glib.h>

#include "bionet-util.h"




int bionet_is_valid_name_component(const char *str) {
    if (str == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_is_valid_name_component(): NULL string passed in!");
        return 0;
    }

    // zero-length string is NOT valid
    if (*str == '\0') return 0;

    while (*str != '\0') {
        if (!isalnum(*str) && (*str != '-')) return 0;
        str ++;
    }

    return 1;
}


