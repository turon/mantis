
#include <errno.h>
#include <string.h>

#include <glib.h>

#include "bionet.h"
#include "libbionet-internal.h"


void bionet_set_nag_hostname(const char *hostname) {
    if (libbionet_nag_hostname != NULL) {
        free(libbionet_nag_hostname);
        libbionet_nag_hostname = NULL;
    }

    if (hostname != NULL) {
        libbionet_nag_hostname = strdup(hostname);
        if (libbionet_nag_hostname == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory while setting NAG hostname to '%s'", hostname);
        }
    }
}

