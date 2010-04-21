

#include <errno.h>
#include <string.h>
#include <sys/time.h>

#include <libxml/tree.h>

#include <glib.h>

#include "libbionet-internal.h"




// FIXME: this treats timeouts and network errors and Nag error messages all the same: return -1...
int libbionet_read_ok_from_nag(void) {
    int r;
    xmlDoc *xml;
    xmlNode *reply;


    r = bionet_connect_to_nag();
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error connecting to NAG");
        return -1;
    }


    {
        const char *accept[] = { "ok", "error", NULL };
        struct timeval timeout;

        timeout.tv_sec = libbionet_nag_timeout;
        timeout.tv_usec = 0;

        r = bionet_nxio_read_acceptable(libbionet_nag_nxio, &timeout, &xml, accept, &libbionet_queued_messages_from_nag);
        if (r < 0) {
            return -1;
        }

        if (xml == NULL) {
            // timeout
            return -1;
        }
    }

    reply = xmlDocGetRootElement(xml);

    if (strcmp((char *)reply->name, "error") == 0) {
        libbionet_handle_error_message(xml);
        return -1;
    }

    // "ok" message, perfect, just what we wanted
    free(libbionet_nag_error);
    libbionet_nag_error = NULL;

    xmlFreeDoc(xml);

    return 0;
}


