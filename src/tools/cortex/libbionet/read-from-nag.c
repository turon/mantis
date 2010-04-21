

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <glib.h>

#include "libbionet-internal.h"
#include "bionet.h"




//
// FIXME: these functions should return errors to the caller
//




void bionet_read_from_nag(void) {
    bionet_read_from_nag_but_dont_handle_messages();
    bionet_handle_queued_nag_messages();
}




void bionet_read_from_nag_but_dont_handle_messages(void) {
    if (bionet_connect_to_nag() < 0) {
        return;
    }

    do {
        xmlDoc *xml;
        int r;
        struct timeval timeout;

        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        r = bionet_nxio_read(libbionet_nag_nxio, &timeout, &xml);
        if (r < 0) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error reading from NAG: %s", strerror(errno));
            libbionet_kill_nag_connection();
            return;
        }

        if (xml == NULL) {
            // nothing from the NAG
            return;
        }

        libbionet_queued_messages_from_nag = g_slist_append(libbionet_queued_messages_from_nag, xml);
    } while(1);
}

