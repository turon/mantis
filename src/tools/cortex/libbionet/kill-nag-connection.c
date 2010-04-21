

#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <sys/socket.h>

#include <glib.h>

#include "libbionet-internal.h"
#include "bionet.h"




void libbionet_kill_nag_connection(void) {
    bionet_nxio_reset(libbionet_nag_nxio);

    if (libbionet_nag_nxio->socket < 0) return;


    // for the half-close scenario

    if (shutdown(libbionet_nag_nxio->socket, 2) != 0) {
        if (errno != ENOTCONN) {
            g_log(
                BIONET_LOG_DOMAIN,
                G_LOG_LEVEL_WARNING,
                "libbionet_kill_nag_connection(): error shutting down socket file descriptor: %s",
                strerror(errno)
            );
        }
    }

    if (close(libbionet_nag_nxio->socket) != 0) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "libbionet_kill_nag_connection(): error closing socket file descriptor: %s",
            strerror(errno)
        );
    }

    libbionet_nag_nxio->socket = -1;
}


