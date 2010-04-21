

#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>

#include <netinet/in.h>
#include <netinet/tcp.h>

#include <sys/socket.h>

#include <glib.h>

#include "bionet-nxio.h"




static int bionet_nxio_va_append(bionet_nxio_t *nxio, const char *fmt, va_list ap) {
    int max_size;
    int len;


    errno = 0;

    max_size = NXIO_MAX_MESSAGE_SIZE - nxio->out_index;

    // append the message to the out-buffer
    len = vsnprintf(&nxio->out_buffer[nxio->out_index], max_size, fmt, ap);
    if (len >= max_size) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_append(): message to send exceeds max message size (%d bytes), dropped", NXIO_MAX_MESSAGE_SIZE);
        nxio->out_index = 0;
        errno = ENOSPC;
        return -1;
    }

    nxio->out_index += len;
    return 0;
}




int bionet_nxio_send(bionet_nxio_t *nxio, const char *fmt, ...) {
    va_list ap;
    int r;

    errno = 0;


    va_start(ap, fmt);
    r = bionet_nxio_va_append(nxio, fmt, ap);
    va_end(ap);
    if (r < 0) {
        // nxio_va_append() will have set errno
        return -1;
    }

    return bionet_nxio_flush(nxio);
}




// appends the string to the outgoing buffer
// return 0 on success, -1 if the outgoing buffer overflows (and is automatically reset)
int bionet_nxio_append(bionet_nxio_t *nxio, const char *fmt, ...) {
    va_list ap;
    int r;

    errno = 0;

    va_start(ap, fmt);
    r = bionet_nxio_va_append(nxio, fmt, ap);
    // nxio_va_append() will have set errno as appropriate
    va_end(ap);

    return r;
}




// writes what's in the out-buffer to the socket
// return 0 on success, -1 if there was a problem and the socket is now closed
int bionet_nxio_flush(bionet_nxio_t *nxio) {
    fd_set w, e;
    struct timeval timeout;

    uint8_t header[NXIO_HEADER_SIZE];

    int r;


    errno = 0;

    if (nxio->socket < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_flush(): nxio with closed socket passed in!");
        errno = EPIPE;
        return -1;
    }


    FD_ZERO(&w);
    FD_SET(nxio->socket, &w);

    FD_ZERO(&e);
    FD_SET(nxio->socket, &e);

    timeout.tv_sec = 0;
    timeout.tv_usec = 10 * 1000; // FIXME: or use 100 ms?  what's the right number?

    r = select(nxio->socket + 1, NULL, &w, &e, &timeout);
    if (r < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_flush(): select error (%s)", strerror(errno));
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    if (r == 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_flush(): timeout waiting to send");
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    if (FD_ISSET(nxio->socket, &e)) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_flush(): exception on socket");
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    if (! FD_ISSET(nxio->socket, &w)) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_flush(): socket not writable");
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }


    *((uint32_t *)&header[0]) = htonl(nxio->out_index);
    header[4] = 0x00;

    r = write(nxio->socket, &header, NXIO_HEADER_SIZE);
    if (r < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error writing to socket: %s", strerror(errno));
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    if (r < NXIO_HEADER_SIZE) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "short write to socket!");
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    r = write(nxio->socket, nxio->out_buffer, nxio->out_index);
    if (r < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error writing to socket: %s", strerror(errno));
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }

    if (r < nxio->out_index) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "short write to socket!");
        bionet_nxio_close(nxio);
        errno = EPIPE;
        return -1;
    }


    // FIXME
    // g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_DEBUG, "wrote %d bytes:", r);
    // clogprint_hex(NXIO_LPC, G_LOG_LEVEL_DEBUG, "    ", nxio->out_buffer, r);

    nxio->out_index = 0;
    return 0;
}


