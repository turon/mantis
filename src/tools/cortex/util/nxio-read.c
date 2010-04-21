

#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <netinet/in.h>

#include <glib.h>

#include "bionet-nxio.h"




int bionet_nxio_read_acceptable(bionet_nxio_t *nxio, struct timeval *timeout, xmlDoc **xml, const char **acceptable, GSList **queue) {
    int i;
    int r;

    struct timeval start, timeout_remaining;


    errno = 0;

    if (nxio->socket < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read_acceptable(): nxio with closed socket passed in!");
        errno = EPIPE;
        return -1;
    }

    timeout_remaining.tv_sec = timeout->tv_sec;
    timeout_remaining.tv_usec = timeout->tv_usec;


    *xml = NULL;


    r = gettimeofday(&start, NULL);
    if (r < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error getting time of day: %s", strerror(errno));
        return -1;
    }


    do {
        int r;
        xmlNode *reply;


        r = bionet_nxio_read(nxio, &timeout_remaining, xml);
        if (r < 0) {
            return r;
        }

        if (*xml == NULL) {
            // timeout
            return 0;
        }

        reply = xmlDocGetRootElement(*xml);

        for (i = 0; acceptable[i] != NULL; i ++) {
            if (strcmp((char *)reply->name, acceptable[i]) == 0) {
                // got an acceptable message, return it to caller
                return 0;
            }
        }

        // not an acceptable message, queue it
        // FIXME: this queue should prolly be in the nxio_t
        *queue = g_slist_append(*queue, *xml);


        *xml = NULL;


        {
            struct timeval now, waited_so_far;
            int r;

            r = gettimeofday(&now, NULL);
            if (r < 0) {
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error getting time of day: %s", strerror(errno));
                return -1;
            }

            waited_so_far.tv_sec = (now.tv_sec - start.tv_sec);
            waited_so_far.tv_usec = (now.tv_usec - start.tv_usec);
            if (waited_so_far.tv_usec < 0) {
                waited_so_far.tv_sec --;
                waited_so_far.tv_usec += 1000000;
            }

            timeout_remaining.tv_sec = timeout->tv_sec - waited_so_far.tv_sec;
            timeout_remaining.tv_usec = timeout->tv_usec - waited_so_far.tv_sec;
            if (timeout_remaining.tv_usec < 0) {
                timeout_remaining.tv_sec --;
                timeout_remaining.tv_usec += 1000000;
            }

            if (timeout_remaining.tv_sec < 0) {
                return 0;
            }
        }
    } while (1);

    // NOT REACHED
    return 0;
}




int bionet_nxio_read(bionet_nxio_t *nxio, struct timeval *timeout, xmlDoc **xml) {
    int bytes_to_read;
    uint32_t expected_packet_size;

    struct timeval start, timeout_remaining;

    int r;


    errno = 0;

    if (nxio->socket < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): nxio with closed socket passed in!");
        errno = EPIPE;
        return -1;
    }


    // FIXME: NULL should mean "block until something comes in"
    if (timeout == NULL) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): NULL timeout passed in");
        errno = EINVAL;
        return -1;
    }

    timeout_remaining.tv_sec = timeout->tv_sec;
    timeout_remaining.tv_usec = timeout->tv_usec;


    *xml = (xmlDoc *)NULL;


#ifdef NXIO_DEBUG
    // FIXME
    g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, "on entry to nxio_read, index=%d", nxio->in_index);
    log_hexdump(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, nxio->in_buffer, nxio->in_index);
#endif

    r = gettimeofday(&start, NULL);
    if (r < 0) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): error getting time of day: %s", strerror(errno));
        return -1;
    }


    do {
        struct timeval now, waited_so_far;
        int r;

        // wait for the socket to become readable
        {
            fd_set read_fds;
            int r;

#ifdef NXIO_DEBUG
            g_log(
                NXIO_LOG_DOMAIN,
                G_LOG_LEVEL_WARNING,
                "nxio_read(): waiting to read, started=%d.%06d, timeout_remaining=%d.%06d",
                (int)start.tv_sec, (int)start.tv_usec,
                (int)timeout_remaining.tv_sec, (int)timeout_remaining.tv_usec
            );
#endif

            FD_ZERO(&read_fds);
            FD_SET(nxio->socket, &read_fds);

            r = select(nxio->socket + 1, &read_fds, NULL, NULL, &timeout_remaining);

            if (r < 0) {
                // error
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): error selecting on socket: %s", strerror(errno));
                bionet_nxio_close(nxio);
                errno = EPIPE;
                return -1;
            }

            if (r == 0) {
                // timeout
                return 0;
            }
        }


        //
        // if we get here, the nxio socket is readable
        //


        // 
        // make sure we've read the framing header before trying to parse it
        // unfortunately this means it takes at least two read(2) calls to read each packet...
        //

        if (nxio->in_index < NXIO_HEADER_SIZE) {
            // we dont have the framing header yet, so try to read it

            int r;

            r = read(nxio->socket, &nxio->in_buffer[nxio->in_index], NXIO_HEADER_SIZE - nxio->in_index);

            if (r == -1) {
                if (errno == EAGAIN) {
                    // tried to read on an empty non-blocking socket
                    // this should never happen if the caller behaves, but hey
                    return 0;
                }
                // read error
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): error reading from socket: %s", strerror(errno));
                bionet_nxio_close(nxio);
                errno = EPIPE;
                return -1;
            }

            if (r == 0) {
                // peer closed connection
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): socket closed on the far side");
                bionet_nxio_close(nxio);
                errno = EPIPE;
                return -1;
            }


            nxio->in_index += r;

#ifdef NXIO_DEBUG
            // FIXME
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, "read %d bytes, now have %d bytes", r, nxio->in_index);
            log_hexdump(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, nxio->in_buffer, nxio->in_index);
#endif

            if (nxio->in_index < NXIO_HEADER_SIZE) {
                // we read all there was, which was not enough, keep waiting
                goto keep_waiting;
            }
        }


        //
        // if we get here, we have the incoming packet header
        //


        expected_packet_size = ntohl(*(uint32_t*)nxio->in_buffer);
        if (expected_packet_size > NXIO_MAX_MESSAGE_SIZE) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "frame size %u exceeds max size %d!", expected_packet_size, NXIO_MAX_MESSAGE_SIZE);
            bionet_nxio_close(nxio);
            errno = EPIPE;
            return -1;
        }

        bytes_to_read = expected_packet_size - (nxio->in_index - NXIO_HEADER_SIZE);

        if (bytes_to_read > 0) {
            int r;

            r = read(nxio->socket, &nxio->in_buffer[nxio->in_index], bytes_to_read);

            if (r == -1) {
                if (errno == EAGAIN) {
                    // tried to read on an empty non-blocking socket
                    // this should never happen if the caller behaves, but hey
                    return 0;
                }
                // read error
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): error reading from socket: %s", strerror(errno));
                bionet_nxio_close(nxio);
                errno = EPIPE;
                return -1;
            }

            if (r == 0) {
                // peer closed connection
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_read(): socket closed on the far side");
                bionet_nxio_close(nxio);
                errno = EPIPE;
                return -1;
            }


            nxio->in_index += r;


#ifdef NXIO_DEBUG
            // FIXME
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, "read %d bytes, now have %d bytes", r, nxio->in_index);
            log_hexdump(NXIO_LOG_DOMAIN, G_LOG_LEVEL_INFO, nxio->in_buffer, nxio->in_index);
#endif

            if (r < bytes_to_read) {
                // we read all there was, which was not enough, keep waiting
                goto keep_waiting;
            }

            break;
        }


keep_waiting:
        r = gettimeofday(&now, NULL);
        if (r < 0) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error getting time of day: %s", strerror(errno));
            return -1;
        }

        waited_so_far.tv_sec = (now.tv_sec - start.tv_sec);
        waited_so_far.tv_usec = (now.tv_usec - start.tv_usec);
        if (waited_so_far.tv_usec < 0) {
            waited_so_far.tv_sec --;
            waited_so_far.tv_usec += 1000000;
        }

        timeout_remaining.tv_sec = timeout->tv_sec - waited_so_far.tv_sec;
        timeout_remaining.tv_usec = timeout->tv_usec - waited_so_far.tv_usec;
        if (timeout_remaining.tv_usec < 0) {
            timeout_remaining.tv_sec --;
            timeout_remaining.tv_usec += 1000000;
        }

#ifdef NXIO_DEBUG
        g_log(
            NXIO_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "nxio_read(): read some but didnt finish, started=%d.%06d, now=%d.%06d, waited_so_far=%d.%06d, timeout_remaining=%d.%06d",
            (int)start.tv_sec, (int)start.tv_usec,
            (int)now.tv_sec, (int)now.tv_usec,
            (int)waited_so_far.tv_sec, (int)waited_so_far.tv_usec,
            (int)timeout_remaining.tv_sec, (int)timeout_remaining.tv_usec
        );
#endif

        if (timeout_remaining.tv_sec < 0) {
            return 0;
        }
    } while (1);


    //
    // if we get here, we have the whole packet
    //


    // is it a packet we know how to deal with?
    // 0x00: XML message
    if (nxio->in_buffer[4] != 0x00) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "unknown message type 0x%02hhX, skipping", nxio->in_buffer[4]);
        nxio->in_index = 0;
        errno = EBADMSG;
        return -1;
    }


    // it's an XML document!
    // parse & DTD-validate
    {
        xmlDoc *xml_doc;

        xml_doc = xmlParseMemory(&nxio->in_buffer[NXIO_HEADER_SIZE], expected_packet_size);
        nxio->in_index = 0;

        if (xml_doc == NULL) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "got unparsable XML message, dropping");
            // FIXME log it
            // clogprint_hex(NXIO_LPC, G_LOG_LEVEL_WARNING, "    ", &nxio->in_buffer[2], nxio->in_index - 2);
            errno = EBADMSG;
            return -1;
        }

        if ((nxio->in_dtd != NULL) && (nxio->in_validation_context != NULL)) {
            int r;
            r = xmlValidateDtd(nxio->in_validation_context, xml_doc, nxio->in_dtd);
            if (r == 0) {
                g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "XML message fails DTD validation");
                // FIXME log it
                // clogprint_hex(NXIO_LPC, G_LOG_LEVEL_WARNING, "    ", &nxio->in_buffer[2], nxio->in_index - 2);
                xmlFreeDoc(xml_doc);
                errno = EBADMSG;
                return -1;
            }
        }

        *xml = xml_doc;
    }

    return 0;
}

