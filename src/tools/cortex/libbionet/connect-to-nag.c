

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <pwd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <glib.h>

#include "nag-to-client-dtd.h"

#include "libbionet-internal.h"
#include "bionet.h"




int bionet_is_connected(void) {
    if (libbionet_nag_nxio == NULL) {
        return 0;
    }

    if (libbionet_nag_nxio->socket < 0) {
        return 0;
    }

    return 1;
}




// 
// Opens a TCP connection to the NAG server specified by
// libbionet_nag_hostname and libbionet_nag_port.
//
// Returns the socket fd if the connection is open, -1 if there's a problem.
//

int bionet_connect_to_nag(void) {
    struct sockaddr_in server_address;
    struct hostent *server_host;

    int r;

    char *hostname;


    if (libbionet_nag_hostname == NULL) {
        hostname = "localhost";
    } else {
        hostname = libbionet_nag_hostname;
    }


    if (libbionet_nag_nxio == NULL) {
        libbionet_nag_nxio = bionet_nxio_new(nag_to_client_dtd);
        if (libbionet_nag_nxio == NULL) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error setting up NAG communications");
            return -1;
        }
    }


    // if the connection is already open we just return it's fd
    if (libbionet_nag_nxio->socket > -1) return libbionet_nag_nxio->socket;


    //
    // If we get here we need to actually open the connection.
    //


    //
    // If the server dies or the connection is lost somehow, writes will
    // cause us to receive SIGPIPE, and the default SIGPIPE handler
    // terminates the process.  So we need to change the handler to ignore
    // the signal, unless the process has explicitly changed the action.
    //

    {
        int r;
        struct sigaction sa;

        r = sigaction(SIGPIPE, NULL, &sa);
        if (r < 0) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_connect_to_nag(): error getting old SIGPIPE sigaction: %s", strerror(errno));
            return -1;
        }

        if (sa.sa_handler == SIG_DFL) {
            sa.sa_handler = SIG_IGN;
            r = sigaction(SIGPIPE, &sa, NULL);
            if (r < 0) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_connect_to_nag(): error setting SIGPIPE sigaction to SIG_IGN: %s", strerror(errno));
                return -1;
            }
        }
    }


    // get the hostent for the server
    server_host = gethostbyname(hostname);

    if (server_host == NULL) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "bionet_connect_to_nag(): gethostbyname(\"%s\"): %s",
            hostname,
            hstrerror(h_errno)
        );
        return -1;
    }


    // create the socket
    if ((libbionet_nag_nxio->socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "bionet_connect_to_nag(): cannot create local socket: %s",
            strerror(errno)
        );
        return -1;
    }


    //  This makes it to the underlying networking code tries to send any
    //  buffered data, even after we've closed the socket.
    {
        struct linger linger;

        linger.l_onoff = 1;
        linger.l_linger = 60;
        if (setsockopt(libbionet_nag_nxio->socket, SOL_SOCKET, SO_LINGER, (char *)&linger, sizeof(linger)) != 0) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_connect_to_nag(): WARNING: cannot make socket linger: %s", strerror(errno));
        }
    }


    // prepare the server address
    memset((char *)&server_address, '\0', sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = inet_addr(inet_ntoa(*(struct in_addr *)(*server_host->h_addr_list)));
    server_address.sin_port = htons(libbionet_nag_port);


    // connect to the server
    if (connect(libbionet_nag_nxio->socket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        g_log(
            BIONET_LOG_DOMAIN,
            G_LOG_LEVEL_WARNING,
            "bionet_connect_to_nag(): failed to connect to server %s:%d: %s",
            server_host->h_name,
            libbionet_nag_port,
            strerror(errno)
        );

        libbionet_kill_nag_connection();

        return -1;
    }




    //
    // Send our ident string to the server, it's useful for keeping
    // statistics and for debugging.
    //
    // If the client app did not specifically set the ID by calling
    // bionet_set_id(), we use the default: "user@host:port (program [pid])"
    //

    if (libbionet_client_id != NULL) {
        r = bionet_nxio_send(libbionet_nag_nxio, "<set-id id='%s'/>", libbionet_client_id);

    } else {
        struct passwd *passwd;

        struct hostent *host;

        struct sockaddr_in local_socket;

        char program_name[100];


        // get the user name
        passwd = getpwuid(getuid());
        if (passwd == NULL) {
            g_log(
                BIONET_LOG_DOMAIN,
                G_LOG_LEVEL_WARNING,
                "bionet_connect_to_nag(): unable to determine user name for ident string: %s",
                strerror(errno)
            );
            libbionet_kill_nag_connection();
            return -1;
        }


        // get the hostent for the local host
        host = gethostbyname("localhost");
        if (host == NULL) {
            g_log(
                BIONET_LOG_DOMAIN,
                G_LOG_LEVEL_WARNING,
                "bionet_connect_to_nag(): unable to gethostbyname(\"localhost\") for ident string (%s), oh well",
                hstrerror(h_errno)
            );
        }


        // get the local socket port number
        {
            int r;
            socklen_t len;

            len = sizeof(struct sockaddr_in);
            r = getsockname(libbionet_nag_nxio->socket, (struct sockaddr *)&local_socket, &len);
            if (r < 0) {
                g_log(
                    BIONET_LOG_DOMAIN,
                    G_LOG_LEVEL_WARNING,
                    "bionet_connect_to_nag(): error getting local socket name: %s",
                    strerror(errno)
                );
                libbionet_kill_nag_connection();
                return -1;
            }
        }


        // get the program name
        // FIXME: this work on Linux only!  not portable!
        {
            int fd;

            char *tmp;


            fd = open("/proc/self/cmdline", O_RDONLY);
            if (fd < 0) {
                g_log(
                    BIONET_LOG_DOMAIN,
                    G_LOG_LEVEL_WARNING,
                    "bionet_connect_to_nag(): error opening /proc/self/cmdline: %s",
                    strerror(errno)
                );
                libbionet_kill_nag_connection();
                return -1;
            } else {
                int r;

                memset(program_name, (char)NULL, sizeof(program_name));

                r = read(fd, program_name, sizeof(program_name));
                if (r < 0) {
                    g_log(
                        BIONET_LOG_DOMAIN,
                        G_LOG_LEVEL_WARNING,
                        "bionet_connect_to_nag(): error reading /proc/self/cmdline: %s",
                        strerror(errno)
                    );
                    libbionet_kill_nag_connection();
                    return -1;
                }

                close(fd);

                while ((tmp = memchr(program_name, '/', strlen(program_name))) != NULL) {
                    int new_len = strlen(tmp+1);
                    memmove(program_name, tmp+1, new_len);
                    program_name[new_len] = (char)NULL;
                }
            }
        }


        r = bionet_nxio_send(
            libbionet_nag_nxio,
            "<set-id id='%s@%s:%d (%s [%d])'/>",
            passwd->pw_name,
            host->h_name,
            ntohs(local_socket.sin_port),
            program_name,
            getpid()
        );
    }

    // r is from the bionet_nxio_send()
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_connect_to_nag(): error sending ident string");
        libbionet_kill_nag_connection();
        return -1;
    }

    r = libbionet_read_ok_from_nag();
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_connect_to_nag(): error setting id: %s", bionet_get_nag_error());
        libbionet_kill_nag_connection();
        return -1;
    }


    return libbionet_nag_nxio->socket;
}


