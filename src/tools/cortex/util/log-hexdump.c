
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <glib.h>


void log_hexdump(const char *log_domain, GLogLevelFlags log_level, const void *data, int count) {
    char hex[60];
    char ascii[20];
    int byte, hex_dst, ascii_dst;


    // FIXME
    // if (! lp_should_logprint(lp, msg_level)) return;


    hex_dst = 0;
    ascii_dst = 0;

    for (byte = 0; byte < count; byte ++) {
        sprintf(&hex[hex_dst], "%02X ", ((unsigned char *)data)[byte]);
        hex_dst += 3;

        if (isprint(((unsigned char *)data)[byte])) {
            sprintf(&ascii[ascii_dst], "%c", ((unsigned char *)data)[byte]);
        } else {
            sprintf(&ascii[ascii_dst], ".");
        }
        ascii_dst ++;

        if (byte % 16 == 15) {
            g_log(log_domain, log_level, "    %s |%s|", hex, ascii);
            hex_dst = 0;
            ascii_dst = 0;
        } else if (byte % 8 == 7) {
            sprintf(&hex[hex_dst], " ");
            hex_dst ++;
            sprintf(&ascii[ascii_dst], " ");
            ascii_dst ++;
        }
    }

    if (byte % 16 != 0) {
        g_log(log_domain, log_level, "    %-49s |%-17s|", hex, ascii);
    }
}


