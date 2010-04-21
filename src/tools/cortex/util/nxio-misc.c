

#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include <glib.h>

#include "bionet-nxio.h"




void bionet_nxio_reset(bionet_nxio_t *nxio) {
    nxio->in_index = 0;
    nxio->out_index = 0;
}




static void bionet_nxio_xml_error(void *unused, const char *fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "xml error!");
    g_logv(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, fmt, ap);
    va_end(ap);
}


static void bionet_nxio_xml_warning(void *unused, const char *fmt, ...) {
    va_list ap;

    va_start(ap, fmt);
    g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "xml warning!");
    g_logv(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, fmt, ap);
    va_end(ap);
}


int bionet_nxio_init(bionet_nxio_t *nxio, const char *dtd_doc_str) {
    nxio->socket = -1;

    if (dtd_doc_str == NULL) {
        nxio->in_dtd = NULL;
        nxio->in_validation_context = NULL;
    } else {
        xmlParserInputBufferPtr buf;

        buf = xmlParserInputBufferCreateMem(dtd_doc_str, strlen(dtd_doc_str), XML_CHAR_ENCODING_NONE);
        if (buf == NULL) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error making ParserInputBuffer from DTD string");
            return -1;
        }

        nxio->in_dtd = xmlIOParseDTD(NULL, buf, XML_CHAR_ENCODING_NONE);
        // xmlFreeParserInputBuffer(buf);
        if (nxio->in_dtd == NULL) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing DTD");
            return -1;
        }

        nxio->in_validation_context = xmlNewValidCtxt();
        if (nxio->in_validation_context == NULL) {
            g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error creating XML validation context");
            free(nxio->in_dtd);
            return -1;
        }
        nxio->in_validation_context->userData = NULL;
        nxio->in_validation_context->error = bionet_nxio_xml_error;
        nxio->in_validation_context->warning = bionet_nxio_xml_warning;
    }

    nxio->in_index  = 0;

    nxio->out_index = 0;

    return 0;
}




bionet_nxio_t *bionet_nxio_new(const char *dtd_doc_str) {
    bionet_nxio_t *nxio;
    int r;

    nxio = (bionet_nxio_t *)malloc(sizeof(bionet_nxio_t));
    if (nxio == NULL) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory");
        return NULL;
    }

    r = bionet_nxio_init(nxio, dtd_doc_str);
    if (r < 0) {
        free(nxio);
        return NULL;
    }

    return nxio;
}




void bionet_nxio_free(bionet_nxio_t *nxio) {
    if (nxio == NULL) {
        g_log(NXIO_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_nxio_free(): NULL nxio passed in");
        return;
    }

    if (nxio->in_dtd != NULL) {
        xmlFreeDtd(nxio->in_dtd);
    }

    if (nxio->in_validation_context != NULL) {
        xmlFreeValidCtxt(nxio->in_validation_context);
    }

    free(nxio);
}


void bionet_nxio_close(bionet_nxio_t *nxio) {
    if (nxio == NULL) {
        return;
    }

    nxio->in_index = 0;
    nxio->out_index = 0;

    if (nxio->socket >= 0) {
        close(nxio->socket);
    }

    nxio->socket = -1;
}


