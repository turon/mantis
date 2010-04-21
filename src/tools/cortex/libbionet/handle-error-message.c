

#include <string.h>

#include <glib.h>

#include "bionet-util.h"
#include "libbionet-internal.h"


void libbionet_handle_error_message(xmlDoc *xml) {
    xmlNode *reply;
    xmlChar *error_msg;

    reply = xmlDocGetRootElement(xml);

    if (strcmp((char *)reply->name, "error") != 0) {
        // wtf?
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "libbionet_handle_error_message() called with non-error message");
        xmlFreeDoc(xml);
        return;
    }


    error_msg = xmlGetProp(reply, (xmlChar *)"message");

    if (error_msg == NULL) {
        // wtf?
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NAG reported error but no error message");
        xmlFreeDoc(xml);
        return;
    }

    g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "NAG request failed (%s)", error_msg);
    if (libbionet_nag_error != NULL) {
        free(libbionet_nag_error);
    }
    libbionet_nag_error = strdup((char *)error_msg);

    xmlFree(error_msg);
    xmlFreeDoc(xml);
}


