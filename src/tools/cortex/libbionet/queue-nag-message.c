
#include "libbionet-internal.h"

void libbionet_queue_nag_message(xmlDoc *xml) {
    libbionet_queued_messages_from_nag = g_slist_append(libbionet_queued_messages_from_nag, xml);
}

