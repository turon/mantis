

#include <stdio.h>


#include "libbionet-internal.h"
#include "bionet.h"




GSList *libbionet_queued_messages_from_nag = NULL;

void (*libbionet_callback_new_hab)(bionet_hab_t *hab) = NULL;
void (*libbionet_callback_lost_hab)(bionet_hab_t *hab) = NULL;

void (*libbionet_callback_new_node)(bionet_node_t *node) = NULL;
void (*libbionet_callback_lost_node)(bionet_node_t *node) = NULL;

void (*libbionet_callback_resource_value)(bionet_resource_t *resource) = NULL;


char *libbionet_nag_hostname = NULL;
unsigned short libbionet_nag_port = 11000;

int libbionet_nag_timeout = 5;

bionet_nxio_t *libbionet_nag_nxio = NULL;

char *libbionet_client_id = NULL;

char *libbionet_nag_error = NULL;

GSList *bionet_habs = NULL;


