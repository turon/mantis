

#ifndef __LIBBIONET_INTERNAL_H
#define __LIBBIONET_INTERNAL_H




#include <glib.h>

#include <libxml/tree.h>

#include "bionet.h"
#include "bionet-util.h"
#include "bionet-nxio.h"




extern GSList *libbionet_queued_messages_from_nag;


extern char *libbionet_nag_hostname;
extern unsigned short libbionet_nag_port;

extern int libbionet_nag_timeout;

extern bionet_nxio_t *libbionet_nag_nxio;

extern char *libbionet_client_id;

extern char *libbionet_nag_error;

extern void (*libbionet_callback_new_hab)(bionet_hab_t *hab);
extern void (*libbionet_callback_lost_hab)(bionet_hab_t *hab);

extern void (*libbionet_callback_new_node)(bionet_node_t *node);
extern void (*libbionet_callback_lost_node)(bionet_node_t *node);

extern void (*libbionet_callback_resource_value)(bionet_resource_t *resource);




void libbionet_kill_nag_connection(void);

int libbionet_send_to_nag(const char *fmt, ...);

int libbionet_read_ok_from_nag(void);

// sets libbionet_nag_error, frees xml
void libbionet_handle_error_message(xmlDoc *xml);

// puts the NAG-Request message on the queue to be dealt with later
void libbionet_queue_nag_message(xmlDoc *xml);

bionet_hab_t *libbionet_parse_hab_from_xml(xmlNode *hab_node);
bionet_node_t *libbionet_parse_node_from_xml(xmlNode *node_node);
bionet_resource_t *libbionet_parse_resource(xmlNode *resource_node);

void libbionet_cache_add_hab(bionet_hab_t *hab);
void libbionet_cache_remove_hab(bionet_hab_t *hab);

void libbionet_cache_add_node(bionet_node_t *node);
void libbionet_cache_remove_node(bionet_node_t *node);

void libbionet_cache_add_resource(bionet_resource_t *resource);
void libbionet_cache_replace_resource(bionet_resource_t *resource);




#endif // __LIBBIONET_INTERNAL_H


