

#ifndef __BIONET_NODE_H
#define __BIONET_NODE_H




#include <stdint.h>
#include <sys/time.h>

#include <glib.h>

#include "bionet-util.h"




// 
// This holds a Node.
// 'private' is for the caller to do with as it pleases.
//

typedef struct {
    char *id;

    // these let you determine what HAB owns this Node
    char *hab_type;
    char *hab_id;

    GSList *resources;   // each is a bionet_resource_t*
    GSList *streams;     // each is a bionet_stream_t*
    GSList *commands;    // each is a bionet_command_t*

    void *private;
} bionet_node_t;



//
//       NAME:  bionet_node_new()
//
//   FUNCTION:  Makes a new node from the strings passed to it.
//
//  ARGUMENTS:  The HAB-Type and HAB-ID of the HAB that owns the new node,
//              the Node-ID of the new Node.  The HAB-Type and HAB-ID are
//              optional.
//
//    RETURNS:  The new Node if all went well, and NULL if there was an error.
//

bionet_node_t* bionet_node_new(const char* hab_type, const char* hab_id, const char* node_id);




//
//       NAME:  bionet_node_add_resource()
//
//   FUNCTION:  Adds a Resource to a Node.
//
//  ARGUMENTS:  The Node to add the Resource to, and string forms of all
//              the Resource fields.  The passed-in Resource fields are
//              copied as appropriate, and the originals belong to the
//              caller.
//
//    RETURNS:  0 if all went well, and -1 if there was an error.
//

int bionet_node_add_resource_with_valuestr_timestr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const char *time_str
);

int bionet_node_add_resource_with_valuestr_timevalptr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const struct timeval *tv
);

int bionet_node_add_resource_with_valueptr_timestr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *value,
    const char *time_str
);

int bionet_node_add_resource_with_valueptr_timevalptr(
    bionet_node_t *node,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *value,
    const struct timeval *tv
);




//
//       NAME:  bionet_node_add_command()
//
//   FUNCTION:  Adds a Command to a Node.
//
//  ARGUMENTS:  The Node to add the Command to, and Command to add.
//
//    RETURNS:  0 if all went well, and -1 if there was an error.
//

int bionet_node_add_command(bionet_node_t *node, bionet_command_t *cmd);




//
//       NAME:  bionet_node_add_stream()
//
//   FUNCTION:  Adds a Stream to a Node.
//
//  ARGUMENTS:  The Node to add the Stream to, and the Stream fields of the
//              Stream to add.  The passed-in Stream fields are copied as
//              appropriate, and the originals belong to the caller.  See
//              the documentation for bionet_stream_new() for details on the fields.
//
//    RETURNS:  0 if all went well, and -1 if there was an error.
//

int bionet_node_add_stream(bionet_node_t *node, bionet_stream_t *stream);

int bionet_node_add_stream_native(
    bionet_node_t *node,
    const char *id,
    bionet_stream_direction_t direction,
    const char *type,
    const char *host,
    uint16_t port
);

int bionet_node_add_stream_from_strings(
    bionet_node_t *node,
    const char *id,
    const char *direction_str,
    const char *type,
    const char *host,
    const char *port_str
);




//
//       NAME:  bionet_node_free()
//
//   FUNCTION:  Frees a Node and all its Resources, Streams, and Commands.
//              NOTE: Nodes have an (optional) 'private' member, and it is
//              the caller's responsibility to manage this pointer.  This
//              function will refuse to touch the Node if the private
//              pointer is not NULL.
//
//  ARGUMENTS:  The Node to free.
//
//    RETURNS:  Nothing.
//

void bionet_node_free(bionet_node_t *node);




//
//       NAME:  bionet_node_set_resource_value()
//
//   FUNCTION:  Updates the Value and Timestamp of a Resource on a Node.
//
//  ARGUMENTS:  The Nore to set the Resource on, the Resource's ID, the new
//              value, and the new timestamp.
//
//    RETURNS:  0 on success, -1 on failure.
//

int bionet_node_set_resource_value(bionet_node_t *node, const char *resource_id, const void *value, const struct timeval *timestamp);
int bionet_node_set_resource_value_with_valuestr(bionet_node_t *node, const char *resource_id, const char *value, const struct timeval *timestamp);




//
//       NAME:  bionet_node_get_resource_by_id()
//
//   FUNCTION:  Looks up a Resource of a Node given its ID.
//
//  ARGUMENTS:  The Node to get the Resource from, and the ID of the
//              Resource to get.
//
//    RETURNS:  A pointer to the specified Resource, or NULL if it was not
//              found.
//

bionet_resource_t *bionet_node_get_resource_by_id(bionet_node_t *node, const char *resource_id);




//
//       NAME:  bionet_node_get_command_by_id()
//
//   FUNCTION:  Looks up a Command of a Node given its ID.
//
//  ARGUMENTS:  The Node to get the Command from, and the ID of the
//              Command to get.
//
//    RETURNS:  A pointer to the specified Command, or NULL if it was not
//              found.
//

bionet_command_t *bionet_node_get_command_by_id(bionet_node_t *node, const char *command_id);




//
//       NAME:  bionet_node_get_stream_by_id()
//
//   FUNCTION:  Looks up a Stream of a Node given its ID.
//
//  ARGUMENTS:  The Node to get the Stream from, and the ID of the Stream
//              to get.
//
//    RETURNS:  A pointer to the specified Stream, or NULL if it was not
//              found.
//

bionet_stream_t *bionet_node_get_stream_by_id(bionet_node_t *node, const char *stream_id);




//
//       NAME:  bionet_node_matches_id()
//              bionet_node_matches_habtype_habid_nodeid()
//
//   FUNCTION:  Checks if a Node matches a name specification.  The Node
//              name can be specified as just a Node-ID (in which case the
//              Node's HAB-Type and HAB-ID are considered matching no
//              matter what) or as a HAB-Type, HAB-ID, and Node-ID (in
//              which case all must match).  The wildcard "*" matches all
//              strings.
//
//  ARGUMENTS:  The Node to test for match, optionally the HAB-Type and
//              HAB-ID, and the Node-ID.
//

int bionet_node_matches_id(const bionet_node_t *node, const char *id);
int bionet_node_matches_habtype_habid_nodeid(const bionet_node_t *node, const char *hab_type, const char *hab_id, const char *node_id);




#endif // __BIONET_NODE_H


