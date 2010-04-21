

#ifndef __BIONET_HAB_H
#define __BIONET_HAB_H




#include <stdint.h>
#include <sys/time.h>

#include <glib.h>

#include "bionet-util.h"




//
// This holds a HAB.
// 'private' is for the caller to do with as they please.
//

typedef struct {
    char *type;
    char *id;

    GSList *nodes;   // each is a bionet_node_t*

    void *private;
} bionet_hab_t;



//
//       NAME:  bionet_hab_new()
//
//   FUNCTION:  Allocates and initializes a new HAB, from
//              strings describing the HAB.
//
//  ARGUMENTS:  The HAB id and the HAB type.
//
//    RETURNS:  The HAB, or NULL on error.
//

bionet_hab_t* bionet_hab_new(char *hab_type, char *hab_id);




//
//       NAME:  bionet_hab_add_node()
//
//   FUNCTION:  Adds a Node to a HAB.
//
//  ARGUMENTS:  The HAB to add a Node to, and the Node to add.
//
//    RETURNS:  0 if all went well, -1 on error.
//

int bionet_hab_add_node(bionet_hab_t *hab, bionet_node_t *node);




//
//       NAME: bionet_hab_get_node_by_id()
//
//   FUNCTION: Finds a specific Node of a HAB, given its Node-ID.
//
//  ARGUMENTS: The HAB to search for the Node in, and the Node's ID.
//
//    RETURNS: The Node if one was found, NULL if it was not found.
//

bionet_node_t *bionet_hab_get_node_by_id(bionet_hab_t *hab, const char *node_id);




//
//       NAME: bionet_hab_remove_node_by_id()
//
//   FUNCTION: Removes a specific Node of a HAB, given its Node-ID.
//
//  ARGUMENTS: The HAB to remove the Node from, and the Node's ID.
//
//    RETURNS: 0 if the Node was removed, -1 on error (including Node not
//             found).
//
//       NOTE: The Node is not freed, it is the responsibility of the
//             caller to free the Node.
//

int bionet_hab_remove_node_by_id(bionet_hab_t *hab, const char *node_id);




//
//       NAME: bionet_hab_remove_all_nodes()
//
//   FUNCTION: Removes all Nodes from a HAB.
//
//  ARGUMENTS: The HAB to remove the Nodes from.
//
//    RETURNS: 0 on success, -1 on failure.
//
//       NOTE: The Nodes are freed as they are removed (using the
//             bionet_node_free() function), assuming they dont have their
//             'private' parts set.
//

int bionet_hab_remove_all_nodes(bionet_hab_t *hab);




//
//       NAME: bionet_hab_matches_type_and_id()
//
//   FUNCTION: Checks if a HAB matches a HAB-Type and HAB-ID pair.  The
//             wildcard "*" matches any string.
//
//  ARGUMENTS: The HAB to test, and the HAB-Type and HAB-ID to compare to.
//
//    RETURNS: TRUE (1) if the HAB matches, FALSE (0) if not.
//

int bionet_hab_matches_type_and_id(const bionet_hab_t *hab, const char *type, const char *id);




//
//       NAME: bionet_hab_free()
//
//   FUNCTION: Frees a HAB and all its Nodes.
//
//  ARGUMENTS: The HAB to free.
//
//    RETURNS: Nothing.
//

void bionet_hab_free(bionet_hab_t *hab);




#endif // __BIONET_HAB_H


