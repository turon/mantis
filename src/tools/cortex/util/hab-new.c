#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"



bionet_hab_t* bionet_hab_new(
	char* hab_type,
	char* hab_id
) {
    bionet_hab_t* hab;

    if (hab_type == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "hab-new(): NULL hab type passed in");
	return NULL;
    }

    if (hab_id == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "hab-new(): NULL hab id passed in");
	return NULL;
    }


    hab = calloc(1, sizeof(bionet_hab_t));
    if (hab == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
	return NULL;
    }

    hab->id = strdup(hab_id);
    if (hab->id == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
	return NULL;
    }
    hab->type = strdup(hab_type);
    if (hab->type == NULL)
    {
	g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_ERROR, "out of memory!");
	return NULL;
    }

    hab->nodes = NULL;

    return hab;
}
