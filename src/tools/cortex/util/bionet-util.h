

#ifndef __BIONET_UTIL_H
#define __BIONET_UTIL_H




#include <stdint.h>
#include <sys/time.h>

#include <glib.h>

#include "bionet-resource.h"
#include "bionet-stream.h"
#include "bionet-command.h"
#include "bionet-node.h"
#include "bionet-hab.h"




//
// The bionet code uses glib's logging facility to log internal messages,
// and this is the domain.
//

#define  BIONET_LOG_DOMAIN  "bionet"




// 
// General utility functions
//


// Returns true (1) if str matches [-a-zA-Z0-9]+, false (0) if not.
int bionet_is_valid_name_component(const char *str);


//
// puts the glib log messages where you want them
//

typedef struct {
    enum {
        BIONET_LOG_TO_STDOUT = 0,
        BIONET_LOG_TO_SYSLOG = 1
    } destination;

    // messages with log_level *below* log_limit are logged, all others are dropped
    // FIXME: really we want a default log_limit and then optional per-domain limits
    GLogLevelFlags log_limit;
} bionet_log_context_t;

void  bionet_glib_log_handler(const gchar *log_domain, GLogLevelFlags log_level, const gchar *message, gpointer log_context);

void log_hexdump(const char *log_domain, GLogLevelFlags log_level, const void *data, int count);




#endif // __BIONET_UTIL_H


