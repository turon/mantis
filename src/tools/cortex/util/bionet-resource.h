

#ifndef __BIONET_RESOURCE_H
#define __BIONET_RESOURCE_H




#include <stdint.h>
#include <sys/time.h>

#include <glib.h>

#include "bionet-util.h"




typedef enum {
    BIONET_RESOURCE_FLAVOR_INVALID = 0,
    BIONET_RESOURCE_FLAVOR_SENSOR,
    BIONET_RESOURCE_FLAVOR_ACTUATOR,
    BIONET_RESOURCE_FLAVOR_PARAMETER
} bionet_resource_flavor_t;





typedef enum {
    BIONET_RESOURCE_DATA_TYPE_INVALID = 0,
    BIONET_RESOURCE_DATA_TYPE_BINARY,
    BIONET_RESOURCE_DATA_TYPE_UINT8,
    BIONET_RESOURCE_DATA_TYPE_INT8,
    BIONET_RESOURCE_DATA_TYPE_UINT16,
    BIONET_RESOURCE_DATA_TYPE_INT16,
    BIONET_RESOURCE_DATA_TYPE_UINT32,
    BIONET_RESOURCE_DATA_TYPE_INT32,
    BIONET_RESOURCE_DATA_TYPE_FLOAT,
    BIONET_RESOURCE_DATA_TYPE_DOUBLE
} bionet_resource_data_type_t;




typedef union {
    int binary_v;

    uint8_t  uint8_v;
    int8_t   int8_v;

    uint16_t uint16_v;
    int16_t  int16_v;

    uint32_t uint32_v;
    int32_t  int32_v;

    float float_v;
    double double_v;
} bionet_resource_value_t;




// 
// This holds a resource.  'flavor', 'id', and 'data_type' are all used by
// the system.  'private' is for the caller to do with as it pleases.
//

typedef struct {
    char *id;
    bionet_resource_flavor_t flavor;
    bionet_resource_data_type_t data_type;
    bionet_resource_value_t value;
    struct timeval time;

    // these let you determine what HAB and Node owns this Resource
    char *hab_type;
    char *hab_id;
    char *node_id;

    void *private;
} bionet_resource_t;




//
//
//       NAME:  bionet_resource_new()
//
//   FUNCTION:  Allocates and initializes a new resource, from strings
//              describing the resource.
//
//  ARGUMENTS:  All passed-in strings are considered the property of the
//              caller.  The function duplicates waht it needs, the caller
//              is free to overwrite or free the strings on return from
//              this function.
//
//              hab_type: The HAB-Type of the HAB that owns the Node that
//              owns this Resource.  (optional)
//
//              hab_id: The HAB-ID of the HAB that owns the Node that owns
//              this Resource.  (optional)
//
//              node_id: The Node-ID of the Node that owns this Resource.
//              (optional)
//
//              data_type_str: The name of the data type of this Resource.
//
//              flavor_str: The flavor of this Resource.
//
//              resource_id: The ID of this Resource.
//
//              value_str: The value of this Resource (will be parsed
//              according to the Resource's data type).
//
//              time_str: The timestamp of the Resource's value.
//
//
//    RETURNS:  The Resource if all went well, NULL on error.
//
//

bionet_resource_t *bionet_resource_new_with_valuestr_timestr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const char *time_str
);

bionet_resource_t *bionet_resource_new_with_valuestr_timevalptr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const char *value_str,
    const struct timeval *tv
);

bionet_resource_t *bionet_resource_new_with_valueptr_timestr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const char *time_str
);

bionet_resource_t *bionet_resource_new_with_valueptr_timevalptr(
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *data_type_str,
    const char *flavor_str,
    const char *resource_id,
    const void *valueptr,
    const struct timeval *tv
);




//
//
//       NAME:  bionet_resource_free()
//
//   FUNCTION:  Frees a Resource created with bionet_resource_new().
//
//  ARGUMENTS:  The Resource to free.
//
//    RETURNS:  Nothing.
//
//

void bionet_resource_free(bionet_resource_t *resource);




//
//
//       NAME:  bionet_resource_value_copy()
//
//   FUNCTION:  Copies one Resource Value to another.
//
//  ARGUMENTS:  The source & destination Resource Values.
//
//    RETURNS:  0 on success, -1 on failure.
//
//

int bionet_resource_value_copy(const bionet_resource_value_t *source, bionet_resource_value_t *dest);




//
//
//       NAME:  bionet_resource_value_dup()
//
//   FUNCTION:  Allocates a new Resource Value and sets its value from the
//              passed-in Resource Value.
//
//  ARGUMENTS:  The Resource Value to duplicate.
//
//    RETURNS:  The dynamically allocated duplicate of the Resource Value,
//              or NULL on failure.
//
//

bionet_resource_value_t *bionet_resource_value_dup(const bionet_resource_value_t *original);




//
//       NAME:  bionet_resource_value_to_string()
//
//   FUNCTION:  Renders a Resource's Value as an ASCII string.
//
//  ARGUMENTS:  The Resource to get the Value from.
//
//    RETURNS:  A statically allocated string containing an ASCII
//              representation of the Resource Value, or NULL on error.
//

const char *bionet_resource_value_to_string(const bionet_resource_t *resource);




//
//       NAME:  bionet_resource_value_to_string_isolated()
//
//   FUNCTION:  Renders a Resource-Value as an ASCII string, without the
//              Resource-Value being wrapped up in a Resource.
//
//  ARGUMENTS:  The Resource-Value and Resource-Data-Type to render.
//
//    RETURNS:  A statically allocated string containing an ASCII
//              representation of the Resource-Value, or NULL on error.
//

const char *bionet_resource_value_to_string_isolated(bionet_resource_data_type_t data_type, const bionet_resource_value_t *value);




//
//       NAME:  bionet_resource_value_from_string()
//   
//   FUNCTION:  Sets a Resource's Value from an ASCII string
//              representation.
//
//  ARGUMENTS:  The string to set the Value from, and the Resource to set
//              the Value on.
//
//    RETURNS:  0 on success, -1 on error.
//

int bionet_resource_value_from_string(const char *value_string, bionet_resource_t *dest_resource);




//
//       NAME:  bionet_resource_value_from_string_isolated()
//   
//   FUNCTION:  Sets a Resource-Value from an ASCII string representation,
//              without the Value being wrapped in a full Resource.
//
//  ARGUMENTS:  The string to set the Value from, and the
//              Resource-Data-Type and Resource-Value to set.
//
//    RETURNS:  0 on success, -1 on error.
//

int bionet_resource_value_from_string_isolated(const char *value_string, bionet_resource_data_type_t data_type, bionet_resource_value_t *value);




//
//       NAME:  bionet_resource_value_from_pointer()
//   
//   FUNCTION:  Sets a Resource's Value from a void pointer pointing at the
//              appropriate value.
//
//  ARGUMENTS:  The void pointer to set the Value from, and the Resource to
//              set the Value on.
//
//    RETURNS:  0 on success, -1 on error.
//
int bionet_resource_value_from_pointer(const void *value, bionet_resource_t *dest_resource);




//
//       NAME:  bionet_resource_flavor_to_string()
//
//   FUNCTION:  Renders a Resource-Flavor as an ASCII string.  The output
//              will be one of "Sensor", "Actuator", or "Parameter".
//
//  ARGUMENTS:  The Resource Flavor to render.
//
//    RETURNS:  A pointer to the statically allocated string on success,
//              NULL on error.
//

const char *bionet_resource_flavor_to_string(bionet_resource_flavor_t flavor);




//
//       NAME:  bionet_resource_flavor_from_string()
//
//   FUNCTION:  Tries to parse an ASCII string as a Resource-Flavor, and
//              returns the Resource-Flavor.  Supported Resource-Flavors are
//              "sensor", "actuator", and "parameter".  Case insensitive.
//
//  ARGUMENTS:  The string to parse the Resource-Flavor from.
//
//    RETURNS:  The Resource-Flavor (which may be
//              BIONET_RESOURCE_FLAVOR_INVALID to indicate error).
//

bionet_resource_flavor_t bionet_resource_flavor_from_string(const char *flavor_string);




//
//       NAME:  bionet_resource_data_type_to_string()
//
//   FUNCTION:  Renders a Resource-Data-Type as an ASCII string.
//
//  ARGUMENTS:  The Resource-Data-Type to render.
//
//    RETURNS:  A pointer to the statically allocated string on success,
//              NULL on error.
//

const char *bionet_resource_data_type_to_string(bionet_resource_data_type_t data_type);




//
//       NAME:  bionet_resource_data_type_from_string()
//
//   FUNCTION:  Tries to parse an ASCII string as a Resource-Data-Type, and
//              returns the Resource-Data-Type.  Case insensitive.
//
//  ARGUMENTS:  The string to parse the Resource-Flavor from.
//
//    RETURNS:  The Resource-Flavor (which may be
//              BIONET_RESOURCE_FLAVOR_INVALID to indicate error).
//

bionet_resource_data_type_t bionet_resource_data_type_from_string(const char *data_type_string);




//
//       NAME:  bionet_resource_time_to_string()
//              bionet_resource_time_to_string_human_readable()
//
//   FUNCTION:  Renders the Resource Timestamp as an ASCII string.  The
//              first function outputs a string of the form
//              "<seconds>.<microseconds>", where <seconds> is the number
//              of seconds since 00:00:00 UTC, January 1, 1970.  The
//              _human_readable() function outputs a string of the form
//              "YYYY-MM-DD HH:MM:SS.microseconds", intended for human
//              consumption.
//
//  ARGUMENTS:  The Resource to get the Timestamp of.
//
//    RETURNS:  A pointer to the statically allocated string on success,
//              NULL on failure.
//

const char *bionet_resource_time_to_string(const bionet_resource_t *resource);
const char *bionet_resource_time_to_string_human_readable(const bionet_resource_t *resource);




//
//       NAME:  bionet_resource_time_from_string()
//              bionet_resource_time_from_timeval()
//              bionet_resource_time_now()
//
//  ARGUMENTS:  The Resource to set the Timestamp of, and optionally the
//              timestamp.  The new timestamp can be submitted as an ASCII
//              string (formatted like the output of the
//              bionet_resource_time_to_string*() functions) or as a struct
//              timeval (as returned, for example, from gettimeofday()), or
//              it can be omitted to indicate "now").  If the passed-in
//              string or timeval is NULL, "now" will be used.
//
//    RETURNS:  0 on succes; -1 on failure.
//

int bionet_resource_time_from_string(const char *time_str, bionet_resource_t *dest_resource);
int bionet_resource_time_from_timeval(const struct timeval *tv, bionet_resource_t *dest_resource);
int bionet_resource_time_now(bionet_resource_t *resource);




//
//       NAME:  bionet_resource_matches_id()
//              bionet_resource_matches_habtype_habid_nodeid_resourceid()
//
//   FUNCTION:  Checks if a Resource matches a name specification.  The
//              Resource name can be specified as just a Resource-ID (in
//              which case the Resource's HAB-Type, HAB-ID, and Node-ID are
//              considered matching no matter what), or as a HAB-Type,
//              HAB-ID, Node-ID, and Resource-ID (in which case all must
//              match).  The wildcard "*" matches all strings.
//
//  ARGUMENTS:  The Resource to test for match, and the relevant Resource
//              name components.
//
//    RETURNS:  TRUE (1) if it matches, FALSE (0) if not.
//

int bionet_resource_matches_id(const bionet_resource_t *resource, const char *id);
int bionet_resource_matches_habtype_habid_nodeid_resourceid(
    const bionet_resource_t *resource,
    const char *hab_type,
    const char *hab_id,
    const char *node_id,
    const char *resource_id
);




#endif // __BIONET_RESOURCE_H


