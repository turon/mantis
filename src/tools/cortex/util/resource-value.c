
#define _GNU_SOURCE  // for strtof(3)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>

#include "bionet-util.h"




const char *bionet_resource_value_to_string(const bionet_resource_t *resource) {
    return bionet_resource_value_to_string_isolated(resource->data_type, &resource->value);
}


const char *bionet_resource_value_to_string_isolated(bionet_resource_data_type_t data_type, const bionet_resource_value_t *value) {
    int r;
    static char val_str[256];


    switch (data_type) {

        case BIONET_RESOURCE_DATA_TYPE_BINARY: {
            r = snprintf(val_str, sizeof(val_str), "%d", (int)value->binary_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_UINT8: {
            r = snprintf(val_str, sizeof(val_str), "%hhu", value->uint8_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_INT8: {
            r = snprintf(val_str, sizeof(val_str), "%hhd", value->int8_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_UINT16: {
            r = snprintf(val_str, sizeof(val_str), "%hu", value->uint16_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_INT16: {
            r = snprintf(val_str, sizeof(val_str), "%hd", value->int16_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_UINT32: {
            r = snprintf(val_str, sizeof(val_str), "%u", value->uint32_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_INT32: {
            r = snprintf(val_str, sizeof(val_str), "%d", value->int32_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_FLOAT: {
            r = snprintf(val_str, sizeof(val_str), "%.7f", value->float_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        case BIONET_RESOURCE_DATA_TYPE_DOUBLE: {
            r = snprintf(val_str, sizeof(val_str), "%.15f", value->double_v);
            if (r >= sizeof(val_str)) {
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated(): value of resource is too big to fit in output string!");
                return NULL;
            }
            return val_str;
        }

        default: {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_to_string_isolated():  unknown data-type %d", data_type);
            return NULL;
        }
    }


    // not reached
    return NULL;
}




int bionet_resource_value_from_pointer(const void *value, bionet_resource_t *dest_resource) {
    switch (dest_resource->data_type) {
        case BIONET_RESOURCE_DATA_TYPE_BINARY:
            dest_resource->value.binary_v = *(int *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_UINT8:
            dest_resource->value.uint8_v = *(uint8_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_INT8:
            dest_resource->value.int8_v = *(int8_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_UINT16:
            dest_resource->value.uint16_v = *(uint16_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_INT16:
            dest_resource->value.int16_v = *(int16_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_UINT32:
            dest_resource->value.uint32_v = *(uint32_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_INT32:
            dest_resource->value.int32_v = *(int32_t *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_FLOAT:
            dest_resource->value.float_v = *(float *)value;
            return 0;

        case BIONET_RESOURCE_DATA_TYPE_DOUBLE:
            dest_resource->value.double_v = *(double *)value;
            return 0;

        default:
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_from_pointer(): invalid data type %d passed in", dest_resource->data_type);
            return -1;
    }

    // NOT REACHED
    return -1;
}


int bionet_resource_value_from_string(const char *value_string, bionet_resource_t *dest_resource) {
    return bionet_resource_value_from_string_isolated(value_string, dest_resource->data_type, &dest_resource->value);
}


int bionet_resource_value_from_string_isolated(const char *value_string, bionet_resource_data_type_t data_type, bionet_resource_value_t *value) {
    if (value_string == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_from_string_isolated(): NULL value string passed in!");
        return -1;
    }

    switch (data_type) {
        case BIONET_RESOURCE_DATA_TYPE_BINARY: {
            if (
                (strcasecmp(value_string, "on") == 0) ||
                (strcasecmp(value_string, "true") == 0) ||
                (strcasecmp(value_string, "yes") == 0) ||
                (strcasecmp(value_string, "1") == 0)
            ) {
                value->binary_v = 1;
                return 0;
            } else if (
                (strcasecmp(value_string, "off") == 0) ||
                (strcasecmp(value_string, "false") == 0) ||
                (strcasecmp(value_string, "no") == 0) ||
                (strcasecmp(value_string, "0") == 0)
            ) {
                value->binary_v = 0;
                return 0;
            } 
            
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "error parsing Binary value from '%s'", value_string);
            return -1;
        }

        case BIONET_RESOURCE_DATA_TYPE_FLOAT:
            {
                char *end_ptr;
                float val;

                val = strtof(value_string, &end_ptr);
                if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) {
                    value->float_v = val;
                    return 0;
                }
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse Float value from '%s'", value_string);
                return -1;
            }

        case BIONET_RESOURCE_DATA_TYPE_DOUBLE:
            {
                char *end_ptr;
                double val;

                val = strtod(value_string, &end_ptr);
                if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) {
                    value->double_v = val;
                    return 0;
                }
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse Double value from '%s'", value_string);
                return -1;
            }

        case BIONET_RESOURCE_DATA_TYPE_INT32:
            {
                char *end_ptr;
                long long val;

                val = strtoll(value_string, &end_ptr, 0);
                if ((val <= INT32_MAX) && (val >= INT32_MIN)) {
                    value->int32_v = (int32_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                }
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse Int32 value from '%s'", value_string);
                return -1;
            }


        case BIONET_RESOURCE_DATA_TYPE_UINT32:
            {
                char *end_ptr;
                unsigned long long val;

                val = strtoull(value_string, &end_ptr, 0);
                if (val <= UINT32_MAX) {
                    value->uint32_v = (uint32_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                }
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse UInt32 value from '%s'", value_string);
                return -1;
            }


        case BIONET_RESOURCE_DATA_TYPE_INT16:
            {
                char *end_ptr;
                long val;

                val = strtol(value_string, &end_ptr, 0);
                if ((val <= INT16_MAX) && (val >= INT16_MIN)) {
                    value->int16_v = (int16_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                } 
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse Int16 value from '%s'", value_string);
                return -1;
            }


        case BIONET_RESOURCE_DATA_TYPE_UINT16:
            {
                char *end_ptr;
                unsigned long val;

                val = strtoul(value_string, &end_ptr, 0);
                if (val <= UINT16_MAX) {
                    value->uint16_v = (uint16_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                } 
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse UInt16 value from '%s'", value_string);
                return -1;
            }


        case BIONET_RESOURCE_DATA_TYPE_INT8:
            {
                char *end_ptr;
                long val;

                val = strtol(value_string, &end_ptr, 0);
                if ((val <= INT8_MAX) && (val >= INT8_MIN)) {
                    value->int8_v = (int8_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                } 
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse Int8 value from '%s'", value_string);
                return -1;
            }


        case BIONET_RESOURCE_DATA_TYPE_UINT8:
            {
                char *end_ptr;
                unsigned long val;

                val = strtoul(value_string, &end_ptr, 0);
                if (val <= UINT8_MAX) {
                    value->uint8_v = (uint8_t)val;
                    if ((*value_string != (char)NULL) && (*end_ptr == (char)NULL)) return 0;
                } 
                g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "cannot parse UInt8 value from '%s'", value_string);
                return -1;
            }

        default: {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_string_to_value_isolated(): invalid data-type %d", data_type);
            return -1;
        }
    }


    // not reached
    return -1;
}




int bionet_resource_value_copy(const bionet_resource_value_t *source, bionet_resource_value_t *dest) {
    if (source == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_copy(): NULL source Resource Value passed in");
        return -1;
    }

    if (dest == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_copy(): NULL destination Resource Value passed in");
        return -1;
    }

    memcpy(dest, source, sizeof(bionet_resource_value_t));

    return 0;
}




bionet_resource_value_t *bionet_resource_value_dup(const bionet_resource_value_t *source_resource_value) {
    bionet_resource_value_t *new;

    if (source_resource_value == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_dup(): NULL source Resource Value passed in");
        return NULL;
    }

    new = (bionet_resource_value_t *)malloc(sizeof(bionet_resource_value_t));
    if (new == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_resource_value_dup(): out of memory");
        return NULL;
    }

    memcpy(new, source_resource_value, sizeof(bionet_resource_value_t));

    return new;
}


