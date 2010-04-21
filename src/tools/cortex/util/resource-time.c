

#define _XOPEN_SOURCE // needed for strptime()

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <glib.h>

#include "bionet-util.h"


const char *bionet_resource_time_to_string_human_readable(const bionet_resource_t *resource) {
    static char time_str[200];

    char usec_str[10];
    struct tm *tm;
    int r;

    tm = gmtime((time_t *)&resource->time.tv_sec);
    if (tm == NULL) {
        return "invalid time";
    }

    r = strftime(time_str, sizeof(time_str), "%Y-%m-%d %T", tm);
    if (r <= 0) {
        return "invalid time";
    }

    sprintf(usec_str, ".%06d", (int)resource->time.tv_usec);
    strcat(time_str, usec_str);

    return time_str;
}


const char *bionet_resource_time_to_string(const bionet_resource_t *resource) {
    static char time_str[100];

    sprintf(time_str, "%d.%06d", (int)resource->time.tv_sec, (int)resource->time.tv_usec);
    return time_str;
}


int bionet_resource_time_from_timeval(const struct timeval *tv, bionet_resource_t *resource) {
    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_timeval(): NULL resource passed in");
        return -1;
    }

    if (tv == NULL) {
        int r;

        r = gettimeofday(&resource->time, NULL);
        if (r < 0) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_timeval(): error getting time: %s", strerror(errno));
            return -1;
        }
        return 0;
    }

    resource->time.tv_sec  = tv->tv_sec;
    resource->time.tv_usec = tv->tv_usec;

    return 0;
}


int bionet_resource_time_from_string(const char *time_str, bionet_resource_t *resource) {
    int r;
    struct tm tm;
    char *p;

    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_string(): NULL resource passed in");
        return -1;
    }

    if (time_str == NULL) {
        r = gettimeofday(&resource->time, NULL);
        if (r < 0) {
            g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_string(): error getting time: %s", strerror(errno));
            return -1;
        }
        return 0;
    }


    // first try full time
    p = strptime(time_str, "%Y-%m-%d %T", &tm);
    if (p != NULL) {
	resource->time.tv_sec = mktime(&tm);

	r = sscanf(p, ".%6d", (int *)&resource->time.tv_usec);
	if (r == 1) {
	    return 0;
	}
    }


    // then try <seconds>.<micro_seconds>
    r = sscanf(time_str, "%d.%6d", (int *)&resource->time.tv_sec, (int *)&resource->time.tv_usec);
    if (r == 2) {
        return 0;
    }


    // finally try just <seconds>
    r = sscanf(time_str, "%d", (int *)&resource->time.tv_sec);
    resource->time.tv_usec = 0;
    if (r == 1) {
        return 0;
    }


    g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_string(): error parsing time from '%s'", time_str);
    return -1;
}


int bionet_resource_time_now(bionet_resource_t *resource) {
    int r;

    if (resource == NULL) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_from_string(): NULL resource passed in");
        return -1;
    }

    r = gettimeofday(&resource->time, NULL);
    if (r < 0) {
        g_log(BIONET_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "bionet_set_resource_time_now(): cannot get time: %s", strerror(errno));
        return -1;
    }

    return 0;
}


