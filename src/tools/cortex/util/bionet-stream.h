


#ifndef __BIONET_STREAM_H
#define __BIONET_STREAM_H


#include <stdint.h>




typedef enum {
    BIONET_STREAM_DIRECTION_INVALID = 0,
    BIONET_STREAM_DIRECTION_PRODUCER,
    BIONET_STREAM_DIRECTION_CONSUMER
} bionet_stream_direction_t;




typedef struct {
    char *id;
    bionet_stream_direction_t direction;
    char *type;
    char *host;
    uint16_t port;

    int socket;

    void *private;
} bionet_stream_t;




//
//
//       NAME:  bionet_stream_new()
//
//
//   FUNCTION:  Creates a new Stream.
//
//
//  ARGUMENTS:  id - the ID of the new stream
//
//              direction - the direction of the new stream
//
//              type - the type of the stream (currently only "audio" is
//              supported)
//
//              host - the host that the stream is available on (optional,
//              defaults to the local host)
//
//              port - the port that the stream is available on
//
//
//    RETURNS:  A pointer to the new Stream if all went well, and NULL if
//              there was an error.
//
//

bionet_stream_t *bionet_stream_new(
    const char *id,
    bionet_stream_direction_t direction,
    const char *type,
    const char *host,
    uint16_t port
);

bionet_stream_t *bionet_stream_new_from_strings(
    const char *id,
    const char *direction_str,
    const char *type,
    const char *host,
    const char *port_str
);




//
//       NAME:  bionet_stream_listen()
//
//   FUNCTION:  Binds the Stream to an ephemeral (random unused) port and
//              starts listening.  Sets the Stream's "port" member to
//              whatever port it got.
//
//  ARGUMENTS:  The Stream to listen on.
//
//    RETURNS:  0 on success, -1 on failure.
//

int bionet_stream_listen(bionet_stream_t *stream);




//
//       NAME:  bionet_stream_accept()
//
//   FUNCTION:  Accepts a connection on a listening Stream.
//
//  ARGUMENTS:  The Stream to accept on.
//
//    RETURNS:  The new socket on success, -1 on failure.
//

int bionet_stream_accept(bionet_stream_t *stream);




//
//       NAME:  bionet_stream_connect()
//
//   FUNCTION:  Makes a connection to a Stream.
//
//  ARGUMENTS:  The Stream to connect to.
//
//    RETURNS:  The new socket on success, -1 on failure.
//

int bionet_stream_connect(bionet_stream_t *stream);




//
//       NAME:  bionet_stream_free()
//
//   FUNCTION:  Frees a Stream.
//
//  ARGUMENTS:  The Stream to free.
//
//    RETURNS:  Nothing.
//

void bionet_stream_free(bionet_stream_t *stream);




// 
//       NAME:  bionet_stream_direction_from_string()
//
//   FUNCTION:  Converts a text string describing a Stream direction into
//              the appropriate bionet_stream_direction_t.
//
//  ARGUMENTS:  The string to convert.
//
//    RETURNS:  The bionet_stream_direction_t.
//

bionet_stream_direction_t bionet_stream_direction_from_string(const char *direction_string);




// 
//       NAME:  bionet_stream_direction_to_string()
//
//   FUNCTION:  Converts a bionet_stream_direction_t into a descriptive
//              text string.
//
//  ARGUMENTS:  The bionet_stream_direction_t to convert.
//
//    RETURNS:  The string.
//

const char *bionet_stream_direction_to_string(bionet_stream_direction_t direction);




// 
//       NAME:  bionet_stream_port_from_string()
//
//   FUNCTION:  Converts a text string describing a Stream port into
//              the appropriate uint16_t.
//
//  ARGUMENTS:  The string to convert.
//
//    RETURNS:  The uint16_t (0 on error).
//

uint16_t bionet_stream_port_from_string(const char *port_string);




// 
//       NAME:  bionet_stream_port_to_string()
//
//   FUNCTION:  Converts a uint16_t representing a Stream port into a
//              text string.
//
//  ARGUMENTS:  The Stream port to convert.
//
//    RETURNS:  The string.
//

const char *bionet_stream_port_to_string(uint16_t port);




#endif // __BIONET_STREAM_H

