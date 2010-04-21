

#ifndef __BIONET_NXIO_H
#define __BIONET_NXIO_H




#include <stdint.h>
#include <sys/time.h>

#include <libxml/tree.h>




// nxio's glib log domain
#define NXIO_LOG_DOMAIN "nxio"

#undef NXIO_DEBUG
// #define NXIO_DEBUG


// 
// The NXIO header looks like this:
//
//     uint32_t Size_of_payload;  // network byte order (big-endian)
//     uint8_t  Packet_type;      // 0x00 for XML, the rest are undefined
//

#define NXIO_MAX_MESSAGE_SIZE (256 * 1024)

#define NXIO_HEADER_SIZE (5)




//
// a Network XML I/O framing system
//

typedef struct {
    int socket;

    char in_buffer[NXIO_MAX_MESSAGE_SIZE];
    int in_index;

    // used by the nxio functions to handle DTD validation of XML messages
    xmlDtd *in_dtd;
    xmlValidCtxt *in_validation_context;


    char out_buffer[NXIO_MAX_MESSAGE_SIZE];
    int out_index;
} bionet_nxio_t;




//
//       NAME:  bionet_nxio_read()
//
//   FUNCTION:  Reads up to one XML document, DTD validates it (if the nxio
//              has a DTD), and returns it.
//
//  ARGUMENTS:  The nxio object to read on, the timeout (as a struct
//              timeval), and a place to put the resulting XML document,
//              if any.
//
//    RETURNS:  0 on success, in which case *xml may point to a document or
//              to NULL (if the message was not fully received yet).
//              
//              -1 on error, in which case errno will be set:
//
//                  EPIPE: I/O error, socket is now closed and set to -1,
//                      and should not be used by the caller any more.
//
//                  EINVAL: Invalid argument passed in.
//
//                  EBADMSG: Invalid message received.  Message dropped,
//                     connection still open.
//

int bionet_nxio_read(bionet_nxio_t *nxio, struct timeval *timeout, xmlDoc **xml);




//
//       NAME:  bionet_nxio_read_acceptable()
//
//   FUNCTION:  Reads messages from an nxio object until one of a
//              specified set is received (or the timeout expires).  Any
//              messages not in the set are put on a queue for later
//              processing by the caller.
//
//  ARGUMENTS:  'nxio' is the object to read on and 'timeout' is the read
//              timeout (as a struct timeval).
//
//              The 'acceptable' argument is an array of "const char **",
//              the last one of which is NULL.  The function keeps reading
//              messages until the timeout expires or it receives one of
//              the acceptable messages.  If any messages are received
//              other than the acceptable ones, they are appended to the
//              'queue'.
//
//    RETURNS:  0 on success.
//
//              -1 on error, in which case errno will be set as per the
//              bionet_nxio_read() function.
// 

int bionet_nxio_read_acceptable(bionet_nxio_t *nxio, struct timeval *timeout, xmlDoc **xml, const char **acceptable, GSList **queue);




//
//       NAME:  bionet_nxio_append()
//
//   FUNCTION:  Appends the specified string to the nxio object's outgoing
//              buffer.
//
//  ARGUMENTS:  The nxio object and the string to append (with the same
//              kind of "fmt, ..." arguments that printf takes).
//
//    RETURNS:  0 on success.
//
//              -1 on failure, in which case errno will be set to ENOSPC,
//              indicating "message too large, dropped"
//

int bionet_nxio_append(bionet_nxio_t *nxio, const char *fmt, ...);




//
//       NAME:  bionet_nxio_send()
//
//   FUNCTION:  Appends the string to the nxio object's outgoing buffer
//              (like bionet_nxio_append() does), then sends the buffer.
//
//  ARGUMENTS:  The nxio object to send on, and the string to send (with
//              the same kind of "fmt, ..." arguments that printf takes).
//
//    RETURNS:  0 on success.
//
//              -1 on failure, in which case errno will be set:
//                  
//                  EPIPE: the far side closed the socket
//
//                  ENOSPC: message too large, dropped
//

int bionet_nxio_send(bionet_nxio_t *nxio, const char *fmt, ...);




//
//       NAME:  bionet_nxio_flush()
//
//   FUNCTION:  Send the data in the nxio object's output buffer.
//
//  ARGUMENTS:  The nxio object to flush the output buffer on.
//
//    RETURNS:  0 on success.
//
//              -1 on failure, in which case errno will be set:
//
//                  EPIPE: error writing, socket closed
//

int bionet_nxio_flush(bionet_nxio_t *nxio);




// nxio misc
bionet_nxio_t *bionet_nxio_new(const char *dtd_document);
void bionet_nxio_free(bionet_nxio_t *nxio);

int bionet_nxio_init(bionet_nxio_t *nxio, const char *dtd_document);
void bionet_nxio_reset(bionet_nxio_t *nxio);

void bionet_nxio_close(bionet_nxio_t *nxio);




#endif // __BIONET_NXIO_H


