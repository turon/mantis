#ifndef _SS_TYPES_H_
#define _SS_TYPES_H_
#include "mos.h"
#include "mica2-gps.h"

#define MSG_HANDSHAKE_BEACON 0x00
#define MSG_HANDSHAKE_QUERY  0x01
#define MSG_HANDSHAKE_END    0x02
#define MSG_DATABASE_ENTRY   0x03
#define MSG_BASESTATION      0x04

typedef struct dbentry_s
{
   uint8_t src_id;
   latlong_t lat;
   latlong_t lon;
   timestamp_t rec_time;
   uint8_t hop_count;
   uint8_t hops[5];
} dbentry_t;


typedef struct handshake_s
{
   uint8_t msg_type;
   uint8_t src_id;
} handshake_t;

 
typedef struct entrypkt_s
{
   uint8_t msg_type; 
   dbentry_t entry;
} entrypkt_t;



#endif
