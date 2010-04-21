//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

/** @file dvdrp.h
 * @brief content-based routing layer
 * @author Cyrus Hall  :  hallcp@colorado.edu
 * @date 07/20/2004
 *
 * An implimentation of DV/DRP, a content-based routing and forwarding 
 * protocol for sensor networks.  For more information on DV/DRP, see  
 * the comments in this file or refer to the paper published at        
 * SECON04.
 */

#ifndef _DVDRP_H_
#define _DVDRP_H_

#include <stdarg.h>

#include "mos.h"

#include "net.h"
#include "com.h"

static const uint8_t DVDRP_PROTO_ID = 12;

////////////////////////////////////////////////////////////////////////////
// Base types needed for packets, route, and forwarding tables.
////
typedef uint32_t bv_set_t;
typedef uint16_t node_id_t;  //!! Note: there is a type conflict
			     //between node_id_t and the size of the
			     //bv_id in the packets below (a uint8_t),
			     //Nodes with ID's higher than 32 will
			     //fail strangely if they try and set a
			     //predicate currently.  This needs to be
			     //fixed with the bv_id arbitration
			     //scheme.

typedef struct dvdrp_attribute_t
{
    uint16_t name;
    uint16_t value;
} dvdrp_attrib;  //4 bytes

typedef uint8_t dvdrp_comp_t;
#define DVDRP_COMPARE_LT 1
#define DVDRP_COMPARE_GT 2
#define DVDRP_COMPARE_EQ 3
#define DVDRP_COMPARE_NE 4

typedef struct dvdrp_constraint_t
{
    uint16_t     name;
    uint16_t     value;
    dvdrp_comp_t compare_type;
} dvdrp_constraint;   //5 bytes

typedef struct dvdrp_filter_t {
    uint8_t           num_constraints;
    dvdrp_constraint  constraints[0];
} dvdrp_filter;

//max numbers for packets
typedef struct dvdrp_predicate_t
{
    uint8_t      num_filters;
    dvdrp_filter filters[0];
} dvdrp_predicate;

//////////////////////////////////////////////////////////////////
// DVDRP packet types.  Every packet contains a dvdrp_pkt up top.
////
typedef struct dvdrp_pkt_t
{
    uint8_t       pkt_type;
    uint8_t       htl;
} dvdrp_pkt;      //2 bytes

static const uint8_t DVDRP_MAX_MESG_ATTRIBS = 10;
typedef struct dvdrp_mesg_pkt_t
{
    dvdrp_pkt     header;        //2 byte
    bv_set_t      split_bv;      //4 bytes (6)
    node_id_t     immed_dest;    //2 bytes (8)
    uint8_t       is_secondary;  //1 byte  (9)
    uint8_t       num_attribs;   //1 byte  (10)
    dvdrp_attrib  attributes[0]; //varaible (64 - each attribute is 4
				 //bytes, so a max of 13 or so - lets
				 //say 10)
} __attribute__((packed)) dvdrp_mesg_pkt;

static const uint8_t DVDRP_MAX_ADVERT_CONSTRAINTS = 8;
static const uint8_t DVDRP_MAX_ADVERT_FILTERS = 4;
typedef struct dvdrp_advertisement_pkt_t
{
    dvdrp_pkt        header;      //2 byte
    node_id_t        receiver_id; //2 bytes (4)
    node_id_t        prev_hop;    //2 bytes (6)
    uint8_t          seq_num;     //1 byte  (7)
    uint8_t          bv_pos;      //1 byte  (8)
    uint32_t         min_delay;   //4 bytes (12)
    dvdrp_predicate  pred;        //variable (64 - each constraint is
                                  //5 bytes so a max of 8 or so
} __attribute__((packed)) dvdrp_advert_pkt;

typedef struct dvdrp_req_repair_pkt_t
{
    dvdrp_pkt header;      //2 bytes
    node_id_t receiver_id; //2 bytes (4)
} dvdrp_req_repair_pkt;

static const uint8_t DVDRP_MESG_PKT   = 0x1;
static const uint8_t DVDRP_ADVERT_PKT = 0x2;
static const uint8_t DVDRP_REPAIR_PKT = 0x3;

//////////////////////////////////////////
// Other DVDRP constants
////
#define DVDRP_RT_MAX_PREDICATES 5
#define DVDRP_RT_MAX_PATHS 2

//#define DVDRP_MAX_RECEIVERS 32
#define DVDRP_MAX_HTL 20
#define DVDRP_INVALID_NODE_ID 0
#define DVDRP_INVALID_BV_ID 32

//ioctl types
enum dvdrp_ioctl_t {
    DVDRP_IOCTL_SETPRED = 1,
    DVDRP_IOCTL_SETFAILURE_LIMIT,   //not impl
    DVDRP_IOCTL_RESET_PROTO         //not impl
};

////////////////////////////////////////
// Memory pools for route tables
////
typedef struct mem_pool_t {
    uint8_t  num_elems;
    uint8_t  type_size;
    uint32_t free_vec;
    void    *pool;
} mem_pool;

typedef struct dvdrp_constraint_ptr_t {
    uint16_t                       name;
    uint16_t                       value;
    dvdrp_comp_t                   compare_type;
    struct dvdrp_constraint_ptr_t *next;
} dvdrp_constraint_list;

typedef struct dvdrp_filter_ptr_t {
    dvdrp_constraint_list     *constraints;
    struct dvdrp_filter_ptr_t *next;
} dvdrp_filter_list;

#define DVDRP_MEM_CONSTRAINTS 20
#define DVDRP_MEM_FILTERS 10
#define DVDRP_MEM_COMBUFS 4
extern mem_pool constraint_pool;
extern mem_pool filter_pool;
extern mem_pool combuf_pool;

//suggested stack size - keep in mind, we've gotten by with a lot
//less, but if your application is using a decent amount of stack
//space (and you're getting strange behavior) think about using some
//more.  See the DV/DRP docs for more information on stack space
//issues and debugging, particularly mos_get_stack_size(...)
#define DVDRP_SUGGESTED_STACK_SIZE 256

void dvdrp_init_mem_pools();
void *pool_malloc(mem_pool *pool);
void pool_free(mem_pool *pool, void *mem);

////////////////////////////////////////////////
// Public function prototypes
////
/* Setup the flooding protocol and register with the net layer. */
/** @brief Setup the DV/DRP protocol and register with the net layer. */
void dvdrp_init();

/* Send a packet using the flooding protocol. */
/** @brief Send a packet using the DV/DRP protocol. 
 * @param pkt Packet to send
 * @param args 
 */
int8_t dvdrp_send(comBuf *pkt, va_list args);

/* Receive a packet that came using the dvdrp protocol. */
// NOTE: Must subtract the sizeof route header from pkt->size so
//       the net layer can determine whether any more data is available.
/** @brief Receive a packet that came using the dvdrping protocol. 
 *
 * NOTE: Must subtract the sizeof route header from pkt->size so
 *       the net layer can determine whether any more data is available.
 * @param pkt Packet received
 * @param footer Location
 * @param port The port number the packet belongs too.
 * @return FALSE if failure, else return TRUE
 */
boolean dvdrp_recv(comBuf *pkt, uint8_t **footer, uint8_t port);

/* Set some protocol specific parameters. */
/** @brief Set some protocol specific parameters. 
 * @param request IO request
 * @param args Arguements
 */
int8_t dvdrp_ioctl(uint8_t request, va_list args);

/////////////////////////////////
// Some basic bit operations.
////

#define mask_set(X, Y) (X |= ((uint32_t)0x0001 << (uint32_t)Y))
#define mask_unset(X, Y) (X &= ~((uint32_t)0x0001 << (uint32_t)Y))
#define mask_query(X, Y) (1 == (((uint32_t)X >> (uint32_t)Y) & (uint32_t)0x0001))
#define mask_clear(X) (X = 0)

#endif
