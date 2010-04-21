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

/**************************************************************************/
/* File:      dvdrp.c                                                     */
/* Author:    Cyrus Hall   :  hallcp@colorado.edu                         */
/* Date:      05/11/05                                                    */
/*                                                                        */
/* An implimentation of DV/DRP, a content-based routing and forwarding    */
/* protocol for sensor networks.  For more information on DV/DRP, see     */
/* the comments in the file or the tech report at:                        */
/* http://www.inf.unisi.ch/carzaniga/papers/cucs-979-04.pdf               */
/**************************************************************************/

/** @file dvdrp.c
 * @brief A DV/DRP implementation for the mos net layer.
 * @author Cyrus Hall
 * @date 07/20/2004
 */

#include <stdlib.h>
#include <string.h>

#include "led.h"
#include "node_id.h"
#include "mutex.h"

#include "dvdrp.h"
//#include "flood.h"

#ifdef DVDRP
uint16_t mos_check_stack(mos_thread_t *t);

////////////////////////////////////////
// Cache realted defines and globals
////
#define CACHE_SIZE   10	                // Packet cache size for flooding
static uint16_t cache_ptr[CACHE_SIZE];
static uint8_t  cache_curpos;

int8_t cache_check(dvdrp_pkt *pkt);

/////////////////////////////////////////////////
// Forward and routing table related variables
////
typedef struct dvdrp_path_t {
    node_id_t  next_hop;
    uint8_t    distance;
    uint8_t    fail_cnt;
} dvdrp_path;

typedef struct dvdrp_route_t {
    //main route/forwading information
    dvdrp_filter_list *pred;
    node_id_t          receiver_id;
    uint8_t            bv_pos;
    dvdrp_path         paths[DVDRP_RT_MAX_PATHS];

    //extra routing information
    uint8_t            seq;

    //forwarding timing
    uint32_t           last_pkt;
    uint32_t           min_delay;
} dvdrp_route;

//the main route table - this, along with the memory pools it uses, is
//the most hefty piece of memory usage in the code.  If you're running
//low on memory (easy to do with only 4/8k), try trimming the pool
//sizes below to the minimum you need and then reducing
//DVDRP_RT_MAX_PREDICATES (see dvdrp.h)
static dvdrp_route route_tbl[DVDRP_RT_MAX_PREDICATES];
//must be locked either when performing a route lookup or update.
static mos_mutex_t route_lock;

//forwarding functions
//static uint32_t dvdrp_get_splitbv(dvdrp_mesg_pkt *attribs);
static void dvdrp_get_splitbv(dvdrp_mesg_pkt *pkt, uint32_t *bv);

static uint8_t dvdrp_mesg_send(dvdrp_mesg_pkt *dvdrp_ptr, comBuf *cbuf);
static uint8_t dvdrp_match(dvdrp_mesg_pkt *, dvdrp_route *);
static uint8_t dvdrp_attr_match(dvdrp_attrib *attr,
				dvdrp_constraint_list *constraint);

//route update/maintance
static uint8_t dvdrp_get_free_bv(uint8_t additional_bv);
static uint8_t dvdrp_send_route_advert(dvdrp_route *route, uint8_t htl);
static uint8_t dvdrp_route_update(dvdrp_route *route, dvdrp_advert_pkt *pkt);
static void dvdrp_route_init(dvdrp_route *route, dvdrp_advert_pkt *pkt);
static uint8_t dvdrp_route_local_update(dvdrp_route *route,
					dvdrp_filter_list *pred);
static uint8_t dvdrp_route_resolve_bv_conflicts(dvdrp_advert_pkt *pkt);

static dvdrp_route *dvdrp_route_find(node_id_t receiver_id);
static dvdrp_route *dvdrp_route_find_unused();

static void dvdrp_route_copy_pkt_pred(dvdrp_route *route,
				      dvdrp_advert_pkt *pkt);
static void dvdrp_route_free_pred(dvdrp_filter_list *pred);
static void dvdrp_path_copy(dvdrp_path *dest, dvdrp_path *src);

//debug only
void print_packet(comBuf *buf);  

////////////////////////////
// Memory pools
////
dvdrp_constraint_list constraint_mem[DVDRP_MEM_CONSTRAINTS];
mem_pool              constraint_pool;
dvdrp_filter_list     filter_mem[DVDRP_MEM_FILTERS];
mem_pool              filter_pool;
comBuf                combuf_mem[DVDRP_MEM_COMBUFS];
mem_pool              combuf_pool;

////////////////////////////////////////
// Private DV/DRP variables
////
static uint8_t   node_seq_num;
static uint8_t   node_bv;  //stored as a decimal offset
static node_id_t node_id;
static uint32_t  node_delay;

//for protocol packets to use
static comBuf    local_pkt;

///////////////////////////////////////
// Public DV/DRP interface
////
/** @brief Setup the DV/DRP protocol and register with the net layer. */
void dvdrp_init()
{
    uint8_t i;
    
    dvdrp_init_mem_pools();

    for(i = 0; i <  DVDRP_RT_MAX_PREDICATES; ++i) {
	route_tbl[i].receiver_id = DVDRP_INVALID_NODE_ID;
    }
    mos_mutex_init(&route_lock);
    
    memset(cache_ptr, 0, sizeof(uint16_t) * CACHE_SIZE);
    cache_curpos = 0;
    
    node_seq_num = 0;
    node_id = mos_node_id_get();
    node_bv = DVDRP_INVALID_BV_ID;
    node_delay = 0;

    srand(node_id);
    
    net_proto_register(DVDRP_PROTO_ID, dvdrp_send, dvdrp_recv, dvdrp_ioctl);
}

/** @brief Send a packet using the DV/DRP protocol. 
 * @param pkt Packet to send
 * @param args 
 */
int8_t dvdrp_send(comBuf *pkt, va_list args)
{
    uint8_t i, orig_size;
    uint8_t num_attribs = va_arg(args, int);
    dvdrp_mesg_pkt *dvdrp_ptr = (dvdrp_mesg_pkt*)&(pkt->data[pkt->size]);
    dvdrp_attrib *attribs = va_arg(args, dvdrp_attrib *);

//    printf("dvdrp_send(%x): orig_size %C\n", attribs, pkt->size);

    orig_size = pkt->size;
    
    if(pkt->size + sizeof(dvdrp_mesg_pkt) + (num_attribs << 2) > COM_DATA_SIZE)
	return NET_BUF_FULL;

    dvdrp_ptr->header.pkt_type = DVDRP_MESG_PKT;
    dvdrp_ptr->header.htl = DVDRP_MAX_HTL;
    dvdrp_ptr->is_secondary = FALSE;
    dvdrp_ptr->num_attribs = num_attribs;

    pkt->size += 10;
    
    for(i = 0; i < num_attribs; ++i) {
//	printf("%x = %d\n", attribs[i].name, attribs[i].value);
	dvdrp_ptr->attributes[i].name = attribs[i].name;
	dvdrp_ptr->attributes[i].value = attribs[i].value;
	pkt->size += sizeof(dvdrp_attrib);
    }

//    printf("new size: %C\n", pkt->size);
    pkt->data[pkt->size++] = pkt->size - orig_size;

    mos_mutex_lock(&route_lock);
    dvdrp_get_splitbv(dvdrp_ptr, &dvdrp_ptr->split_bv);
    if(dvdrp_ptr->split_bv != 0) {
	dvdrp_mesg_send(dvdrp_ptr, pkt);
	i = pkt->size;
	//!!!!need to check for local delivery here - maybe we should
	//do such in dvdrp_mesg_send and return bytes instead
    } else {
	//no valid destinations, drop packet, don't do work
	printf(".NR");
	i = 0;
    }
    mos_mutex_unlock(&route_lock);

    return i;
}

/** @brief Receive a packet that came using the dvdrping protocol. 
 *
 * NOTE: Must subtract the sizeof route header from pkt->size so
 *       the net layer can determine whether any more data is available.
 * @param pkt Packet received
 * @param footer Location
 * @return FALSE if failure, else return TRUE
 */
boolean dvdrp_recv(comBuf *pkt, uint8_t **footer, uint8_t port)
{
    uint8_t hdr_size = pkt->data[pkt->size-1];
    uint8_t matches_local = FALSE;
    dvdrp_pkt *hdr = (dvdrp_pkt*)&pkt->data[pkt->size - hdr_size - 1];

    hdr->htl--;
    if(hdr->pkt_type == DVDRP_ADVERT_PKT) {
	printf(".ad");
	mos_led_toggle(2);
	mos_mutex_lock(&route_lock);
	dvdrp_route_update(dvdrp_route_find(((dvdrp_advert_pkt *)hdr)->receiver_id),
			   (dvdrp_advert_pkt *)hdr);
	mos_mutex_unlock(&route_lock);
    } else if(hdr->pkt_type == DVDRP_MESG_PKT) {
	printf(".msg");
	//note, we do not match the message again, but just match bv_id's
	if(((dvdrp_mesg_pkt *)hdr)->immed_dest == node_id) {
	    mos_mutex_lock(&route_lock);
	    matches_local = dvdrp_mesg_send((dvdrp_mesg_pkt *)hdr, pkt);
	    mos_mutex_unlock(&route_lock);
	} //else we drop the mo'fo
    } else if(hdr->pkt_type == DVDRP_REPAIR_PKT) {
	printf(".rep");
    }
    
    // Pull our header off. 
    pkt->size -= hdr_size + 1;  // +1 for size at eop

    //Set the footer if we need to deliver info to the application
    *footer = (uint8_t *)hdr;

    //Should the higher layer give a shit?
    if(matches_local) {
	printf(".ml\n");
	return TRUE;
    } else {
	printf("\n");
	return FALSE;
    }
}

/** @brief Set some protocol specific parameters. 
 * @param request IO request
 * @param args Arguements
 */
int8_t dvdrp_ioctl(uint8_t request, va_list args)
{
    uint8_t ret = TRUE;
    dvdrp_route *route;
    dvdrp_filter_list *pred;

    switch(request) { 
    case DVDRP_IOCTL_SETPRED: // Change subscription predicate
       //printf("dvdrp_ioctl(): setpred\n");
       pred = va_arg(args, dvdrp_filter_list *);
       node_delay = va_arg(args, uint32_t);
       
       mos_mutex_lock(&route_lock);
       route = dvdrp_route_find(node_id);
       if(!route) {
	  ret = dvdrp_route_local_update(NULL, pred);
       } else {
	  ret = dvdrp_route_local_update(route, pred);
       }
       mos_mutex_unlock(&route_lock);
       
       break;
    case DVDRP_IOCTL_SETFAILURE_LIMIT:   
    case DVDRP_IOCTL_RESET_PROTO:        
    default:
	break;
    } 
   
    return ret;
}

static uint8_t dvdrp_send_route_advert(dvdrp_route *route, uint8_t htl) {
    dvdrp_advert_pkt *advert_pkt = (dvdrp_advert_pkt*)&(local_pkt.data[0]);
    dvdrp_filter_list *f_list;
    dvdrp_filter *cur_f;
    dvdrp_constraint_list *c_list;
    uint8_t num_c, num_f, constraint_size, orig_size;

    orig_size = local_pkt.size = 0;
    
    advert_pkt->header.pkt_type = DVDRP_ADVERT_PKT;
    advert_pkt->header.htl = htl;
    advert_pkt->receiver_id = route->receiver_id;
    advert_pkt->prev_hop = node_id;
    advert_pkt->seq_num = route->seq;
    advert_pkt->bv_pos = route->bv_pos;
    advert_pkt->min_delay = route->min_delay;

    local_pkt.size = 13;   //bytes up to this point, inc. pred.num_filters
    
    f_list = route->pred;
    num_f = 0;
    cur_f = &advert_pkt->pred.filters[0];
    while(f_list) {
	c_list = f_list->constraints;
	constraint_size = num_c = 0;
	while(c_list) {
	    cur_f->constraints[num_c].name = c_list->name;
	    cur_f->constraints[num_c].value = c_list->value;
	    cur_f->constraints[num_c++].compare_type = c_list->compare_type;
	    c_list = c_list->next;
	    constraint_size += sizeof(dvdrp_constraint);
//	    printf("c\n");
	}
	cur_f->num_constraints = num_c;
	constraint_size += sizeof(uint8_t);

	cur_f += constraint_size;
	local_pkt.size += constraint_size;

	f_list = f_list->next;
	num_f++;
//	printf("f\n");
    }
    advert_pkt->pred.num_filters = num_f;

    local_pkt.data[local_pkt.size++] = local_pkt.size - orig_size;
    /* Add the proto ID to the end of the packet. */
    local_pkt.data[local_pkt.size++] = DVDRP_PROTO_ID;

//    print_packet(&local_pkt);

    com_send(IFACE_RADIO, &local_pkt);

//    printf("Sent advert.\n");
    
    return local_pkt.size;
}

/** Functions private to this file. **/
///////////////////////////////////////
// Foward/Route table maintance.
////
//forwarding funcs

static void dvdrp_get_splitbv(dvdrp_mesg_pkt *pkt, uint32_t *bv) {
    uint8_t  i;

    *bv = 0;
    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	if(dvdrp_match(pkt, &route_tbl[i])) {
	    mask_set(*bv, route_tbl[i].bv_pos);
	}
    }
}

static uint8_t dvdrp_mesg_send(dvdrp_mesg_pkt *dvdrp_ptr, comBuf *pkt) {
    node_id_t cur_next_hop = 0;  //to make build clean
    uint32_t  /*global,*/ local_bv;
    uint8_t   global;
    uint8_t   i, ret;
    
    // Add the proto ID to the end of the packet.
    pkt->data[pkt->size++] = DVDRP_PROTO_ID;

    // This is a rather rough function to write without dynamicly
    // sized DSs, so here it is unoptimized:
    // 1) create a global bit vector marked with matching routes
    // 2) take the first marked route and record it's next_hop
    // 3) unmark that route from the global bv ---\ !!
    // 4) mark in the local bv                 ---/ now rolled into step 5!
    // 5) iterate through the rest of the marked routes
    //   a) if next_hop is the same, repeat 3 & 4 for route
    // 6) local bv is packets bvs; assign and send out
    // 7) goto 2
    //
    // This strategy, while slow, saves on the need for larger arrays
    // or other memory hungary data structures.  We can pay for
    // smaller memory size with a little bit of energy.
    //
    // Note: I've looked at the asm for this function and it's horrid.
    // avr-libc's 32-bit number support isn't poor, but rather I
    // didn't realize what an impact emulating 32-bit operations would
    // have.  This function will need to get a serious facelift soon.
    // If the use of global can be reduced by doing inline marking in
    // the route table, or some other method, it'll be a major victory
    // toward reducing the size.

    // 1)
    global = 0;
    ret = FALSE;
    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	if(route_tbl[i].receiver_id != DVDRP_INVALID_NODE_ID
	   &&
	   mask_query(dvdrp_ptr->split_bv, route_tbl[i].bv_pos))
	{
//	    printf(".r;%d:%C", route_tbl[i].receiver_id, route_tbl[i].bv_pos);
	    mask_set(global, i);
	}
    }

    // 2)
    while(1) {
	for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	    if(mask_query(global, i)) {
		cur_next_hop = route_tbl[i].paths[0].next_hop;
		break;
	    }
	}

	if(i == DVDRP_RT_MAX_PREDICATES) {
//	    printf(".outp");
	    break;
	}

	// 5)
	local_bv = 0;
	for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	    // 5a)
	    if(mask_query(global, i) &&
	       cur_next_hop == route_tbl[i].paths[0].next_hop)
	    {
		// 5a - 3)
		mask_unset(global, i);
		// 5a - 4)
		mask_set(local_bv, route_tbl[i].bv_pos);
	    }
	}

	//check for local delivery
	if(cur_next_hop == node_id) {
	    ret = TRUE;
//	    printf(".mf;l");
	} else {
	    dvdrp_ptr->immed_dest = cur_next_hop;
	    dvdrp_ptr->split_bv = local_bv;
	    
//	    printf(".mf;%d", dvdrp_ptr->immed_dest);
	    com_send(IFACE_RADIO, pkt);
	    //mos_led_toggle(1);
	}
    }

//    printf(".pts;%d\n", mos_check_stack(mos_thread_current()));
    
    return ret;
}

static uint8_t dvdrp_match(dvdrp_mesg_pkt *dvdrp_ptr, dvdrp_route* r) {
    dvdrp_filter_list     *f_list;
    dvdrp_constraint_list *c_list;
    uint8_t counter, filter_size, i;

    //for each filter in the predicate
    for(f_list = r->pred; f_list; f_list = f_list->next) {
	//for each constraint in the filter
//	printf("\nf.");
	counter = filter_size = 0;   //reset size of filter
	for(c_list = f_list->constraints; c_list; c_list = c_list->next) {
	    filter_size++;
	    //for each attribute in the message
//	    printf("c.");
	    for(i = 0; i < dvdrp_ptr->num_attribs; ++i) {
//		printf("a.");
		//check to see if attribute is covered by the constraint
		if(dvdrp_attr_match(&dvdrp_ptr->attributes[i], c_list)) {
		    counter++;
		}
	    }
	}
	
	//if all contraints are covered, packet needs to filtered down
	//interface
	if(counter == filter_size) {
//	    printf("  Fwd match: recv = %d.\n", r->receiver_id);
	    //if this filter matches, skip the rest of the predicate
	    return TRUE;
	}
    }
    
    return FALSE;    
}

static uint8_t dvdrp_attr_match(dvdrp_attrib *attr,
				dvdrp_constraint_list *constraint)
{
    uint8_t ret = FALSE;

    if(attr->name == constraint->name) {
	switch (constraint->compare_type) {
	case DVDRP_COMPARE_EQ:
	    ret = (attr->value == constraint->value);
	    break;
	case DVDRP_COMPARE_GT:
	    ret = (attr->value > constraint->value);
	    break;
	case DVDRP_COMPARE_LT:
	    ret = (attr->value < constraint->value);
	    break;
	case DVDRP_COMPARE_NE:
	    ret = (attr->value != constraint->value);
	    break;
	}
    }

    return ret;
}

/////////////////////////
//Route table funcs
//

//get free bv id for a new route - returns 32 (DVDRP_INVALID_BV_ID) if
//no free bv
static uint8_t dvdrp_get_free_bv(uint8_t additional) {
    uint32_t bv = 0;
    uint8_t i, j;

    //first marked all used bits
    if(additional != DVDRP_INVALID_BV_ID) {
	mask_set(bv, additional);
    }
    
    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	if(route_tbl[i].pred) {
	    mask_set(bv, route_tbl[i].bv_pos);
	}
    }

    //now find free bv
    j = 0;
    if(bv == 0xFFFF) {  //we're full, so sorry
	i = DVDRP_INVALID_BV_ID;
    } else {
	i = rand() % 32;

	for( ; j < 32; ++j) {
	    if(!mask_set(bv, (i + j) % 32)) {
		break;
	    }
	}
    }

    return (i + j) % 32;
}

//pass NULL for local init
static void dvdrp_route_init(dvdrp_route *route, dvdrp_advert_pkt *pkt) {
    uint8_t i;

    if(!route)
	return;

    if(pkt) {
	route->receiver_id = pkt->receiver_id;
	route->bv_pos = pkt->bv_pos;  
	route->paths[0].next_hop = pkt->prev_hop;
	route->paths[0].distance = DVDRP_MAX_HTL - pkt->header.htl;
	route->seq = pkt->seq_num;
	route->min_delay = pkt->min_delay;
    } else {
	route->receiver_id = node_id;
	route->bv_pos = node_bv;  
	route->paths[0].next_hop = node_id;
	route->paths[0].distance = 0;
	route->seq = node_seq_num;
	route->min_delay = node_delay;
    }

    route->paths[0].fail_cnt = 0;
    route->last_pkt = 0;
    for(i = 1; i < DVDRP_RT_MAX_PATHS; ++i) {
	route->paths[i].next_hop = DVDRP_INVALID_NODE_ID;
	route->paths[i].distance = DVDRP_MAX_HTL;
	route->paths[i].fail_cnt = 0;
    }
}

static uint8_t dvdrp_route_local_update(dvdrp_route *route,
					dvdrp_filter_list *pred)
{
    if(!route) {
	//the local node has not yet set a route, lets do so
	route = dvdrp_route_find_unused();
	node_bv = dvdrp_get_free_bv(DVDRP_INVALID_BV_ID);

//	printf(".bvid;%C\n", node_bv);

	if(!route || node_bv == DVDRP_INVALID_BV_ID)  { //no unused routes
	    dvdrp_route_free_pred(pred);
	    
	    return FALSE;
	}

	dvdrp_route_init(route, NULL); 
    }

//    printf("pred_passed: %x\n", pred);
    
    //predicate is localy allocated by the application - use their
    //version of the pred to save space and force ioctl for updates
    route->pred = pred;
    route->seq = ++node_seq_num;

    return dvdrp_send_route_advert(route, DVDRP_MAX_HTL);
}

static uint8_t dvdrp_route_update(dvdrp_route *route, dvdrp_advert_pkt *pkt) {
    uint8_t pkt_dist;
    uint8_t i, j;
    uint8_t ret = FALSE;

    pkt_dist = DVDRP_MAX_HTL - pkt->header.htl;
    if(!route) {
	//first we need to check for and resolve any BV conflicts
	//also, make sure there is an available route
	if(!dvdrp_route_resolve_bv_conflicts(pkt) ||    //bv conflicts?
	   !(route = dvdrp_route_find_unused()))        //free route?
	{
	    return FALSE;
	}
	
//	printf("New sub (rcv = %d)\n", pkt->receiver_id);
	
	dvdrp_route_init(route, pkt);

	dvdrp_route_copy_pkt_pred(route, pkt);
	dvdrp_send_route_advert(route, pkt->header.htl);

	///!!!! Note the return here - below code does not run
	return TRUE;
    }
    
    for(i = 0; i < DVDRP_RT_MAX_PATHS; ++i) {
	if(pkt_dist < route->paths[i].distance || pkt->seq_num > route->seq) {
//	    printf("Updating sub, idx %C: %C < %C (%C > %C)\n",
//		   i, pkt_dist, route->paths[i].distance, pkt->seq_num,
//		   route->seq);
//	    printf("(rcv = %d)\n", pkt->receiver_id);

	    j = DVDRP_RT_MAX_PATHS - 1;
	    while(j > i) {
		dvdrp_path_copy(&route->paths[j], &route->paths[j-1]);
		--j;
	    }
	    
	    route->paths[i].next_hop = pkt->prev_hop;
	    route->paths[i].distance = pkt_dist;
	    
	    if(pkt->seq_num > route->seq) {  //note: i == 0 gaureenteed
		for(j = 1; j < DVDRP_RT_MAX_PATHS; ++j) {
		    route->paths[j].next_hop = DVDRP_INVALID_NODE_ID;
		    route->paths[j].distance = DVDRP_MAX_HTL;
		}

		if(pkt->pred.num_filters == 0) {
		    //we need to kill this route!
		    route->receiver_id = DVDRP_INVALID_NODE_ID;
		    dvdrp_route_free_pred(route->pred);  //free the old pred
		    route_tbl[i].pred = NULL;
		} else {
		    dvdrp_route_free_pred(route->pred);  //free the old pred
		    dvdrp_route_copy_pkt_pred(route, pkt);
		    route->seq = pkt->seq_num;
		    route->min_delay = pkt->min_delay;

		    //there are timing situation in bv arbitration
		    //that force this to be updated here
		    route->bv_pos = pkt->bv_pos;
		}
	    }

	    //we only want to propigate the subscription if we update
	    //the PRIMARY - a second broadcast only adds noise to an
	    //already received route.
	    if(i == 0) {
		dvdrp_send_route_advert(route, pkt->header.htl);
	    }

	    ret = TRUE;
	    break;
	}
	
	//protect against a next_hop being listed twice
	if(route->paths[i].next_hop == pkt->prev_hop) {
	    break;
	}

    }

    //make sure that there arn't multiple paths using the same node
    for(j = i+1; j < DVDRP_RT_MAX_PATHS; ++j) {
	if(route->paths[j].next_hop == route->paths[i].next_hop) {
	    route->paths[j].next_hop = DVDRP_INVALID_NODE_ID;
	    route->paths[j].distance = DVDRP_MAX_HTL;
	}
    }
    
    if(!ret) {
//	printf("Dropping subscription pkt.\n");
    }

    return ret;
}

static uint8_t dvdrp_route_resolve_bv_conflicts(dvdrp_advert_pkt *pkt) {
    uint8_t i, ret = TRUE;
    
    // Here's the layout:
    // 1) Check for any other route with the same bv
    //   a) if found, check receiver id;
    //   b) if new_mesg.recv_id < old_bv
    //     i) check if *we* were the loser - readvertise with new bv
    //    ii) else kill the old route - they need to readvertise
    //   c) else return FALSE, as this new advert is the loser

    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	//1)
	if(route_tbl[i].bv_pos == pkt->bv_pos) {
	    //a)
	    if(pkt->receiver_id < route_tbl[i].receiver_id) {
		//b)
		if(route_tbl[i].receiver_id == node_id) {
		    //i)
		    //get a new node_bv
		    node_bv = dvdrp_get_free_bv(pkt->bv_pos);
		    route_tbl[i].bv_pos = node_bv;

		    //send replacement advertisement
		    dvdrp_send_route_advert(&route_tbl[i], DVDRP_MAX_HTL);
		} else {
		    //kill old route
		    route_tbl[i].receiver_id = DVDRP_INVALID_NODE_ID;
		    dvdrp_route_free_pred(route_tbl[i].pred);
		    route_tbl[i].pred = NULL;
		}
		break;
	    } else {
		//c)
		ret = FALSE;
		break;
	    }
	}
    }
    
    return ret;
}

static dvdrp_route *dvdrp_route_find(node_id_t receiver_id) {
    uint8_t i;

    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	if(route_tbl[i].receiver_id == receiver_id) {
	    return &route_tbl[i];
	}
    }

    return NULL;
}

static dvdrp_route *dvdrp_route_find_unused() {
    uint8_t i;

    for(i = 0; i < DVDRP_RT_MAX_PREDICATES; ++i) {
	if(route_tbl[i].receiver_id == DVDRP_INVALID_NODE_ID) {
	    return &route_tbl[i];
	}
    }

    return NULL;
}

static void dvdrp_route_copy_pkt_pred(dvdrp_route *route,
				      dvdrp_advert_pkt *pkt)
{
    dvdrp_filter_list     **f_list;
    dvdrp_filter           *cur_filter;
    dvdrp_constraint_list **c_list;
    uint8_t                 i, j, size;

    f_list = &route->pred;
    cur_filter = &(pkt->pred.filters[0]);
//    printf("\np %C\n", pkt->pred.num_filters);
    for(i = 0; i < pkt->pred.num_filters; ++i) {
	*f_list = pool_malloc(&filter_pool);
	size = sizeof(uint8_t);
	c_list = &((*f_list)->constraints);

//	printf(" f %C (%d)\n", cur_filter->num_constraints,
//	       (void*)cur_filter - (void*)pkt);
 	for(j = 0; j < cur_filter->num_constraints; ++j) {
	    *c_list = pool_malloc(&constraint_pool);
	    (*c_list)->name = cur_filter->constraints[j].name;
	    (*c_list)->value = cur_filter->constraints[j].value;
	    (*c_list)->compare_type = cur_filter->constraints[j].compare_type;

//	    printf("   c %C %C %C\n", cur_filter->constraints[j].name,
//		   cur_filter->constraints[j].value,
//		   cur_filter->constraints[j].compare_type);
	    
	    c_list = &((*c_list)->next);
	    size += sizeof(dvdrp_constraint);
	}

	*c_list = NULL;
	cur_filter += size;
	f_list = &(*f_list)->next;
    }
//    printf("\\p\n");
    
    *f_list = NULL;
}

static void dvdrp_route_free_pred(dvdrp_filter_list *pred) {
    dvdrp_filter_list *f_list;
    dvdrp_constraint_list *c_list;
    void *temp_mem;

    f_list = pred;
    while(f_list) {
	c_list = f_list->constraints;
	while(c_list) {
	    temp_mem = c_list;
	    c_list = c_list->next;
	    pool_free(&constraint_pool, temp_mem);
	}
	temp_mem = f_list;
	f_list = f_list->next;
	pool_free(&filter_pool, temp_mem);
    }
}

static void dvdrp_path_copy(dvdrp_path *dest, dvdrp_path *src) {
    dest->next_hop = src->next_hop;
    dest->distance = src->distance;
    dest->fail_cnt = src->fail_cnt;
}

///////////////////////////////////////
// Cache maintance.
////
/** @brief Check the cache for the given packet, and cache it if we haven't
 * already seen it. 
 * @param pkt Packet 
 * @return FALSE if this is an old or non-existent packet, else return TRUE
 */
int8_t cache_check(dvdrp_pkt * pkt)
{
    return FALSE;
}


///////////////////////////////////////
// Pool maintance functions.
////
void dvdrp_init_mem_pools() {
    constraint_pool.num_elems = DVDRP_MEM_CONSTRAINTS;
    constraint_pool.type_size = sizeof(dvdrp_constraint_list);
    constraint_pool.free_vec  = 0xFFFF;
    constraint_pool.pool = &constraint_mem[0];

    filter_pool.num_elems = DVDRP_MEM_FILTERS;
    filter_pool.type_size = sizeof(dvdrp_filter_list);
    filter_pool.free_vec  = 0xFFFF;
    filter_pool.pool = &filter_mem[0];

    combuf_pool.num_elems = DVDRP_MEM_COMBUFS;
    combuf_pool.type_size = sizeof(comBuf);
    combuf_pool.free_vec  = 0xFFFF;
    combuf_pool.pool = &combuf_mem[0];
}

void *pool_malloc(mem_pool *pool) {
    uint8_t i;

    for(i = 0; i < pool->num_elems; ++i) {
	if(mask_query(pool->free_vec, i)) {
	    mask_unset(pool->free_vec, i);
	    return (pool->pool + (i * pool->type_size));
	}
    }

    return NULL;
}

void pool_free(mem_pool *pool, void *mem) {
    uint8_t bit;

    bit = (mem - pool->pool) / pool->type_size;
    mask_set(pool->free_vec, bit);
}

void print_packet(comBuf *buf) {
    uint8_t i;
    
    printf("\n");
    for(i = 0; i < buf->size; ++i) {
	printf("%C ", buf->data[i]);
    }
}
 
#endif