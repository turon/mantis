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

#include "mos.h"
#include "optsconfig.h"
#include "com.h"

#ifndef _MOS_DEBUGGING_H_
#define _MOS_DEBUGGING_H_

#ifdef MOS_DEBUG

#define DEBUG_PACKET 254
#define DEBUG_ACK 253

#define DEBUG_TRACE_SIZE  200    //size of the trace length in bytes
#define DEBUG_TRACE_START 0
#define DEBUG_TRACE_END   DEBUG_TRACE_SIZE-1
#define DBCODE_SIZE  4      //size of an error code in bits


#define LOWER_BIT_MASK   15     //00001111
#define UPPER_BIT_MASK   240    //11110000

// leave the 0000 code as the start code
#define DBCODE_CONTEXT_SWITCH   1
#define DBCODE_PROCEDURE_CALL   2
#define DBCODE_PROCEDURE_RETURN 3
#define DBCODE_INTERRUPT        4
#define DBCODE_THREAD_BLOCK     5
#define DBCODE_THREAD_UNBLOCK   6
#define DBCODE_TIMER_SET        7
#define DBCODE_TIMER_FIRED      8
#define DBCODE_THREAD_SLEEP     9
#define DBCODE_THREAD_WAKEUP    10
#define DBCODE_NODE_SLEEP       11
#define DBCODE_NODE_WAKEUP      12
#define DBCODE_THREAD_NEW       13
#define DBCODE_THREAD_EXIT      14
#define DBCODE_BREAKPOINT       15

#define DBCODE_THREAD1          16
#define DBCODE_THREAD2          17
#define DBCODE_THREAD3          18
#define DBCODE_THREAD4          19
#define DBCODE_THREAD5          20
#define DBCODE_THREAD6          21

//todo:
//idle mode
//ints enabled/disabled/
//somehow do mutexes

#define SIGNAL_STACK_OVERFLOW   1
#define SIGNAL_DEADLOCK         2
#define SIGNAL_ASSERTION_FAILED 3
#define SIGNAL_WATCHDOG_EXPIRED 4

#define DEBUG_DATA_SIZE 30
#define DEBUG_STATUS_CHECK_INTERVAL 1500 // 1.5 seconds, must be less than the WDT reset (2 seconds)

typedef struct debug_checkpoint_ {
   uint32_t timeout;
   uint32_t last_timestamp;
   struct debug_checkpoint_ *next;
} debug_checkpoint_t;

typedef struct {
    uint8_t type;
    uint8_t src;
    uint8_t dest;
    uint8_t next_hop;
    uint8_t last_hop;
    uint8_t seqno;
    uint8_t pkt_dtb;
    uint8_t sender_dtb;
} debug_net_packet_t;

typedef struct {
   uint8_t error_code;
   uint8_t offset;
   uint8_t size;
   uint8_t data[DEBUG_DATA_SIZE]; //10 bytes for header info
} debug_packet_t;


#define ASSERT(condition) if(!(condition)) mos_debug_error_report(SIGNAL_ASSERTION_FAILED)

#define MOS_DEBUG_PRE_SLEEP() {               \
      mos_debug_set_trace(DBCODE_NODE_SLEEP); \
      wdt_disable(      );                    \
}

#define MOS_DEBUG_POST_SLEEP() {              \
      mos_debug_set_trace(DBCODE_INTERRUPT);  \
      mos_debug_set_trace(DBCODE_NODE_WAKEUP);\
      wdt_enable(WDTO_2S);                    \
}
void mos_debug_init();
void mos_debug_post_init();
boolean mos_debug_enabled();
void mos_debug_set_trace(uint8_t code);
void mos_debug_clear_trace();
void mos_debug_error_report(uint8_t signal);

void mos_debug_register_checkpoint(debug_checkpoint_t *cp, uint32_t ms);
void mos_debug_checkpoint_reached(debug_checkpoint_t *cp);
void mos_debug_status_check();


void mos_debug_check_stack();
#endif

#endif
