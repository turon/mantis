#include "mos.h"
#include "msched.h"
#include "cc2420.h"
#include "inttypes.h"
#include "dev.h"
#include "printf.h"
#include "cond.h"
#include "led.h"
#include <string.h>

/** For net layer interface **/
#define CTP_NEW_PACKET      1 
#define CTP_EXISTING_PACKET 0
#define CTP_DUMMY_PORT      45
#define CTP_LISTENING_PORT  46

#define CTP_SET_IS_BASE     0      
#define CTP_SET_IS_NODE     1
#define CTP_SET_POWER       2
#define CTP_SET_TESTBED     3

/** registered interfaces on net layer **/
int8_t  net_ctp_proto_send(comBuf *buf, va_list args);
boolean net_ctp_proto_recv(comBuf *buf, uint8_t **footer, uint8_t port);
int8_t  net_ctp_proto_ioctl(uint8_t request, va_list args);


/** Assumptions on the network **/
#define CTP_NETWORK_DIAMETER  20

/** controls for routing computation **/
#define PRR_WINDOWSIZE      16
#define HOP_COUNT           10
#define MAX_LQI             26    // = (HOP_COUNT + PRR_WINDOWSIZE)
#define MAX_ETX             520   // = MAX_LQI * CTP_NETWORK_DIAMETER
#define OPT_ETX			50	// used to modify X-mit power to achieve OPT-ETX
#define NEW_FRESH           80
#define NEW_DL_FRESH        40
// rssi_val ranges from -60 to 60.  To facilate calculation, we force it to be positive,
// by adding 128.  The corresponding input power is given by 
//    P = rssi_val + rssi_offset [dbm]
// where rssi_offset is found empirically around -45.
// cc2420 rx sensitivity threshold is said to be -90dbm (?), 
// so the norminal rssi_val threshold is -45dbm.
// We will map this threshold to positive value by adding 128.
#define RSSI_THRESHOLD      80 //was 83, changed to 80 based on the observation in experiment 


/** CTP FRAME TYPES **/
#define AM_CTP_DATA         0
#define AM_CTP_ROUTING      1
#define AM_CTP_DL_DATA      2
#define AM_CTP_DL_ROUTING   3

/** Higher Layger Protocol (App usage) **/
#define AM_OSCILLOSCOPE     0      // possible usage for upstream
#define AM_DIAGNOSIS        1      // possible usage for downstream
#define AM_CROSSLAYER       2      // possible usage for downstream

/** Control bit in the CTP frame header **/
#define CTP_OPT_PULL        0x01
#define CTP_OPT_ECN         0x02  // congestion bit

/** CTP frame header formats **/
/** CTP_ROUTING **/
#define CTP_ROUTING_HEADER_SIZE 8 
#define MAX_LIENTRIES           3
typedef struct ctp_routing_header_t{
	uint8_t  type;
	uint8_t  leepEntries;
	uint8_t  seqno;
	uint8_t  options;
	uint8_t  parentHigh;
	uint8_t  parentLow;
	uint8_t  etxHigh;
	uint8_t  etxLow;	
	uint8_t  data[MAX_LIENTRIES*3];
} ctp_routing_header_t;


/** CTP_DL_ROUTING **/
#define CTP_DL_ROUTING_HEADER_SIZE  9 
#define MAX_DESTENTRIES  3
typedef struct ctp_dl_routing_header_t{
	uint8_t  type;
	uint8_t  leepEntries;
	uint8_t  seqno;
	uint8_t  options;
	uint8_t  parentHigh;
	uint8_t  parentLow;
	uint8_t  etxHigh;
	uint8_t  etxLow;	
	uint8_t  destEntries;
	uint8_t  data[MAX_LIENTRIES*3 + MAX_DESTENTRIES * 6];	
} ctp_dl_routing_header_t;

/** CTP_DATA **/
#define CTP_DATA_HEADER_SIZE 9 
typedef struct ctp_data_header_t{
	uint8_t type;
	uint8_t options;
	uint8_t etxHigh;
	uint8_t etxLow;
	uint8_t thl;
	uint8_t originSeqNo;
	uint8_t originHigh;     
	uint8_t originLow;      
	uint8_t collectId;
} ctp_data_header_t;

/** CTP_DL_DATA **/
#define CTP_DL_DATA_HEADER_SIZE 11 
typedef struct ctp_dl_data_header_t{
	uint8_t type;
	uint8_t options;
	uint8_t etxHigh;
	uint8_t etxLow;
	uint8_t thl;
	uint8_t originSeqNo;
	uint8_t originHigh;     
	uint8_t originLow;  
	uint8_t destHigh;
	uint8_t destLow;
	uint8_t collectId;
} ctp_dl_data_header_t;

/** LEEP (appened to CTP_ROUTING and CTP_DL_ROUTING) **/
#define LEEP_ENTRY_SIZE 3
typedef struct leep_entry_t{  
	uint8_t node_idHigh;
	uint8_t node_idLow;
	uint8_t lq;
} leep_entry_t;

/** downstream destination list (appended to CTP_DL_ROUTING) **/
#define DEST_ENTRY_SIZE 6
typedef struct dest_entry_t {
  uint8_t node_idHigh;
  uint8_t node_idLow;
  uint8_t r_parentHigh;
  uint8_t r_parentLow;
  uint8_t r_etxHigh;
  uint8_t r_etxLow;
} dest_entry_t;


#define BEACON_TIMER_MAX    64
#define BEACON_TIMER_MIN    1
#define BEACON_INTERVAL     200


/** Internal data structure **/
#define MAX_NUM_DEST    3          // number of the nodes that Base can connect to at the same time
#define MAX_NEIGHBORS   5          // ensure that MAX_NEIGHBORS >= MAX_NUM_DEST + 1

//#define CTP_QUEUE_SIZE 10
//#define CTP_FORWARDING_SIZE 5

/* modified routing table to include both up- and downstream routing info */
typedef struct routing_entry_t{
	uint16_t  addr;                  // neighbor node id
	uint16_t  etx;                   // forward ETX to Base
	uint16_t  prr; 
	uint8_t   freshness;
	uint8_t   ib_lqi;
	uint8_t   ob_lqi;
	uint8_t   lastrecvdseqno;
  
	uint16_t  r_etx[MAX_NUM_DEST];   // reverse ETXs to destination nodes    
	uint8_t   flag;
} routing_entry_t;

typedef struct destnode_t {
    uint8_t  freshness;
    uint8_t  r_parent;
    uint16_t id;
    uint16_t r_bestETX;
    uint16_t r_nextHop;
    uint16_t beaconTimer;
    uint16_t beaconTimerCount;
} destnode_t;


// munipulate beacon speed. Minumum granularity is defined as BEACON_INTERVAL
void inline increaseBeaconInterval(destnode_t *dt);
void inline resetBeaconInterval(destnode_t *dt);
void inline beaconFast(destnode_t *dt);

/** other external controls to user **/
extern boolean is_base;
extern boolean ctp_append_hops;

void ctp_proto_init(void);

typedef enum CTP_frameCategory {ROUTING_FRAME, DATA_FRAME, UNKNOWN_FRAME} CTP_frameCategory;
CTP_frameCategory inline getFrameCat(uint8_t frametype);
uint8_t inline getHeaderSize(uint8_t frametype);

void  seed_random();
uint8_t rand_mlcg_8();

uint8_t  inline getOptions();
uint16_t inline getNextHop(uint8_t frametype, uint16_t dest);
uint16_t inline getETX(uint8_t frametype, uint16_t dest);
boolean  inline isDestination(uint8_t frametype, uint16_t dest);


/** setup or remove the connections **/
uint8_t connectTo(uint16_t dest);
boolean blockConnectTo(uint16_t dest);
void close(uint16_t dest);


