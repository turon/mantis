#include "mos.h"
#include "msched.h"
#include "cc2420.h"
#include "inttypes.h"
#include "dev.h"
#include "printf.h"
#include "cond.h"
#include "led.h"
#include <string.h>



#define CTP_NEW_PACKET      1 
#define CTP_EXISTING_PACKET 0
#define CTP_DUMMY_PORT      45
#define CTP_LISTENING_PORT  46

#define CTP_SET_IS_BASE     0      
#define CTP_SET_IS_NODE     1
#define CTP_SET_POWER       2
#define CTP_SET_TESTBED 3

#define CTP_OPT_PULL 0x01

#define CTP_OPT_ECN 0x02  // congestion bit

#define BEACON_TIMER_MAX 64

#define BEACON_TIMER_MIN 1

#define CTP_QUEUE_SIZE 10

#define CTP_FORWARDING_SIZE 5

#define MAX_NEIGHBORS 5

#define MAX_LIENTRIES 3

#define APP_SEND_INTERVAL 2000

//#define BROADCAST_ADDR 0

#define AM_CTP_ROUTING   1
#define AM_CTP_DATA      0
#define AM_CTP_BAD_FRAME 0xff


#define PRR_WINDOWSIZE 16

#define MAX_LQI 10+PRR_WINDOWSIZE

#define MAX_ETX 520

#define AM_OSCILLOSCOPE 0 

// rssi_val ranges from -60 to 60.  To facilate calculation, we force it to be positive,
// by adding 128.  The corresponding input power is given by 
//    P = rssi_val + rssi_offset [dbm]
// where rssi_offset is found empirically around -45.
// cc2420 rx sensitivity threshold is said to be -90dbm (?), 
// so the norminal rssi_val threshold is -45dbm.
// We will map this threshold to positive value by adding 128.
#define RSSI_THRESHOLD  80 //was 83, changed to 78 based on the observation in experiment
//50

#define NEW_FRESH 80 

typedef struct routing_entry_t{
	uint16_t addr;
	uint16_t etx;
        uint16_t prr;
	uint8_t freshness;
	uint8_t ib_lqi;
        uint8_t ob_lqi;
        uint8_t lastrecvdseqno;
} routing_entry_t;

#define LEEP_ENTRY_SIZE 3
typedef struct leep_entry_t{
  //uint16_t node_id;
	uint8_t node_idHigh;
	uint8_t node_idLow;
	uint8_t lq;
} leep_entry_t;

#define CTP_ROUTING_HEADER_SIZE 8 
typedef struct ctp_routing_header_t{
	uint8_t type;
  uint8_t leepEntries;
	uint8_t seqno;
	uint8_t options;
	uint16_t parent;
	uint8_t etxHigh;
	uint8_t etxLow;	
	uint8_t data[MAX_LIENTRIES*3]; //[MAX_NEIGHBORS*3];
	//uint16_t signal;
} ctp_routing_header_t;

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

void get_leep_entry(uint8_t* data, uint8_t offset, leep_entry_t* entry);

uint16_t htons(uint16_t v);

//void send_thread();

//void source_thread();

//void chooseNewParent();

//void routing_thread();

void updateParent();

void updateRoutingTable(uint16_t source, ctp_routing_header_t* rh);

void receive_thread();

void beacon_thread();

void test_recv();

void test_send();

void seed_random();

static uint32_t rand_mlcg();;

uint8_t inline getOptions();

uint16_t getEtx();

uint16_t  getNextHop();

uint8_t send_packet(comBuf* buf);

void enqueueDataPacket();

void enqueueBeacon();

void inline enqueuePacket(comBuf* buf);

comBuf* dequeuePacket();

comBuf* allocatePacket();

void freePacket(comBuf * buf);

int packetsLeft();

void resetBeaconInterval();

void beaconFast();

int8_t  net_ctp_proto_send(comBuf *buf, va_list args);
boolean net_ctp_proto_recv(comBuf *buf, uint8_t **footer, uint8_t port);
int8_t  net_ctp_proto_ioctl(uint8_t request, va_list args);
