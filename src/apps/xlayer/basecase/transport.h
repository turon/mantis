#ifndef TRANSPORT_H_
#define TRANSPORT_H_

#include "clock.h"
#include "queue.h"

#define RECV_Q_UNDERFLOW Q_UNDERFLOW
#define RECV_Q_OK Q_OK
#define CONNECT_OK 1
#define CONNECT_BAD -1

// #define NODE_ID 1
// #define BASE_NODE_ID 1
// #define LAST_NODE_ID 2
#define ACK_TIMEOUT 10000 // Wait for up to 10 seconds for an ACK
#define COM_RECV_TIMEOUT 100
#define ACK_FLAG 1

// Array indices into packet header for data members
#define SIZE_INDEX     0
#define SEQ_NUM_INDEX  1
// #define NODE_ID_INDEX  5
#define ACK_INDEX      5
#define DEST_PORT_INDEX     6  // Destination port
#define SRC_PORT_INDEX      7  // Source port
#define CONN_ID_INDEX  8 // Connection id
#define DATA_INDEX     9 // Starting index for packet data

// Each packet contains up to 16 bytes of data
#define PKT_DATA_SIZE 55
#define STORED_HEADER_SIZE 3

// Sizes in bytes for packets and ACK packets 
#define PKT_SIZE      DATA_INDEX + PKT_DATA_SIZE
#define ACK_PKT_SIZE  CONN_ID_INDEX + 1

// Max number of retransmits attempted for each packet sent
#define MAX_RETRANSMITS 255

// Max port number
#define MAX_PORT_NUM 3

// Max data elements in each port queue.  Each data element contains the data
// payload for one packet.
#define MAX_QUEUE_LEN 3

#define MAX_CONNECTIONS 4

// Default port to use for transport protocol
#define TRANSPORT_LISTENING_PORT 0

// Maximum possible CTP++ ETX value per hop
#define MAX_ETX_PER_HOP (MAX_LQI / 2)

// Timeout value per hop is 500 ms
#define TIMEOUT_PER_HOP 1000

// Extra timeout margin for initial send
#define EXTRA_INITIAL_TIMEOUT 2000

// The hop count delta that will cause timeout to be recalculated
#define HOP_COUNT_DELTA 1

// Rate at which a packet is transmitted.  One packet is sent every SEND_RATE
// ms
#define SEND_RATE 2500

// TBD: Add support for source port numbers, to allow for simultaneous 
// connections to one dest port for multiple source ports on the same node
// Struct for connection state
typedef struct {
    uint8_t     connectionBusy;
    uint8_t     connectionOpen;
    // uint8_t     connectionClosed;
    uint32_t    currSeqNum;
    uint32_t    recvSeqNum;
    mos_alarm_t ackAlarm; // Alarm for ACK timeout
    uint8_t     ackTimeoutFlag;
    uint8_t     ackRecvd;
    // uint8_t     packetRecvd;
    uint8_t     destNodeId;  // Node id for destination
    uint8_t     srcPortNum;  // Port number for source
    uint8_t     destPortNum; // Port number for destination
    uint16_t    originator;  // Originator of received packet
    
    // Flag to indicate a resend should be performed
    uint8_t     resendFlag;
    
    // Counter for current number of retransmits sent
    uint8_t     numRetransmits;
    
    // Buffer for message to send in connection management reply
    char connReplyMessBuff[PKT_DATA_SIZE];
    
    // Flag set by receiveThread() to indicate that a connection management 
    // reply message should be sent for this connection
    uint8_t sendConnReplyFlag;
} connectState;

// Function prototypes
void sendThread();
void receiveThread();
void ackTimeoutCallBack(void* data);
void addToReceiveQueue(uint8_t *packetData, uint8_t packetDataLen, 
    uint8_t port);
void receivePacket(uint8_t portNum);
int receiveData(uint8_t portNum, uint8_t* buffer, uint8_t bufferLen, 
    uint8_t blocking);
void sendPacket(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen, 
		uint16_t destNodeId);
void sendData(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen, 
		uint16_t destNodeId);
int connect(uint8_t portNum, uint16_t destNodeId);
int closeConn(uint8_t portNum, uint16_t destNodeId);
void sendAck(uint8_t portNum, uint16_t destNodeId);
void transportInit(boolean isBaseNode);

#endif /*TRANSPORT_H_*/
