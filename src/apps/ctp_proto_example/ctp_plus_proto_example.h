#define APP_DATA  0
#define APP_ACK   1

#define INVALID_NODEID      0xffff
#define ACK_INTERVAL        500
#define APP_SEND_INTERVAL   3500


#define APP_HEADER_SIZE    3
typedef struct app_header_t{
  uint8_t type;
  uint8_t seqNo;
  uint8_t length;  
} app_header_t;


typedef struct app_ack_t {
  uint16_t to_addr;
  uint8_t  ackSeqNo;
  uint8_t freshness;
} app_ack_t;


#ifdef KEEP_CTP_HEADER  
  const char header_info2[] = "\nQueued %s. seq# %C. to %d\n";
  const char header_info[]  = "\nReceived %s. len %C seq# %C. from %d\n";
  const char header_info1[] = "\nReceived unknown App packet. len %C type %C from %d.\n";
#else
  const char header_info2[] = "\nQueued %s. seq# %C\n";
  const char header_info[]  = "\nReceived %s. len %C seq# %C\n";
  const char header_info1[] = "\nReceived unknown App packet. len %C type %C\n";
#endif 
const char header_info3[] = "\nQueued %s. seq# %C. to BASE\n";






   
