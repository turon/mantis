#include "net.h"
#include "ctp_plus.h"
#include "ctp_plus_proto_example.h"

#define MAX_NUM_ACK     3
#define NEW_ACK_FRESH   3

app_ack_t ackQueue[MAX_NUM_ACK];
uint8_t  ackQueueHead, ackQueueSize;

mos_mutex_t ackQueueMutex;
mos_cond_t  ackQueueCond;

uint16_t app_myid;
boolean  ack_debug = false;

/**  print debug message: ackQueue status **/
void printAckQueue(boolean in)
{
   uint8_t i;
   printf("\n%s ack list(head %C size %C): 1(%d,%C,%C) 2(%d,%C,%C) (%d,%C,%C)%C", in ? ">>" : "<<", 
                              ackQueueHead, ackQueueSize, 
                              ackQueue[0].to_addr, ackQueue[0].ackSeqNo, ackQueue[0].freshness, 
                              ackQueue[1].to_addr, ackQueue[1].ackSeqNo, ackQueue[1].freshness, MAX_NUM_ACK==2?'\n':' ');
   for(i = 2; i < MAX_NUM_ACK; i++) 
     printf("%C(%d,%C,%C)%C", i, ackQueue[i].to_addr, ackQueue[i].ackSeqNo, ackQueue[i].freshness, i==MAX_NUM_ACK-1?'\n':' ');
}  

/* ----------------------------------------------------------- */

void enqueueAck(uint16_t dest, uint8_t seqNo, uint8_t fresh)
{
   uint8_t ackQueueTail;
   if(ackQueueSize < MAX_NUM_ACK) {     
     ackQueueTail =  (ackQueueHead+ackQueueSize) % MAX_NUM_ACK;
     ackQueue[ackQueueTail].to_addr = dest;
     ackQueue[ackQueueTail].ackSeqNo = seqNo;
     ackQueue[ackQueueTail].freshness = fresh;
     ackQueueSize++;
   }
   mos_cond_signal(&ackQueueCond);
}

uint8_t dequeueAck()
{
   uint8_t prev_Qhead;
   prev_Qhead = ackQueueHead;
   ackQueueHead = (ackQueueHead+1) % MAX_NUM_ACK;
   ackQueueSize--;
   return prev_Qhead;
}

void initAckEntry(uint8_t i)
{
   ackQueue[i].to_addr = INVALID_NODEID;
   ackQueue[i].ackSeqNo = 0;
   ackQueue[i].freshness = 0;
}

void app_send()
{   
  static comBuf dataPacket; 
  static uint8_t ack_indx, ackSeqNo, freshness;
  static uint16_t dest;  
  
  dataPacket.size = 3;
  // app header
  dataPacket.data[0] = APP_ACK;  // type
  dataPacket.data[1] = 0;        // seqNo
  dataPacket.data[2] = 0;        // length
  
  while (1) {         
     mos_mutex_lock(&ackQueueMutex);
     while (ackQueueSize == 0) {
        mos_cond_wait(&ackQueueCond, &ackQueueMutex);
     }
     ack_indx = dequeueAck();      
     
     dest = ackQueue[ack_indx].to_addr;
     freshness = ackQueue[ack_indx].freshness;
     ackSeqNo = ackQueue[ack_indx].ackSeqNo;

     /* empty this ack entry */
     initAckEntry(ack_indx);    
     
     mos_mutex_unlock(&ackQueueMutex); 

     /* send ack frame to downstream node */         
     if (dest != INVALID_NODEID)
     {
       dataPacket.data[1] = ackSeqNo;
       if(net_send(&dataPacket, CTP_PROTO_ID, CTP_LISTENING_PORT, CTP_NEW_PACKET, AM_CTP_DL_DATA, dest, CTP_LISTENING_PORT) ==1) 
       {
       	  // successfully enqueued into ctp routing data queue
          printf(header_info2, dataPacket.data[0] == APP_ACK?"APP_ACK":"APP_DATA", ackSeqNo, dest);
       }        
       else 
       { 
         //printf("route not found yet. cannot send out ACK packet to node %d.\n", dest);
         if(freshness > 0) {
            mos_mutex_lock(&ackQueueMutex);
            enqueueAck(dest, ackSeqNo, freshness-1);            
            mos_mutex_unlock(&ackQueueMutex); 
         }
       }
     }       
     mos_thread_sleep (ACK_INTERVAL);
  }
}


void app_recv() {
    static comBuf * buffer;
    static uint8_t  headerSize;
    static uint16_t originator = 0;

    while (1) {
        /* receive data frames from nodes */
        buffer = net_recv(CTP_LISTENING_PORT);   
     
#ifdef KEEP_CTP_HEADER
        ctp_data_header_t * dh = (ctp_data_header_t *) buffer->data;
        if((is_base && (dh->type != AM_CTP_DATA)) || ((!is_base) && (dh->type != AM_CTP_DL_DATA)) ) 
        {
           printf("\nReceived a wrong frame type %C, not %s", dh->type, is_base?"CTP_DATA":"CTP_DL_DATA");
        }
        else
        {
           originator = (dh->originHigh << 8) | (dh->originLow & 0xff);
        }
        headerSize = getHeaderSize(dh->type);
        memcpy(buffer->data, &(buffer->data[headerSize]), buffer->size-headerSize);
        buffer->size -= headerSize;   
#else
        originator = buffer->source;     
#endif
        /* parse app layer header. */
        app_header_t * appHeader = (app_header_t *) buffer->data;
        if(appHeader->type == APP_ACK || appHeader->type == APP_DATA) {        	
           printf(header_info, appHeader->type == APP_ACK?"APP_ACK":"APP_DATA", buffer->size, appHeader->seqNo, originator);
           mos_mutex_lock(&ackQueueMutex);
           enqueueAck(originator, appHeader->seqNo, NEW_ACK_FRESH);
           mos_mutex_unlock(&ackQueueMutex);
        } else {
           printf(header_info1, buffer->size, appHeader->type, originator);
        }      

        com_free_buf(buffer);
        
#ifdef CTP_PLUS_DEBUG        
        printf("\n\ncom free buf 2 %x\n", buffer);
        comBuf * bf = check_free_combuf(); 
        printf("\nfree comBuf pool:\t%x -> %x -> %x\n", bf, bf==NULL?NULL:bf->next, (bf==NULL || bf->next==NULL)?NULL:bf->next->next);        
#endif        
   }  
}

void start(void)
{
  uint8_t i;

  app_myid = mos_node_id_get();

  mos_mutex_init(&ackQueueMutex);
  mos_cond_init(&ackQueueCond);

  ackQueueSize = 0;
  ackQueueHead = 0;
  for(i = 0; i < MAX_NUM_ACK; i++) 
     initAckEntry(i);
  
  is_base = true;
 
  /* must start the net thread */
   net_init();
   
  /* start the CTP backends */
   ctp_proto_init();

  /* wait a while till the routing is possibly established */     
   mos_mdelay(10); 
  
   //net_ioctl(CTP_PROTO_ID, CTP_SET_IS_NODE);
   net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
//com_ioctl_IFACE_RADIO(CC2420_HIGH_POWER_MODE);
//com_ioctl_IFACE_RADIO(CC2420_TX_POWER, 25);

  /* This is the basic app thread */
   mos_thread_new(app_recv, 288, PRIORITY_NORMAL);   
   mos_thread_new(app_send, 288, PRIORITY_NORMAL);
}
