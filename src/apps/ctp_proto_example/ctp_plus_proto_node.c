#include "net.h"
#include "ctp_plus.h"
#include "ctp_plus_proto_example.h"


uint16_t app_myid;

void app_send()
{   
  static comBuf dataPacket; 
  static uint8_t seqNo = 0;
    
  dataPacket.size = 3;
  // app header
  dataPacket.data[0] = APP_DATA;  // type
  dataPacket.data[1] = 0;        // seqNo
  dataPacket.data[2] = 0;        // length
  
  while (1) 
  {
    dataPacket.data[1] = seqNo++;
    if( net_send(&dataPacket, CTP_PROTO_ID, CTP_LISTENING_PORT, CTP_NEW_PACKET, AM_CTP_DATA, CTP_LISTENING_PORT) == 2 ) 
    { 
    //  printf("route not found yet. cannot send out data packet.\n");
    }
    else {
    	printf(header_info3, dataPacket.data[0] == APP_ACK?"APP_ACK":"APP_DATA", dataPacket.data[1]);
    }	  
    mos_thread_sleep (APP_SEND_INTERVAL);
   }
}

void app_recv() {
    static comBuf * buffer;
    static uint8_t  headerSize;
    static uint8_t  from_seqNo = 0;
    static uint16_t originator = 0;
   
    while (1) {
        /* receive data frames from base */
        buffer = net_recv(CTP_LISTENING_PORT);

#ifdef KEEP_CTP_HEADER
        /* parse ctp header */
        ctp_dl_data_header_t * dh = (ctp_dl_data_header_t *) buffer->data;
        if((is_base && (dh->type != AM_CTP_DATA)) || ((!is_base) && (dh->type != AM_CTP_DL_DATA)) )
        {
           printf("\nReceived a frame type %C, not %s", dh->type, is_base?"CTP_DATA":"CTP_DL_DATA");
        }
        else 
        {
           originator = (dh->originHigh << 8) | (dh->originLow & 0xff);
           from_seqNo =  dh->originSeqNo;
        }
        headerSize = getHeaderSize(dh->type);
        memcpy(buffer->data, &(buffer->data[headerSize]), buffer->size-headerSize);
        buffer->size -= headerSize;
#endif
        /* parse app layer header. Header format (app_type, seqNo) */
        app_header_t * appHeader = (app_header_t *) buffer->data;
        if(appHeader->type == APP_ACK) { 
           printf(header_info, "APP_ACK", buffer->size, appHeader->seqNo, originator);
        } 
        else if(appHeader->type == APP_DATA) {
           printf(header_info, "APP_DATA", buffer->size, appHeader->seqNo, originator);
        } else {
           printf(header_info1, buffer->size, appHeader->type, originator);
        }  
          
        com_free_buf(buffer);
    }
}

void start(void)
{
  app_myid = mos_node_id_get();

  is_base = 0;
 
  /* must start the net thread */
   net_init();
   
  /* start the CTP backends */
   ctp_proto_init();

  /* wait a while till the routing is possibly established */     
   mos_mdelay(10); 
  
   //net_ioctl(CTP_PROTO_ID, CTP_SET_IS_NODE);
   if(app_myid == 36 || app_myid == 112 || app_myid == 0)
     net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
   else //com_ioctl_IFACE_RADIO(CC2420_HIGH_POWER_MODE);
     com_ioctl_IFACE_RADIO(CC2420_TX_POWER, 35);

  /* This is the basic app thread */
   mos_thread_new(app_recv, 288, PRIORITY_NORMAL);   
   mos_thread_new(app_send, 288, PRIORITY_NORMAL);
  
}
