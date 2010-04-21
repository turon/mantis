#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "cc1000.h"

void recv();

uint32_t seqno = 0;

void start()
{
   
   com_mode (IFACE_RADIO, IF_LISTEN);
   com_ioctl_IFACE_RADIO (CC1000_TX_POWER, 0x40);
   
   mos_thread_new (recv, 128, PRIORITY_NORMAL);
    
}

void recv()
{

   comBuf *recv_pkt;  //give us a packet pointer
   uint16_t pkts = 0;
   
   printf("seqNo\tvolt\tlinkStrength\t%%pktsRecvd\n");
      
   while(1)
   {
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      
//       if(recv_pkt->data[0] == 0)
//       {
//          printf("Final packets recieved percentage: %d%%\n", pkts/100);
//       	 while(1)
//       	 {
//       	    mos_thread_sleep(1000);
//       	 }
//       }
      
      if(recv_pkt->data[3] == 'q')
      {
      seqno++;
      pkts+=100;      
   printf("%d\t%d\t%d\t\t%d\n",recv_pkt->data[0], recv_pkt->data[1]+(recv_pkt->data[2])*256, recv_pkt->signal, pkts/recv_pkt->data[0]);
      }
      
      

      com_free_buf(recv_pkt);  //free the recv'd packet to the pool
      
      mos_led_toggle(1);
   }
   
}
