#include "mos.h"
#include "msched.h"
#include "led.h"
#include "avr-adc.h"
//#include "adc.h"
#include "dev.h"
#include "com.h"
#include "printf.h"
#include "bedrest_config.h"
#include "bedrest_shared.h"
#include "node_net_event.h"
#include "cc1000.h"

static comBuf out_packet;
net_event_t *event;
net_event_t *revent;
uint16_t seqNum;
comBuf *recv_packet;


void send()
{
   seqNum = 0;
   out_packet.size = 4;
   event = (net_event_t *) out_packet.data;
   event->to = 0;
   event->from = 10;
   while(1)
   {
      mos_thread_sleep(100);
//      seqNum++;
//      event->from = seqNum;
      com_send(IFACE_RADIO,&out_packet);
      printf(".");
      mos_led_toggle(0);
   }
}

void recv()
{
   mos_led_on(2);
   
   com_mode(IFACE_RADIO,IF_LISTEN);
   while(1)
   {
      recv_packet = com_recv(IFACE_RADIO);
      revent = (net_event_t *)recv_packet->data;
      if (revent->from==0)
      {
         mos_led_toggle(1);
         printf("\b");
      }
      else
         mos_led_toggle(2);
      com_free_buf(recv_packet);
   }
}


void start()
{
   mos_thread_new(send,128, PRIORITY_NORMAL);
   mos_thread_new(recv,128, PRIORITY_NORMAL);
}
