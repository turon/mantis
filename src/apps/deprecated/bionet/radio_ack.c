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

net_event_t *event;
comBuf *recv_pkt;

void ack()
{
   com_mode(IFACE_RADIO,IF_LISTEN);
   while(1)
   {
      mos_led_toggle(2);
      recv_pkt = com_recv(IFACE_RADIO); //blocking recv a packet
      printf("r");
      
      event = (net_event_t *)recv_pkt->data;
      if(event->to == 0)
      { //packet meant for base-station..
            event->to = event->from;
            event->from = 0;
            recv_pkt->size=4;
            com_send (IFACE_RADIO, recv_pkt);
            
      }
      com_free_buf(recv_pkt);
   }
}

void start()
{
   mos_thread_new(ack,128, PRIORITY_NORMAL);
}

