
#include "config.h"
#include "node_net_event.h"
#include "dev.h"
#include "com.h"
#include "bedrest_shared.h"
#include "bedrest_config.h"
static comBuf out_packet;


void battery_update(void)
{
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICAZ)
   bedrest_t *packet;
   
   net_event_t *event;
   event = (net_event_t *)out_packet.data;
   event->from = 0;
   event->to = 0;
   event->event = BEDREST_PACKET;

   packet = (bedrest_t *)&(out_packet.data[6]);
   packet->temp = 0;
//   packet->light = 0;
   packet->avg_accelx = 0;
   packet->avg_accely = 0;
   packet->txpower = 255;


   uint8_t seqno = 0;
   
   while(1) {
      out_packet.size = 17;
      //battery
      dev_open(DEV_MICA2_BATTERY);
      dev_read(DEV_MICA2_BATTERY, &(packet->battery), sizeof(packet->battery));
      dev_close(DEV_MICA2_BATTERY);

      packet->avg_accelx = seqno++;
      
      com_send(IFACE_SERIAL, &out_packet);
      mos_thread_sleep(5000);
   }
#endif
}


