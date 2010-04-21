#include "mos.h"
#include "telos-flash.h"
#include "com.h"
#include "dev.h"
#include "node_id.h"
#include "printf.h"


#define DEST_ADDRESS 256 // where in flash to start writing the data.
#define PRESERVE_NODE_ID // define to re-write node id after erasing flash.

static comBuf ack;

void start(void)
{
   ack.size = 2;
   ack.data[0] = ':';
   ack.data[1] = ')';
   uint16_t id = mos_node_id_get();
   
   com_mode(IFACE_SERIAL2, IF_LISTEN);
   printf("flash receiver\n");
   printf("erasing flash....");
   
   dev_open(DEV_TELOS_FLASH);
   dev_mode(DEV_TELOS_FLASH, DEV_MODE_ON);
   
   dev_ioctl(DEV_TELOS_FLASH, TELOS_FLASH_BULK_ERASE);

#ifdef PRESERVE_NODE_ID
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, (uint32_t)0);
   dev_write(DEV_TELOS_FLASH, &id, sizeof(id));
#endif
   
   printf("done. Waiting for file.\n");
   
   comBuf* p;

   p = com_recv(IFACE_SERIAL2);
   uint32_t total = buf_extract32(p->data, 0);
  
   printf("%l total bytes to receive (size %d)\n", total, p->size);
   com_send(IFACE_SERIAL2, &ack);
   
   com_free_buf(p);


   
   uint32_t addr = DEST_ADDRESS;
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);

   while(total != 0)
   {
      p = com_recv(IFACE_SERIAL2);
      if (!p)
      {
	 mos_led_toggle(1);
	 continue;
      }
      
      dev_write(DEV_TELOS_FLASH, p->data, p->size);
      addr += p->size;
      total -= p->size;
      dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, addr);
      
      printf("%l bytes left (size %d) wrote @ %x\n", total, p->size, (uint16_t)(addr - p->size));
      mos_led_toggle(0);
      com_send(IFACE_SERIAL2, &ack);
      
      com_free_buf(p);
   }

   printf("all done\n");

   mos_led_on(2);
   dev_close(DEV_TELOS_FLASH);
   
}
