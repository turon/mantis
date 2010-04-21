#include "mos.h"
#include "msched.h" 
#include "led.h"
#include "dev.h"
#include "com.h"

comBuf send_buf;
uint8_t my_id[4];

void send_thread(void)
{
   uint8_t i;
   for(i = 0; i < sizeof(my_id); ++i)
      send_buf.data[i] = my_id[i];

   strcpy(&send_buf.data[i], "hello, world!");
   send_buf.size = strlen("hello, world!") + sizeof(my_id);
   
   while(1)
   {
      mos_led_toggle(0);
      com_send(IFACE_RADIO, &send_buf);
      mos_thread_sleep(3000);
   }
   
}

void recv_thread(void)
{
   uint8_t sender_id[4];
   uint8_t i;
   
   while(1)
   {
      comBuf* p = com_recv(IFACE_RADIO);
      mos_led_toggle(1);
      
      for(i = 0; i < sizeof(sender_id); ++i)
	 sender_id[i] = p->data[i];

      p->data[p->size] = '\0';
      
      printf("Node 0x%02h%02h%02h%02h says: %s\n",
	     sender_id[0], sender_id[1],
	     sender_id[2], sender_id[3],
	     &p->data[i]);
	     
	  com_free_buf(p);
   }
   
}

void print_thread(void)
{
   printf("this node is Node 0x%02h%02h%02h%02h\n",
	  my_id[0], my_id[1],
	  my_id[2], my_id[3]);

   while(1)
   {
      printf("(this node): hello, world!\n");
      mos_led_toggle(2);
      mos_thread_sleep(3000);
   }
   
}

void start(void)
{
   com_mode(IFACE_RADIO, IF_LISTEN);

   dev_read(DEV_HARDWARE_ID, my_id, sizeof(my_id));
   
   mos_thread_new(send_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(recv_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(print_thread, 128, PRIORITY_NORMAL);
}
