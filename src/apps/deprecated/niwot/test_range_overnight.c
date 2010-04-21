#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "cc1000.h"

static comBuf send_pkt;

void send();

void start()
{
   
   com_mode(IFACE_RADIO, IF_LISTEN);
   com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x25);
   
   mos_thread_new (send, 128, PRIORITY_NORMAL);
    
}

void send()
{
   uint16_t volt = 0;
   uint8_t i;
   send_pkt.size = 3; //3 bytes
   
   while(1) //keep sending until battery dies!!!!
   {  
      mos_thread_sleep(230000);
      for(i=0; i<5; i++)
      { 
         mos_thread_sleep(2000);
         mos_led_toggle(0);
      
         dev_open(DEV_MICA2_BATTERY); //--------------------------------battery----------------------------
         dev_read(DEV_MICA2_BATTERY, &volt, sizeof(volt)); //-----------voltage----------------------------
         dev_close(DEV_MICA2_BATTERY); //--------------------------------read------------------------------
      
         send_pkt.data[1] = volt; //least significant byte loaded
         send_pkt.data[2] = volt/256; //most significant byte loaded
         send_pkt.data[3] = 'q'; //packet marker.  this is our packet
      
         com_send(IFACE_RADIO, &send_pkt);
       }
         
       mos_led_off(0);
   }
   
   while(1)
   {
   send_pkt.data[0] = 0;
   com_send(IFACE_RADIO, &send_pkt);
   }
}

  